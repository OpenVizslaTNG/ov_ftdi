from migen import *
from misoc.interconnect.stream import Endpoint
from migen.fhdl.bitcontainer import bits_for
from migen.genlib.fsm import FSM, NextState
from migen.genlib.misc import WaitTimer

from ovhw.whacker.util import dmatpl, Acc
from ovhw.constants import *

from ovhw.ov_types import D_LAST, ULPI_DATA_D

def _inc(signal, modulo, dest_signal=None):
    if type(dest_signal) == type(None):
        dest_signal = signal

    assert modulo == 2**len(signal)
    assert len(dest_signal) == len(signal)
    return dest_signal.eq(signal + 1)

class Consumer(Module):
    def __init__(self, port, depth, debug_discard):
        # Sink MUST keep payload constant after asserting stb until ack
        self.sink = Endpoint(dmatpl(depth))
        self.source = Endpoint(D_LAST)
        self.busy = Signal()

        self.pos = Signal(max=depth, reset=0)
        self.pos_next = Signal(max=depth, reset=0)
        self.ct = Signal(max=depth, reset=0)
        self.ct_next = Signal(max=depth)


        self.comb += [
                self.ct_next.eq(self.ct),

                self.pos_next.eq(self.pos),
                port.adr.eq(self.pos_next),
                ]

        self.sync += [
            self.pos.eq(self.pos_next),
            self.ct.eq(self.ct_next)
            ]

        # Convert packet metadata into header fields
        pkt_flags = Signal(8)
        pkt_truncated = Signal()

        self.comb += [
            pkt_truncated.eq(self.sink.payload.count > MAX_PACKET_SIZE),
            pkt_flags.eq(
                Mux(self.sink.payload.flag_first, HF0_FIRST, 0) |
                Mux(self.sink.payload.flag_last, HF0_LAST, 0) |
                Mux(self.sink.payload.flag_ovf, HF0_OVF, 0) |
                Mux(self.sink.payload.flag_err, HF0_ERR, 0) |
                Mux(pkt_truncated, HF0_TRUNC, 0) |
                (self.sink.payload.speed << HF0_SPEED_SHIFT)
            ),
        ]

        # Output variable-length delta timestamp relative to previous packet
        delta_ts_size = Acc(3)
        delta_timestamp = Acc(len(self.sink.payload.ts))
        previous_timestamp = Acc(len(self.sink.payload.ts))
        self.submodules += previous_timestamp, delta_timestamp, delta_ts_size

        # Prevent last byte waiting excessive amount of time on SDRAM write by
        # lazily sending filler magic byte if total number of bytes sent is odd
        filler_needed = Acc(1)
        filler_timer = WaitTimer(FILLER_TIMEOUT)
        self.submodules += filler_needed, filler_timer
        self.comb += filler_needed.set(filler_needed.v ^ (self.source.stb & self.source.ack))

        self.submodules.fsm = FSM()

        self.fsm.act("IDLE",
            self.busy.eq(0),
            filler_timer.wait.eq(filler_needed.v & ~self.sink.stb),
            If(self.sink.stb,
                self.busy.eq(1),
                delta_timestamp.set(self.sink.payload.ts - previous_timestamp.v),
                self.ct_next.eq(Mux(pkt_truncated, MAX_PACKET_SIZE, self.sink.payload.count)),
                NextState("PACKET"),
            ).Elif(filler_timer.done,
                NextState("FILLER"),
            )
        )

        self.fsm.act("PACKET",
            If(delta_timestamp.v[56:64],
                delta_ts_size.set(7)
            ).Elif(delta_timestamp.v[48:56],
                delta_ts_size.set(6)
            ).Elif(delta_timestamp.v[40:48],
                delta_ts_size.set(5)
            ).Elif(delta_timestamp.v[32:40],
                delta_ts_size.set(4)
            ).Elif(delta_timestamp.v[24:32],
                delta_ts_size.set(3)
            ).Elif(delta_timestamp.v[16:24],
                delta_ts_size.set(2)
            ).Elif(delta_timestamp.v[8:16],
                delta_ts_size.set(1)
            ).Else(
                delta_ts_size.set(0)
            ),
            If(self.sink.payload.discard & ~debug_discard,
                If(self.ct,
                    # Update consumer watermark
                    self.pos_next.eq(self.sink.payload.start - 1 + self.ct),
                ),
                # Metadata is no longer needed
                self.sink.ack.eq(1),
                NextState("IDLE")
            ).Else(
                # Delta timestamp will be sent, update base timestamp
                previous_timestamp.set(self.sink.payload.ts),
                If(self.sink.payload.discard,
                    NextState("WH0D")
                ).Else(
                    NextState("WH0")
                )
            )
        )

        def write_hdr(statename, nextname, val):
            self.fsm.act(statename,
                self.busy.eq(1),
                self.source.stb.eq(1),
                self.source.payload.d.eq(val),
                If(self.source.ack,
                    NextState(nextname)
                )
            )

        # Header format:
        # A0 - magic byte (A2 if packet is discarded and debug is enabled)
        # F0 - flags
        # SL SH - packet size (lower 13 bits) and delta timestamp size (3 bits)
        # T0 T1 T2 T3 T4 T5 T6 T7 - delta timestamp from previous packet
        # d0....dN - captured USB packet data
        #
        # Delta timestamp size is 1 + value encoded in delta timestamp bits.
        # Captured USB packet data is at most MAX_PACKET_SIZE bytes long. Packet
        # size can be greater than MAX_PACKET_SIZE, however such size indicate
        # errors either on the bus itself (babble) or capture path.
        #
        # Flags format
        #
        # F0.0 - ERR  - Line level error (ULPI.RXERR asserted during packet)
        # F0.1 - OVF  - RX Path Overflow (can happen on high-speed traffic)
        # F0.2 - CLIP - Filter clipped (we do not set yet)
        # F0.3 - PERR - Protocol level err (but ULPI was fine, ???)
        write_hdr("WH0", "WRF0", 0xA0)
        write_hdr("WH0D", "WRF0", 0xA2)
        write_hdr("WRF0", "WRSL", pkt_flags)
        # 13 bits (8191) for packet size is more than enough
        # Use the upper 3 bits to encode delta timestamp size
        write_hdr("WRSL", "WRSH", self.sink.payload.count[:8])
        write_hdr("WRSH", "WRTS", delta_ts_size.v[:3] << 5 | self.sink.payload.count[8:13])

        # Write variable length delta timestamp value
        self.fsm.act("WRTS",
            self.busy.eq(1),
            self.source.stb.eq(1),
            self.source.payload.d.eq(delta_timestamp.v[:8]),
            If(self.source.ack,
                If(delta_ts_size.v,
                    delta_ts_size.set(delta_ts_size.v - 1),
                    delta_timestamp.set(delta_timestamp.v[8:])
                ).Elif(self.ct,
                    self.pos_next.eq(self.sink.payload.start),
                    self.ct_next.eq(self.ct - 1),
                    NextState("d")
                ).Else(
                    # No actual payload, it is a stuff packet
                    self.sink.ack.eq(1),
                    NextState("IDLE")
                )
            )
        )

        self.fsm.act("d",
                self.busy.eq(1),
                self.source.stb.eq(1),
                self.source.payload.d.eq(port.dat_r),

                If(self.ct == 0,
                    self.source.payload.last.eq(1)),

                If(self.source.ack,
                    If(self.ct,
                        _inc(self.pos, depth, self.pos_next),
                        self.ct_next.eq(self.ct - 1),
                    ).Else(
                        self.sink.ack.eq(1),
                        NextState("IDLE")
                    )
                )
            )

        self.fsm.act("FILLER",
            self.busy.eq(1),
            self.source.stb.eq(1),
            self.source.payload.d.eq(FILLER_MAGIC),
            self.source.payload.last.eq(1),

            If(self.source.ack,
                NextState("IDLE")
            )
        )
