from migen import *
from misoc.interconnect.stream import Endpoint
from migen.genlib.fsm import FSM, NextState

from ovhw.ov_types import ULPI_DATA_TAG
from ovhw.constants import *
from ovhw.whacker.util import *


class Producer(Module):

    def __init__(self, wrport, depth, consume_watermark, ena, la_filters=[]):
        self.ulpi_sink = Endpoint(ULPI_DATA_TAG)

        self.out_addr = Endpoint(dmatpl(depth))


        # Produce side
        self.submodules.produce_write = Acc_inc(max=depth)
        self.submodules.produce_header = Acc(max=depth)

        self.submodules.pid = Acc(4)
        self.submodules.pid_valid = Acc(1)

        self.submodules.discard = Acc(1)

        self.submodules.size = Acc_inc_sat(13)
        self.submodules.flag_first = Acc(1)
        self.submodules.flag_last = Acc(1)
        self.submodules.flag_ovf = Acc(1)
        self.submodules.flag_err = Acc(1)
        self.submodules.speed = Acc(2)

        self.submodules.to_start = Acc(1)

        self.submodules.fsm = FSM()

        has_space = Signal()

        self.comb += has_space.eq(((consume_watermark - self.produce_write.v - 1) & (depth - 1)) > 1)

        # Grab packet timestamp at SOP or on stuffed packet
        grab_timestamp = Signal()
        pkt_timestamp = Signal(len(self.ulpi_sink.payload.ts))
        self.sync += If(grab_timestamp | (self.ulpi_sink.payload.is_start & self.ulpi_sink.ack),
                pkt_timestamp.eq(self.ulpi_sink.payload.ts))

        payload_is_rxcmd = Signal()
        self.comb += payload_is_rxcmd.eq(
            self.ulpi_sink.payload.is_start | 
            self.ulpi_sink.payload.is_end | 
            self.ulpi_sink.payload.is_err |
            self.ulpi_sink.payload.is_ovf)

        # Packet first/last bits
        clear_acc_flags = Signal()

        en_last = Signal()
        self.sync += en_last.eq(ena)
        self.submodules.packet_first = Acc(1)
        self.submodules.packet_last = Acc(1)

        # Stuff-packet bit
        # At start-of-capture or end-of-capture, we stuff a packet to
        # indicate the exact time of capture
        stuff_packet = Signal()
        self.comb += stuff_packet.eq(self.packet_first.v | self.packet_last.v)

        self.comb += If(ena & ~en_last, 
            self.packet_first.set(1)).Elif(clear_acc_flags,
            self.packet_first.set(0))

        self.comb += If(~ena & en_last, 
            self.packet_last.set(1)).Elif(clear_acc_flags,
            self.packet_last.set(0))

        # Combine outputs of filters
        la_resets = [f.reset.eq(1) for f in la_filters]
        filter_done = 1
        filter_reject = 0
        for f in la_filters:
            filter_done = f.done & filter_done
            filter_reject = f.reject | filter_reject

        self.fsm.act("IDLE",
                If(
                    ((self.ulpi_sink.stb | self.to_start.v) & ena 
                     | stuff_packet) & has_space,

                    If(~self.to_start.v | (self.ulpi_sink.stb & stuff_packet), self.ulpi_sink.ack.eq(1)),

                    # Produce header equals write when there is no USB data.
                    # Produce header points to first captured USB data byte and
                    # produce write points after last written byte.
                    self.produce_write.set(self.produce_header.v),
                    self.pid.set(0),
                    self.pid_valid.set(0),
                    self.discard.set(0),
                    self.size.set(0),
                    self.flag_first.set(self.packet_first.v),
                    self.flag_last.set(self.packet_last.v),
                    self.flag_ovf.set(0),
                    self.flag_err.set(0),
                    self.speed.set(self.ulpi_sink.payload.speed),
                    self.to_start.set(0),

                    la_resets,
                    
                    If(self.ulpi_sink.payload.is_start | self.to_start.v,
                        NextState("DATA")

                    ).Elif(stuff_packet,
                        # Capture reference timestamp
                        grab_timestamp.eq(1),
                        NextState("waitdone"),
                    )

                # If not enabled, we just dump RX'ed data
                ).Elif(~ena,
                    self.ulpi_sink.ack.eq(1)
                )
        )

        do_filter_write = Signal()

        # Feed data to lookaside filters
        for f in la_filters:
            self.comb += [
                f.write.eq(do_filter_write),
                f.dat_w.eq(self.ulpi_sink.payload)
            ]
            
        packet_too_long = Signal()
        self.comb += packet_too_long.eq(self.size.v >= MAX_PACKET_SIZE)

        self.fsm.act("DATA",
                If(has_space & self.ulpi_sink.stb,
                    self.ulpi_sink.ack.eq(1),
                    If(payload_is_rxcmd,

                        # Got another start-of-packet
                        If(self.ulpi_sink.payload.is_start,
                            self.flag_ovf.set(1),

                            # If we got a SOP, we need to skip RXCMD det in IDLE
                            self.to_start.set(1)

                        # Mark error if we hit an error
                        ).Elif(self.ulpi_sink.payload.is_err,
                            self.flag_err.set(1),

                        # Mark overflow if we got a stuffed overflow
                        ).Elif(self.ulpi_sink.payload.is_ovf,
                            self.flag_ovf.set(1),
                        ),

                        # In any case (including END), we're done RXing
                        NextState("waitfilter")
                    ).Else(
                        self.size.inc(),
                        If(~packet_too_long,
                            self.produce_write.inc(),
                            wrport.adr.eq(self.produce_write.v),
                            wrport.dat_w.eq(self.ulpi_sink.payload.d),
                            wrport.we.eq(1),
                            do_filter_write.eq(1)
                        ),
                        If(self.size.v == 0,
                            self.pid.set(self.ulpi_sink.payload.d[:4]),
                            self.pid_valid.set(self.ulpi_sink.payload.d[:4] == Cat(~self.ulpi_sink.payload.d[4:8])),
                        )
                    )
                )
            )

        self.fsm.act("waitfilter",
            If(filter_done,
                self.discard.set(filter_reject),
                NextState("waitdone")
            )
        )

        self.fsm.act("waitdone",
            clear_acc_flags.eq(self.flag_first.v | self.flag_last.v),
            NextState("SEND")
        )

        self.fsm.act("SEND",
            self.out_addr.stb.eq(1),
            self.out_addr.payload.ts.eq(pkt_timestamp),
            self.out_addr.payload.pid.eq(self.pid.v),
            self.out_addr.payload.pid_valid.eq(self.pid_valid.v),
            self.out_addr.payload.discard.eq(self.discard.v),
            self.out_addr.payload.flag_first.eq(self.flag_first.v),
            self.out_addr.payload.flag_last.eq(self.flag_last.v),
            self.out_addr.payload.flag_ovf.eq(self.flag_ovf.v),
            self.out_addr.payload.flag_err.eq(self.flag_err.v),
            self.out_addr.payload.speed.eq(self.speed.v),
            self.out_addr.payload.start.eq(self.produce_header.v),
            self.out_addr.payload.count.eq(self.size.v),
            If(self.out_addr.ack,
                self.produce_header.set(self.produce_write.v),
                NextState("IDLE")
            )
        )

