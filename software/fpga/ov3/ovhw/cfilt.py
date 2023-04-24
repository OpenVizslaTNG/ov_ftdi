from migen import *
from migen.genlib.fsm import FSM, NextState
from misoc.interconnect.stream import Endpoint

from ovhw.constants import *
from ovhw.ov_types import ULPI_DATA_D, ULPI_DATA_TAG

class RXCmdFilter(Module):
    # Merges/drops unnecessary RXCMDs for packet parsing

    def __init__(self):
        self.sink = Endpoint(ULPI_DATA_D)
        self.source = Endpoint(ULPI_DATA_TAG)

        is_sop = Signal()
        is_eop = Signal()
        is_ovf = Signal()
        is_nop = Signal()

        is_active = Signal()
        is_nactive = Signal()
        is_error = Signal()

        ts_counter = Signal(len(self.source.payload.ts))

        self.comb += [
                is_sop.eq(self.sink.payload.rxcmd & (self.sink.payload.d == RXCMD_MAGIC_SOP)),
                is_eop.eq(self.sink.payload.rxcmd & (self.sink.payload.d == RXCMD_MAGIC_EOP)),
                is_ovf.eq(self.sink.payload.rxcmd & (self.sink.payload.d == RXCMD_MAGIC_OVF)),
                is_nop.eq(self.sink.payload.rxcmd & (self.sink.payload.d == RXCMD_MAGIC_NOP)),

                is_active.eq(self.sink.payload.rxcmd & 
                    ~self.sink.payload.d[6] & 
                    (self.sink.payload.d[4:6] == 0x1)),
                is_nactive.eq(self.sink.payload.rxcmd &
                    ~self.sink.payload.d[6] &
                    (self.sink.payload.d[4:6] == 0x0)),
                is_error.eq(self.sink.payload.rxcmd & 
                    ~self.sink.payload.d[6] & 
                    (self.sink.payload.d[4:6] == 0x3)),

                self.source.payload.d.eq(self.sink.payload.d),
                self.source.payload.speed.eq(self.sink.payload.speed),
                self.source.payload.ts.eq(ts_counter)
                ]

        self.sync += If(self.sink.ack, ts_counter.eq(ts_counter + 1))

        self.submodules.fsm = FSM()

        def pass_(state):
            return send(state, 0, 0, 0, 0)

        def send(state, is_start, is_end, is_err, is_ovf):
            return [
                self.source.stb.eq(1),
                self.source.payload.is_start.eq(is_start),
                self.source.payload.is_end.eq(is_end),
                self.source.payload.is_err.eq(is_err),
                self.source.payload.is_ovf.eq(is_ovf),
                If(self.source.ack,
                    self.sink.ack.eq(1),
                    NextState(state)
                )
                ]

        def skip(state):
            return [
                self.sink.ack.eq(1),
                NextState(state)
                ]

        def act(state, *args):
            self.fsm.act(state, 
                If(self.sink.stb,
                    If(~self.sink.payload.rxcmd,
                        pass_(state)
                    ).Elif(is_nop,
                        self.sink.ack.eq(1),
                    ).Else(*args)))


        act("NO_PACKET",
            If(is_sop | is_active,
                send("PACKET", 1, 0, 0, 0)
            ).Else(
                skip("NO_PACKET")
            ))

        act("PACKET",
            If(is_eop | is_nactive,
                send("NO_PACKET", 0, 1, 0, 0)
            ).Elif(is_error,
                send("NO_PACKET", 0, 0, 1, 0)
            ).Elif(is_ovf,
                send("NO_PACKET", 0, 0, 0, 1)
            ).Else(
                skip("PACKET")
            ))
