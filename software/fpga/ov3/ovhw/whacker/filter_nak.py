# Copyright (c) 2023 Tomasz Mon <desowin@gmail.com>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *
from misoc.interconnect.stream import Endpoint, SyncFIFO
from migen.genlib.fsm import FSM, NextState

from ovhw.constants import *
from ovhw.whacker.util import *


class FilterNAK(Module):
    def __init__(self, depth, enable):
        self.input = Endpoint(dmatpl(depth))
        self.output = Endpoint(dmatpl(depth))

        pid_is_out = Signal()
        pid_is_data0 = Signal()
        pid_is_ping = Signal()
        pid_is_split = Signal()
        pid_is_in = Signal()
        pid_is_nak = Signal()
        pid_is_data1 = Signal()

        self.comb += [
            # When enable is 0 all pid checks will be 0 and thus state machine
            # will enter FORWARD and then alternate between DEFAULT and FORWARD
            If(self.input.payload.pid_valid & enable,
                pid_is_out.eq(self.input.payload.pid == PID_OUT),
                pid_is_data0.eq(self.input.payload.pid == PID_DATA0),
                pid_is_ping.eq(self.input.payload.pid == PID_PING),
                pid_is_split.eq(self.input.payload.pid == PID_SPLIT),
                pid_is_in.eq(self.input.payload.pid == PID_IN),
                pid_is_nak.eq(self.input.payload.pid == PID_NAK),
                pid_is_data1.eq(self.input.payload.pid == PID_DATA1),
            )
        ]

        self.submodules.queue = SyncFIFO(dmatpl(depth), 3)

        self.comb += [
            self.queue.sink.payload.eq(self.input.payload),
            # Connect all payload signals except discard
            self.output.payload.ts.eq(self.queue.source.payload.ts),
            self.output.payload.pid.eq(self.queue.source.payload.pid),
            self.output.payload.pid_valid.eq(self.queue.source.payload.pid_valid),
            self.output.payload.flag_first.eq(self.queue.source.payload.flag_first),
            self.output.payload.flag_last.eq(self.queue.source.payload.flag_last),
            self.output.payload.flag_ovf.eq(self.queue.source.payload.flag_ovf),
            self.output.payload.flag_err.eq(self.queue.source.payload.flag_err),
            self.output.payload.speed.eq(self.queue.source.payload.speed),
            self.output.payload.start.eq(self.queue.source.payload.start),
            self.output.payload.count.eq(self.queue.source.payload.count),
        ]

        self.submodules.fsm = FSM()

        self.fsm.act("DEFAULT",
            self.queue.sink.stb.eq(self.input.stb),
            self.input.ack.eq(self.queue.sink.ack),
            If(self.input.stb & self.queue.sink.ack,
                If(pid_is_split,
                    NextState("SPLIT")
                ).Elif(pid_is_in | pid_is_ping,
                    NextState("HANDSHAKE")
                ).Elif(pid_is_out,
                    NextState("OUT")
                ).Else(
                    NextState("FORWARD")
                )
            )
        )

        self.fsm.act("SPLIT",
            If(self.input.stb,
                If(pid_is_split,
                    # Do not ack input, it will be acked in DEFAULT state
                    NextState("FORWARD")
                ).Else(
                    # No matter what it is, forward it. NAK filter currently
                    # does not support filtering NAKed SPLIT transactions.
                    self.queue.sink.stb.eq(self.input.stb),
                    self.input.ack.eq(self.queue.sink.ack),
                    If(self.queue.sink.ack,
                        NextState("FORWARD")
                    )
                )
            )
        )

        self.fsm.act("HANDSHAKE",
            If(self.input.stb,
                If(pid_is_nak,
                    self.queue.sink.stb.eq(self.input.stb),
                    self.input.ack.eq(self.queue.sink.ack),
                    If(self.queue.sink.ack,
                        NextState("DISCARD")
                    )
                ).Else(
                    # Do not ack input, it will be acked in DEFAULT state
                    NextState("FORWARD")
                )
            )
        )

        self.fsm.act("OUT",
            If(self.input.stb,
                If(pid_is_data0 | pid_is_data1,
                    self.queue.sink.stb.eq(self.input.stb),
                    self.input.ack.eq(self.queue.sink.ack),
                    If(self.queue.sink.ack,
                        NextState("HANDSHAKE")
                    )
                ).Else(
                    # Do not ack input, it will be acked in DEFAULT state
                    NextState("FORWARD")
                )
            )
        )

        self.fsm.act("FORWARD",
            If(self.queue.source.stb,
                # Forward all queued packets, keeping discard signal intact
                self.output.payload.discard.eq(self.queue.source.payload.discard),
                self.output.stb.eq(self.queue.source.stb),
                self.queue.source.ack.eq(self.output.ack),
            ).Else(
                NextState("DEFAULT")
            )
        )

        self.fsm.act("DISCARD",
            If(self.queue.source.stb,
                # Forward all queued packets, ensuring discard signal is set
                self.output.payload.discard.eq(1),
                self.output.stb.eq(self.queue.source.stb),
                self.queue.source.ack.eq(self.output.ack),
            ).Else(
                NextState("DEFAULT")
            )
        )
