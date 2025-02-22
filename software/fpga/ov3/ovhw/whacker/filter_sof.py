# Copyright (c) 2023 Tomasz Mon <desowin@gmail.com>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *
from misoc.interconnect.stream import Endpoint

from ovhw.constants import *
from ovhw.whacker.util import *


class FilterSOF(Module):
    def __init__(self, depth, enable):
        self.input = Endpoint(dmatpl(depth))
        self.output = Endpoint(dmatpl(depth))

        discard = Signal()

        self.comb += [
            If(self.input.payload.pid_valid & enable,
                discard.eq(self.input.payload.pid == PID_SOF),
            ),
            # Pass through all signals from input to output
            self.output.stb.eq(self.input.stb),
            self.input.ack.eq(self.output.ack),
            self.output.eop.eq(self.input.eop),
            self.output.payload.ts.eq(self.input.payload.ts),
            self.output.payload.pid.eq(self.input.payload.pid),
            self.output.payload.pid_valid.eq(self.input.payload.pid_valid),
            self.output.payload.flag_first.eq(self.input.payload.flag_first),
            self.output.payload.flag_last.eq(self.input.payload.flag_last),
            self.output.payload.flag_ovf.eq(self.input.payload.flag_ovf),
            self.output.payload.flag_err.eq(self.input.payload.flag_err),
            self.output.payload.speed.eq(self.input.payload.speed),
            self.output.payload.start.eq(self.input.payload.start),
            self.output.payload.count.eq(self.input.payload.count),
            # Actual filter - set discard if packet is SOF and filter enabled
            self.output.payload.discard.eq(self.input.payload.discard | discard),
        ]
