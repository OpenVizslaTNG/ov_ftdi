from migen import *
from migen.fhdl.bitcontainer import bits_for

def dmatpl(depth):
    b = bits_for(depth-1)
    return [('ts', 64),
            # USB PID used by filters. Do not use if pid_valid is 0.
            ('pid', 4),
            ('pid_valid', 1),
            # Filter indicates packet should be discarded
            ('discard', 1),
            # Capture flags. Note that TRUNC is derived from count.
            ('flag_first', 1),
            ('flag_last', 1),
            ('flag_ovf', 1),
            ('flag_err', 1),
            ('speed', 2),
            # Start address of actual USB packet start in ring buffer
            ('start', b),
            # Packet size, but only up to MAX_PACKET_SIZE bytes are captured
            ('count', 13)]

class Acc(Module):
    def __init__(self, *args, **kwargs):
        self.v = Signal(*args, **kwargs)

        self._n = Signal(*args, **kwargs)
        self._s = Signal(1)

        self.sync += If(self._s, self.v.eq(self._n))

    def set(self, val):
        return self._n.eq(val), self._s.eq(1)

class Acc_inc(Acc):
    def inc(self):
        return self._n.eq(self.v+1), self._s.eq(1)

class Acc_inc_sat(Acc):
    def inc(self):
        return If(self.v != (1<<len(self.v))-1, self._n.eq(self.v+1), self._s.eq(1))

class Acc_or(Acc):
    def _or(self, v):
        return self._n.eq(self.v | v), self._s.eq(1)

