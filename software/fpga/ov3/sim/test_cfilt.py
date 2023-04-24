import unittest

from migen import *
from migen.sim import run_simulation, passive

from ovhw.constants import *
from ovhw.ov_types import ULPI_DATA_TAG
from ovhw.cfilt import RXCmdFilter


# USB334x Table 6-3: ULPI RX CMD encondig
RXEVENT_IDLE = 0
RXEVENT_ACTIVE = 1
RXEVENT_ERROR = 2
RXEVENT_DISCONNECT = 3


def process_ulpi(byte_list):
    # Process ULPI data entrirely in Python to generate expected output that is
    # later checked against the simulated migen module.
    packet = False
    expected = []
    for i, (rxcmd, d) in enumerate(byte_list):
        rx_event = None
        is_start = is_end = is_ovf = is_nop = is_err = 0
        if rxcmd:
            is_start = (d == RXCMD_MAGIC_SOP)
            is_end = (d == RXCMD_MAGIC_EOP)
            is_ovf = (d == RXCMD_MAGIC_OVF)
            is_nop = (d == RXCMD_MAGIC_NOP)
            is_err = 0

            if d & ~RXCMD_MASK:
                # It has to be one of out magic values
                assert is_start or is_end or is_ovf or is_nop
            else:
                # RX CMD as reported by ULPI PHY
                rx_event = (d & 0x30) >> 4

            if packet:
                is_err = (rx_event == RXEVENT_ERROR)
                is_end |= (rx_event == RXEVENT_IDLE)
                if is_end or is_ovf or is_err:
                    packet = False
                else:
                    # Filter out RX CMD
                    continue
            else:
                is_start |= (rx_event == RXEVENT_ACTIVE)
                if is_nop or not is_start:
                    # Filter out RX CMD
                    continue
                packet = True
        elif not packet:
            # Packet byte outside packet boundary (either overflow or bug)
            continue

        expected.append({"ts": i, "d": d, "is_start": int(is_start),
            "is_end": int(is_end), "is_ovf": int(is_ovf),
            "is_err": int(is_err), "speed": int(0)})

    return expected


class TestBench(Module):
    def __init__(self):
        self.submodules.tr = RXCmdFilter()
        self.comb += self.tr.source.ack.eq(self.tr.source.stb)
        self.byte_list = [(1,0x40), (0,0xCA), (1,0x10), (0, 0xFE), (1, 0x41)]

    @passive
    def reader(self, output):
        while True:
            if (yield self.tr.source.stb):
                out = {}
                for field in [item[0] for item in ULPI_DATA_TAG]:
                    out[field] = yield getattr(self.tr.source.payload, field)
                output.append(out)
                print(out)
            yield

    def gen(self, byte_list):
        for _ in range(0, 6):
            yield
        for rxcmd, b in byte_list:
            print("WR %s" % repr((rxcmd, b)))
            yield self.tr.sink.stb.eq(1)
            yield self.tr.sink.payload.d.eq(b)
            yield self.tr.sink.payload.rxcmd.eq(rxcmd)
            yield
            # Make every other sys clock cycle without data
            yield self.tr.sink.stb.eq(0)
            yield


class TestRXCmdFilter(unittest.TestCase):
    def setUp(self):
        self.tb = TestBench()
    
    def test_cfilt(self):
        inp = [(1,0x40), (0,0xCA), (1,0x10), (0, 0xFE), (1, 0x41)]
        output = []
        expected = process_ulpi(inp)
        run_simulation(self.tb, [self.tb.gen(inp), self.tb.reader(output)],
                       vcd_name="cfilt.vcd")
        self.assertSequenceEqual(expected, output)


if __name__ == "__main__":
    unittest.main()
