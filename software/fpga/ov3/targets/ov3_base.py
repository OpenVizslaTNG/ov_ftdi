#!/usr/bin/env python3
"""Shared OV3 board base.

Contains clock and reset generator that shall be common to every OV3 bitstream.
"""

from migen import *
from ovhw.ftdi_bus import FTDI_sync245
from ovhw.ulpi import ULPI_pl


class _CRG(Module):
    def __init__(self, platform):
        clkin = platform.request("clk12") # 12mhz reference clock from which all else is derived

        self.clk_sys = Signal()
        self.clk_sdram = Signal()
        self.clk_sdram_sample = Signal()
        self.cd_sys = ClockDomain()
        self.pll_locked = Signal()

        clkout0, clkout2 = Signal(), Signal()
        dcm_locked = Signal()
        clk2x = Signal()   # raw DCM CLK2X output (drives the PLL directly)
        clkfb = Signal()   # CLK2X buffered through a BUFG, fed back to CLKFB

        feedback = Signal()
        # Simple 2X: 12MHz -> 24MHz
        #
        # Feedback is buffered to match UG382 Spartan-6 FPGA Clocking Resources
        # Figure 3-15: DCM Driving a PLL. We do use CLK2X instead of CLK0 as
        # feedback in order to meet minimum PLL frequency requirement.
        # All BUFGs are in the central CLKC tile (they are global, region-less),
        # so the loop runs DCM(bottom) -> BUFG(center) -> global GCLK spine ->
        # DCM.CLKFB; the DLL compensates the whole loop delay.
        #
        # Input chain: P50(clk12) -> IBUFG -> BUFIO2 -> DCM.CLKIN.
        # IBUFG + BUFIO2 are made explicit only so the BUFIO2 can be LOC'd
        clkin_g = Signal()
        clkin_b = Signal()
        self.specials += Instance("IBUFG", i_I=clkin, o_O=clkin_g)
        self.specials.bufio2 = Instance("BUFIO2",
            i_I=clkin_g, o_DIVCLK=clkin_b,
            p_DIVIDE=1, p_DIVIDE_BYPASS="TRUE",
            p_I_INVERT="FALSE", p_USE_DOUBLER="FALSE")
        self.specials.dcm = Instance("DCM_SP",
            i_CLKIN=clkin_b, i_CLKFB=clkfb, i_RST=0, i_PSEN=0,
            o_CLK2X=clk2x, o_LOCKED=dcm_locked,
            p_CLK_FEEDBACK="2X")
        self.specials += Instance("BUFG", i_I=clk2x, o_O=clkfb)

        # VCO 400.1000MHz
        # PFD 19..400MHz
        # 24MHz in, /1 24MHz PFD, x25 600MHz VCO, /6 100MHz CLKOUT
        self.specials.pll = Instance("PLL_BASE",
            i_CLKIN=clk2x, i_CLKFBIN=feedback, i_RST=~dcm_locked,
            o_CLKFBOUT=feedback, o_CLKOUT0=clkout0, o_CLKOUT1=self.clk_sdram,
            o_CLKOUT2=clkout2, o_LOCKED = self.pll_locked,
            p_BANDWIDTH="LOW",
            p_COMPENSATION="DCM2PLL", p_CLK_FEEDBACK="CLKFBOUT",
            p_DIVCLK_DIVIDE=1, p_CLKFBOUT_MULT=25,
            p_CLKOUT0_DIVIDE=6, p_CLKOUT0_PHASE=0.0,
            p_CLKOUT1_DIVIDE=6, p_CLKOUT1_PHASE=180.0,
            p_CLKOUT2_DIVIDE=6, p_CLKOUT2_PHASE=180.0,
            p_CLKOUT3_DIVIDE=18, p_CLKOUT3_PHASE=0.0,
        )

        # Pin the whole clock chain into clk12's clock region (X0Y0) so the
        # pin -> buffer -> DCM path is region-coherent:
        #   P50 (region X0Y0)
        #     -> BUFIO2_X1Y1   (region X0Y0)
        #     -> DCM_X0Y0      (region X0Y0)
        #     -> PLL_ADV_X0Y0  (region X0Y1; the bottom CMT's PLL and the only
        #                       PLL anywhere near the bottom-edge clk12 pin).
        # DCM_X0Y0 and PLL_ADV_X0Y0 are in the same (bottom) CMT, so the
        # dedicated in-tile DCM->PLL route required by COMPENSATION=DCM2PLL is
        # used; the DCM->PLL region boundary is crossed only on that hard route.
        platform.add_platform_command("""
INST "{dcm}" LOC = "DCM_X0Y0";
INST "{pll}" LOC = "PLL_ADV_X0Y0";
INST "{bufio2}" LOC = "BUFIO2_X1Y1";
""", dcm=self.dcm, pll=self.pll, bufio2=self.bufio2)

        self.specials += [
            Instance("BUFG", i_I=clkout0, o_O=self.clk_sys),
            Instance("BUFG", i_I=clkout2, o_O=self.clk_sdram_sample),
        ]

        # Reset generator: 4 cycles in reset after PLL is locked
        rst_ctr = Signal(max=4)
        self.clock_domains.cd_rst = ClockDomain()
        self.cd_sys.rst.reset = 1
        self.sync.rst += If(rst_ctr == 3,
                            self.cd_sys.rst.eq(0)
                         ).Else(
                            rst_ctr.eq(rst_ctr+1)
                         )
        self.comb += [
            self.cd_rst.clk.eq(self.clk_sys),
            self.cd_rst.rst.eq(~self.pll_locked),
            self.cd_sys.clk.eq(self.clk_sys),
        ]


class OV3BaseSoC(Module):
    def __init__(self, plat):
        # Clocking
        self.submodules.crg = _CRG(plat)
        self.clock_domains.cd_sys = self.crg.cd_sys

        # Request init_b to prevent unused pin pull-down from forming voltage
        # divider with the external pull-up
        self.add_dummy_input(plat.request("init_b"))

    def add_dummy_input(self, pin):
        # Connect input pin to permanently disabled (CE=0) ILOGIC FF
        dummy_input_q = Signal()
        dummy_input_q.attr.add(("S", "TRUE"))

        self.specials += Instance("FDE",
            i_D=pin, i_C=ClockSignal(), i_CE=dummy_input_q, o_Q=dummy_input_q,
            attr={("IOB", "TRUE")})

    def _connect_ulpi(self, ulpi_pins, ulpi_pl):
        # Combinational signals: rst, nxt, dir
        self.comb += [
            ulpi_pins.rst.eq(ulpi_pl.rst),
            ulpi_pl.nxt.eq(ulpi_pins.nxt),
            ulpi_pl.dir.eq(ulpi_pins.dir),
        ]

        # STP: registered output, IOB packed
        ulpi_pins.stp.attr.add(("IOB", "TRUE"))
        self.sync.ulpi += ulpi_pins.stp.eq(ulpi_pl.stp)

        # Registered data tristate enable, default to Hi-Z at reset.
        # Disable tristate (drive data bus) according to d_oe signal.
        # Enable tristate during turnaround cycle (when dir goes high), so we
        # are already Hi-Z once turnaround cycle ends.
        t_ff = Signal(reset=1)
        self.specials += Instance("FDPE",
            p_INIT=1, i_D=~ulpi_pl.d_oe, i_PRE=ulpi_pins.dir,
            i_CE=1, i_C=ClockSignal("ulpi"), o_Q=t_ff,
            attr={("IOB", "TRUE")})

        # Data bus: per-bit registered output, combinational input, IOB packed
        for i in range(8):
            o_ff = Signal(name="ulpi_dq%d_o" % i, reset_less=True)
            o_ff.attr.add(("IOB", "TRUE"))
            self.sync.ulpi += o_ff.eq(ulpi_pl.d_o[i])
            self.specials += Instance("IOBUF",
                i_I=o_ff, o_O=ulpi_pl.d_i[i], i_T=t_ff,
                io_IO=ulpi_pins.d[i],
                p_DRIVE=2, p_SLEW="QUIETIO",
            )

    def add_ulpi_pl(self, ulpi_pins, ulpi_cd_rst, ulpi_stp_ovr):
        # ULPI clock domain (driven by USB3343 PHY).
        self.clock_domains.cd_ulpi = ClockDomain()
        self.cd_ulpi.clk = ulpi_pins.clk
        self.cd_ulpi.rst = ulpi_cd_rst

        # Instantiate ULPI physical layer logical adapter and connect signals
        self.submodules.ulpi_pl = ULPI_pl(ulpi_stp_ovr)
        self._connect_ulpi(ulpi_pins, self.ulpi_pl)
        return self.ulpi_pl

    def _connect_ftdi(self, ftdi_pins, ftdi_bus):
        # Combinational signals: FPGA inputs (RXF#, TXE#) + async SIWUA#
        self.comb += [
            ftdi_bus.rxf_n.eq(ftdi_pins.rxf_n),
            ftdi_bus.txe_n.eq(ftdi_pins.txe_n),
            ftdi_pins.siwua_n.eq(ftdi_bus.siwua_n),
        ]

        # Registered control outputs, IOB packed
        # Generate internal single-fanout signals here that are eligible for IOB
        # packing and for which XST correctly infers FF's INIT value.
        ftdi_rd_n = Signal(reset=ftdi_bus.rd_n.reset)
        ftdi_oe_n = Signal(reset=ftdi_bus.oe_n.reset)
        ftdi_wr_n = Signal(reset=ftdi_bus.wr_n.reset)
        self.sync.ftdi += [
            ftdi_rd_n.eq(ftdi_bus.rd_n),
            ftdi_oe_n.eq(ftdi_bus.oe_n),
            ftdi_wr_n.eq(ftdi_bus.wr_n),
        ]
        self.comb += [
            ftdi_pins.rd_n.eq(ftdi_rd_n),
            ftdi_pins.oe_n.eq(ftdi_oe_n),
            ftdi_pins.wr_n.eq(ftdi_wr_n),
        ]
        ftdi_pins.rd_n.attr.add(("IOB", "TRUE"))
        ftdi_pins.oe_n.attr.add(("IOB", "TRUE"))
        ftdi_pins.wr_n.attr.add(("IOB", "TRUE"))

        # Registered data tristate enable, default to Hi-Z at reset
        t_ff = Signal(reset=1)
        t_ff.attr.add(("IOB", "TRUE"))
        self.sync.ftdi += t_ff.eq(~ftdi_bus.d_oe)

        # Data bus: per-bit registered output, combinational input, IOB packed
        for i in range(8):
            o_ff = Signal(name="ftdi_dq%d_o" % i)
            o_ff.attr.add(("IOB", "TRUE"))
            self.sync.ftdi += o_ff.eq(ftdi_bus.d_o[i])
            self.specials += Instance("IOBUF",
                i_I=o_ff, o_O=ftdi_bus.d_i[i], i_T=t_ff,
                io_IO=ftdi_pins.d[i],
                p_DRIVE=4, p_SLEW="FAST",
            )

    def add_ftdi_bus(self, ftdi_pins):
        # FTDI clock domain (driven by FT2232H)
        self.clock_domains.cd_ftdi = ClockDomain()
        self.cd_ftdi.clk = ftdi_pins.clk
        self.cd_ftdi.rst = self.crg.cd_sys.rst

        # Instantiate FTDI bus controller and connect signals
        self.submodules.ftdi_bus = ftdi_bus = FTDI_sync245()
        self._connect_ftdi(ftdi_pins, ftdi_bus)
        return ftdi_bus
