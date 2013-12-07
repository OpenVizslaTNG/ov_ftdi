#!/usr/bin/env python3

from migen.fhdl.std import *
from migen.genlib.cdc import NoRetiming, MultiReg
from migen.genlib.fifo import AsyncFIFO
from migen.genlib.fsm import FSM, NextState
from migen.genlib.record import Record, DIR_M_TO_S
from migen.flow.network import DataFlowGraph, CompositeActor
from migen.flow.actor import Source, Sink
import migen.actorlib.fifo as al_fifo
from migen.bus.csr import Interconnect
from migen.bank.csrgen import BankArray
from migen.bank.description import AutoCSR, CSRStorage, CSRStatus, CSR

import ovhw.clocking as clocking
from ovhw.sdramctl import SDRAMCTL
from ovhw.sdram_mux import SdramMux
from ovhw.sdram_bist import SdramBist
from ovhw.sdrambistcfg import SdramBistCfg
from ovhw.ulpi import ULPI_ctrl, ULPI_pl, ULPI_BUS, ULPI_REG, ULPI_DATA
from ovhw.leds import LED_outputs
from ovhw.buttons import BTN_status
from ovhw.whacker.whacker import Whacker
from ovhw.ovf_insert import OverflowInserter
from ovhw.cmdproc import CmdProc
from ovhw.ftdi_bus import FTDI_sync245
from ovhw.ftdi_lfsr_test import FTDI_randtest
from ovhw.ulpicfg import ULPICfg
from ovhw.cfilt import RXCmdFilter
import ovplatform.sdram_params

class OV3(Module):
    def __init__(self, plat):
        # Clocking
        clk_ref = plat.request("clk50")
        self.submodules.clockgen = clocking.ClockGen(clk_ref)
        self.clock_domains.cd_sys = self.clockgen.cd_sys

        # SDRAM Controller
        sd_param = ovplatform.sdram_params.getSDRAMParams('mt48lc16m16a2')
        self.submodules.sdramctl = SDRAMCTL(
            plat.request("sdram"),
            clk_out=self.clockgen.clk_sdram,
            clk_sample=self.clockgen.clk_sdram_sample,
            **sd_param._asdict()
        )

        # SDRAM Master arbiter
        self.submodules.sdram_mux = SdramMux(self.sdramctl.hostif)

        # SDRAM BIST
        memsize = 2 ** (sd_param.colbits + sd_param.rowbits + sd_param.bankbits)
        self.submodules.bist = SdramBist(self.sdram_mux.getPort(), memsize)
        self.submodules.sdram_test = SdramBistCfg(self.bist)


        # ULPI Interfce

        # Diagnostics/Testing signals
        ulpi_cd_rst = Signal()
        ulpi_stp_ovr = Signal(1)
        
        # ULPI physical layer
        self.submodules.ulpi_pl = ULPI_pl(
            plat.request("ulpi"), ulpi_cd_rst, ulpi_stp_ovr)
        self.clock_domains.cd_ulpi = self.ulpi_pl.cd_ulpi
        
        # ULPI controller
        ulpi_reg = Record(ULPI_REG)
        self.submodules.ulpi = RenameClockDomains(
          ULPI_ctrl(self.ulpi_pl.ulpi_bus, ulpi_reg),
          {"sys": "ulpi"}
        )

        # ULPI register R/W CSR interface
        self.submodules.ucfg = ULPICfg(
            self.cd_ulpi.clk, ulpi_cd_rst, self.ulpi_pl.ulpi_bus.rst,
            ulpi_stp_ovr, ulpi_reg)


        # Receive Path
        self.submodules.ovf_insert = RenameClockDomains(
            OverflowInserter(),
            {"sys": "ulpi"}
        )

        self.submodules.udata_fifo = RenameClockDomains(
            al_fifo.AsyncFIFO(ULPI_DATA, 1024),
            {"write":"ulpi", "read":"sys"}
        )

        self.submodules.cfilt = RXCmdFilter()
        self.submodules.cstream = Whacker(1024)
        self.comb += [
                self.ovf_insert.sink.connect(self.ulpi.data_out_source),
                self.udata_fifo.sink.connect(self.ovf_insert.source),
                self.cfilt.sink.connect(self.udata_fifo.source),
                self.cstream.sink.connect(self.cfilt.source)
                ]


        # FTDI bus interface
        ftdi_io = plat.request("ftdi")
        self.submodules.ftdi_bus = ftdi_bus = FTDI_sync245(self.clockgen.cd_sys.rst,
                ftdi_io)

        # FTDI command processor
        self.submodules.randtest = FTDI_randtest()
        self.submodules.cmdproc = CmdProc(self.ftdi_bus,
                [self.randtest, self.cstream])

        # GPIOs (leds/buttons)
        self.submodules.leds = LED_outputs(plat.request('leds'),
                [
                    [self.bist.busy, self.ftdi_bus.tx_ind],
                    [0, self.ftdi_bus.rx_ind],
                    [0]
                ], active=0)

        self.submodules.buttons = BTN_status(~plat.request('btn'))


        # Bind all device CSRs
        self.csr_map = {
                'leds': 0,
                'buttons' : 1,
                'ucfg' : 2,
                'randtest' : 3,
                'cstream' : 4,
                'sdram_test' : 5,
                }

        self.submodules.csrbankarray = BankArray(self,
            lambda name, _: self.csr_map[name])

        # Connect FTDI CSR Master to CSR bus
        self.submodules.incon = Interconnect(self.cmdproc.master, self.csrbankarray.get_buses())
