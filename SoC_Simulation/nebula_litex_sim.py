#!/usr/bin/env python3
"""
nebula_litex_sim.py — Simulação LiteX do Nebula Core
"""

import os
from migen import *
from litex.build.generic_platform import *
from litex.build.sim import SimPlatform
from litex.build.sim.config import SimConfig
from litex.soc.cores.cpu import CPUS
from litex.soc.cores.cpu import CPU, CPU_GCC_TRIPLE_RISCV64
from litex.soc.interconnect import axi
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *


# =============================================================================
# 1. Definição do CPU para o ecossistema LiteX
# =============================================================================
class NebulaCPU(CPU):
    name                 = "nebula"
    data_width           = 64
    endianness           = "little"
    gcc_triple           = CPU_GCC_TRIPLE_RISCV64
    linker_output_format = "elf64-littleriscv"
    nop                  = "nop"
    io_regions           = {0xF0000000: 0x10000000} # FIX: Periféricos no topo da memória
    family               = "riscv"
    category             = "softcore"
    variants             = ["standard"]

    @property
    def mem_map(self):
        return {
            "rom":      0x10000000,
            "sram":     0x11000000,
            "main_ram": 0x40000000, # FIX: RAM realocada para o padrão da indústria
            "csr":      0xF0000000, # FIX: CSRs e UART no topo
        }

    @property
    def gcc_flags(self):
        return "-mcmodel=medany -march=rv64imafdc_zicsr_zifencei -mabi=lp64d -D__nebula__"

    def __init__(self, platform, variant="standard"):
        super().__init__()
        self.platform = platform
        self.reset    = Signal()
        self.reset_address = 0x10000000

        # AXI de 64 bits para o LiteX
        self.axi      = axi.AXIInterface(data_width=64, address_width=32, id_width=4)
        self.periph_buses = [self.axi]
        self.memory_buses = [self.axi]

    def do_finalize(self):
        rtl_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "rtl"))

        # Package primeiro
        pkg_file = os.path.join(rtl_dir, "common/nebula_pkg.sv")
        self.platform.add_source(pkg_file)

        # Resto do RTL
        self.platform.add_source_dir(rtl_dir)

        # =====================================================================
        # Instância do nebula_core_axi_top
        # =====================================================================
        self.specials += Instance("nebula_core_axi_top",
            i_clk   = ClockSignal("sys"),
            i_rst_n = (ResetSignal("sys") == 0),

            i_i_timer_irq    = 0,
            i_i_external_irq = 0,
            i_i_software_irq = 0,

            i_i_m_axi_i_arready = 0,
            i_i_m_axi_i_rdata   = 0,
            i_i_m_axi_i_rresp   = 0,
            i_i_m_axi_i_rlast   = 0,
            i_i_m_axi_i_rvalid  = 0,

            o_o_m_axi_d_awvalid = self.axi.aw.valid,
            i_i_m_axi_d_awready = self.axi.aw.ready,
            o_o_m_axi_d_awaddr  = self.axi.aw.addr,
            o_o_m_axi_d_awid    = self.axi.aw.id,
            o_o_m_axi_d_awlen   = self.axi.aw.len,
            o_o_m_axi_d_awsize  = self.axi.aw.size,
            o_o_m_axi_d_awburst = self.axi.aw.burst,

            o_o_m_axi_d_wvalid  = self.axi.w.valid,
            i_i_m_axi_d_wready  = self.axi.w.ready,
            o_o_m_axi_d_wdata   = self.axi.w.data,
            o_o_m_axi_d_wstrb   = self.axi.w.strb,
            o_o_m_axi_d_wlast   = self.axi.w.last,

            i_i_m_axi_d_bvalid  = self.axi.b.valid,
            o_o_m_axi_d_bready  = self.axi.b.ready,
            i_i_m_axi_d_bresp   = self.axi.b.resp,
            i_i_m_axi_d_bid     = self.axi.b.id,

            o_o_m_axi_d_arvalid = self.axi.ar.valid,
            i_i_m_axi_d_arready = self.axi.ar.ready,
            o_o_m_axi_d_araddr  = self.axi.ar.addr,
            o_o_m_axi_d_arid    = self.axi.ar.id,
            o_o_m_axi_d_arlen   = self.axi.ar.len,
            o_o_m_axi_d_arsize  = self.axi.ar.size,
            o_o_m_axi_d_arburst = self.axi.ar.burst,

            i_i_m_axi_d_rvalid  = self.axi.r.valid,
            o_o_m_axi_d_rready  = self.axi.r.ready,
            i_i_m_axi_d_rdata   = self.axi.r.data,
            i_i_m_axi_d_rresp   = self.axi.r.resp,
            i_i_m_axi_d_rlast   = self.axi.r.last,
        )

CPUS[NebulaCPU.name] = NebulaCPU

# =============================================================================
# 2. IOs virtuais para o simulador
# =============================================================================
_io = [
    ("sys_clk", 0, Pins(1)),
    ("sys_rst", 0, Pins(1)),
    ("serial", 0,
        Subsignal("source_valid", Pins(1)),
        Subsignal("source_ready", Pins(1)),
        Subsignal("source_data",  Pins(8)),
        Subsignal("sink_valid",   Pins(1)),
        Subsignal("sink_ready",   Pins(1)),
        Subsignal("sink_data",    Pins(8)),
    ),
]

# =============================================================================
# 3. CRG — domínio de relógio
# =============================================================================
class CRG(Module):
    def __init__(self, sys_clk, sys_rst):
        self.clock_domains.cd_sys = ClockDomain()
        self.comb += [
            self.cd_sys.clk.eq(sys_clk),
            self.cd_sys.rst.eq(sys_rst),
        ]

# =============================================================================
# 4. SoC completo
# =============================================================================
class NebulaSimSoC(SoCCore):
    def __init__(self):
        platform     = SimPlatform("sim", _io)
        sys_clk_freq = int(1e6)

        SoCCore.__init__(self, platform, clk_freq=sys_clk_freq,
            cpu_type              = "nebula",
            integrated_rom_size   = 0x8000,
            integrated_main_ram_size = 0x100000,
            uart_name             = "sim",
        )
        self.add_constant("ROM_BOOT_ADDRESS", self.mem_map["rom"])
        self.add_constant("SIM_TRACE", 1)

        self.add_config("BIOS_NO_MEMTEST")
        self.add_config("BIOS_NO_DELAYS")

        sys_clk = platform.request("sys_clk")
        sys_rst = platform.request("sys_rst")
        self.submodules.crg = CRG(sys_clk, sys_rst)

# =============================================================================
# 5. Compilação e simulação
# =============================================================================
if __name__ == "__main__":
    soc     = NebulaSimSoC()
    builder = Builder(soc, compile_software=True, compile_gateware=True)

    sim_config = SimConfig(default_clk="sys_clk")
    sim_config.add_module("serial2console", "serial")

    builder.build(sim_config=sim_config, run=True, trace=True)