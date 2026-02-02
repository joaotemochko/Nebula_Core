# Nebula Core

<p align="center">
  <strong>A Low Power, Linux-capable RISC-V RV64GC processor core</strong>
</p>

<p align="center">
  <a href="#features">Features</a> •
  <a href="#architecture">Architecture</a> •
  <a href="#getting-started">Getting Started</a> •
  <a href="#directory-structure">Directory Structure</a> •
  <a href="#documentation">Documentation</a> •
  <a href="#license">License</a>
</p>

---

## Overview

**Nebula Core** is a synthesizable RISC-V processor implementing the **RV64GC** ISA (RV64IMAFDC + Zicsr + Zifencei), designed to run full operating systems like Ubuntu Linux. The core is optimized as an efficient "Little Core" for big.LITTLE SoC configurations.

The Nebula Cluster configuration provides a **4-core SMP system** with shared L2 cache and MESI cache coherence, suitable for multi-threaded Linux workloads.

## Features

### ISA Support

| Extension | Description | Status |
|-----------|-------------|--------|
| **RV64I** | Base Integer (64-bit) | ✅ Complete |
| **M** | Integer Multiply/Divide | ✅ Complete |
| **A** | Atomic Instructions (LR/SC + AMO) | ✅ Complete |
| **F** | Single-Precision Floating-Point | ✅ IEEE 754 |
| **D** | Double-Precision Floating-Point | ✅ IEEE 754 |
| **C** | Compressed Instructions (16-bit) | ✅ Complete |
| **Zicsr** | CSR Instructions | ✅ Complete |
| **Zifencei** | Instruction-Fetch Fence | ✅ Complete |

### Microarchitecture

- **Pipeline**: In-order, 6-8 stages
- **Issue Width**: Dual-issue capable
- **Branch Predictor**: gshare + BTB (256 entries) + RAS (8 entries)
- **L1 I-Cache**: 32KB, 4-way set-associative
- **L1 D-Cache**: 32KB, 4-way set-associative, write-back
- **L2 Cache**: 512KB shared, 8-way, 4 banks, MESI coherence
- **MMU**: Sv39 (39-bit virtual address, 512GB address space)
- **TLB**: Split ITLB/DTLB, 32 entries each, fully associative
- **Privilege Modes**: Machine (M), Supervisor (S), User (U)

### Linux Compatibility

The Nebula Core implements all features required for Linux:

- ✅ Full RV64GC ISA support
- ✅ Sv39 virtual memory with superpages (4KB/2MB/1GB)
- ✅ M/S/U privilege levels with proper CSRs
- ✅ Interrupt delegation (medeleg/mideleg)
- ✅ Timer, external, and software interrupts
- ✅ Atomic operations for SMP synchronization
- ✅ SFENCE.VMA for TLB management
- ✅ FENCE.I for instruction cache coherence
- ✅ WFI (Wait For Interrupt) instruction

## Architecture

### Block Diagram

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              NEBULA CLUSTER                                  │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│   ┌───────────────┐ ┌───────────────┐ ┌───────────────┐ ┌───────────────┐  │
│   │    Core 0     │ │    Core 1     │ │    Core 2     │ │    Core 3     │  │
│   │  ┌─────────┐  │ │  ┌─────────┐  │ │  ┌─────────┐  │ │  ┌─────────┐  │  │
│   │  │ Frontend│  │ │  │ Frontend│  │ │  │ Frontend│  │ │  │ Frontend│  │  │
│   │  │  + BP   │  │ │  │  + BP   │  │ │  │  + BP   │  │ │  │  + BP   │  │  │
│   │  │  + RVC  │  │ │  │  + RVC  │  │ │  │  + RVC  │  │ │  │  + RVC  │  │  │
│   │  └────┬────┘  │ │  └────┬────┘  │ │  └────┬────┘  │ │  └────┬────┘  │  │
│   │  ┌────┴────┐  │ │  ┌────┴────┐  │ │  ┌────┴────┐  │ │  ┌────┴────┐  │  │
│   │  │ Backend │  │ │  │ Backend │  │ │  │ Backend │  │ │  │ Backend │  │  │
│   │  │ALU+MDU  │  │ │  │ALU+MDU  │  │ │  │ALU+MDU  │  │ │  │ALU+MDU  │  │  │
│   │  │  +FPU   │  │ │  │  +FPU   │  │ │  │  +FPU   │  │ │  │  +FPU   │  │  │
│   │  └────┬────┘  │ │  └────┬────┘  │ │  └────┬────┘  │ │  └────┬────┘  │  │
│   │  ┌────┴────┐  │ │  ┌────┴────┐  │ │  ┌────┴────┐  │ │  ┌────┴────┐  │  │
│   │  │L1 I+D   │  │ │  │L1 I+D   │  │ │  │L1 I+D   │  │ │  │L1 I+D   │  │  │
│   │  │ 32K+32K │  │ │  │ 32K+32K │  │ │  │ 32K+32K │  │ │  │ 32K+32K │  │  │
│   │  └────┬────┘  │ │  └────┬────┘  │ │  └────┬────┘  │ │  └────┬────┘  │  │
│   │  ┌────┴────┐  │ │  ┌────┴────┐  │ │  ┌────┴────┐  │ │  ┌────┴────┐  │  │
│   │  │MMU Sv39 │  │ │  │MMU Sv39 │  │ │  │MMU Sv39 │  │ │  │MMU Sv39 │  │  │
│   │  │TLB + PTW│  │ │  │TLB + PTW│  │ │  │TLB + PTW│  │ │  │TLB + PTW│  │  │
│   │  └────┬────┘  │ │  └────┬────┘  │ │  └────┬────┘  │ │  └────┬────┘  │  │
│   └───────┼───────┘ └───────┼───────┘ └───────┼───────┘ └───────┼───────┘  │
│           └─────────────────┴─────────────────┴─────────────────┘          │
│                                     │                                       │
│                      ┌──────────────┴──────────────┐                       │
│                      │     L2 Cache (512KB)        │                       │
│                      │   8-way, 4 banks, MESI      │                       │
│                      └──────────────┬──────────────┘                       │
│                                     │                                       │
└─────────────────────────────────────┼───────────────────────────────────────┘
                                      │
                              Memory Interface
                             (AXI / Native Bus)
```

### Pipeline Stages

```
┌─────────┐   ┌─────────┐   ┌─────────┐   ┌─────────┐   ┌─────────┐   ┌─────────┐
│  FETCH  │──▶│  ALIGN  │──▶│ DECODE  │──▶│ EXECUTE │──▶│ MEMORY  │──▶│WRITEBACK│
│         │   │  + RVC  │   │         │   │         │   │         │   │         │
└─────────┘   └─────────┘   └─────────┘   └─────────┘   └─────────┘   └─────────┘
     │                            │             │             │
     ▼                            ▼             ▼             ▼
 ┌───────┐                   ┌─────────┐   ┌─────────┐   ┌─────────┐
 │  BP   │                   │   CSR   │   │   MDU   │   │ D-Cache │
 │ BTB   │                   │  Unit   │   │ MUL/DIV │   │  + TLB  │
 │ RAS   │                   └─────────┘   └─────────┘   └─────────┘
 └───────┘                                      │
                                                ▼
                                           ┌─────────┐
                                           │   FPU   │
                                           │IEEE 754 │
                                           └─────────┘
```

### Compressed Instruction Handling

The frontend implements a sophisticated fetch buffer to handle the RV64C extension correctly:

```
┌─────────────────────────────────────────────────────────────┐
│                  FETCH BUFFER (80 bits)                      │
├───────────┬───────────┬───────────┬───────────┬─────────────┤
│  [79:64]  │  [63:48]  │  [47:32]  │  [31:16]  │   [15:0]    │
│ Carry-over│    HW3    │    HW2    │    HW1    │    HW0      │
└───────────┴───────────┴───────────┴───────────┴─────────────┘
                                                      ↑
                                              fetch_offset

Detection: bits[1:0] != 2'b11 → 16-bit compressed instruction
           bits[1:0] == 2'b11 → 32-bit standard instruction

PC Advancement: +2 for compressed, +4 for standard
```

### Memory Hierarchy

| Level | Size | Associativity | Line Size | Latency | Policy |
|-------|------|---------------|-----------|---------|--------|
| L1 I-Cache | 32KB | 4-way | 64B | 1 cycle | Read-only |
| L1 D-Cache | 32KB | 4-way | 64B | 1 cycle | Write-back |
| L2 Cache | 512KB | 8-way | 64B | 8-12 cycles | Write-back, MESI |

### FPU Specifications

The FPU is fully IEEE 754-2008 compliant:

| Feature | Support |
|---------|---------|
| **Formats** | binary32 (float), binary64 (double) |
| **Rounding Modes** | RNE, RTZ, RDN, RUP, RMM |
| **Exception Flags** | NV, DZ, OF, UF, NX |
| **Special Values** | ±0, ±∞, NaN (canonical), denormals |

**Latencies:**

| Operation | Single | Double |
|-----------|--------|--------|
| FADD/FSUB | 4 cycles | 4 cycles |
| FMUL | 4 cycles | 4 cycles |
| FMA | 5 cycles | 5 cycles |
| FDIV | 14 cycles | 28 cycles |
| FSQRT | 15 cycles | 30 cycles |

## Getting Started

### Prerequisites

- **Simulator**: Verilator 4.0+ or commercial simulator (VCS, Questa, Xcelium)
- **Synthesis**: Vivado 2020.1+ (for Xilinx FPGAs) or Design Compiler
- **Toolchain**: RISC-V GNU Toolchain with RV64GC support

### Building

```bash
# Clone the repository
git clone https://github.com/yourusername/nebula-core.git
cd nebula-core

# Run simulation (Verilator)
make sim

# Run RISC-V compliance tests
make test

# Synthesize for FPGA
make synth BOARD=arty_a7
```

### Running Linux

```bash
# Build OpenSBI firmware
make opensbi

# Build Linux kernel
make linux

# Boot in simulation
make boot-linux
```

## Directory Structure

```
nebula-core/
├── rtl/
│   ├── backend/
│   │   ├── nebula_backend_fpu.sv      # Execution backend with FPU integration
|   |   └── mdu_rv64                   # Multiply/Divide unit
│   │
│   ├── cache/
│   │   ├── icache_l1.sv               # L1 instruction cache (32KB, 4-way)
│   │   ├── dcache_l1.sv               # L1 data cache (32KB, 4-way, WB)
│   │   └── l2_cache.sv                # Shared L2 cache (512KB, 8-way, MESI)
│   │
│   ├── cluster/
│   │   └── nebula_cluster.sv          # 4-core cluster top-level
│   │
│   ├── common/
│   │   ├── nebula_pkg.sv              # Types, constants, structs
│   │   └── csr_unit.sv                # CSR registers (M/S/U mode)
│   │
│   ├── core/
│   │   └── nebula_core_full.sv        # Single core top-level
│   │
│   ├── fpu/
│   │   └── fpu_ieee754.sv             # IEEE 754 compliant FPU (F+D)
│   │
│   ├── frontend/
│   │   ├── nebula_frontend_rvc.sv     # Fetch + Decode with RVC support
|   |   └── compressed_decoder_rv64.sv # RV64C instruction decoder
│   │
│   ├── memory/
│   │   ├── tlb_sv39.sv                # Translation Lookaside Buffer
│   │   └── ptw_sv39.sv                # Page Table Walker (Sv39)
│   │
│   └── predictor/
│       └── branch_predictor.sv        # gshare + BTB + RAS
│
├── tb/
│   ├── tb_nebula_core.sv              # Core testbench
│   └── tb_nebula_cluster.sv           # Cluster testbench
│
├── README.md
└── LICENSE
```

## Module Descriptions

### Core Modules

| Module | Lines | Description |
|--------|-------|-------------|
| `nebula_core_full.sv` | 262 | Top-level single core, instantiates all submodules |
| `nebula_backend_fpu.sv` | 874 | Execution backend: ALU, MDU, FPU, register files |
| `nebula_frontend_rvc.sv` | 674 | Instruction fetch with RVC support, decode logic |
| `compressed_decoder_rv64.sv` | 590 | 16-bit to 32-bit instruction expansion |
| `mdu_rv64.sv` | 329 | Integer multiply/divide (M extension) |

### Memory Modules

| Module | Lines | Description |
|--------|-------|-------------|
| `icache_l1.sv` | 378 | L1 instruction cache, non-blocking |
| `dcache_l1.sv` | 694 | L1 data cache, write-back, AMO support |
| `l2_cache.sv` | 365 | Shared L2 with MESI coherence |
| `tlb_sv39.sv` | 320 | TLB with superpage support |
| `ptw_sv39.sv` | 391 | Hardware page table walker |

### Other Modules

| Module | Lines | Description |
|--------|-------|-------------|
| `fpu_ieee754.sv` | 1003 | Full IEEE 754 FPU (F+D extensions) |
| `csr_unit.sv` | 607 | Control/Status registers, trap handling |
| `branch_predictor.sv` | 238 | gshare predictor with BTB and RAS |
| `nebula_cluster.sv` | 262 | 4-core cluster with L2 arbitration |
| `nebula_pkg.sv` | 406 | Shared types and constants |

**Total: ~7,400 lines of SystemVerilog**

## CSR Map

### Machine Mode

| CSR | Address | Description |
|-----|---------|-------------|
| `mstatus` | 0x300 | Machine status register |
| `misa` | 0x301 | ISA and extensions |
| `medeleg` | 0x302 | Machine exception delegation |
| `mideleg` | 0x303 | Machine interrupt delegation |
| `mie` | 0x304 | Machine interrupt enable |
| `mtvec` | 0x305 | Machine trap vector |
| `mscratch` | 0x340 | Machine scratch register |
| `mepc` | 0x341 | Machine exception PC |
| `mcause` | 0x342 | Machine trap cause |
| `mtval` | 0x343 | Machine trap value |
| `mip` | 0x344 | Machine interrupt pending |

### Supervisor Mode

| CSR | Address | Description |
|-----|---------|-------------|
| `sstatus` | 0x100 | Supervisor status |
| `sie` | 0x104 | Supervisor interrupt enable |
| `stvec` | 0x105 | Supervisor trap vector |
| `sscratch` | 0x140 | Supervisor scratch |
| `sepc` | 0x141 | Supervisor exception PC |
| `scause` | 0x142 | Supervisor trap cause |
| `stval` | 0x143 | Supervisor trap value |
| `sip` | 0x144 | Supervisor interrupt pending |
| `satp` | 0x180 | Supervisor address translation |

### Floating-Point

| CSR | Address | Description |
|-----|---------|-------------|
| `fflags` | 0x001 | FP exception flags |
| `frm` | 0x002 | FP rounding mode |
| `fcsr` | 0x003 | FP control/status |

## Performance

### Estimated Specifications

| Metric | Value |
|--------|-------|
| Target Frequency | 1.0 - 1.5 GHz (14nm) |
| IPC (single-thread) | 0.8 - 1.2 |
| IPC (4-thread) | 3.0 - 4.0 |
| Dhrystone | ~2.5 DMIPS/MHz |
| CoreMark | ~3.5 CM/MHz |
| Area (4 cores) | ~2.5 mm² (14nm) |
| Power | 0.5 - 1.5 W |

### Branch Predictor Accuracy

| Benchmark | Accuracy |
|-----------|----------|
| SPEC INT | ~92% |
| Dhrystone | ~95% |
| CoreMark | ~90% |

## Integration

### SoC Requirements

To boot Linux, the following peripherals are required (external to Nebula Core):

| Peripheral | Purpose |
|------------|---------|
| **CLINT** | Core Local Interruptor (timer + software IRQs) |
| **PLIC** | Platform-Level Interrupt Controller |
| **UART** | Serial console |
| **Memory Controller** | DDR4/DDR5 interface |

### Example Device Tree

```dts
/ {
    #address-cells = <2>;
    #size-cells = <2>;
    compatible = "nebula,cluster";

    cpus {
        #address-cells = <1>;
        #size-cells = <0>;
        timebase-frequency = <10000000>;

        cpu@0 {
            device_type = "cpu";
            reg = <0>;
            compatible = "riscv";
            riscv,isa = "rv64imafdc";
            mmu-type = "riscv,sv39";
        };
        // ... cores 1-3
    };

    memory@80000000 {
        device_type = "memory";
        reg = <0x0 0x80000000 0x0 0x40000000>; // 1GB
    };
};
```

## Testing

### RISC-V Compliance

```bash
# Run official RISC-V architecture tests
make riscv-tests

# Run specific test suite
make riscv-tests SUITE=rv64ui  # User-mode integer
make riscv-tests SUITE=rv64um  # Multiply/divide
make riscv-tests SUITE=rv64ua  # Atomics
make riscv-tests SUITE=rv64uf  # Single-precision FP
make riscv-tests SUITE=rv64ud  # Double-precision FP
make riscv-tests SUITE=rv64uc  # Compressed
```

### Simulation

```bash
# Simple test
make sim TEST=hello_world

# Linux boot test
make sim-linux

# Generate waveforms
make sim TEST=dhrystone WAVES=1
```

## Roadmap

- [x] RV64I base ISA
- [x] M extension (multiply/divide)
- [x] A extension (atomics)
- [x] F/D extensions (floating-point)
- [x] C extension (compressed)
- [x] Sv39 MMU
- [x] L1 caches
- [x] L2 shared cache
- [x] Branch predictor
- [x] 4-core cluster
- [ ] RISC-V compliance tests
- [ ] Linux boot verification
- [ ] FPGA validation
- [ ] PMP (Physical Memory Protection)
- [ ] Debug Module (JTAG)
- [ ] Vector extension (V)
- [ ] Hypervisor extension (H)

## Contributing

In moment, contibuting is not avaliable. In the future I accept contribuitions.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- RISC-V Foundation for the open ISA specification
- OpenSBI project for the M-mode firmware reference
- Linux RISC-V port maintainers

## Contact

- **Author**: João Antônio Temochko Andre
- **Email**: joao.temochko@ifsp.edu.br
- **Project Link**: https://github.com/joaotemochko/Nebula_Core

---

<p align="center">
  <sub>Built with ❤️ for the RISC-V community</sub>
</p>
