# Nebula Core

<p align="center">
  <strong>Núcleo RISC-V RV64GC dual-issue in-order capaz de rodar Linux</strong>
</p>

---

## Visão geral

**Nebula Core** é um processador RISC-V **RV64IMAFDC + Zicsr + Zifencei** com modos
M/S/U e memória virtual **Sv39**, escrito em SystemVerilog. O pipeline é
**dual-issue em ordem (in-order)**: emite, executa e efetiva até 2 instruções por
ciclo, sempre na ordem do programa.

Estado verificado (simulação Verilator):

| Verificação | Resultado |
|---|---|
| riscv-tests (rv64ui/um/ua/uc/uf/ud/si/mi, ambientes `-p` e `-v`) | **241/244** (as 3 restantes exigem acesso desalinhado em HW e PMP — opcionais) |
| Co-simulação lockstep com Spike (toda instrução comparada) | sem divergências em ISA, Embench e boot do Linux |
| Embench IoT (19 benchmarks) | **19/19 corretos**, IPC média geométrica **1.28**, IPC ≥ 1.0 em **18/19** |
| Linux 6.19 (OpenSBI 1.7 + kernel + initramfs BusyBox/glibc) | boota até o shell e executa comandos |

## Microarquitetura

```
 ┌──────────── FETCH ────────────┐   ┌──── DECODE/ISSUE ────┐   ┌──── EX ────┐   ┌──── MEM ────┐   ┌──── WB ────┐
 │ PC ─► BTB(1024)/RAS ─► next PC│   │ extrai 2 instruções   │   │ ALU0  ALU1 │   │ LSU: D$ 32K │   │ commit x2  │
 │ ITLB ─► I$ 32KB 8 vias        │──►│ RVC ─► decode x2      │──►│ BRU (1/par)│──►│  8 vias WT  │──►│ CSRs/traps │
 │ bloco 16B/ciclo ─► FQ (16 hw) │   │ hazards/pareamento    │   │ AGU        │   │ store buffer│   │ interrupç. │
 └───────────────────────────────┘   │ RF (bypass do WB)     │   │ DIV  FPU   │   │ DTLB/PTW    │   │ xRET/FENCE │
                                     └───────────────────────┘   └────────────┘   │ MUL         │   └────────────┘
                                                                                  └─────────────┘
```

- **Fetch**: busca 16 bytes alinhados por ciclo do I-Cache (VIPT, 32 KB, 8 vias, linhas de 64 B).
  BTB de 1024 conjuntos indexada pelo bloco que contém o fim do desvio (desvio tomado previsto
  sem bolhas), contadores de 2 bits, RAS de 8 entradas especulativo com recuperação a partir de
  um RAS arquitetural. Fila de 16 halfwords desacopla fetch e decode (instruções de 16/32 bits
  misturadas, inclusive atravessando blocos/páginas).
- **Decode/Issue**: expande RVC, decodifica 2 instruções, detecta ilegais (inclusive FP com
  `mstatus.FS=Off`, rounding mode inválido e instruções privilegiadas), corrige predições
  simples (JAL não previsto, BTFN para desvios para trás) e emite 0/1/2 instruções.
  Regras de pareamento: no máximo 1 acesso à memória, 1 desvio e 1 MUL por par; sem RAW/WAW
  entre as duas; CSR/SYSTEM/FENCE/AMO/DIV/FPU emitem sozinhas.
- **EX**: 2 ALUs completas, resolução de desvios, AGU, divisor iterativo (2 bits/ciclo com
  salto de zeros) e FPU multiciclo. Forwarding completo de MEM e WB (latência ALU = 1).
- **MEM**: LSU com D-Cache VIPT 32 KB 8 vias **write-through**, hit em 1 ciclo; store buffer
  de 8 linhas **coalescente**; DTLB (16 entradas, superpáginas); PTW Sv39 compartilhado;
  LR/SC e AMOs; acessos MMIO não-cacheáveis. Multiplicador de 1 ciclo. Load-use = 1 bolha.
- **WB**: commit em ordem de até 2 instruções; exceções e interrupções precisas; CSRs
  executados no commit; MRET/SRET, WFI, FENCE.I (drena o store buffer e invalida o I$),
  SFENCE.VMA.
- **FPU**: IEEE 754-2008 (F e D). FMA exato com arredondamento único (add/sub/mul usam o
  mesmo caminho), div/sqrt por recorrência de dígitos, conversões com saturação RISC-V,
  NaN canônico, NaN-boxing, 5 flags com tininess *após* arredondamento.
- **MMU**: Sv39 com superpáginas 2 MB/1 GB, comportamento Svade (A/D=0 geram page fault, o
  kernel ajusta os bits), SUM/MXR/MPRV. `satp.ASID` é somente-leitura zero (Linux usa
  SFENCE.VMA global). PMP não implementado (0 regiões).

### Interface do núcleo (`nebula_core_full`)

| Grupo | Sinais |
|---|---|
| Reset | `reset_vector[63:0]` (PC inicial) |
| I-mem | `imem_req/addr/ack/data[511:0]/error` — leitura de linha de 64 B |
| D-mem | `dmem_req/we/addr/wdata[511:0]/wstrb[63:0]/uncached/ack/rdata/error` — refill de linha, escrita com byte-strobes, leituras MMIO de 8 B |
| PTW | `ptw_mem_req/addr/ack/data[63:0]/error` |
| Interrupções | `timer_irq`, `software_irq`, `external_irq` (M), `s_external_irq` (S), `mtime[63:0]` |
| Trace | `trace_*` — instruções efetivadas e traps (verificação/co-simulação) |

## Estrutura do repositório

```
rtl/
  common/   nebula_pkg.sv (tipos, uop), nebula_csr.sv (CSRs M/S/U, traps, interrupções)
  frontend/ nebula_fetch.sv (PC, BTB, RAS, FQ), nebula_decoder.sv, nebula_rvc_expander.sv
  cache/    nebula_icache.sv, l2_cache.sv (cluster legado)
  memory/   nebula_tlb.sv, nebula_ptw.sv
  backend/  nebula_lsu.sv (D$, store buffer, AMO, DTLB), nebula_div.sv
  fpu/      nebula_fpu.sv
  core/     nebula_core_full.sv (pipeline dual-issue)
  cluster/, wrapper/  integração com L2/AXI/LiteX (não verificada com o núcleo novo)
tb/
  sim/      nebula_sim.cpp (SoC simulado), cosim_spike.cpp (lockstep com Spike), Makefile
  run_isa_tests.py
benchmarks/
  embench-iot/ (submódulo), nebula/ (board support), run_embench.py
linux/      nebula_sim.dts/.dtb, run_linux.sh
```

## Como usar

Pré-requisitos: Verilator ≥ 5, `riscv64-unknown-elf-gcc`, Python 3. Para co-simulação: Spike
instalado como biblioteca (`libriscv.so` + headers). Os caminhos padrão assumem `RISCV=/opt/riscv`.

```bash
git submodule update --init --recursive
make sim            # simulador Verilator
make isa            # riscv-tests (baixa e compila na primeira vez)
make embench        # compila e roda os 19 benchmarks, imprime IPC
make linux          # boota Linux (imagens de /opt/riscv/buildroot/output/images)

make cosim          # simulador com lockstep Spike
make isa ISA_FLAGS="--sim build/sim_cosim/Vnebula_core_full --extra=--cosim"
linux/run_linux.sh --cosim
```

Interagir com o shell do Linux (roteiro expect/send pela UART):

```bash
linux/run_linux.sh --expect='# ' --send='uname -a; cat /proc/cpuinfo; poweroff\n'
```

Opções úteis do simulador: `--trace=arquivo` (log de commits), `--heartbeat=N`,
`--rd-lat/--wr-lat` (latência de memória), `--max-cycles=N`.

### SoC simulado (`tb/sim/nebula_sim.cpp`)

| Endereço | Dispositivo |
|---|---|
| `0x0000_1000` | Boot ROM (a0=hartid, a1=DTB, salta para `0x8000_0000`) |
| `0x0010_0000` | Finisher `sifive,test0` (fim da simulação / poweroff) |
| `0x0200_0000` | CLINT (msip, mtimecmp, mtime) |
| `0x0C00_0000` | PLIC (contextos M e S) |
| `0x1000_0000` | UART 16550A |
| `0x8000_0000` | RAM (256 MB) |

Memória modelada com latência de leitura de 10 ciclos e escrita de 2 ciclos (equivalente a
um L2). `mtime` avança a cada 100 ciclos (núcleo a 100 MHz, timebase de 1 MHz).

## Desempenho — Embench IoT

Compilado com GCC 16.1 `-O2 -march=rv64imafdc`, medido entre `start_trigger()` e
`stop_trigger()` (mcycle/minstret), caches aquecidas:

| benchmark | IPC | benchmark | IPC |
|---|---|---|---|
| aha-mont64 | 1.43 | nsichneu | 0.92 |
| crc32 | 1.37 | picojpeg | 1.38 |
| depthconv | 1.33 | qrduino | 1.21 |
| edn | 1.33 | sglib-combined | 1.02 |
| huffbench | 1.21 | slre | 1.22 |
| matmult-int | 1.15 | statemate | 1.05 |
| md5sum | 1.67 | tarfind | 1.22 |
| nettle-aes | 1.76 | ud | 1.20 |
| nettle-sha256 | 1.64 | wikisort | 1.34 |
| | | xgboost | 1.19 |

Média geométrica: **1.28**.

## Limitações conhecidas

- Um único hart: a D-Cache write-through não participa de coerência, portanto o cluster
  multi-core (`rtl/cluster`) não suporta SMP com o núcleo novo.
- Acessos desalinhados geram exceção (o OpenSBI os emula para o Linux).
- PMP, triggers de debug e ASIDs não implementados.
- Erros de barramento em stores são ignorados (stores são efetivados no store buffer).

## Licença

MIT — veja [LICENSE](LICENSE).
