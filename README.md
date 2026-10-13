# Nebula Core

<p align="center">
  <strong>Núcleo RISC-V RV64GC dual-issue in-order, SMP, capaz de rodar Linux</strong>
</p>

---

## Visão geral

**Nebula Core** é um processador RISC-V **RV64IMAFDC + Zicsr + Zifencei + Zicclsm + Sdtrig**,
com modos M/S/U, memória virtual **Sv39** (com ASIDs) e **PMP**, escrito em SystemVerilog.
O pipeline é **dual-issue em ordem (in-order)**: emite, executa e efetiva até 2 instruções
por ciclo, sempre na ordem do programa. O **cluster** agrupa N núcleos com memória coerente
e uma porta de memória (nativa ou AXI4).

Estado verificado (simulação Verilator):

| Verificação | Resultado |
|---|---|
| riscv-tests (rv64ui/um/ua/uc/uf/ud/si/mi, ambientes `-p` e `-v`) | **244/244** |
| Testes dirigidos (PMA, PMP aleatório, ASID/SFENCE, desalinhados, triggers, SMP) | **6/6** |
| Co-simulação lockstep com Spike (toda instrução comparada) | sem divergências em ISA, testes dirigidos, Embench e boot do Linux |
| Embench IoT (19 benchmarks) | **19/19 corretos**, IPC média geométrica **1.28**, IPC ≥ 1.0 em **18/19** |
| SMP bare-metal (AMO, LR/SC, spinlock, compartilhamento falso, passagem de mensagem) | 1, 2 e 4 harts |
| Ponte AXI4 (escravo com atrasos aleatórios) | ISA 244/244, dirigidos 5/5, SMP 4 harts |
| Linux 6.19 (OpenSBI 1.7 + kernel + initramfs BusyBox/glibc), 1 hart | boot até o shell, comandos, `poweroff` (também sob cosim) |
| Linux 6.19 SMP (kernel `CONFIG_SMP=y`), 2 e 4 harts | "Brought up 1 node, N CPUs", 4× `md5sum` paralelos corretos, `poweroff` |

## Microarquitetura do núcleo

```
 ┌──────────── FETCH ────────────┐   ┌──── DECODE/ISSUE ────┐   ┌──── EX ────┐   ┌──── MEM ────┐   ┌──── WB ────┐
 │ PC ─► BTB(1024)/RAS ─► next PC│   │ extrai 2 instruções   │   │ ALU0  ALU1 │   │ LSU: D$ 32K │   │ commit x2  │
 │ ITLB ─► I$ 32KB 8 vias        │──►│ RVC ─► decode x2      │──►│ BRU (1/par)│──►│  8 vias WT  │──►│ CSRs/traps │
 │ bloco 16B/ciclo ─► FQ (16 hw) │   │ hazards/pareamento    │   │ AGU        │   │ store buffer│   │ interrupç. │
 │ PMA/PMP/triggers de execução  │   │ RF (bypass do WB)     │   │ DIV  FPU   │   │ DTLB/PTW    │   │ xRET/FENCE │
 └───────────────────────────────┘   └───────────────────────┘   └────────────┘   │ MUL         │   └────────────┘
                                                                                  └─────────────┘
```

- **Fetch**: busca 16 bytes alinhados por ciclo do I-Cache (VIPT, 32 KB, 8 vias, linhas de 64 B).
  BTB de 1024 conjuntos indexada pelo bloco que contém o fim do desvio (desvio tomado previsto
  sem bolhas), contadores de 2 bits, RAS de 8 entradas especulativo com recuperação a partir de
  um RAS arquitetural. Fila de 16 halfwords desacopla fetch e decode. PMA e PMP são checados
  por setor de 4 bytes do bloco.
- **Decode/Issue**: expande RVC, decodifica 2 instruções, detecta ilegais, aplica triggers de
  execução (por parcela de 16 bits), corrige predições simples (JAL não previsto, BTFN) e emite
  0/1/2 instruções. Pareamento: no máximo 1 acesso à memória, 1 desvio e 1 MUL por par; sem
  RAW/WAW entre as duas; CSR/SYSTEM/FENCE/AMO/DIV/FPU emitem sozinhas.
- **EX**: 2 ALUs completas, resolução de desvios, AGU, divisor iterativo (2 bits/ciclo com salto
  de zeros) e FPU multiciclo. Forwarding completo de MEM e WB (latência ALU = 1).
- **MEM**: LSU com D-Cache VIPT 32 KB 8 vias **write-through**, hit em 1 ciclo; store buffer de 8
  linhas **coalescente**; DTLB (16 entradas, superpáginas, ASID); acessos **desalinhados em
  hardware** (até 2 fases, inclusive cruzando linha/página); triggers de load/store; MMIO;
  LR/SC/AMO enviados à interconexão. Multiplicador de 1 ciclo. Load-use = 1 bolha.
- **WB**: commit em ordem de até 2 instruções; exceções e interrupções precisas; CSRs executados
  no commit; MRET/SRET, WFI, FENCE (drena o store buffer), FENCE.I (drena e invalida o I$),
  SFENCE.VMA seletivo (VA/ASID).
- **FPU**: IEEE 754-2008 (F e D). FMA exato com arredondamento único, div/sqrt por recorrência de
  dígitos, conversões com saturação RISC-V, NaN canônico, NaN-boxing, 5 flags (tininess após
  arredondamento).
- **MMU**: Sv39 com superpáginas 2 MB/1 GB e **ASIDs de 16 bits** (TLBs etiquetadas, bit G,
  SFENCE.VMA nas 4 formas); comportamento Svade (A/D=0 geram page fault); SUM/MXR/MPRV.
- **Proteção física**: **PMA** parametrizável (endereço inexistente → access fault precisa, sem
  transação no barramento) e **PMP** com 16 regiões (OFF/TOR/NA4/NAPOT, bit L, granularidade
  4 B), checado no fetch, no LSU e no PTW.
- **Triggers de debug (Sdtrig)**: 4 triggers `mcontrol`/`mcontrol6` com ação *breakpoint*:
  execução, load e store por endereço (EQUAL/NAPOT/GE/LT/máscaras) ou por valor, encadeamento,
  bits `hit`, temporização antes/após, regras de habilitação por MIE/SIE.

## Cluster SMP e coerência

```
  núcleo 0 ─┐  I, D, PTW
  núcleo 1 ─┼──────────►  nebula_interconnect  ──►  porta de memória (linha 64 B, strobes)
     ...    │             • árbitro round-robin             │
  núcleo N ─┘             • invalidação das L1D             └─► nebula_axi_bridge ─► AXI4 64 bits
                          • AMO / LR / SC atômicos
                          • reservas LR por hart
```

- **Coerência**: as L1D são write-through; a interconexão é o ponto único de serialização e,
  no mesmo ciclo em que efetiva uma escrita, invalida a linha nas L1D dos outros núcleos.
  Isso garante atomicidade multi-cópia (RVWMO).
- **Atômicos**: AMO, LR e SC são executados na interconexão, como leitura + escrita sem
  intercalação; uma escrita de outro hart na linha reservada cancela a reserva.
- **Ordenação**: FENCE drena o store buffer; o I$ não é coerente por hardware (FENCE.I e IPI,
  como define a ISA).

## Estrutura do repositório

```
rtl/
  common/   nebula_pkg.sv (tipos, uop, triggers, AMO), nebula_csr.sv (CSRs, traps, PMP, triggers)
  frontend/ nebula_fetch.sv (PC, BTB, RAS, FQ), nebula_decoder.sv, nebula_rvc_expander.sv
  cache/    nebula_icache.sv
  memory/   nebula_tlb.sv, nebula_ptw.sv, nebula_pma.sv, nebula_pmp.sv
  backend/  nebula_lsu.sv (D$, store buffer, desalinhados, triggers), nebula_div.sv
  fpu/      nebula_fpu.sv
  core/     nebula_core_full.sv (pipeline dual-issue)
  cluster/  nebula_cluster.sv (N núcleos), nebula_interconnect.sv (coerência, atômicos)
  wrapper/  nebula_axi_bridge.sv, nebula_axi_top.sv (cluster + AXI4 para SoC/LiteX)
tb/
  sim/      nebula_sim.cpp (SoC simulado), soc_model.h, cosim_spike.cpp (lockstep com Spike),
            nebula_axi_sim.cpp (escravo AXI4 com atrasos aleatórios), Makefile
  tests/    testes dirigidos (PMA, PMP, ASID, desalinhados, triggers, SMP)
  run_isa_tests.py, run_directed.py
benchmarks/  embench-iot/ (submódulo), nebula/ (board support), run_embench.py
linux/       DTS/DTB (1, 2 e 4 harts), run_linux.sh
SoC_Simulation/  integração LiteX (nebula_axi_top)
```

## Como usar

Pré-requisitos: Verilator ≥ 5, `riscv64-unknown-elf-gcc`, Python 3. Para co-simulação: Spike
instalado como biblioteca (`libriscv.so` + headers). Os caminhos padrão assumem `RISCV=/opt/riscv`.

```bash
git submodule update --init --recursive
make sim            # simulador Verilator (1 hart)
make isa            # riscv-tests (baixa e compila na primeira vez)
make directed       # testes dirigidos com co-simulação
make embench        # compila e roda os 19 benchmarks, imprime IPC
make linux          # boota Linux (imagens de /opt/riscv/buildroot/output/images)

make cosim          # simulador com lockstep Spike
python3 tb/run_isa_tests.py --sim build/sim_cosim/Vnebula_cluster --extra=--cosim
linux/run_linux.sh --cosim

make -C tb/sim NCORES=4          # cluster com 4 harts
make -C tb/sim axi NCORES=4      # cluster + ponte AXI4 com escravo AXI aleatório
```

Interagir com o shell do Linux (roteiro expect/send pela UART):

```bash
linux/run_linux.sh --expect='login: ' --send='root\n' --expect='# ' --send='uname -a; poweroff\n'
```

Opções úteis do simulador: `--trace=arquivo` (log de commits), `--trace-from=N`,
`--trace-mmio-only`, `--heartbeat=N` (PC de cada hart), `--dump-ram=arquivo`,
`--rd-lat/--wr-lat` (latência de memória), `--max-cycles=N`, `--stall-limit=N`.

### SoC simulado (`tb/sim/soc_model.h`)

| Endereço | Dispositivo |
|---|---|
| `0x0000_1000` | Boot ROM (a0=hartid, a1=DTB, salta para `0x8000_0000`) |
| `0x0010_0000` | Finisher `sifive,test0` (poweroff/reboot, fim da simulação) |
| `0x0200_0000` | CLINT (msip e mtimecmp por hart, mtime) |
| `0x0C00_0000` | PLIC (contextos M e S por hart) |
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
| | | xgboost | 1.18 |

Média geométrica: **1.28**.

## Linux

O kernel do buildroot (6.19, uniprocessador) boota em 1 hart. Para SMP é preciso um kernel com
`CONFIG_SMP=y`; ele pode ser compilado a partir da mesma árvore do buildroot:

```bash
cp -a /opt/riscv/buildroot/output/build/linux-6.19.14 build/linux-smp/src
cd build/linux-smp/src
./scripts/config --enable SMP --set-val NR_CPUS 8 --disable FORCE_NR_CPUS
sed -i 's#^CONFIG_INITRAMFS_SOURCE=.*#CONFIG_INITRAMFS_SOURCE="/opt/riscv/buildroot/output/images/rootfs.cpio"#' .config
make ARCH=riscv CROSS_COMPILE=riscv64-buildroot-linux-gnu- olddefconfig Image -j16
cd ../../.. && make -C tb/sim NCORES=4
NCORES=4 KERNEL=build/linux-smp/src/arch/riscv/boot/Image linux/run_linux.sh
```

Boot completo + 4 `md5sum` em paralelo + `poweroff`: ~528M ciclos com 2 harts, ~544M com 4 harts.
A saída do console só começa após ~60–100M ciclos (OpenSBI + descompressão/early boot).

## Limitações conhecidas

- Sem cache L2 no cluster (a porta de memória é a fronteira do cluster).
- Stores são efetivados no store buffer: endereços inexistentes geram access fault precisa
  (via PMA), mas um erro devolvido pelo barramento numa escrita a uma região válida é descartado.
- Triggers de dado (`select=1`) em AMO/SC são avaliados após a operação (o acesso já ocorreu);
  todos os demais casos seguem exatamente a semântica do Spike.
- A integração LiteX (`SoC_Simulation/`) foi atualizada para `nebula_axi_top`, mas não foi
  executada (LiteX não está instalado no ambiente de verificação); a ponte AXI em si é
  verificada pelo testbench AXI.
- Simulação do Linux é lenta (~0,3–0,5 MHz por hart no Verilator).

## Licença

MIT — veja [LICENSE](LICENSE).
