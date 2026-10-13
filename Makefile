# =============================================================================
# Nebula Core — fluxo de verificação
#   make sim          simulador Verilator (build/sim)
#   make cosim        simulador com co-simulação lockstep Spike (build/sim_cosim)
#   make riscv-tests  baixa e compila riscv-tests (build/riscv-tests)
#   make isa          roda a suíte ISA            (ISA_FLAGS=--cosim p/ lockstep)
#   make directed     testes dirigidos (PMA, PMP, ASID, desalinhados, triggers) com cosim
#   make smp          testes de coerência/atômicos com 2 e 4 harts
#   make embench      compila e roda Embench IoT  (IPC por benchmark)
#   make linux        boota Linux (OpenSBI + kernel/initramfs) no RTL
# =============================================================================
RISCV ?= /opt/riscv
export PATH := $(RISCV)/bin:$(PATH)

sim:
	$(MAKE) -C tb/sim
cosim:
	$(MAKE) -C tb/sim COSIM=1

build/riscv-tests/isa/rv64ui-p-add:
	mkdir -p build && cd build && (test -d riscv-tests || git clone --recursive https://github.com/riscv-software-src/riscv-tests.git)
	$(MAKE) -C build/riscv-tests/isa XLEN=64 rv64ui rv64um rv64ua rv64uf rv64ud rv64uc rv64si rv64mi
riscv-tests: build/riscv-tests/isa/rv64ui-p-add

isa: sim riscv-tests
	python3 tb/run_isa_tests.py $(ISA_FLAGS)

directed: cosim
	$(MAKE) -C tb/tests
	python3 tb/run_directed.py

smp:
	$(MAKE) -C tb/tests
	$(MAKE) -C tb/sim NCORES=2
	$(MAKE) -C tb/sim NCORES=4
	build/sim_smp2/Vnebula_cluster build/tests/smp_test_2.elf
	build/sim_smp4/Vnebula_cluster build/tests/smp_test_4.elf

embench: sim
	$(MAKE) -C benchmarks/nebula
	python3 benchmarks/run_embench.py --json build/embench_results.json

linux: sim
	linux/run_linux.sh

.PHONY: sim cosim riscv-tests isa directed smp embench linux
