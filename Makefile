# =============================================================================
# Nebula Core — fluxo de verificação
#   make sim          simulador Verilator (build/sim)
#   make cosim        simulador com co-simulação lockstep Spike (build/sim_cosim)
#   make riscv-tests  baixa e compila riscv-tests (build/riscv-tests)
#   make isa          roda a suíte ISA            (ISA_FLAGS=--cosim p/ lockstep)
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

embench: sim
	$(MAKE) -C benchmarks/nebula
	python3 benchmarks/run_embench.py --json build/embench_results.json

linux: sim
	linux/run_linux.sh

.PHONY: sim cosim riscv-tests isa embench linux
