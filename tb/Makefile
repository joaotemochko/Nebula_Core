# =============================================================================
# Makefile — Etapa 1: Testes de Unidades Isoladas do Nebula Core
#
# Uso:
#   make test_mdu                        — testa MDU
#   make test_alu                        — testa ALU via nebula_backend_fpu
#   make test_branch                     — testa branch predictor
#   make test_all                        — roda todos
#   make test_mdu   WAVES=1              — com geração de VCD
#   make test_mdu   RTL_DIR=/outro/caminho
#   make help                            — mostra esta ajuda
# =============================================================================

# Caminho para o RTL original do projeto
# Padrão: um nível acima do diretório tb/
RTL_DIR ?= $(abspath $(dir $(lastword $(MAKEFILE_LIST)))/../rtl)

# Caminho para os arquivos RTL corrigidos (outputs desta sessão)
# Por padrão assume que estão no mesmo nível que rtl/ como rtl_fixed/
# Ajuste se necessário.
FIXED_DIR ?= $(abspath $(dir $(lastword $(MAKEFILE_LIST)))/../rtl_fixed)

WAVES ?= 0

# Verificar se RTL_DIR existe antes de rodar qualquer teste
_check_rtl:
	@test -d "$(RTL_DIR)" || \
	  (echo "" && \
	   echo "ERRO: RTL_DIR não encontrado: $(RTL_DIR)" && \
	   echo "      Rode com: make test_mdu RTL_DIR=/caminho/para/rtl" && \
	   echo "" && exit 1)

# =============================================================================
# MDU
# Arquivos: nebula_pkg.sv + mdu_rv64.sv (versão corrigida)
# =============================================================================
test_mdu: _check_rtl
	@echo ""
	@echo "=== Testando MDU ==="
	RTL_DIR=$(RTL_DIR) WAVES=$(WAVES) \
	  pytest test_mdu.py -v --tb=short 2>&1
	@echo ""

# =============================================================================
# ALU — via nebula_backend_fpu completo
# Arquivos necessários (em ordem de compilação):
#   nebula_pkg.sv
#   csr_unit.sv
#   mdu_rv64.sv        (usado pelo backend mas mockado no teste)
#   nebula_backend_fpu.sv
# =============================================================================
test_alu: _check_rtl
	@echo ""
	@echo "=== Testando ALU (via nebula_backend_fpu) ==="
	RTL_DIR=$(RTL_DIR) WAVES=$(WAVES) \
	  pytest test_alu.py -v --tb=short 2>&1
	@echo ""

# =============================================================================
# Branch Predictor
# Arquivos: nebula_pkg.sv + branch_predictor.sv
# =============================================================================
test_branch: _check_rtl
	@echo ""
	@echo "=== Testando Branch Predictor ==="
	RTL_DIR=$(RTL_DIR) WAVES=$(WAVES) \
	  pytest test_branch_predictor.py -v --tb=short 2>&1
	@echo ""

# =============================================================================
# Todos os testes
# =============================================================================
test_all: test_mdu test_branch test_alu

# =============================================================================
# Limpar artefatos de simulação
# =============================================================================
clean:
	rm -rf sim_build __pycache__ .pytest_cache \
	       results.xml *.vcd *.fst cocotb_sim_out.log

# =============================================================================
# Ajuda
# =============================================================================
help:
	@echo ""
	@echo "Etapa 1 — Testes de Unidades Isoladas do Nebula Core"
	@echo ""
	@echo "Alvos disponíveis:"
	@echo "  make test_mdu     — MDU: MUL/MULH/MULHSU/MULHU/DIV/DIVU/REM/REMU + W"
	@echo "  make test_alu     — ALU: ADD/SUB/SLL/SRL/SRA/SLT/XOR/OR/AND/LUI/AUIPC + W"
	@echo "  make test_branch  — Branch predictor: BTB/BHT/RAS"
	@echo "  make test_all     — Todos os testes acima"
	@echo "  make clean        — Remove artefatos de simulação"
	@echo ""
	@echo "Variáveis:"
	@echo "  RTL_DIR=<path>    — Caminho para o RTL (padrão: ../rtl)"
	@echo "  WAVES=1           — Gerar VCD para GTKWave"
	@echo ""
	@echo "Exemplos:"
	@echo "  make test_mdu"
	@echo "  make test_all RTL_DIR=/home/user/Nebula_Core/rtl"
	@echo "  make test_mdu WAVES=1 && gtkwave sim_build/*.vcd"
	@echo ""
	@echo "RTL_DIR atual: $(RTL_DIR)"
	@echo ""

.PHONY: test_mdu test_alu test_branch test_all clean help _check_rtl

# =============================================================================
# riscv-tests oficiais
# =============================================================================
RISCV_TESTS ?= /opt/riscv-tests/isa

# Compilar Verilator uma vez e rodar todos os testes rv64ui + rv64um
test_riscv:
	RTL_DIR=$(RTL_DIR) RISCV_TESTS=$(RISCV_TESTS) \
	  python3 run_riscv_tests.py

# Só inteiros base (rv64ui) — mais rápido para validação inicial
test_riscv_ui:
	RTL_DIR=$(RTL_DIR) RISCV_TESTS=$(RISCV_TESTS) \
	  python3 run_riscv_tests.py --isa rv64ui

# Só extensão M (mul/div)
test_riscv_um:
	RTL_DIR=$(RTL_DIR) RISCV_TESTS=$(RISCV_TESTS) \
	  python3 run_riscv_tests.py --isa rv64um

# Teste específico — ex: make test_one TEST=add
test_one:
	RTL_DIR=$(RTL_DIR) RISCV_TESTS=$(RISCV_TESTS) \
	  python3 run_riscv_tests.py --test $(TEST)

# Só compilar (verificar erros de síntese)
compile_riscv:
	RTL_DIR=$(RTL_DIR) python3 run_riscv_tests.py --compile-only

.PHONY: test_riscv test_riscv_ui test_riscv_um test_one compile_riscv

# Teste de Dual Issue e comportamento
test_dual_issue:
	RTL_DIR=$(RTL_DIR) WAVES=1 pytest test_nebula_dual_issue.py -s