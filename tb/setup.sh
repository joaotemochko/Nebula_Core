#!/usr/bin/env bash
# =============================================================================
# setup.sh — Instala dependências e verifica ambiente para os testbenches
# Executar uma vez antes de rodar os testes.
# =============================================================================
set -e

echo "=== Verificando Python ==="
python3 --version

echo ""
echo "=== Instalando cocotb e dependências ==="
pip install --upgrade "cocotb>=1.8" cocotb-test pytest pytest-html

echo ""
echo "=== Verificando Verilator ==="
if command -v verilator &>/dev/null; then
    verilator --version
    VER=$(verilator --version | grep -oP '\d+\.\d+' | head -1)
    MAJOR=$(echo $VER | cut -d. -f1)
    if [ "$MAJOR" -lt 5 ]; then
        echo "AVISO: Verilator < 5.0 detectado. Recomendado >= 5.0 para --timing."
        echo "       Instale via: sudo apt install verilator  (ou compile do fonte)"
    else
        echo "Verilator OK (>= 5.0)"
    fi
else
    echo "ERRO: Verilator não encontrado."
    echo "Instale com:"
    echo "  Ubuntu/Debian: sudo apt install verilator"
    echo "  Ou compile do fonte: https://github.com/verilator/verilator"
    exit 1
fi

echo ""
echo "=== Verificando riscv-gcc (opcional, para etapas futuras) ==="
if command -v riscv64-unknown-elf-gcc &>/dev/null; then
    riscv64-unknown-elf-gcc --version | head -1
else
    echo "riscv64-unknown-elf-gcc não encontrado (necessário apenas na Etapa 3)"
    echo "Instale com: sudo apt install gcc-riscv64-unknown-elf"
fi

echo ""
echo "=== Ambiente pronto. Para rodar os testes: ==="
echo "  cd tb/"
echo "  make test_mdu        # testa MDU isolado"
echo "  make test_alu        # testa ALU isolada"
echo "  make test_branch     # testa branch predictor"
echo "  make test_all        # roda todos"
echo ""
echo "Para ver waveforms (VCD):"
echo "  make test_mdu WAVES=1"
echo "  gtkwave sim_build/*.vcd"
