#!/usr/bin/env bash
# =============================================================================
# setup_riscv_tests.sh
# Compila riscv-tests oficiais — apenas variante -p- (physical, sem VM)
# =============================================================================
set -e

TESTS_DIR="${1:-/opt/riscv-tests}"

echo "=== Verificando riscv64-unknown-elf-gcc ==="
if ! command -v riscv64-unknown-elf-gcc &>/dev/null; then
    echo "Instalando toolchain RISC-V..."
    sudo apt-get install -y gcc-riscv64-unknown-elf binutils-riscv64-unknown-elf
fi
riscv64-unknown-elf-gcc --version | head -1

echo ""
echo "=== Clonando riscv-tests ==="
if [ ! -d "$TESTS_DIR" ]; then
    sudo git clone --depth 1 \
        https://github.com/riscv/riscv-tests.git \
        "$TESTS_DIR"
    sudo chown -R "$USER" "$TESTS_DIR"
fi

echo ""
echo "=== Configurando ==="
cd "$TESTS_DIR"
git submodule update --init --recursive
autoconf
./configure --prefix="$TESTS_DIR/target"

echo ""
echo "=== Estrutura do repositório ==="
echo "Arquivos .S encontrados:"
find "$TESTS_DIR" -name "*.S" | grep "rv64" | head -10
echo ""
echo "Conteúdo de isa/:"
ls "$TESTS_DIR/isa/" | head -20

echo ""
echo "=== Compilando apenas variante -p- via make ==="
cd "$TESTS_DIR"

# O Makefile do riscv-tests aceita alvos individuais por nome
# Vamos pegar a lista de alvos -p- do próprio Makefile
TARGETS=$(make -C isa -n 2>/dev/null \
    | grep -oE 'rv64u[imafd]-p-[a-z0-9_]+' \
    | sort -u \
    | tr '\n' ' ')

if [ -z "$TARGETS" ]; then
    echo "Tentando compilação direta sem filtro de alvos..."
    # Alternativa: compilar tudo e ignorar erros dos -v-
    make -C isa -j"$(nproc)" -k 2>&1 | grep -v "vm.c" | tail -20 || true
else
    echo "Alvos encontrados: $(echo $TARGETS | wc -w)"
    make -C isa -j"$(nproc)" $TARGETS
fi

echo ""
echo "=== Verificando binários gerados ==="
UI=$(find "$TESTS_DIR/isa" -maxdepth 1 -name "rv64ui-p-*" \
     ! -name "*.dump" ! -name "*.S" ! -name "*.o" | wc -l)
UM=$(find "$TESTS_DIR/isa" -maxdepth 1 -name "rv64um-p-*" \
     ! -name "*.dump" ! -name "*.S" ! -name "*.o" | wc -l)
echo "rv64ui-p: $UI binários"
echo "rv64um-p: $UM binários"

if [ "$UI" -gt 0 ]; then
    echo ""
    echo "=== Pronto. Próximos passos: ==="
    echo "  cd tb/"
    echo "  make compile_riscv RTL_DIR=../rtl"
    echo "  make test_riscv_ui"
else
    echo ""
    echo "Nenhum binário -p- gerado. Estrutura atual de isa/:"
    ls "$TESTS_DIR/isa/" | head -30
    echo ""
    echo "DICA: verifique se o repositório tem uma estrutura diferente com:"
    echo "  find /opt/riscv-tests -name 'rv64ui-p-add*'"
fi
