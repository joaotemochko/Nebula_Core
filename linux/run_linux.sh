#!/usr/bin/env bash
# Boota Linux (OpenSBI fw_jump + kernel com initramfs) no simulador RTL do Nebula Core.
#   linux/run_linux.sh [--cosim] [opções extras do nebula_sim]
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/.." && pwd)
IMG=${IMG:-/opt/riscv/buildroot/output/images}
SIM=$ROOT/build/sim/Vnebula_core_full
for a in "$@"; do [ "$a" = "--cosim" ] && SIM=$ROOT/build/sim_cosim/Vnebula_core_full; done
exec "$SIM" --boot-rom --ram-mb=256 \
     --load=$IMG/fw_jump.bin@0x80000000 \
     --load=$IMG/Image@0x80200000 \
     --dtb=$HERE/nebula_sim.dtb@0x82200000 \
     --mtime-div=100 --stall-limit=1000000000 --max-cycles=${MAX_CYCLES:-4000000000} "$@"
