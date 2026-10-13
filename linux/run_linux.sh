#!/usr/bin/env bash
# Boota Linux (OpenSBI fw_jump + kernel com initramfs) no simulador RTL do Nebula Core.
#   linux/run_linux.sh [--cosim] [opções extras do nebula_sim]
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/.." && pwd)
IMG=${IMG:-/opt/riscv/buildroot/output/images}
NCORES=${NCORES:-1}
if [ "$NCORES" = "1" ]; then
    SIM=$ROOT/build/sim/Vnebula_cluster
    DTB=$HERE/nebula_sim.dtb
else
    SIM=$ROOT/build/sim_smp$NCORES/Vnebula_cluster
    DTB=$HERE/nebula_sim_smp$NCORES.dtb
fi
for a in "$@"; do [ "$a" = "--cosim" ] && SIM=${COSIM_SIM:-$ROOT/build/sim_cosim/Vnebula_cluster}; done
exec "$SIM" --boot-rom --ram-mb=256 \
     --load=$IMG/fw_jump.bin@0x80000000 \
     --load=${KERNEL:-$IMG/Image}@0x80200000 \
     --dtb=$DTB@0x82200000 \
     --mtime-div=100 --stall-limit=1000000000 --max-cycles=${MAX_CYCLES:-4000000000} "$@"
