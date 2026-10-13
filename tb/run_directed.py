#!/usr/bin/env python3
"""Roda os testes dirigidos (build/tests/*.elf) no simulador, opcionalmente com cosim."""
import argparse, glob, os, subprocess, sys
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
ap = argparse.ArgumentParser()
ap.add_argument('--sim', default=os.path.join(ROOT, 'build/sim_cosim/Vnebula_cluster'))
ap.add_argument('--no-cosim', action='store_true')
a = ap.parse_args()
fails = 0
for t in sorted(glob.glob(os.path.join(ROOT, 'build/tests/*.elf'))):
    b = os.path.basename(t)
    if b.startswith('smp_test_') and b != 'smp_test_1.elf':
        continue   # testes multi-hart: 'make smp'
    cmd = [a.sim, '--quiet', '--max-cycles=20000000'] + ([] if a.no_cosim else ['--cosim']) + [t]
    p = subprocess.run(cmd, capture_output=True, text=True)
    ok = p.returncode == 0
    fails += not ok
    print(f"{'PASS' if ok else 'FAIL'} {os.path.basename(t)[:-4]:20s} {(p.stderr.strip().splitlines() or [''])[-1]}")
sys.exit(1 if fails else 0)
