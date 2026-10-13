#!/usr/bin/env python3
"""Executa a suíte riscv-tests (ISA) no simulador Verilator do Nebula Core.

Uso: tb/run_isa_tests.py [--sim build/sim/Vnebula_cluster] [--tests DIR] [filtro...]
"""
import argparse, glob, os, subprocess, sys
from concurrent.futures import ThreadPoolExecutor

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
ap = argparse.ArgumentParser()
ap.add_argument('--sim', default=os.path.join(ROOT, 'build/sim/Vnebula_cluster'))
ap.add_argument('--tests', default=os.path.join(ROOT, 'build/riscv-tests/isa'))
ap.add_argument('--extra', default='')
ap.add_argument('-j', type=int, default=os.cpu_count())
ap.add_argument('filters', nargs='*', default=['rv64ui-p', 'rv64um-p', 'rv64ua-p', 'rv64uc-p',
                                               'rv64uf-p', 'rv64ud-p', 'rv64si-p', 'rv64mi-p',
                                               'rv64ui-v', 'rv64um-v', 'rv64ua-v', 'rv64uc-v',
                                               'rv64uf-v', 'rv64ud-v'])
a = ap.parse_args()

tests = sorted(t for t in glob.glob(os.path.join(a.tests, 'rv64*'))
               if not t.endswith('.dump') and '-' in os.path.basename(t) and any(os.path.basename(t).startswith(f) for f in a.filters))

def run(t):
    cmd = [a.sim, '--max-cycles=3000000', '--quiet'] + a.extra.split() + [t]
    p = subprocess.run(cmd, capture_output=True, text=True)
    last = (p.stderr.strip().splitlines() or [''])[-1]
    return os.path.basename(t), p.returncode, last

fails = []
with ThreadPoolExecutor(a.j) as ex:
    for name, rc, last in ex.map(run, tests):
        ok = rc == 0
        if not ok: fails.append(name)
        print(f"{'PASS' if ok else 'FAIL'} {name:28s} {last}")
print(f"\n{len(tests) - len(fails)}/{len(tests)} passaram")
if fails: print("Falhas:", ' '.join(fails))
sys.exit(1 if fails else 0)
