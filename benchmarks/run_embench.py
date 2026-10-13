#!/usr/bin/env python3
"""Roda todos os benchmarks Embench IoT no simulador RTL do Nebula Core.

Mede ciclos e instruções entre start_trigger() e stop_trigger() (mcycle/minstret)
e reporta IPC por benchmark e a média geométrica.
Uso: benchmarks/run_embench.py [--sim ...] [--json out.json] [bench ...]
"""
import argparse, glob, json, math, os, re, subprocess, sys, time
from concurrent.futures import ThreadPoolExecutor

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
ap = argparse.ArgumentParser()
ap.add_argument('--sim', default=os.path.join(ROOT, 'build/sim/Vnebula_cluster'))
ap.add_argument('--dir', default=os.path.join(ROOT, 'build/embench'))
ap.add_argument('--json', default=None)
ap.add_argument('--extra', default='')
ap.add_argument('-j', type=int, default=os.cpu_count())
ap.add_argument('benches', nargs='*')
a = ap.parse_args()

elfs = sorted(glob.glob(os.path.join(a.dir, '*.elf')))
if a.benches:
    elfs = [e for e in elfs if os.path.basename(e)[:-4] in a.benches]

def run(elf):
    t0 = time.time()
    p = subprocess.run([a.sim, '--max-cycles=400000000'] + a.extra.split() + [elf],
                       capture_output=True, text=True)
    m = re.search(r'NEBULA_RESULT cycles=(\d+) instret=(\d+) correct=(\d)', p.stdout)
    name = os.path.basename(elf)[:-4]
    if not m:
        return name, None, p.stdout[-300:] + p.stderr[-300:], time.time() - t0
    cyc, ins, ok = int(m[1]), int(m[2]), m[3] == '1'
    return name, dict(cycles=cyc, instret=ins, ipc=ins / cyc, correct=ok and p.returncode == 0), '', time.time() - t0

res = {}
print(f"{'benchmark':16s} {'ciclos':>12s} {'instruções':>12s} {'IPC':>6s}  ok   tempo-sim")
with ThreadPoolExecutor(a.j) as ex:
    for name, r, err, dt in ex.map(run, elfs):
        if r is None:
            print(f"{name:16s} ERRO: {err.strip()}")
            res[name] = None
            continue
        res[name] = r
        print(f"{name:16s} {r['cycles']:12d} {r['instret']:12d} {r['ipc']:6.3f}  {'sim' if r['correct'] else 'NÃO'}  {dt:6.1f}s")
ok = [r for r in res.values() if r]
if ok:
    g = math.exp(sum(math.log(r['ipc']) for r in ok) / len(ok))
    print(f"\nIPC média geométrica: {g:.3f}   |   IPC >= 1.0 em {sum(r['ipc'] >= 1.0 for r in ok)}/{len(ok)} benchmarks"
          f"   |   corretos: {sum(r['correct'] for r in ok)}/{len(res)}")
if a.json:
    json.dump(res, open(a.json, 'w'), indent=2)
sys.exit(0 if all(r and r['correct'] for r in res.values()) else 1)
