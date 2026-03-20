#!/usr/bin/env python3
"""
run_riscv_tests.py — Runner para riscv-tests oficiais no Nebula Core

Uso:
    python3 run_riscv_tests.py                    # todos os testes rv64ui + rv64um
    python3 run_riscv_tests.py --isa rv64ui       # só testes de inteiros
    python3 run_riscv_tests.py --test add         # só rv64ui-p-add
    python3 run_riscv_tests.py --isa rv64um       # extensão M (mul/div)

Requisitos:
    - riscv-tests compilados (./setup_riscv_tests.sh)
    - Verilator >= 5.036
    - riscv64-unknown-elf-objcopy (para gerar .hex dos ELFs)
"""

import os
import sys
import subprocess
import argparse
import glob
import re
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor, as_completed

# =============================================================================
# Configuração
# =============================================================================
RTL_DIR       = Path(os.environ.get("RTL_DIR", "../rtl"))
TESTS_DIR     = Path(os.environ.get("RISCV_TESTS", "/opt/riscv-tests/isa"))
BUILD_DIR     = Path("sim_build_riscv")
TB_FILE       = Path(__file__).parent / "nebula_tb.sv"
TIMEOUT_CYC   = 2_000_000

# Grupos de teste suportados
TEST_GROUPS = {
    "rv64ui": "Instruções inteiras RV64I (ADD, SUB, LW, SW, BEQ, JAL...)",
    "rv64um": "Extensão M (MUL, DIV, REM...)",
    "rv64ua": "Extensão A (AMO...)",
    "rv64uf": "Extensão F (float simples)",
    "rv64ud": "Extensão D (float dupla)",
}

# Cores para output
GREEN  = "\033[92m"
RED    = "\033[91m"
YELLOW = "\033[93m"
RESET  = "\033[0m"
BOLD   = "\033[1m"

# =============================================================================
# ELF → HEX
# =============================================================================

def elf_to_hex(elf_path: Path, hex_path: Path) -> bool:
    """Converte ELF para formato hex que o $readmemh entende."""
    try:
        # Usar objcopy para extrair seção .text e .data como binário flat
        bin_path = hex_path.with_suffix(".bin")
        subprocess.run([
            "riscv64-unknown-elf-objcopy",
            "-O", "binary",
            "--only-section=.text",
            "--only-section=.data",
            "--only-section=.tohost",
            str(elf_path), str(bin_path)
        ], check=True, capture_output=True)

        # Converter binário para hex (1 palavra de 64 bits por linha)
        with open(bin_path, "rb") as f:
            data = f.read()

        # Pad para múltiplo de 8 bytes
        while len(data) % 8:
            data += b'\x00'

        with open(hex_path, "w") as f:
            for i in range(0, len(data), 8):
                word = int.from_bytes(data[i:i+8], 'little')
                f.write(f"{word:016x}\n")

        return True
    except subprocess.CalledProcessError as e:
        print(f"  ERRO objcopy: {e.stderr.decode()}")
        return False

# =============================================================================
# Compilar Verilator (uma vez)
# =============================================================================

def compile_verilator() -> bool:
    """Compila o testbench com Verilator. Roda apenas uma vez."""
    BUILD_DIR.mkdir(exist_ok=True)
    stamp = BUILD_DIR / ".compiled"
    if stamp.exists():
        return True

    print(f"{BOLD}=== Compilando Verilator ==={RESET}")

    rtl_files = [
        RTL_DIR / "common" / "nebula_pkg.sv",
        RTL_DIR / "common" / "csr_unit.sv",
        RTL_DIR / "backend"    / "mdu_rv64.sv",
        RTL_DIR / "backend"    / "nebula_backend_fpu.sv",
        RTL_DIR / "cache"      / "icache_l1.sv",
        RTL_DIR / "cache"      / "dcache_l1.sv",
        RTL_DIR / "fpu"        / "fpu_ieee754.sv",
        RTL_DIR / "predictor"  / "branch_predictor.sv",
        RTL_DIR / "frontend"   / "compressed_decoder_rv64.sv",
        RTL_DIR / "frontend"   / "nebula_frontend_rvc.sv",
        RTL_DIR / "memory"     / "tlb_sv39.sv",
        RTL_DIR / "memory"     / "ptw_sv39.sv",
        RTL_DIR / "core"       / "nebula_core_full.sv",
        TB_FILE,
    ]

    # Verificar que todos os arquivos existem
    missing = [f for f in rtl_files if not f.exists()]
    if missing:
        print(f"{RED}ERRO: arquivos RTL não encontrados:{RESET}")
        for f in missing:
            print(f"  {f}")
        return False

    cmd = [
        "verilator", "--binary", "--timing",
        "-Mdir", str(BUILD_DIR),
        "--top-module", "nebula_tb",
        "-o", "nebula_tb_sim",
        "-Wall",
        "-Wno-IMPORTSTAR", "-Wno-VARHIDDEN", "-Wno-UNOPTFLAT",
        "-Wno-WIDTHTRUNC", "-Wno-WIDTHEXPAND",
        "-Wno-ENUMVALUE",  "-Wno-UNUSED", "-Wno-UNDRIVEN",
        "-Wno-PINCONNECTEMPTY", "-Wno-DECLFILENAME",
        "-Wno-LATCH", "-Wno-BLKSEQ",
        "--assert",
    ] + [str(f) for f in rtl_files]

    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print(f"{RED}ERRO na compilação Verilator:{RESET}")
        print(result.stdout[-3000:])
        print(result.stderr[-3000:])
        return False

    # Com --binary o Verilator compila e linka em um só passo.
    # O binário fica em BUILD_DIR/nebula_tb_sim
    sim_bin = BUILD_DIR / "nebula_tb_sim"
    if not sim_bin.exists():
        print(f"{RED}ERRO: binário não gerado em {sim_bin}{RESET}")
        print("Conteúdo do BUILD_DIR:")
        for f in BUILD_DIR.iterdir():
            print(f"  {f.name}")
        return False

    stamp.touch()
    print(f"{GREEN}Compilação OK{RESET}")
    return True

# =============================================================================
# Rodar um teste
# =============================================================================

def run_test(elf_path: Path) -> dict:
    """Roda um único teste riscv-test. Retorna dict com resultado."""
    name    = elf_path.stem
    hex_dir = BUILD_DIR / "hex"
    hex_dir.mkdir(exist_ok=True)
    hex_path = hex_dir / f"{name}.hex"

    # Converter ELF → HEX
    if not elf_to_hex(elf_path, hex_path):
        return {"name": name, "status": "ERROR", "msg": "Falha no objcopy"}

    # Executar simulação
    sim_bin = BUILD_DIR / "nebula_tb_sim"
    cmd = [str(sim_bin), f"+HEXFILE={hex_path}"]

    try:
        result = subprocess.run(
            cmd, capture_output=True, text=True,
            timeout=60   # 60s máximo por teste
        )
        output = result.stdout + result.stderr

        if "[PASS]" in output:
            # Extrair ciclos
            m = re.search(r"ciclos=(\d+)", output)
            cycles = int(m.group(1)) if m else -1
            return {"name": name, "status": "PASS", "cycles": cycles}
        elif "[FAIL]" in output:
            m = re.search(r"tohost=(\S+)", output)
            tohost = m.group(1) if m else "?"
            return {"name": name, "status": "FAIL", "msg": f"tohost={tohost}"}
        elif "[TIMEOUT]" in output:
            return {"name": name, "status": "TIMEOUT", "msg": f"{TIMEOUT_CYC} ciclos"}
        else:
            return {"name": name, "status": "ERROR", "msg": output[-200:]}

    except subprocess.TimeoutExpired:
        return {"name": name, "status": "TIMEOUT", "msg": "60s wall clock"}

# =============================================================================
# Main
# =============================================================================

def main():
    parser = argparse.ArgumentParser(description="Runner riscv-tests para Nebula Core")
    parser.add_argument("--isa",    default=None, help="Grupo ISA (ex: rv64ui, rv64um)")
    parser.add_argument("--test",   default=None, help="Teste específico (ex: add)")
    parser.add_argument("--jobs",   type=int, default=4, help="Testes em paralelo")
    parser.add_argument("--compile-only", action="store_true")
    args = parser.parse_args()

    # Compilar Verilator
    if not compile_verilator():
        sys.exit(1)
    if args.compile_only:
        sys.exit(0)

    # Encontrar testes
    if args.test:
        # Teste específico
        pattern = str(TESTS_DIR / f"rv64*-p-{args.test}")
        elfs = [Path(p) for p in glob.glob(pattern)
                if not p.endswith(".dump")]
    elif args.isa:
        pattern = str(TESTS_DIR / f"{args.isa}-p-*")
        elfs = [Path(p) for p in glob.glob(pattern)
                if not p.endswith(".dump")]
    else:
        # Todos: rv64ui + rv64um
        elfs = []
        for grp in ["rv64ui", "rv64um"]:
            pattern = str(TESTS_DIR / f"{grp}-p-*")
            elfs += [Path(p) for p in glob.glob(pattern)
                     if not p.endswith(".dump")]

    if not elfs:
        print(f"{RED}ERRO: nenhum teste encontrado em {TESTS_DIR}{RESET}")
        print("Rode: ./setup_riscv_tests.sh")
        sys.exit(1)

    elfs.sort()
    print(f"\n{BOLD}=== Rodando {len(elfs)} testes ==={RESET}\n")

    # Executar em paralelo
    results = []
    with ThreadPoolExecutor(max_workers=args.jobs) as ex:
        futures = {ex.submit(run_test, e): e for e in elfs}
        for fut in as_completed(futures):
            r = fut.result()
            results.append(r)
            status = r["status"]
            name   = r["name"]
            if status == "PASS":
                cyc = r.get("cycles", "?")
                print(f"  {GREEN}PASS{RESET}  {name:<40} ({cyc} ciclos)")
            elif status == "FAIL":
                print(f"  {RED}FAIL{RESET}  {name:<40} {r.get('msg','')}")
            elif status == "TIMEOUT":
                print(f"  {YELLOW}TIME{RESET}  {name:<40} {r.get('msg','')}")
            else:
                print(f"  {RED}ERR {RESET}  {name:<40} {r.get('msg','')[:60]}")

    # Sumário
    passed  = sum(1 for r in results if r["status"] == "PASS")
    failed  = sum(1 for r in results if r["status"] == "FAIL")
    timeout = sum(1 for r in results if r["status"] == "TIMEOUT")
    errors  = sum(1 for r in results if r["status"] == "ERROR")
    total   = len(results)

    print(f"\n{'='*60}")
    print(f"{BOLD}RESULTADO: {passed}/{total} testes passaram{RESET}")
    if failed:  print(f"  {RED}Falhas:   {failed}{RESET}")
    if timeout: print(f"  {YELLOW}Timeouts: {timeout}{RESET}")
    if errors:  print(f"  {RED}Erros:    {errors}{RESET}")
    print(f"{'='*60}\n")

    sys.exit(0 if failed == 0 and errors == 0 else 1)

if __name__ == "__main__":
    main()
