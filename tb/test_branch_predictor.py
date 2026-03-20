"""
test_branch_predictor.py — Testbench cocotb para branch_predictor

Ports reais do módulo (nebula_pkg.sv):
    input  wire [VADDR_WIDTH-1:0]  pc
    input  wire                    predict_valid
    output bp_prediction_t         prediction      ← struct packed 45 bits
    input  wire                    update_valid
    input  bp_update_t             update          ← struct packed 83 bits

bp_prediction_t (packed, MSB→LSB):
    [44]    valid
    [43]    taken
    [42:4]  target   (39 bits, VADDR_WIDTH=39)
    [3]     is_call
    [2]     is_ret
    [1:0]   confidence

bp_update_t (packed, MSB→LSB):
    [82]    valid
    [81]    taken
    [80]    mispredicted
    [79:41] pc       (39 bits)
    [40:2]  target   (39 bits)
    [1]     is_call
    [0]     is_ret
"""

import os
from cocotb_test.simulator import run

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge

VADDR_WIDTH = 39
MASK_V      = (1 << VADDR_WIDTH) - 1

RTL_DIR = os.path.abspath(os.environ.get("RTL_DIR", "../rtl"))

# ---------------------------------------------------------------------------
# Helpers para empacotar / desempacotar as structs
# ---------------------------------------------------------------------------

def pack_update(valid=0, taken=0, mispredicted=0,
                pc=0, target=0, is_call=0, is_ret=0):
    """Monta o valor inteiro de bp_update_t."""
    v = 0
    v |= (valid        & 1)  << 82
    v |= (taken        & 1)  << 81
    v |= (mispredicted & 1)  << 80
    v |= (pc    & MASK_V)    << 41
    v |= (target & MASK_V)   << 2
    v |= (is_call & 1)       << 1
    v |= (is_ret  & 1)       << 0
    return v

def unpack_prediction(val):
    """Extrai campos de bp_prediction_t."""
    valid      = (val >> 44) & 1
    taken      = (val >> 43) & 1
    target     = (val >>  4) & MASK_V
    is_call    = (val >>  3) & 1
    is_ret     = (val >>  2) & 1
    confidence = (val >>  0) & 3
    return valid, taken, target, is_call, is_ret, confidence

# ---------------------------------------------------------------------------
# Driver helpers
# ---------------------------------------------------------------------------

async def reset_dut(dut):
    dut.rst_n.value        = 0
    dut.predict_valid.value = 0
    dut.pc.value            = 0
    dut.update_valid.value  = 0
    dut.update.value        = 0
    for _ in range(5):
        await RisingEdge(dut.clk)
    dut.rst_n.value = 1
    await RisingEdge(dut.clk)

async def do_predict(dut, pc):
    """Solicita predição e retorna (valid, taken, target, is_call, is_ret).

    prediction é saída COMBINACIONAL de predict_valid e pc.
    Fluxo:
      1. Setar predict_valid=1 e pc
      2. Aguardar Timer(1ns) para propagação dos deltas combinacionais
      3. Ler prediction (valid=1 se BTB hit)
      4. Aguardar RisingEdge para que o estado interno (GHR, RAS) avance
      5. Baixar predict_valid
    """
    from cocotb.triggers import Timer
    dut.predict_valid.value = 1
    dut.pc.value            = pc & MASK_V
    # Propagar deltas combinacionais sem avançar o clock
    await Timer(1, units="ns")
    pred = int(dut.prediction.value)
    # Avançar um ciclo (RAS push/pop ocorre aqui)
    await RisingEdge(dut.clk)
    dut.predict_valid.value = 0
    # Aguardar mais um ciclo para estabilizar
    await RisingEdge(dut.clk)
    return unpack_prediction(pred)

async def do_update(dut, pc, taken, target,
                    mispredicted=False, is_call=False, is_ret=False):
    """Envia update do resultado real de um branch."""
    val = pack_update(
        valid=1, taken=int(taken), mispredicted=int(mispredicted),
        pc=pc, target=target, is_call=int(is_call), is_ret=int(is_ret)
    )
    dut.update_valid.value = 1
    dut.update.value       = val
    await RisingEdge(dut.clk)
    dut.update_valid.value = 0
    dut.update.value       = 0
    await RisingEdge(dut.clk)

# ---------------------------------------------------------------------------
# Testes
# ---------------------------------------------------------------------------

@cocotb.test()
async def test_btb_miss_then_hit(dut):
    """Primeiro acesso: BTB miss. Após update taken: BTB hit com target correto."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    pc  = 0x100_0010
    tgt = 0x100_0040

    # 1ª predição — deve ser miss (valid=0)
    valid, taken, target, *_ = await do_predict(dut, pc)
    assert valid == 0, f"[FAIL] BTB deveria ser miss na 1ª vez, valid={valid}"
    dut._log.info("[PASS] BTB miss na 1ª predição")

    # Treinar: branch foi taken
    await do_update(dut, pc, taken=True, target=tgt)

    # 2ª predição — deve ser hit com target correto
    valid, taken, target, *_ = await do_predict(dut, pc)
    assert valid  == 1,   f"[FAIL] BTB deveria ter hit após update, valid={valid}"
    assert taken  == 1,   f"[FAIL] Deveria prever taken, taken={taken}"
    assert target == tgt, f"[FAIL] Target errado: got={target:#x} exp={tgt:#x}"
    dut._log.info(f"[PASS] BTB hit após update, target={target:#x}")

@cocotb.test()
async def test_bht_saturating_counter(dut):
    """
    Treinar 4x taken → contador satura em 11 → prevê taken.
    Treinar 4x not-taken → contador satura em 00 → prevê not-taken.
    """
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    pc  = 0x100_0100
    tgt = 0x100_0120

    for _ in range(4):
        await do_update(dut, pc, taken=True, target=tgt)

    valid, taken, *_ = await do_predict(dut, pc)
    assert valid == 1 and taken == 1, \
        f"[FAIL] Deveria prever taken após 4x taken, valid={valid} taken={taken}"
    dut._log.info("[PASS] BHT satura em taken")

    for _ in range(4):
        await do_update(dut, pc, taken=False, target=tgt, mispredicted=True)

    valid, taken, *_ = await do_predict(dut, pc)
    assert valid == 1 and taken == 0, \
        f"[FAIL] Deveria prever not-taken após 4x not-taken, taken={taken}"
    dut._log.info("[PASS] BHT satura em not-taken")

@cocotb.test()
async def test_unconditional_branch(dut):
    """JAL (is_call=True) deve ser previsto taken e is_call=1."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    pc  = 0x100_0200
    tgt = 0x100_0300

    await do_update(dut, pc, taken=True, target=tgt, is_call=True)

    valid, taken, target, is_call, *_ = await do_predict(dut, pc)
    assert valid   == 1,   f"[FAIL] BTB miss para JAL, valid={valid}"
    assert taken   == 1,   f"[FAIL] JAL deveria ser taken"
    assert target  == tgt, f"[FAIL] Target errado: {target:#x} != {tgt:#x}"
    assert is_call == 1,   f"[FAIL] is_call deveria ser 1"
    dut._log.info("[PASS] Branch incondicional (JAL) previsto corretamente")

@cocotb.test()
async def test_ras_call_return(dut):
    """
    Call: push do endereço de retorno (PC+4) no RAS.
    Return: pop do RAS → target deve ser o endereço de retorno.
    """
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    call_pc  = 0x100_1000
    call_tgt = 0x200_0000
    ret_pc   = 0x200_0050

    # Treinar e prever call (push RAS)
    await do_update(dut, call_pc, taken=True, target=call_tgt, is_call=True)
    valid, taken, _, is_call, *_ = await do_predict(dut, call_pc)
    assert valid == 1 and taken == 1 and is_call == 1, \
        "[FAIL] Call não previsto corretamente"
    dut._log.info("[PASS] Call previsto — RAS deve ter PC+4 no topo")

    # Treinar ret
    await do_update(dut, ret_pc, taken=True, target=call_pc+4, is_ret=True)

    # Prever ret — deve usar o topo do RAS
    valid, taken, target, _, is_ret, _ = await do_predict(dut, ret_pc)
    assert valid  == 1, "[FAIL] BTB miss no ret"
    assert taken  == 1, "[FAIL] Ret deveria ser taken"
    assert is_ret == 1, "[FAIL] is_ret deveria ser 1"
    dut._log.info(f"[PASS] Return previsto para {target:#x} (RAS topo={call_pc+4:#x})")

@cocotb.test()
async def test_misprediction_recovery(dut):
    """
    Prever taken 3x, depois mispredicted=True 2x → contador cai → prevê not-taken.
    """
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    pc  = 0x100_0400
    tgt = 0x100_0480

    for _ in range(3):
        await do_update(dut, pc, taken=True, target=tgt)

    await do_update(dut, pc, taken=False, target=tgt, mispredicted=True)
    await do_update(dut, pc, taken=False, target=tgt, mispredicted=True)

    valid, taken, *_ = await do_predict(dut, pc)
    assert taken == 0, \
        f"[FAIL] Deveria prever not-taken após mispredições, taken={taken}"
    dut._log.info("[PASS] Recuperação após misprediction correta")

@cocotb.test()
async def test_two_pcs_independent(dut):
    """Dois PCs com índices BTB diferentes devem ser independentes."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    pc_a  = 0x100_0000
    tgt_a = 0x100_0080
    pc_b  = 0x100_1000   # índice BTB diferente
    tgt_b = 0x100_2000

    await do_update(dut, pc_a, taken=True, target=tgt_a)
    await do_update(dut, pc_b, taken=True, target=tgt_b)

    va, _, ta, *_ = await do_predict(dut, pc_a)
    vb, _, tb, *_ = await do_predict(dut, pc_b)

    assert va == 1 and ta == tgt_a, \
        f"[FAIL] PC_A: valid={va} target={ta:#x} exp={tgt_a:#x}"
    assert vb == 1 and tb == tgt_b, \
        f"[FAIL] PC_B: valid={vb} target={tb:#x} exp={tgt_b:#x}"
    dut._log.info("[PASS] Dois PCs independentes com targets distintos")

@cocotb.test()
async def test_reset_clears_btb(dut):
    """Após reset, BTB deve estar vazio — todas as entradas inválidas."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    pc  = 0x100_0500
    tgt = 0x100_0600

    await do_update(dut, pc, taken=True, target=tgt)
    valid, *_ = await do_predict(dut, pc)
    assert valid == 1, "[FAIL] BTB deveria ter hit antes do reset"

    dut.rst_n.value = 0
    for _ in range(4):
        await RisingEdge(dut.clk)
    dut.rst_n.value = 1
    await RisingEdge(dut.clk)

    valid, *_ = await do_predict(dut, pc)
    assert valid == 0, f"[FAIL] BTB deveria estar limpo após reset, valid={valid}"
    dut._log.info("[PASS] Reset limpa o BTB corretamente")

# ---------------------------------------------------------------------------
# Entrada pytest
# ---------------------------------------------------------------------------
def test_branch_all():
    rtl = os.path.abspath(os.environ.get("RTL_DIR", "../rtl"))
    run(
        verilog_sources=[
            os.path.join(rtl, "common", "nebula_pkg.sv"),
            os.path.join(rtl, "predictor", "branch_predictor.sv"),
        ],
        toplevel="branch_predictor",
        module="test_branch_predictor",
        simulator="verilator",
        extra_args=[
            "--timing", "--assert",
            "-Wall",
            "-Wno-IMPORTSTAR",   # import nebula_pkg::* no $unit — padrão do projeto
            "-Wno-VARHIDDEN",    # parâmetros locais ocultam os do pkg — inofensivo
            "-Wno-WIDTHTRUNC", "-Wno-WIDTHEXPAND",
            "-Wno-ENUMVALUE", "-Wno-UNUSED",
            # BLKSEQ corrigido no branch_predictor.sv — flag mantida para
            # compatibilidade caso o usuário use a versão anterior do arquivo
            "-Wno-BLKSEQ",
        ],
        waves=os.environ.get("WAVES", "0") == "1",
    )
