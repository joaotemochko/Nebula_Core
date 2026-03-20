"""
test_alu.py — Testbench cocotb para ALU via nebula_backend_fpu

Estratégia:
  - Instancia nebula_backend_fpu com todos os sinais de cache/MDU/FPU
    mockados (sempre prontos, sem resposta de dados).
  - Injeta instruções ALU via frontend_packet_t (decoded_instr_t).
  - Aguarda S_WRITEBACK (instr_retired=1) e lê o resultado pelo
    sinal de writeback do regfile interno — mas como o regfile é
    interno, usamos a saída de backend_ctrl e monitoramos dcache_wdata
    ou lemos via força de sinal do Verilator.
  - Na prática: injeta instrução, espera instr_retired, lê exec_result
    monitorando o sinal result que vai para o regfile no S_WRITEBACK.

decoded_instr_t campos relevantes (nebula_pkg.sv, struct packed):
  Ordem MSB→LSB conforme declaração no pkg:
    opcode[6:0], rd[4:0], funct3[2:0], rs1[4:0], rs2[4:0], rs3[4:0],
    funct7[6:0], imm[63:0], csr_addr[11:0], valid, is_compressed,
    is_alu, is_alu_w, is_branch, is_jal, is_jalr, is_load, is_store,
    is_amo, is_lr, is_sc, is_mdu, is_fp, is_fp_load, is_fp_store,
    is_fma, is_fp_single, is_fp_double, fpu_op[4:0], rm[2:0],
    is_csr, is_fence, is_fence_i, is_sfence_vma, is_ecall, is_ebreak,
    is_mret, is_sret, is_wfi, pc[38:0], bp_pred[44:0]

frontend_packet_t:
    instr0[decoded_instr_t], instr1[decoded_instr_t],
    instr0_valid, instr1_valid, dual_issue

Nota: calcular o offset exato de cada campo numa struct packed grande
é frágil e propenso a erros. A abordagem mais robusta com Verilator é
usar sinais hierárquicos diretos via dut._id() ou forçar via DPI-C.

Abordagem adotada aqui: usar o Verilator com -flatten e acessar os
campos da struct diretamente pelo nome hierárquico que o Verilator expõe,
ex: dut.frontend_in__DOT__instr0__DOT__is_alu

Se o Verilator não expõe os campos da struct como sinais separados
(depende da versão), usamos bit manipulation no valor inteiro.
"""

import os
import ctypes
from cocotb_test.simulator import run

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer

XLEN    = 64
VADDR   = 39
MASK64  = (1 << 64) - 1
MASK32  = (1 << 32) - 1
MASK39  = (1 << 39) - 1

RTL_DIR = os.path.abspath(os.environ.get("RTL_DIR", "../rtl"))

# ---------------------------------------------------------------------------
# Tamanho de decoded_instr_t em bits
# Contando campos do nebula_pkg.sv em ordem de declaração (packed = MSB first):
#
#  opcode        7
#  rd            5
#  funct3        3
#  rs1           5
#  rs2           5
#  rs3           5
#  funct7        7
#  imm          64
#  csr_addr     12
#  valid         1
#  is_compressed 1
#  is_alu        1
#  is_alu_w      1
#  is_branch     1
#  is_jal        1
#  is_jalr       1
#  is_load       1
#  is_store      1
#  is_amo        1
#  is_lr         1
#  is_sc         1
#  is_mdu        1
#  is_fp         1
#  is_fp_load    1
#  is_fp_store   1
#  is_fma        1
#  is_fp_single  1
#  is_fp_double  1
#  fpu_op        5
#  rm            3
#  is_csr        1
#  is_fence      1
#  is_fence_i    1
#  is_sfence_vma 1
#  is_ecall      1
#  is_ebreak     1
#  is_mret       1
#  is_sret       1
#  is_wfi        1
#  pc           39
#  bp_pred      45   (bp_prediction_t)
#  ──────────────
#  TOTAL       236
# ---------------------------------------------------------------------------

# Offsets LSB de cada campo em decoded_instr_t (calculados de trás para frente)
_F = {}
_pos = 0
def _add(name, width):
    _F[name] = (_pos, width)
    return width

# ordem inversa (LSB first para construção do valor)
_pos += _add('bp_pred',       45)
_pos += _add('pc',            39)
_pos += _add('is_wfi',         1)
_pos += _add('is_sret',        1)
_pos += _add('is_mret',        1)
_pos += _add('is_ebreak',      1)
_pos += _add('is_ecall',       1)
_pos += _add('is_sfence_vma',  1)
_pos += _add('is_fence_i',     1)
_pos += _add('is_fence',       1)
_pos += _add('is_csr',         1)
_pos += _add('rm',             3)
_pos += _add('fpu_op',         5)
_pos += _add('is_fp_double',   1)
_pos += _add('is_fp_single',   1)
_pos += _add('is_fma',         1)
_pos += _add('is_fp_store',    1)
_pos += _add('is_fp_load',     1)
_pos += _add('is_fp',          1)
_pos += _add('is_mdu',         1)
_pos += _add('is_sc',          1)
_pos += _add('is_lr',          1)
_pos += _add('is_amo',         1)
_pos += _add('is_store',       1)
_pos += _add('is_load',        1)
_pos += _add('is_jalr',        1)
_pos += _add('is_jal',         1)
_pos += _add('is_branch',      1)
_pos += _add('is_alu_w',       1)
_pos += _add('is_alu',         1)
_pos += _add('is_compressed',  1)
_pos += _add('valid',          1)
_pos += _add('csr_addr',      12)
_pos += _add('imm',           64)
_pos += _add('funct7',         7)
_pos += _add('rs3',            5)
_pos += _add('rs2',            5)
_pos += _add('rs1',            5)
_pos += _add('funct3',         3)
_pos += _add('rd',             5)
_pos += _add('opcode',         7)
INSTR_BITS = _pos  # deve ser 233

def build_instr(**fields):
    """Constrói o valor inteiro de decoded_instr_t com os campos fornecidos."""
    v = 0
    for name, val in fields.items():
        assert name in _F, f"Campo desconhecido: {name}"
        lsb, width = _F[name]
        mask = (1 << width) - 1
        v |= (int(val) & mask) << lsb
    return v

# frontend_packet_t: instr1 | instr0 | instr1_valid | instr0_valid | dual_issue
# instr1 ocupa bits [2*INSTR_BITS+2 : INSTR_BITS+3]
# instr0 ocupa bits [INSTR_BITS+2   : 3]
# instr1_valid = bit 2
# instr0_valid = bit 1
# dual_issue   = bit 0
PKT_BITS = 2 * INSTR_BITS + 3

def build_packet(instr0, instr0_valid=1, instr1=0, instr1_valid=0, dual_issue=0):
    # frontend_packet_t (struct packed, MSB first = ordem de declaração):
    #   instr0 (declarado 1º) → bits [468:236]  ← MSB
    #   instr1 (declarado 2º) → bits [235:3]
    #   instr1_valid          → bit 2
    #   instr0_valid          → bit 1
    #   dual_issue            → bit 0            ← LSB
    mask = (1 << INSTR_BITS) - 1
    v  = (instr0       & mask) << (INSTR_BITS + 3)  # instr0 no MSB
    v |= (instr1       & mask) << 3                  # instr1 abaixo
    v |= (instr1_valid & 1)    << 2
    v |= (instr0_valid & 1)    << 1
    v |= (dual_issue   & 1)    << 0
    return v

# Opcodes RV64I
OP_IMM   = 0b0010011   # ADDI, SLTI, XORI, ORI, ANDI, SLLI, SRLI, SRAI
OP_REG   = 0b0110011   # ADD, SUB, SLL, SLT, SLTU, XOR, SRL, SRA, OR, AND
OP_IMM_W = 0b0011011   # ADDIW
OP_REG_W = 0b0111011   # ADDW, SUBW, etc.
OP_LUI   = 0b0110111
OP_AUIPC = 0b0010111

F3_ADD  = 0b000
F3_SLL  = 0b001
F3_SLT  = 0b010
F3_SLTU = 0b011
F3_XOR  = 0b100
F3_SRL  = 0b101
F3_OR   = 0b110
F3_AND  = 0b111

# ---------------------------------------------------------------------------
# Modelos de referência
# ---------------------------------------------------------------------------
def s64(v): return ctypes.c_int64(v & MASK64).value
def s32(v): return ctypes.c_int32(v & MASK32).value
def sext32(v): return ctypes.c_int64(ctypes.c_int32(v & MASK32).value).value & MASK64

def ref_add(a,b):    return (a+b) & MASK64
def ref_sub(a,b):    return (a-b) & MASK64
def ref_sll(a,sh):   return (a << (sh & 63)) & MASK64
def ref_slt(a,b):    return 1 if s64(a) < s64(b) else 0
def ref_sltu(a,b):   return 1 if (a&MASK64) < (b&MASK64) else 0
def ref_xor(a,b):    return (a ^ b) & MASK64
def ref_srl(a,sh):   return (a & MASK64) >> (sh & 63)
def ref_sra(a,sh):   return (s64(a) >> (sh & 63)) & MASK64
def ref_or(a,b):     return (a | b) & MASK64
def ref_and(a,b):    return (a & b) & MASK64
def ref_addw(a,b):   return sext32((a+b) & MASK32)
def ref_subw(a,b):   return sext32((a-b) & MASK32)
def ref_sllw(a,sh):  return sext32((a << (sh & 31)) & MASK32)
def ref_srlw(a,sh):  return sext32((a & MASK32) >> (sh & 31))
def ref_sraw(a,sh):  return sext32((s32(a) >> (sh & 31)) & MASK32)

# ---------------------------------------------------------------------------
# Reset e helpers de drive
# ---------------------------------------------------------------------------

async def reset_dut(dut):
    """Reset completo — zera todos os inputs."""
    dut.rst_n.value             = 0
    dut.frontend_valid.value    = 0
    dut.frontend_in.value       = 0
    # Cache mocks: sempre ready, nunca responde dados
    dut.dcache_ready.value      = 1
    dut.dcache_resp_valid.value = 0
    dut.dcache_resp_data.value  = 0
    dut.dcache_resp_error.value = 0
    # TLB mock: sempre hit, sem page fault
    dut.dtlb_hit.value          = 1
    dut.dtlb_ppn.value          = 0
    dut.dtlb_page_fault.value   = 0
    # PTW mock: nunca requisitado
    dut.ptw_ready.value         = 1
    dut.ptw_resp_valid.value    = 0
    dut.ptw_page_fault.value    = 0
    # MDU mock: sempre ready, nunca responde
    dut.mdu_ready.value         = 1
    dut.mdu_resp_valid.value    = 0
    dut.mdu_result.value        = 0
    # FPU mock: sempre ready, nunca responde
    dut.fpu_ready.value         = 1
    dut.fpu_resp_valid.value    = 0
    dut.fpu_result.value        = 0
    dut.fpu_int_result.value    = 0
    dut.fpu_fflags.value        = 0
    # CSR mock
    dut.csr_rdata.value         = 0
    dut.csr_fault.value         = 0
    # Trap mocks
    dut.trap_vector.value       = 0
    dut.return_pc.value         = 0
    dut.return_priv.value       = 0
    dut.interrupt_pending.value = 0
    dut.interrupt_cause.value   = 0
    # Frontend exception
    dut.frontend_exception.value       = 0
    dut.frontend_exception_cause.value = 0
    dut.frontend_exception_value.value = 0
    # Control
    dut.current_priv.value      = 0b11   # Machine mode
    dut.mmu_enabled.value       = 0
    dut.dcache_flush_done.value = 1

    for _ in range(6):
        await RisingEdge(dut.clk)
    dut.rst_n.value = 1
    await RisingEdge(dut.clk)

async def inject_and_wait(dut, instr_val, timeout=30):
    """
    Injeta uma instrução decodificada e aguarda instr_retired=1.
    Retorna o valor de exec_result capturado no ciclo de writeback.

    backend_ctrl_t (packed, MSB→LSB):
        stall(1) | flush(1) | redirect(1) | redirect_pc(39)
        bit 41 = stall
    """
    from cocotb.triggers import FallingEdge

    BACKEND_CTRL_BITS = 1 + 1 + 1 + VADDR  # 42 bits
    STALL_BIT = BACKEND_CTRL_BITS - 1       # bit 41

    pkt = build_packet(instr0=instr_val, instr0_valid=1)

    # Esperar backend livre (stall=0)
    t = 0
    while (int(dut.backend_ctrl.value) >> STALL_BIT) & 1:
        await RisingEdge(dut.clk)
        t += 1
        assert t < 50, "Timeout aguardando backend livre (stall)"

    dut.frontend_valid.value = 1
    dut.frontend_in.value    = pkt
    await RisingEdge(dut.clk)
    dut.frontend_valid.value = 0
    dut.frontend_in.value    = 0

    # Aguardar instr_retired — lê exec_result no FallingEdge desse ciclo
    t = 0
    while not int(dut.instr_retired.value):
        await RisingEdge(dut.clk)
        t += 1
        if t % 5 == 0:
            try:
                state_val = int(dut.state.value)
            except Exception:
                state_val = -1
            dut._log.info(f"  ciclo={t} instr_retired=0 state={state_val}")
        assert t < timeout, (
            f"Timeout aguardando instr_retired após {t} ciclos. "
            f"Verifique se frontend_valid chegou ao backend e se "
            f"não há stall permanente."
        )

    # Ler exec_result no meio do ciclo (após propagação combinacional)
    await FallingEdge(dut.clk)
    try:
        result = int(dut.exec_result.value)
    except AttributeError:
        # exec_result não exposto — tentar via nome completo do Verilator
        try:
            result = int(dut.nebula_backend_fpu__DOT__exec_result.value)
        except AttributeError:
            raise AttributeError(
                "exec_result não acessível. Verifique se --public-flat-rw "
                "está sendo passado ao Verilator."
            )
    await RisingEdge(dut.clk)
    return result

def chk(dut, name, got, exp):
    exp &= MASK64
    assert got == exp, (
        f"\n[FAIL] {name}\n"
        f"  got={got:#018x}\n"
        f"  exp={exp:#018x}\n"
    )
    dut._log.info(f"[PASS] {name}")

# ---------------------------------------------------------------------------
# Constantes para build_instr (rs1=x1, rs2=x2, rd=x3, PC fixo)
# ---------------------------------------------------------------------------
RS1 = 1   # x1
RS2 = 2   # x2
RD  = 3   # x3
PC  = 0x1000_0000

# Pré-carregar x1 e x2 no regfile interno não é possível diretamente.
# Usamos a seguinte estratégia:
#   1. Executar LUI x1, imm  → x1 = imm << 12  (mas LUI carrega no RD, não RS)
#   Alternativa: usar instruções ADDI x1, x0, imm (imm 12 bits signed)
#   Para valores maiores: LUI xN, upper + ADDI xN, xN, lower
#
# Para simplicidade dos testes iniciais, limitamos os operandos a
# valores que cabem em 12 bits signed (-2048..2047) e usamos
# ADDI rs_dest, x0, imm para carregar o valor, depois fazemos a op.
#
# Fluxo de cada teste ALU:
#   1. ADDI x1, x0, val_a   → carrega val_a em x1
#   2. ADDI x2, x0, val_b   → carrega val_b em x2
#   3. OP   x3, x1, x2      → executa a operação, resultado em x3
#   4. Ler exec_result no ciclo de writeback do passo 3

async def load_reg(dut, rd, imm12):
    """ADDI rd, x0, imm12 — carrega imm12 (signed 12-bit) em rd."""
    instr = build_instr(
        opcode=OP_IMM, rd=rd, funct3=F3_ADD, rs1=0, rs2=0,
        imm=imm12 & MASK64, is_alu=1, valid=1, pc=PC,
    )
    await inject_and_wait(dut, instr)

# ---------------------------------------------------------------------------
# Testes
# ---------------------------------------------------------------------------

SMALL_PAIRS = [
    (0, 0), (1, 2), (100, 50), (-1, 1), (-5, -3), (2047, -2048),
]

@cocotb.test()
async def test_add(dut):
    """ADD x3, x1, x2 para vários pares de operandos."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    for a, b in SMALL_PAIRS:
        await load_reg(dut, RS1, a)
        await load_reg(dut, RS2, b)
        instr = build_instr(
            opcode=OP_REG, rd=RD, funct3=F3_ADD, rs1=RS1, rs2=RS2,
            funct7=0, is_alu=1, valid=1, pc=PC,
        )
        r = await inject_and_wait(dut, instr)
        chk(dut, f"ADD {a}+{b}", r, ref_add(a & MASK64, b & MASK64))

@cocotb.test()
async def test_sub(dut):
    """SUB x3, x1, x2 — funct7[5]=1."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    for a, b in SMALL_PAIRS:
        await load_reg(dut, RS1, a)
        await load_reg(dut, RS2, b)
        instr = build_instr(
            opcode=OP_REG, rd=RD, funct3=F3_ADD, rs1=RS1, rs2=RS2,
            funct7=0b0100000, is_alu=1, valid=1, pc=PC,
        )
        r = await inject_and_wait(dut, instr)
        chk(dut, f"SUB {a}-{b}", r, ref_sub(a & MASK64, b & MASK64))

@cocotb.test()
async def test_addi(dut):
    """ADDI x3, x1, imm — I-type."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    for a, b in SMALL_PAIRS:
        await load_reg(dut, RS1, a)
        instr = build_instr(
            opcode=OP_IMM, rd=RD, funct3=F3_ADD, rs1=RS1,
            imm=b & MASK64, is_alu=1, valid=1, pc=PC,
        )
        r = await inject_and_wait(dut, instr)
        chk(dut, f"ADDI {a}+{b}", r, ref_add(a & MASK64, b & MASK64))

@cocotb.test()
async def test_xor_or_and(dut):
    """XOR, OR, AND com pares pequenos."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    for a, b in SMALL_PAIRS:
        await load_reg(dut, RS1, a)
        await load_reg(dut, RS2, b)
        for f3, ref in [(F3_XOR, ref_xor), (F3_OR, ref_or), (F3_AND, ref_and)]:
            instr = build_instr(
                opcode=OP_REG, rd=RD, funct3=f3, rs1=RS1, rs2=RS2,
                funct7=0, is_alu=1, valid=1, pc=PC,
            )
            r = await inject_and_wait(dut, instr)
            name = {F3_XOR:"XOR", F3_OR:"OR", F3_AND:"AND"}[f3]
            chk(dut, f"{name} {a} {b}", r, ref(a & MASK64, b & MASK64))

@cocotb.test()
async def test_slt_sltu(dut):
    """SLT e SLTU."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    pairs = [(0,1),(1,0),(-1,0),(0,-1),(-2,-1)]
    for a, b in pairs:
        await load_reg(dut, RS1, a)
        await load_reg(dut, RS2, b)
        instr = build_instr(
            opcode=OP_REG, rd=RD, funct3=F3_SLT, rs1=RS1, rs2=RS2,
            funct7=0, is_alu=1, valid=1, pc=PC,
        )
        r = await inject_and_wait(dut, instr)
        chk(dut, f"SLT {a}<{b}", r, ref_slt(a & MASK64, b & MASK64))

        instr = build_instr(
            opcode=OP_REG, rd=RD, funct3=F3_SLTU, rs1=RS1, rs2=RS2,
            funct7=0, is_alu=1, valid=1, pc=PC,
        )
        r = await inject_and_wait(dut, instr)
        chk(dut, f"SLTU {a}<{b}", r, ref_sltu(a & MASK64, b & MASK64))

@cocotb.test()
async def test_shifts(dut):
    """SLL, SRL, SRA com shamt pequeno."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    pairs = [(0xFF, 4), (0xFF, 8), (-1, 1), (-128, 3)]
    for a, sh in pairs:
        await load_reg(dut, RS1, a)
        await load_reg(dut, RS2, sh)

        for f3, f7, ref, name in [
            (F3_SLL, 0,          ref_sll, "SLL"),
            (F3_SRL, 0,          ref_srl, "SRL"),
            (F3_SRL, 0b0100000,  ref_sra, "SRA"),
        ]:
            instr = build_instr(
                opcode=OP_REG, rd=RD, funct3=f3, rs1=RS1, rs2=RS2,
                funct7=f7, is_alu=1, valid=1, pc=PC,
            )
            r = await inject_and_wait(dut, instr)
            chk(dut, f"{name} {a}>>{sh}", r, ref(a & MASK64, sh))

@cocotb.test()
async def test_lui(dut):
    """LUI rd, imm → rd = imm (já com shift aplicado no decode)."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    for imm in [0x1000, 0x12000, 0x7FFFF000]:
        instr = build_instr(
            opcode=OP_LUI, rd=RD,
            imm=imm & MASK64,
            is_alu=1, valid=1, pc=PC,
        )
        r = await inject_and_wait(dut, instr)
        chk(dut, f"LUI {imm:#x}", r, imm & MASK64)

@cocotb.test()
async def test_auipc(dut):
    """AUIPC rd, imm → rd = PC + imm."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    for imm in [0x1000, 0x5000]:
        instr = build_instr(
            opcode=OP_AUIPC, rd=RD,
            imm=imm & MASK64,
            is_alu=1, valid=1, pc=PC,
        )
        r = await inject_and_wait(dut, instr)
        chk(dut, f"AUIPC PC={PC:#x} imm={imm:#x}", r, (PC + imm) & MASK64)

@cocotb.test()
async def test_addw_subw(dut):
    """ADDW e SUBW — operações 32-bit com sign-extension."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    for a, b in [(1,2),(100,50),(-1,1),(-5,-3)]:
        await load_reg(dut, RS1, a)
        await load_reg(dut, RS2, b)

        instr = build_instr(
            opcode=OP_REG_W, rd=RD, funct3=F3_ADD, rs1=RS1, rs2=RS2,
            funct7=0, is_alu=1, is_alu_w=1, valid=1, pc=PC,
        )
        r = await inject_and_wait(dut, instr)
        chk(dut, f"ADDW {a}+{b}", r, ref_addw(a & MASK64, b & MASK64))

        instr = build_instr(
            opcode=OP_REG_W, rd=RD, funct3=F3_ADD, rs1=RS1, rs2=RS2,
            funct7=0b0100000, is_alu=1, is_alu_w=1, valid=1, pc=PC,
        )
        r = await inject_and_wait(dut, instr)
        chk(dut, f"SUBW {a}-{b}", r, ref_subw(a & MASK64, b & MASK64))

# ---------------------------------------------------------------------------
# Entrada pytest
# ---------------------------------------------------------------------------
def test_alu_all():
    rtl = os.path.abspath(os.environ.get("RTL_DIR", "../rtl"))
    run(
        verilog_sources=[
            os.path.join(rtl, "common", "nebula_pkg.sv"),
            os.path.join(rtl, "common", "csr_unit.sv"),
            os.path.join(rtl, "backend", "mdu_rv64.sv"),
            os.path.join(rtl, "backend", "nebula_backend_fpu.sv"),
        ],
        toplevel="nebula_backend_fpu",
        module="test_alu",
        simulator="verilator",
        extra_args=[
            "--timing", "--assert",
            "-Wall",
            # Warnings suprimidos com justificativa:
            # IMPORTSTAR: import nebula_pkg::* no $unit scope — padrão do projeto
            "-Wno-IMPORTSTAR",
            # VARHIDDEN: parâmetros do módulo ocultam os do pkg — inofensivo,
            #            o parâmetro local tem precedência corretamente
            "-Wno-VARHIDDEN",
            # UNOPTFLAT: backend_ctrl tem lógica combinacional circular aparente
            #            porque backend_ctrl.stall depende de state e state
            #            depende de next_state que depende de backend_ctrl.flush —
            #            na prática não há ciclo real pois stall e flush são
            #            registros separados. Suprimido para simulação.
            "-Wno-UNOPTFLAT",
            "-Wno-WIDTHTRUNC", "-Wno-WIDTHEXPAND",
            "-Wno-ENUMVALUE", "-Wno-UNUSED", "-Wno-UNDRIVEN",
            "--top-module", "nebula_backend_fpu",
            # Expor sinais internos (exec_result, regfile, etc.)
            # necessário para verificar resultado sem instanciar módulo externo
            "--public-flat-rw",
        ],
        waves=os.environ.get("WAVES", "0") == "1",
    )
