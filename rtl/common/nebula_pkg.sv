`timescale 1ns/1ps
`ifndef NEBULA_PKG_SV
`define NEBULA_PKG_SV

/**
 * @package nebula_pkg
 * @brief Definições globais do Nebula Core / Nebula Cluster
 *
 * ISA: RV64GC (RV64IMAFDC + Zicsr + Zifencei), M/S/U, Sv39
 *
 * Microarquitetura (núcleo): dual-issue in-order, 5 estágios de backend
 *   F (fetch + BTB/RAS) -> FQ (fila de instruções) -> D (decode/issue x2)
 *   -> EX (2 ALUs, branch, AGU, DIV/FPU) -> MEM (LSU, MUL) -> WB (commit)
 */
package nebula_pkg;

    // =========================================================================
    // Parâmetros de arquitetura
    // =========================================================================
    parameter int NUM_CORES        = 4;
    parameter int CLUSTER_ID_WIDTH = 4;

    parameter int XLEN        = 64;
    parameter int FLEN        = 64;
    parameter int ILEN        = 32;
    parameter int CLEN        = 16;
    parameter int PADDR_WIDTH = 56;
    parameter int VADDR_WIDTH = 39;
    parameter int VPN_WIDTH   = 27;
    parameter int PPN_WIDTH   = 44;
    parameter int ASID_WIDTH  = 16;

    parameter int L1D_LINE_SIZE = 64;
    parameter int L2_LINE_SIZE  = 64;

    typedef logic [XLEN-1:0]        xreg_t;
    typedef logic [FLEN-1:0]        freg_t;
    typedef logic [PADDR_WIDTH-1:0] paddr_t;
    typedef logic [VADDR_WIDTH-1:0] vaddr_t;
    typedef logic [4:0]             reg_addr_t;
    typedef logic [$clog2(NUM_CORES)-1:0] core_id_t;

    // =========================================================================
    // Privilégio
    // =========================================================================
    typedef enum logic [1:0] {
        PRIV_USER       = 2'b00,
        PRIV_SUPERVISOR = 2'b01,
        PRIV_RESERVED   = 2'b10,
        PRIV_MACHINE    = 2'b11
    } priv_t;

    // =========================================================================
    // Causas de exceção (mcause, bit de interrupção separado)
    // =========================================================================
    localparam logic [4:0] EXC_INSTR_MISALIGNED   = 5'd0;
    localparam logic [4:0] EXC_INSTR_ACCESS_FAULT = 5'd1;
    localparam logic [4:0] EXC_ILLEGAL_INSTR      = 5'd2;
    localparam logic [4:0] EXC_BREAKPOINT         = 5'd3;
    localparam logic [4:0] EXC_LOAD_MISALIGNED    = 5'd4;
    localparam logic [4:0] EXC_LOAD_ACCESS_FAULT  = 5'd5;
    localparam logic [4:0] EXC_STORE_MISALIGNED   = 5'd6;
    localparam logic [4:0] EXC_STORE_ACCESS_FAULT = 5'd7;
    localparam logic [4:0] EXC_ECALL_U            = 5'd8;
    localparam logic [4:0] EXC_ECALL_S            = 5'd9;
    localparam logic [4:0] EXC_ECALL_M            = 5'd11;
    localparam logic [4:0] EXC_INSTR_PAGE_FAULT   = 5'd12;
    localparam logic [4:0] EXC_LOAD_PAGE_FAULT    = 5'd13;
    localparam logic [4:0] EXC_STORE_PAGE_FAULT   = 5'd15;

    // =========================================================================
    // Unidades funcionais / classes de micro-op
    // =========================================================================
    typedef enum logic [3:0] {
        FU_ALU    = 4'd0,
        FU_BRANCH = 4'd1,   // branch condicional
        FU_JAL    = 4'd2,
        FU_JALR   = 4'd3,
        FU_LOAD   = 4'd4,   // inclui FLW/FLD
        FU_STORE  = 4'd5,   // inclui FSW/FSD
        FU_AMO    = 4'd6,   // LR/SC/AMO*
        FU_MUL    = 4'd7,
        FU_DIV    = 4'd8,
        FU_FPU    = 4'd9,
        FU_CSR    = 4'd10,
        FU_SYS    = 4'd11,  // MRET/SRET/WFI/SFENCE.VMA/FENCE.I/FENCE
        FU_NONE   = 4'd15   // instrução que só gera exceção
    } fu_t;

    typedef enum logic [3:0] {
        ALU_ADD  = 4'd0,
        ALU_SUB  = 4'd1,
        ALU_SLL  = 4'd2,
        ALU_SLT  = 4'd3,
        ALU_SLTU = 4'd4,
        ALU_XOR  = 4'd5,
        ALU_SRL  = 4'd6,
        ALU_SRA  = 4'd7,
        ALU_OR   = 4'd8,
        ALU_AND  = 4'd9
    } alu_op_t;

    typedef enum logic [2:0] {
        SYS_NONE   = 3'd0,
        SYS_MRET   = 3'd1,
        SYS_SRET   = 3'd2,
        SYS_WFI    = 3'd3,
        SYS_SFENCE = 3'd4,
        SYS_FENCEI = 3'd5,
        SYS_FENCE  = 3'd6
    } sys_op_t;

    // Operações da FPU (decodificadas no estágio D)
    typedef enum logic [4:0] {
        FOP_ADD    = 5'd0,
        FOP_SUB    = 5'd1,
        FOP_MUL    = 5'd2,
        FOP_DIV    = 5'd3,
        FOP_SQRT   = 5'd4,
        FOP_SGNJ   = 5'd5,
        FOP_SGNJN  = 5'd6,
        FOP_SGNJX  = 5'd7,
        FOP_MIN    = 5'd8,
        FOP_MAX    = 5'd9,
        FOP_CVT_FF = 5'd10,  // fcvt.s.d / fcvt.d.s
        FOP_CVT_IF = 5'd11,  // fcvt.{w,wu,l,lu}.{s,d} (fp -> int)
        FOP_CVT_FI = 5'd12,  // fcvt.{s,d}.{w,wu,l,lu} (int -> fp)
        FOP_MV_XF  = 5'd13,  // fmv.x.w / fmv.x.d
        FOP_MV_FX  = 5'd14,  // fmv.w.x / fmv.d.x
        FOP_CLASS  = 5'd15,
        FOP_EQ     = 5'd16,
        FOP_LT     = 5'd17,
        FOP_LE     = 5'd18,
        FOP_MADD   = 5'd19,
        FOP_MSUB   = 5'd20,
        FOP_NMSUB  = 5'd21,
        FOP_NMADD  = 5'd22
    } fpu_op_t;

    // =========================================================================
    // Micro-op (saída do decodificador, carregada pelo pipeline)
    // =========================================================================
    typedef struct packed {
        logic           valid;
        logic [63:0]    pc;
        logic [31:0]    instr;       // bits originais (16b zero-extend p/ RVC)
        logic           is_rvc;
        fu_t            fu;
        // operandos
        logic [4:0]     rs1, rs2, rs3, rd;
        logic           rs1_en, rs2_en, rs3_en, rd_en;
        logic           rs1_fp, rs2_fp, rd_fp;   // rs3 é sempre FP
        logic [63:0]    imm;
        // ALU
        alu_op_t        alu_op;
        logic           op_a_pc;     // operando A = PC (AUIPC)
        logic           op_a_zero;   // operando A = 0 (LUI)
        logic           op_b_imm;    // operando B = imediato
        logic           word;        // operação *W (32 bits)
        logic [2:0]     funct3;      // cond. de branch / tamanho mem / op mul-div / op csr
        // memória
        logic           mem_fp;      // FLW/FLD/FSW/FSD
        logic [4:0]     amo_op;      // funct5 do AMO
        // FPU
        fpu_op_t        fop;
        logic           fp_dbl;      // formato D (senão S)
        logic           fp_src_dbl;  // formato da fonte em FCVT.S.D/D.S
        logic [1:0]     fp_int_sz;   // 0=W 1=WU 2=L 3=LU (conversões)
        logic [2:0]     rm;          // rounding mode já resolvido (DYN -> frm)
        // sistema
        logic [11:0]    csr;
        sys_op_t        sys;
        logic           serial;      // serializante: emite sozinho, flush no commit
        // exceção detectada no front/decode
        logic           exc;
        logic [4:0]     exc_cause;
        logic [63:0]    exc_tval;
        // predição
        logic           pred_taken;
        logic [63:0]    pred_target;
    } uop_t;

    // =========================================================================
    // FPU flags
    // =========================================================================
    typedef struct packed {
        logic NV;
        logic DZ;
        logic OF;
        logic UF;
        logic NX;
    } fflags_t;

    typedef enum logic [2:0] {
        RM_RNE = 3'b000,
        RM_RTZ = 3'b001,
        RM_RDN = 3'b010,
        RM_RUP = 3'b011,
        RM_RMM = 3'b100,
        RM_DYN = 3'b111
    } rounding_mode_t;

    // =========================================================================
    // Tipos legados usados pelo cluster / L2 (mantidos por compatibilidade)
    // =========================================================================
    typedef enum logic [1:0] {
        CACHE_INVALID   = 2'b00,
        CACHE_SHARED    = 2'b01,
        CACHE_EXCLUSIVE = 2'b10,
        CACHE_MODIFIED  = 2'b11
    } cache_state_t;

    typedef enum logic [2:0] {
        COH_NONE,
        COH_READ,
        COH_READ_EX,
        COH_UPGRADE,
        COH_WRITEBACK,
        COH_INVALIDATE
    } coherence_op_t;

    typedef struct packed {
        logic           valid;
        core_id_t       core_id;
        logic           is_ifetch;
        logic           is_write;
        logic           is_amo;
        logic [4:0]     amo_op;
        paddr_t         addr;
        logic [L1D_LINE_SIZE*8-1:0] wdata;
        logic           upgrade;
    } l2_req_t;

    typedef struct packed {
        logic           valid;
        core_id_t       core_id;
        logic           is_ifetch;
        logic [L2_LINE_SIZE*8-1:0] rdata;
        logic           error;
        cache_state_t   state;
    } l2_resp_t;

    typedef struct packed {
        logic           valid;
        coherence_op_t  op;
        paddr_t         addr;
        core_id_t       requester;
    } snoop_req_t;

    typedef struct packed {
        logic           valid;
        logic           has_data;
        logic [L1D_LINE_SIZE*8-1:0] data;
        cache_state_t   state;
    } snoop_resp_t;

endpackage

`endif
