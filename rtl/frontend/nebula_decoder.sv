`timescale 1ns/1ps
`default_nettype none

import nebula_pkg::*;

/**
 * @module nebula_decoder
 * @brief Decodificador RV64GC (32 bits) -> micro-op.
 *
 * Combinacional. Detecta instruções ilegais (incluindo FP com mstatus.FS=Off,
 * rounding mode inválido e instruções privilegiadas fora do modo permitido).
 * Acessos a CSR têm a legalidade verificada no commit (nebula_csr).
 */
module nebula_decoder (
    input  wire [31:0]  instr,        // instrução (expandida se RVC)
    input  wire [31:0]  raw,          // bits originais (para mtval)
    input  wire         is_rvc,
    input  wire         rvc_illegal,
    input  wire [63:0]  pc,
    // estado arquitetural relevante (estável: escritas de CSR fazem flush)
    input  wire [1:0]   priv,
    input  wire [1:0]   fs,           // mstatus.FS
    input  wire         tvm, tw, tsr,
    input  wire [2:0]   frm,
    output uop_t        uop
);

    logic [6:0] opcode;
    logic [2:0] f3;
    logic [6:0] f7;
    logic [4:0] rd, rs1, rs2, rs3;
    logic [63:0] imm_i, imm_s, imm_b, imm_u, imm_j;

    assign opcode = instr[6:0];
    assign f3     = instr[14:12];
    assign f7     = instr[31:25];
    assign rd     = instr[11:7];
    assign rs1    = instr[19:15];
    assign rs2    = instr[24:20];
    assign rs3    = instr[31:27];
    assign imm_i  = {{52{instr[31]}}, instr[31:20]};
    assign imm_s  = {{52{instr[31]}}, instr[31:25], instr[11:7]};
    assign imm_b  = {{51{instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0};
    assign imm_u  = {{32{instr[31]}}, instr[31:12], 12'b0};
    assign imm_j  = {{43{instr[31]}}, instr[31], instr[19:12], instr[20], instr[30:21], 1'b0};

    logic illegal;
    logic fp_off;
    assign fp_off = (fs == 2'b00);

    // rounding mode resolvido; inválido -> ilegal
    logic [2:0] rm_eff;
    logic       rm_bad;
    always_comb begin
        rm_eff = (f3 == 3'b111) ? frm : f3;
        rm_bad = (rm_eff == 3'b101) || (rm_eff == 3'b110) || (rm_eff == 3'b111);
    end

    always_comb begin
        uop          = '0;
        uop.valid    = 1'b1;
        uop.pc       = pc;
        uop.instr    = raw;
        uop.is_rvc   = is_rvc;
        uop.fu       = FU_ALU;
        uop.rs1      = rs1;
        uop.rs2      = rs2;
        uop.rs3      = rs3;
        uop.rd       = rd;
        uop.funct3   = f3;
        uop.alu_op   = ALU_ADD;
        uop.rm       = rm_eff;
        uop.sys      = SYS_NONE;
        uop.csr      = instr[31:20];
        uop.amo_op   = instr[31:27];
        illegal      = 1'b0;

        case (opcode)
            // ----------------------------------------------------------------
            7'b0110111: begin // LUI
                uop.rd_en = 1'b1; uop.op_a_zero = 1'b1; uop.op_b_imm = 1'b1; uop.imm = imm_u;
            end
            7'b0010111: begin // AUIPC
                uop.rd_en = 1'b1; uop.op_a_pc = 1'b1; uop.op_b_imm = 1'b1; uop.imm = imm_u;
            end
            7'b1101111: begin // JAL
                uop.fu = FU_JAL; uop.rd_en = 1'b1; uop.imm = imm_j;
            end
            7'b1100111: begin // JALR
                uop.fu = FU_JALR; uop.rd_en = 1'b1; uop.rs1_en = 1'b1; uop.imm = imm_i;
                illegal = (f3 != 3'b000);
            end
            7'b1100011: begin // BRANCH
                uop.fu = FU_BRANCH; uop.rs1_en = 1'b1; uop.rs2_en = 1'b1; uop.imm = imm_b;
                illegal = (f3 == 3'b010) || (f3 == 3'b011);
            end
            7'b0000011: begin // LOAD
                uop.fu = FU_LOAD; uop.rd_en = 1'b1; uop.rs1_en = 1'b1; uop.imm = imm_i;
                illegal = (f3 == 3'b111);
            end
            7'b0100011: begin // STORE
                uop.fu = FU_STORE; uop.rs1_en = 1'b1; uop.rs2_en = 1'b1; uop.imm = imm_s;
                illegal = f3[2];
            end
            7'b0010011: begin // OP-IMM
                uop.rd_en = 1'b1; uop.rs1_en = 1'b1; uop.op_b_imm = 1'b1; uop.imm = imm_i;
                case (f3)
                    3'b000: uop.alu_op = ALU_ADD;
                    3'b010: uop.alu_op = ALU_SLT;
                    3'b011: uop.alu_op = ALU_SLTU;
                    3'b100: uop.alu_op = ALU_XOR;
                    3'b110: uop.alu_op = ALU_OR;
                    3'b111: uop.alu_op = ALU_AND;
                    3'b001: begin uop.alu_op = ALU_SLL; illegal = (instr[31:26] != 6'b000000); end
                    3'b101: begin
                        uop.alu_op = instr[30] ? ALU_SRA : ALU_SRL;
                        illegal = (instr[31] != 1'b0) || (instr[29:26] != 4'b0000);
                    end
                endcase
            end
            7'b0011011: begin // OP-IMM-32
                uop.rd_en = 1'b1; uop.rs1_en = 1'b1; uop.op_b_imm = 1'b1; uop.imm = imm_i; uop.word = 1'b1;
                case (f3)
                    3'b000: uop.alu_op = ALU_ADD;
                    3'b001: begin uop.alu_op = ALU_SLL; illegal = (f7 != 7'b0000000); end
                    3'b101: begin
                        uop.alu_op = instr[30] ? ALU_SRA : ALU_SRL;
                        illegal = (instr[31] != 1'b0) || (instr[29:25] != 5'b00000);
                    end
                    default: illegal = 1'b1;
                endcase
            end
            7'b0110011: begin // OP
                uop.rd_en = 1'b1; uop.rs1_en = 1'b1; uop.rs2_en = 1'b1;
                if (f7 == 7'b0000001) begin
                    uop.fu = f3[2] ? FU_DIV : FU_MUL;
                end else if (f7 == 7'b0000000) begin
                    case (f3)
                        3'b000: uop.alu_op = ALU_ADD;
                        3'b001: uop.alu_op = ALU_SLL;
                        3'b010: uop.alu_op = ALU_SLT;
                        3'b011: uop.alu_op = ALU_SLTU;
                        3'b100: uop.alu_op = ALU_XOR;
                        3'b101: uop.alu_op = ALU_SRL;
                        3'b110: uop.alu_op = ALU_OR;
                        3'b111: uop.alu_op = ALU_AND;
                    endcase
                end else if (f7 == 7'b0100000) begin
                    case (f3)
                        3'b000: uop.alu_op = ALU_SUB;
                        3'b101: uop.alu_op = ALU_SRA;
                        default: illegal = 1'b1;
                    endcase
                end else illegal = 1'b1;
            end
            7'b0111011: begin // OP-32
                uop.rd_en = 1'b1; uop.rs1_en = 1'b1; uop.rs2_en = 1'b1; uop.word = 1'b1;
                if (f7 == 7'b0000001) begin
                    uop.fu = f3[2] ? FU_DIV : FU_MUL;
                    illegal = (f3 == 3'b001) || (f3 == 3'b010) || (f3 == 3'b011); // sem MULHW
                end else if (f7 == 7'b0000000) begin
                    case (f3)
                        3'b000: uop.alu_op = ALU_ADD;
                        3'b001: uop.alu_op = ALU_SLL;
                        3'b101: uop.alu_op = ALU_SRL;
                        default: illegal = 1'b1;
                    endcase
                end else if (f7 == 7'b0100000) begin
                    case (f3)
                        3'b000: uop.alu_op = ALU_SUB;
                        3'b101: uop.alu_op = ALU_SRA;
                        default: illegal = 1'b1;
                    endcase
                end else illegal = 1'b1;
            end
            7'b0001111: begin // MISC-MEM
                if (f3 == 3'b000) begin
                    uop.fu = FU_SYS; uop.sys = SYS_FENCE;
                end else if (f3 == 3'b001) begin
                    uop.fu = FU_SYS; uop.sys = SYS_FENCEI; uop.serial = 1'b1;
                end else illegal = 1'b1;
            end
            7'b1110011: begin // SYSTEM
                if (f3 == 3'b000) begin
                    uop.fu = FU_SYS;
                    uop.serial = 1'b1;
                    if (instr == 32'h0000_0073) begin          // ECALL
                        uop.fu = FU_NONE; uop.exc = 1'b1;
                        uop.exc_cause = (priv == PRIV_MACHINE)    ? EXC_ECALL_M :
                                        (priv == PRIV_SUPERVISOR) ? EXC_ECALL_S : EXC_ECALL_U;
                    end else if (instr == 32'h0010_0073) begin // EBREAK
                        uop.fu = FU_NONE; uop.exc = 1'b1; uop.exc_cause = EXC_BREAKPOINT;
                        uop.exc_tval = pc;
                    end else if (instr == 32'h3020_0073) begin // MRET
                        uop.sys = SYS_MRET;
                        illegal = (priv != PRIV_MACHINE);
                    end else if (instr == 32'h1020_0073) begin // SRET
                        uop.sys = SYS_SRET;
                        illegal = (priv == PRIV_USER) || (priv == PRIV_SUPERVISOR && tsr);
                    end else if (instr == 32'h1050_0073) begin // WFI
                        uop.sys = SYS_WFI;
                        illegal = (priv == PRIV_USER) || (priv == PRIV_SUPERVISOR && tw);
                    end else if (f7 == 7'b0001001 && rd == 5'd0) begin // SFENCE.VMA
                        uop.sys = SYS_SFENCE; uop.rs1_en = 1'b1; uop.rs2_en = 1'b1;
                        illegal = (priv == PRIV_USER) || (priv == PRIV_SUPERVISOR && tvm);
                    end else illegal = 1'b1;
                end else if (f3 == 3'b100) begin
                    illegal = 1'b1;
                end else begin                                  // CSR*
                    uop.fu     = FU_CSR;
                    uop.serial = 1'b1;
                    uop.rd_en  = 1'b1;
                    uop.rs1_en = !f3[2];
                    uop.imm    = {59'b0, rs1};
                end
            end
            7'b0101111: begin // AMO
                uop.fu = FU_AMO; uop.rd_en = 1'b1; uop.rs1_en = 1'b1;
                uop.rs2_en = (instr[31:27] != 5'b00010);
                case (instr[31:27])
                    5'b00010: illegal = (rs2 != 5'd0);          // LR
                    5'b00011, 5'b00001, 5'b00000, 5'b00100, 5'b01100,
                    5'b01000, 5'b10000, 5'b10100, 5'b11000, 5'b11100: ;
                    default: illegal = 1'b1;
                endcase
                if (f3 != 3'b010 && f3 != 3'b011) illegal = 1'b1;
            end
            // ----------------------------------------------------------------
            // Ponto flutuante
            // ----------------------------------------------------------------
            7'b0000111: begin // FLW/FLD
                uop.fu = FU_LOAD; uop.mem_fp = 1'b1; uop.rd_en = 1'b1; uop.rd_fp = 1'b1;
                uop.rs1_en = 1'b1; uop.imm = imm_i;
                illegal = fp_off || !(f3 == 3'b010 || f3 == 3'b011);
            end
            7'b0100111: begin // FSW/FSD
                uop.fu = FU_STORE; uop.mem_fp = 1'b1; uop.rs1_en = 1'b1;
                uop.rs2_en = 1'b1; uop.rs2_fp = 1'b1; uop.imm = imm_s;
                illegal = fp_off || !(f3 == 3'b010 || f3 == 3'b011);
            end
            7'b1000011, 7'b1000111, 7'b1001011, 7'b1001111: begin // FMADD/FMSUB/FNMSUB/FNMADD
                uop.fu = FU_FPU; uop.serial = 1'b0;
                uop.rd_en = 1'b1; uop.rd_fp = 1'b1;
                uop.rs1_en = 1'b1; uop.rs1_fp = 1'b1;
                uop.rs2_en = 1'b1; uop.rs2_fp = 1'b1;
                uop.rs3_en = 1'b1;
                uop.fp_dbl = instr[25];
                case (opcode[3:2])
                    2'b00: uop.fop = FOP_MADD;
                    2'b01: uop.fop = FOP_MSUB;
                    2'b10: uop.fop = FOP_NMSUB;
                    2'b11: uop.fop = FOP_NMADD;
                endcase
                illegal = fp_off || instr[26] || rm_bad;
            end
            7'b1010011: begin // OP-FP
                uop.fu = FU_FPU;
                uop.fp_dbl = instr[25];
                uop.rd_en = 1'b1; uop.rd_fp = 1'b1;
                uop.rs1_en = 1'b1; uop.rs1_fp = 1'b1;
                uop.rs2_en = 1'b1; uop.rs2_fp = 1'b1;
                illegal = fp_off || instr[26];
                case (instr[31:27])
                    5'b00000: begin uop.fop = FOP_ADD;  if (rm_bad) illegal = 1'b1; end
                    5'b00001: begin uop.fop = FOP_SUB;  if (rm_bad) illegal = 1'b1; end
                    5'b00010: begin uop.fop = FOP_MUL;  if (rm_bad) illegal = 1'b1; end
                    5'b00011: begin uop.fop = FOP_DIV;  if (rm_bad) illegal = 1'b1; end
                    5'b01011: begin
                        uop.fop = FOP_SQRT; uop.rs2_en = 1'b0;
                        if (rm_bad || rs2 != 5'd0) illegal = 1'b1;
                    end
                    5'b00100: begin
                        case (f3)
                            3'b000: uop.fop = FOP_SGNJ;
                            3'b001: uop.fop = FOP_SGNJN;
                            3'b010: uop.fop = FOP_SGNJX;
                            default: illegal = 1'b1;
                        endcase
                    end
                    5'b00101: begin
                        case (f3)
                            3'b000: uop.fop = FOP_MIN;
                            3'b001: uop.fop = FOP_MAX;
                            default: illegal = 1'b1;
                        endcase
                    end
                    5'b01000: begin // FCVT.S.D (fmt=S, rs2=1) / FCVT.D.S (fmt=D, rs2=0)
                        uop.fop = FOP_CVT_FF; uop.rs2_en = 1'b0;
                        uop.fp_src_dbl = rs2[0];
                        if (rm_bad || rs2[4:1] != 4'b0000 || (rs2[0] == instr[25])) illegal = 1'b1;
                    end
                    5'b10100: begin
                        uop.rd_fp = 1'b0;
                        case (f3)
                            3'b010: uop.fop = FOP_EQ;
                            3'b001: uop.fop = FOP_LT;
                            3'b000: uop.fop = FOP_LE;
                            default: illegal = 1'b1;
                        endcase
                    end
                    5'b11000: begin // FCVT.int.fp
                        uop.fop = FOP_CVT_IF; uop.rd_fp = 1'b0; uop.rs2_en = 1'b0;
                        uop.fp_int_sz = rs2[1:0];
                        if (rm_bad || rs2[4:2] != 3'b000) illegal = 1'b1;
                    end
                    5'b11010: begin // FCVT.fp.int
                        uop.fop = FOP_CVT_FI; uop.rs1_fp = 1'b0; uop.rs2_en = 1'b0;
                        uop.fp_int_sz = rs2[1:0];
                        if (rm_bad || rs2[4:2] != 3'b000) illegal = 1'b1;
                    end
                    5'b11100: begin
                        uop.rd_fp = 1'b0; uop.rs2_en = 1'b0;
                        if (f3 == 3'b000)      uop.fop = FOP_MV_XF;
                        else if (f3 == 3'b001) uop.fop = FOP_CLASS;
                        else illegal = 1'b1;
                        if (rs2 != 5'd0) illegal = 1'b1;
                    end
                    5'b11110: begin
                        uop.fop = FOP_MV_FX; uop.rs1_fp = 1'b0; uop.rs2_en = 1'b0;
                        if (f3 != 3'b000 || rs2 != 5'd0) illegal = 1'b1;
                    end
                    default: illegal = 1'b1;
                endcase
            end
            default: illegal = 1'b1;
        endcase

        if (is_rvc && rvc_illegal) illegal = 1'b1;

        // x0 como destino: sem escrita (exceto FP)
        if (uop.rd_en && !uop.rd_fp && uop.rd == 5'd0) uop.rd_en = 1'b0;
        if (!uop.rs1_en) uop.rs1 = '0;
        if (!uop.rs2_en) uop.rs2 = '0;
        if (!uop.rs3_en) uop.rs3 = '0;

        if (illegal) begin
            uop.fu        = FU_NONE;
            uop.exc       = 1'b1;
            uop.exc_cause = EXC_ILLEGAL_INSTR;
            uop.exc_tval  = {32'b0, raw};
            uop.rd_en     = 1'b0;
            uop.rs1_en    = 1'b0;
            uop.rs2_en    = 1'b0;
            uop.rs3_en    = 1'b0;
            uop.serial    = 1'b0;
        end
        if (uop.exc) begin
            uop.rd_en  = 1'b0;
            uop.serial = 1'b0;
        end
        // FPU, DIV, AMO e CSR emitem sozinhos (slot 0) — tratado no issue
    end

endmodule
