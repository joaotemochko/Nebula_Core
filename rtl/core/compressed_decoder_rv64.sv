`timescale 1ns/1ps
`default_nettype none

/**
 * @module compressed_decoder_rv64
 * @brief Decodificador de Instruções Comprimidas RV64C Completo
 *
 * @details Expande instruções de 16-bit para 32-bit equivalentes.
 * Implementação completa de RV64C conforme especificação RISC-V.
 *
 * RV64C Instruction Set:
 * 
 * Quadrant 0 (op=00):
 *   C.ADDI4SPN  - Add immediate * 4 to sp, store in rd'
 *   C.FLD       - FP load double from memory
 *   C.LW        - Load word from memory
 *   C.LD        - Load doubleword from memory (RV64)
 *   C.FSD       - FP store double to memory
 *   C.SW        - Store word to memory
 *   C.SD        - Store doubleword to memory (RV64)
 *
 * Quadrant 1 (op=01):
 *   C.NOP       - No operation
 *   C.ADDI      - Add immediate to register
 *   C.ADDIW     - Add immediate word (RV64)
 *   C.LI        - Load immediate
 *   C.ADDI16SP  - Add immediate * 16 to sp
 *   C.LUI       - Load upper immediate
 *   C.SRLI      - Shift right logical immediate
 *   C.SRAI      - Shift right arithmetic immediate
 *   C.ANDI      - And immediate
 *   C.SUB       - Subtract
 *   C.XOR       - Exclusive or
 *   C.OR        - Or
 *   C.AND       - And
 *   C.SUBW      - Subtract word (RV64)
 *   C.ADDW      - Add word (RV64)
 *   C.J         - Jump
 *   C.BEQZ      - Branch if equal to zero
 *   C.BNEZ      - Branch if not equal to zero
 *
 * Quadrant 2 (op=10):
 *   C.SLLI      - Shift left logical immediate
 *   C.FLDSP     - FP load double from stack
 *   C.LWSP      - Load word from stack
 *   C.LDSP      - Load doubleword from stack (RV64)
 *   C.JR        - Jump register
 *   C.MV        - Move register
 *   C.EBREAK    - Environment break
 *   C.JALR      - Jump and link register
 *   C.ADD       - Add registers
 *   C.FSDSP     - FP store double to stack
 *   C.SWSP      - Store word to stack
 *   C.SDSP      - Store doubleword to stack (RV64)
 *
 * Nota: C.JAL (RV32 only) não é suportado em RV64
 * Nota: C.FLW/C.FSW (RV32 only) não são suportados em RV64
 *       Em RV64, os slots de C.FLW/C.FSW são usados para C.LD/C.SD
 *
 * Latência: Combinacional (0 ciclos)
 */
module compressed_decoder_rv64 (
    input  wire [15:0]  cinstr,         // Compressed instruction (16-bit)
    output logic [31:0] instr,          // Expanded instruction (32-bit)
    output logic        valid,          // Valid compressed instruction
    output logic        illegal,        // Illegal instruction
    output logic        is_compressed   // Input was a compressed instruction
);

    // =========================================================================
    // Opcode Fields
    // =========================================================================
    
    wire [1:0]  op     = cinstr[1:0];       // Quadrant
    wire [2:0]  funct3 = cinstr[15:13];     // Function code
    wire [1:0]  funct2 = cinstr[6:5];       // For C.SUB/XOR/OR/AND and C.SUBW/ADDW
    
    // =========================================================================
    // Register Fields
    // =========================================================================
    
    // Full 5-bit register specifiers (for Quadrant 2)
    wire [4:0]  rd_rs1_full = cinstr[11:7];  // rd/rs1 in bits [11:7]
    wire [4:0]  rs2_full    = cinstr[6:2];   // rs2 in bits [6:2]
    
    // Compressed 3-bit register specifiers (maps to x8-x15)
    wire [4:0]  rd_prime  = {2'b01, cinstr[4:2]};   // rd' -> x8-x15
    wire [4:0]  rs1_prime = {2'b01, cinstr[9:7]};   // rs1' -> x8-x15
    wire [4:0]  rs2_prime = {2'b01, cinstr[4:2]};   // rs2' -> x8-x15
    
    // =========================================================================
    // Immediate Fields - Carefully extracted per instruction type
    // =========================================================================
    
    // C.ADDI4SPN: nzuimm[5:4|9:6|2|3] -> addi rd', x2, nzuimm
    // Bits: [12:11|10:7|6|5] -> [5:4|9:6|2|3]
    wire [9:0] imm_addi4spn = {cinstr[10:7], cinstr[12:11], cinstr[5], cinstr[6], 2'b00};
    
    // C.FLD/C.LD: uimm[5:3|7:6] -> fld/ld rd', uimm(rs1')
    // Bits: [12:10|6:5] -> [5:3|7:6]
    wire [7:0] imm_ld = {cinstr[6:5], cinstr[12:10], 3'b000};
    
    // C.LW: uimm[5:3|2|6] -> lw rd', uimm(rs1')
    // Bits: [12:10|6|5] -> [5:3|2|6]
    wire [6:0] imm_lw = {cinstr[5], cinstr[12:10], cinstr[6], 2'b00};
    
    // C.FSD/C.SD: uimm[5:3|7:6] -> fsd/sd rs2', uimm(rs1')
    // Same encoding as C.FLD/C.LD
    wire [7:0] imm_sd = {cinstr[6:5], cinstr[12:10], 3'b000};
    
    // C.SW: uimm[5:3|2|6] -> sw rs2', uimm(rs1')
    // Same encoding as C.LW
    wire [6:0] imm_sw = {cinstr[5], cinstr[12:10], cinstr[6], 2'b00};
    
    // C.ADDI/C.NOP/C.LI/C.ANDI: nzimm[5|4:0] -> various
    // Bits: [12|6:2] -> [5|4:0]
    wire [5:0]  imm_ci     = {cinstr[12], cinstr[6:2]};
    wire [11:0] imm_ci_sex = {{6{cinstr[12]}}, cinstr[12], cinstr[6:2]};
    
    // C.ADDI16SP: nzimm[9|4|6|8:7|5] -> addi x2, x2, nzimm
    // Bits: [12|6|5|4:3|2] -> [9|4|6|8:7|5]
    wire [9:0]  imm_addi16sp     = {cinstr[12], cinstr[4:3], cinstr[5], cinstr[2], cinstr[6], 4'b0000};
    wire [11:0] imm_addi16sp_sex = {{2{cinstr[12]}}, cinstr[12], cinstr[4:3], cinstr[5], cinstr[2], cinstr[6], 4'b0000};
    
    // C.LUI: nzimm[17|16:12] -> lui rd, nzimm
    // Bits: [12|6:2] -> [17|16:12]
    wire [5:0]  imm_lui_raw = {cinstr[12], cinstr[6:2]};
    wire [19:0] imm_lui     = {{14{cinstr[12]}}, cinstr[12], cinstr[6:2]};
    
    // C.J: imm[11|4|9:8|10|6|7|3:1|5] -> jal x0, imm
    // Bits: [12|11|10:9|8|7|6|5:3|2] -> [11|4|9:8|10|6|7|3:1|5]
    wire [11:0] imm_j = {cinstr[12], cinstr[8], cinstr[10:9], cinstr[6], 
                         cinstr[7], cinstr[2], cinstr[11], cinstr[5:3], 1'b0};
    wire [20:0] imm_j_sex = {{9{cinstr[12]}}, cinstr[12], cinstr[8], cinstr[10:9], 
                             cinstr[6], cinstr[7], cinstr[2], cinstr[11], cinstr[5:3], 1'b0};
    
    // C.BEQZ/C.BNEZ: imm[8|4:3|7:6|2:1|5] -> beq/bne rs1', x0, imm
    // Bits: [12|11:10|6:5|4:3|2] -> [8|4:3|7:6|2:1|5]
    wire [8:0]  imm_b     = {cinstr[12], cinstr[6:5], cinstr[2], cinstr[11:10], cinstr[4:3], 1'b0};
    wire [12:0] imm_b_sex = {{4{cinstr[12]}}, cinstr[12], cinstr[6:5], cinstr[2], 
                             cinstr[11:10], cinstr[4:3], 1'b0};
    
    // C.SLLI/C.SRLI/C.SRAI: shamt[5|4:0] (RV64: full 6-bit shift)
    // Bits: [12|6:2] -> [5|4:0]
    wire [5:0] shamt = {cinstr[12], cinstr[6:2]};
    
    // C.LWSP: uimm[5|4:2|7:6] -> lw rd, uimm(x2)
    // Bits: [12|6:4|3:2] -> [5|4:2|7:6]
    wire [7:0]  imm_lwsp     = {cinstr[3:2], cinstr[12], cinstr[6:4], 2'b00};
    wire [11:0] imm_lwsp_zex = {4'b0000, cinstr[3:2], cinstr[12], cinstr[6:4], 2'b00};
    
    // C.LDSP/C.FLDSP: uimm[5|4:3|8:6] -> ld/fld rd, uimm(x2)
    // Bits: [12|6:5|4:2] -> [5|4:3|8:6]
    wire [8:0]  imm_ldsp     = {cinstr[4:2], cinstr[12], cinstr[6:5], 3'b000};
    wire [11:0] imm_ldsp_zex = {3'b000, cinstr[4:2], cinstr[12], cinstr[6:5], 3'b000};
    
    // C.SWSP: uimm[5:2|7:6] -> sw rs2, uimm(x2)
    // Bits: [12:9|8:7] -> [5:2|7:6]
    wire [7:0]  imm_swsp     = {cinstr[8:7], cinstr[12:9], 2'b00};
    wire [11:0] imm_swsp_zex = {4'b0000, cinstr[8:7], cinstr[12:9], 2'b00};
    
    // C.SDSP/C.FSDSP: uimm[5:3|8:6] -> sd/fsd rs2, uimm(x2)
    // Bits: [12:10|9:7] -> [5:3|8:6]
    wire [8:0]  imm_sdsp     = {cinstr[9:7], cinstr[12:10], 3'b000};
    wire [11:0] imm_sdsp_zex = {3'b000, cinstr[9:7], cinstr[12:10], 3'b000};
    
    // =========================================================================
    // RV32I/RV64I Base Opcodes
    // =========================================================================
    
    localparam [6:0] OP_LOAD     = 7'b0000011;  // LB, LH, LW, LD, LBU, LHU, LWU
    localparam [6:0] OP_LOAD_FP  = 7'b0000111;  // FLW, FLD
    localparam [6:0] OP_STORE    = 7'b0100011;  // SB, SH, SW, SD
    localparam [6:0] OP_STORE_FP = 7'b0100111;  // FSW, FSD
    localparam [6:0] OP_IMM      = 7'b0010011;  // ADDI, SLTI, SLTIU, XORI, ORI, ANDI, SLLI, SRLI, SRAI
    localparam [6:0] OP_IMM_W    = 7'b0011011;  // ADDIW, SLLIW, SRLIW, SRAIW (RV64)
    localparam [6:0] OP_REG      = 7'b0110011;  // ADD, SUB, SLL, SLT, SLTU, XOR, SRL, SRA, OR, AND
    localparam [6:0] OP_REG_W    = 7'b0111011;  // ADDW, SUBW, SLLW, SRLW, SRAW (RV64)
    localparam [6:0] OP_LUI      = 7'b0110111;  // LUI
    localparam [6:0] OP_BRANCH   = 7'b1100011;  // BEQ, BNE, BLT, BGE, BLTU, BGEU
    localparam [6:0] OP_JALR     = 7'b1100111;  // JALR
    localparam [6:0] OP_JAL      = 7'b1101111;  // JAL
    localparam [6:0] OP_SYSTEM   = 7'b1110011;  // ECALL, EBREAK, CSR*
    
    // =========================================================================
    // Funct3 codes
    // =========================================================================
    
    localparam [2:0] F3_ADD_SUB = 3'b000;
    localparam [2:0] F3_SLL     = 3'b001;
    localparam [2:0] F3_SLT     = 3'b010;
    localparam [2:0] F3_SLTU    = 3'b011;
    localparam [2:0] F3_XOR     = 3'b100;
    localparam [2:0] F3_SRL_SRA = 3'b101;
    localparam [2:0] F3_OR      = 3'b110;
    localparam [2:0] F3_AND     = 3'b111;
    
    localparam [2:0] F3_LW      = 3'b010;
    localparam [2:0] F3_LD      = 3'b011;
    localparam [2:0] F3_SW      = 3'b010;
    localparam [2:0] F3_SD      = 3'b011;
    localparam [2:0] F3_FLD     = 3'b011;
    localparam [2:0] F3_FSD     = 3'b011;
    
    localparam [2:0] F3_BEQ     = 3'b000;
    localparam [2:0] F3_BNE     = 3'b001;
    
    // =========================================================================
    // Decode Logic
    // =========================================================================
    
    always_comb begin
        // Defaults
        instr = 32'h0000_0000;
        valid = 1'b0;
        illegal = 1'b0;
        is_compressed = (op != 2'b11);
        
        if (!is_compressed) begin
            // Not a compressed instruction - pass through
            valid = 1'b0;
            illegal = 1'b0;
        end
        else begin
            case (op)
                // =============================================================
                // QUADRANT 0 (op = 2'b00)
                // =============================================================
                2'b00: begin
                    case (funct3)
                        3'b000: begin
                            // C.ADDI4SPN: addi rd', x2, nzuimm
                            // ILLEGAL if nzuimm = 0
                            if (imm_addi4spn == 0) begin
                                illegal = 1'b1;
                            end
                            else begin
                                valid = 1'b1;
                                // addi rd', x2, imm
                                instr = {{2{1'b0}}, imm_addi4spn, 5'd2, F3_ADD_SUB, rd_prime, OP_IMM};
                            end
                        end
                        
                        3'b001: begin
                            // C.FLD: fld rd', offset(rs1')
                            valid = 1'b1;
                            instr = {{4{1'b0}}, imm_ld, rs1_prime, F3_FLD, rd_prime, OP_LOAD_FP};
                        end
                        
                        3'b010: begin
                            // C.LW: lw rd', offset(rs1')
                            valid = 1'b1;
                            instr = {{5{1'b0}}, imm_lw, rs1_prime, F3_LW, rd_prime, OP_LOAD};
                        end
                        
                        3'b011: begin
                            // C.LD: ld rd', offset(rs1') [RV64 only]
                            // Note: In RV32, this would be C.FLW
                            valid = 1'b1;
                            instr = {{4{1'b0}}, imm_ld, rs1_prime, F3_LD, rd_prime, OP_LOAD};
                        end
                        
                        3'b100: begin
                            // Reserved
                            illegal = 1'b1;
                        end
                        
                        3'b101: begin
                            // C.FSD: fsd rs2', offset(rs1')
                            valid = 1'b1;
                            instr = {imm_sd[7:5], rs2_prime, rs1_prime, F3_FSD, imm_sd[4:0], OP_STORE_FP};
                        end
                        
                        3'b110: begin
                            // C.SW: sw rs2', offset(rs1')
                            valid = 1'b1;
                            instr = {imm_sw[6:5], rs2_prime, rs1_prime, F3_SW, imm_sw[4:0], OP_STORE};
                        end
                        
                        3'b111: begin
                            // C.SD: sd rs2', offset(rs1') [RV64 only]
                            // Note: In RV32, this would be C.FSW
                            valid = 1'b1;
                            instr = {imm_sd[7:5], rs2_prime, rs1_prime, F3_SD, imm_sd[4:0], OP_STORE};
                        end
                        
                        default: illegal = 1'b1;
                    endcase
                end
                
                // =============================================================
                // QUADRANT 1 (op = 2'b01)
                // =============================================================
                2'b01: begin
                    case (funct3)
                        3'b000: begin
                            // C.NOP (rd=0, imm=0) or C.ADDI (rd!=0 or imm!=0)
                            // addi rd, rd, nzimm
                            // C.NOP: addi x0, x0, 0 (hint, but valid)
                            valid = 1'b1;
                            // Note: HINT when rd=0 and imm!=0, but still valid
                            instr = {imm_ci_sex, rd_rs1_full, F3_ADD_SUB, rd_rs1_full, OP_IMM};
                        end
                        
                        3'b001: begin
                            // C.ADDIW: addiw rd, rd, imm [RV64 only]
                            // Note: In RV32, this would be C.JAL
                            // ILLEGAL if rd = 0
                            if (rd_rs1_full == 0) begin
                                illegal = 1'b1;
                            end
                            else begin
                                valid = 1'b1;
                                instr = {imm_ci_sex, rd_rs1_full, F3_ADD_SUB, rd_rs1_full, OP_IMM_W};
                            end
                        end
                        
                        3'b010: begin
                            // C.LI: addi rd, x0, imm
                            // HINT if rd = 0
                            valid = 1'b1;
                            instr = {imm_ci_sex, 5'd0, F3_ADD_SUB, rd_rs1_full, OP_IMM};
                        end
                        
                        3'b011: begin
                            if (rd_rs1_full == 5'd2) begin
                                // C.ADDI16SP: addi x2, x2, nzimm
                                // ILLEGAL if nzimm = 0
                                if (imm_addi16sp == 0) begin
                                    illegal = 1'b1;
                                end
                                else begin
                                    valid = 1'b1;
                                    instr = {imm_addi16sp_sex, 5'd2, F3_ADD_SUB, 5'd2, OP_IMM};
                                end
                            end
                            else begin
                                // C.LUI: lui rd, nzimm
                                // ILLEGAL if rd = 0 or nzimm = 0
                                // rd = 2 is handled above (C.ADDI16SP)
                                if (rd_rs1_full == 0 || imm_lui_raw == 0) begin
                                    illegal = 1'b1;
                                end
                                else begin
                                    valid = 1'b1;
                                    instr = {imm_lui, rd_rs1_full, OP_LUI};
                                end
                            end
                        end
                        
                        3'b100: begin
                            // C.SRLI, C.SRAI, C.ANDI, C.SUB, C.XOR, C.OR, C.AND, C.SUBW, C.ADDW
                            case (cinstr[11:10])
                                2'b00: begin
                                    // C.SRLI: srli rd', rd', shamt
                                    // ILLEGAL if shamt = 0 (but NSE - reserved for custom)
                                    // RV64: shamt[5:0] is valid
                                    valid = 1'b1;
                                    instr = {6'b000000, shamt, rs1_prime, F3_SRL_SRA, rs1_prime, OP_IMM};
                                end
                                
                                2'b01: begin
                                    // C.SRAI: srai rd', rd', shamt
                                    // RV64: shamt[5:0] is valid
                                    valid = 1'b1;
                                    instr = {6'b010000, shamt, rs1_prime, F3_SRL_SRA, rs1_prime, OP_IMM};
                                end
                                
                                2'b10: begin
                                    // C.ANDI: andi rd', rd', imm
                                    valid = 1'b1;
                                    instr = {imm_ci_sex, rs1_prime, F3_AND, rs1_prime, OP_IMM};
                                end
                                
                                2'b11: begin
                                    // Register-register operations
                                    if (cinstr[12] == 1'b0) begin
                                        // RV32/RV64 base operations
                                        case (funct2)
                                            2'b00: begin
                                                // C.SUB: sub rd', rd', rs2'
                                                valid = 1'b1;
                                                instr = {7'b0100000, rs2_prime, rs1_prime, F3_ADD_SUB, rs1_prime, OP_REG};
                                            end
                                            
                                            2'b01: begin
                                                // C.XOR: xor rd', rd', rs2'
                                                valid = 1'b1;
                                                instr = {7'b0000000, rs2_prime, rs1_prime, F3_XOR, rs1_prime, OP_REG};
                                            end
                                            
                                            2'b10: begin
                                                // C.OR: or rd', rd', rs2'
                                                valid = 1'b1;
                                                instr = {7'b0000000, rs2_prime, rs1_prime, F3_OR, rs1_prime, OP_REG};
                                            end
                                            
                                            2'b11: begin
                                                // C.AND: and rd', rd', rs2'
                                                valid = 1'b1;
                                                instr = {7'b0000000, rs2_prime, rs1_prime, F3_AND, rs1_prime, OP_REG};
                                            end
                                        endcase
                                    end
                                    else begin
                                        // RV64-only operations (cinstr[12] = 1)
                                        case (funct2)
                                            2'b00: begin
                                                // C.SUBW: subw rd', rd', rs2' [RV64 only]
                                                valid = 1'b1;
                                                instr = {7'b0100000, rs2_prime, rs1_prime, F3_ADD_SUB, rs1_prime, OP_REG_W};
                                            end
                                            
                                            2'b01: begin
                                                // C.ADDW: addw rd', rd', rs2' [RV64 only]
                                                valid = 1'b1;
                                                instr = {7'b0000000, rs2_prime, rs1_prime, F3_ADD_SUB, rs1_prime, OP_REG_W};
                                            end
                                            
                                            2'b10, 2'b11: begin
                                                // Reserved
                                                illegal = 1'b1;
                                            end
                                        endcase
                                    end
                                end
                            endcase
                        end
                        
                        3'b101: begin
                            // C.J: jal x0, offset
                            valid = 1'b1;
                            // JAL encoding: imm[20|10:1|11|19:12] rd opcode
                            instr = {imm_j_sex[20], imm_j_sex[10:1], imm_j_sex[11], 
                                    imm_j_sex[19:12], 5'd0, OP_JAL};
                        end
                        
                        3'b110: begin
                            // C.BEQZ: beq rs1', x0, offset
                            valid = 1'b1;
                            // BEQ encoding: imm[12|10:5] rs2 rs1 funct3 imm[4:1|11] opcode
                            instr = {imm_b_sex[12], imm_b_sex[10:5], 5'd0, rs1_prime, 
                                    F3_BEQ, imm_b_sex[4:1], imm_b_sex[11], OP_BRANCH};
                        end
                        
                        3'b111: begin
                            // C.BNEZ: bne rs1', x0, offset
                            valid = 1'b1;
                            instr = {imm_b_sex[12], imm_b_sex[10:5], 5'd0, rs1_prime, 
                                    F3_BNE, imm_b_sex[4:1], imm_b_sex[11], OP_BRANCH};
                        end
                        
                        default: illegal = 1'b1;
                    endcase
                end
                
                // =============================================================
                // QUADRANT 2 (op = 2'b10)
                // =============================================================
                2'b10: begin
                    case (funct3)
                        3'b000: begin
                            // C.SLLI: slli rd, rd, shamt
                            // HINT if rd = 0
                            // RV64: shamt[5:0] valid (shamt=0 is HINT)
                            if (rd_rs1_full == 0) begin
                                // HINT - valid but NOP-like
                                valid = 1'b1;
                                instr = {6'b000000, shamt, rd_rs1_full, F3_SLL, rd_rs1_full, OP_IMM};
                            end
                            else begin
                                valid = 1'b1;
                                instr = {6'b000000, shamt, rd_rs1_full, F3_SLL, rd_rs1_full, OP_IMM};
                            end
                        end
                        
                        3'b001: begin
                            // C.FLDSP: fld rd, offset(x2)
                            // Note: rd can be any FP register (including f0)
                            valid = 1'b1;
                            instr = {imm_ldsp_zex, 5'd2, F3_FLD, rd_rs1_full, OP_LOAD_FP};
                        end
                        
                        3'b010: begin
                            // C.LWSP: lw rd, offset(x2)
                            // ILLEGAL if rd = 0
                            if (rd_rs1_full == 0) begin
                                illegal = 1'b1;
                            end
                            else begin
                                valid = 1'b1;
                                instr = {imm_lwsp_zex, 5'd2, F3_LW, rd_rs1_full, OP_LOAD};
                            end
                        end
                        
                        3'b011: begin
                            // C.LDSP: ld rd, offset(x2) [RV64 only]
                            // Note: In RV32, this would be C.FLWSP
                            // ILLEGAL if rd = 0
                            if (rd_rs1_full == 0) begin
                                illegal = 1'b1;
                            end
                            else begin
                                valid = 1'b1;
                                instr = {imm_ldsp_zex, 5'd2, F3_LD, rd_rs1_full, OP_LOAD};
                            end
                        end
                        
                        3'b100: begin
                            // C.JR, C.MV, C.EBREAK, C.JALR, C.ADD
                            if (cinstr[12] == 1'b0) begin
                                if (rs2_full == 0) begin
                                    // C.JR: jalr x0, rs1, 0
                                    // ILLEGAL if rs1 = 0
                                    if (rd_rs1_full == 0) begin
                                        illegal = 1'b1;
                                    end
                                    else begin
                                        valid = 1'b1;
                                        instr = {12'b0, rd_rs1_full, 3'b000, 5'd0, OP_JALR};
                                    end
                                end
                                else begin
                                    // C.MV: add rd, x0, rs2
                                    // HINT if rd = 0
                                    valid = 1'b1;
                                    instr = {7'b0000000, rs2_full, 5'd0, F3_ADD_SUB, rd_rs1_full, OP_REG};
                                end
                            end
                            else begin
                                if (rs2_full == 0) begin
                                    if (rd_rs1_full == 0) begin
                                        // C.EBREAK: ebreak
                                        valid = 1'b1;
                                        instr = 32'h0010_0073;  // EBREAK encoding
                                    end
                                    else begin
                                        // C.JALR: jalr x1, rs1, 0
                                        valid = 1'b1;
                                        instr = {12'b0, rd_rs1_full, 3'b000, 5'd1, OP_JALR};
                                    end
                                end
                                else begin
                                    // C.ADD: add rd, rd, rs2
                                    // HINT if rd = 0
                                    valid = 1'b1;
                                    instr = {7'b0000000, rs2_full, rd_rs1_full, F3_ADD_SUB, rd_rs1_full, OP_REG};
                                end
                            end
                        end
                        
                        3'b101: begin
                            // C.FSDSP: fsd rs2, offset(x2)
                            valid = 1'b1;
                            instr = {imm_sdsp_zex[11:5], rs2_full, 5'd2, F3_FSD, imm_sdsp_zex[4:0], OP_STORE_FP};
                        end
                        
                        3'b110: begin
                            // C.SWSP: sw rs2, offset(x2)
                            valid = 1'b1;
                            instr = {imm_swsp_zex[11:5], rs2_full, 5'd2, F3_SW, imm_swsp_zex[4:0], OP_STORE};
                        end
                        
                        3'b111: begin
                            // C.SDSP: sd rs2, offset(x2) [RV64 only]
                            // Note: In RV32, this would be C.FSWSP
                            valid = 1'b1;
                            instr = {imm_sdsp_zex[11:5], rs2_full, 5'd2, F3_SD, imm_sdsp_zex[4:0], OP_STORE};
                        end
                        
                        default: illegal = 1'b1;
                    endcase
                end
                
                // =============================================================
                // QUADRANT 3 (op = 2'b11) - Not compressed
                // =============================================================
                2'b11: begin
                    is_compressed = 1'b0;
                    valid = 1'b0;
                end
                
                default: begin
                    illegal = 1'b1;
                end
            endcase
        end
    end

endmodule
