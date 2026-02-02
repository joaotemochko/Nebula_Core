`timescale 1ns/1ps
`default_nettype none

import nebula_pkg::*;

/**
 * @module compressed_decoder
 * @brief Decodificador de Instruções Comprimidas RV64C
 *
 * @details Expande instruções de 16-bit para 32-bit equivalentes.
 * Suporta todos os quadrantes de RV64C:
 * - C0: Loads, Stores (stack e memory)
 * - C1: Arithmetic, Branches, Jumps
 * - C2: Loads, Stores, Jumps, Adds
 *
 * Latência: Combinacional (0 ciclos)
 */
module compressed_decoder (
    input  wire [15:0]  cinstr,     // Compressed instruction
    output logic [31:0] instr,      // Expanded instruction
    output logic        valid,      // Valid compressed instruction
    output logic        is_compressed
);

    // =========================================================================
    // Compressed Instruction Fields
    // =========================================================================
    
    wire [1:0]  op = cinstr[1:0];
    wire [2:0]  funct3 = cinstr[15:13];
    
    // Register addresses (compressed use x8-x15 mapped to 3-bit)
    wire [4:0]  rd_rs1 = cinstr[11:7];
    wire [4:0]  rs2 = cinstr[6:2];
    wire [4:0]  rd_prime = {2'b01, cinstr[4:2]};   // x8-x15
    wire [4:0]  rs1_prime = {2'b01, cinstr[9:7]};  // x8-x15
    wire [4:0]  rs2_prime = {2'b01, cinstr[4:2]};  // x8-x15
    
    // =========================================================================
    // Immediate Extraction
    // =========================================================================
    
    // C.ADDI, C.LI, C.ANDI
    wire [5:0] imm_ci = {cinstr[12], cinstr[6:2]};
    wire [11:0] imm_ci_sext = {{6{imm_ci[5]}}, imm_ci};
    
    // C.ADDI16SP
    wire [9:0] imm_addi16sp = {cinstr[12], cinstr[4:3], cinstr[5], cinstr[2], cinstr[6], 4'b0};
    wire [11:0] imm_addi16sp_sext = {{2{imm_addi16sp[9]}}, imm_addi16sp};
    
    // C.LUI
    wire [17:0] imm_lui = {cinstr[12], cinstr[6:2], 12'b0};
    wire [31:0] imm_lui_sext = {{14{imm_lui[17]}}, imm_lui};
    
    // C.J, C.JAL
    wire [11:0] imm_cj = {cinstr[12], cinstr[8], cinstr[10:9], cinstr[6], 
                         cinstr[7], cinstr[2], cinstr[11], cinstr[5:3], 1'b0};
    wire [20:0] imm_j_sext = {{9{imm_cj[11]}}, imm_cj};
    
    // C.BEQZ, C.BNEZ
    wire [8:0] imm_cb = {cinstr[12], cinstr[6:5], cinstr[2], cinstr[11:10], cinstr[4:3], 1'b0};
    wire [12:0] imm_b_sext = {{4{imm_cb[8]}}, imm_cb};
    
    // C.LWSP, C.FLWSP
    wire [7:0] imm_lwsp = {cinstr[3:2], cinstr[12], cinstr[6:4], 2'b0};
    wire [11:0] imm_lwsp_zext = {4'b0, imm_lwsp};
    
    // C.LDSP, C.FLDSP
    wire [8:0] imm_ldsp = {cinstr[4:2], cinstr[12], cinstr[6:5], 3'b0};
    wire [11:0] imm_ldsp_zext = {3'b0, imm_ldsp};
    
    // C.SWSP, C.FSWSP
    wire [7:0] imm_swsp = {cinstr[8:7], cinstr[12:9], 2'b0};
    wire [11:0] imm_swsp_zext = {4'b0, imm_swsp};
    
    // C.SDSP, C.FSDSP
    wire [8:0] imm_sdsp = {cinstr[9:7], cinstr[12:10], 3'b0};
    wire [11:0] imm_sdsp_zext = {3'b0, imm_sdsp};
    
    // C.LW, C.FLW
    wire [6:0] imm_cl_w = {cinstr[5], cinstr[12:10], cinstr[6], 2'b0};
    wire [11:0] imm_cl_w_zext = {5'b0, imm_cl_w};
    
    // C.LD, C.FLD
    wire [7:0] imm_cl_d = {cinstr[6:5], cinstr[12:10], 3'b0};
    wire [11:0] imm_cl_d_zext = {4'b0, imm_cl_d};
    
    // C.SW, C.FSW
    wire [6:0] imm_cs_w = {cinstr[5], cinstr[12:10], cinstr[6], 2'b0};
    
    // C.SD, C.FSD
    wire [7:0] imm_cs_d = {cinstr[6:5], cinstr[12:10], 3'b0};
    
    // C.ADDI4SPN
    wire [9:0] imm_addi4spn = {cinstr[10:7], cinstr[12:11], cinstr[5], cinstr[6], 2'b0};
    wire [11:0] imm_addi4spn_zext = {2'b0, imm_addi4spn};
    
    // =========================================================================
    // Decodificação
    // =========================================================================
    
    always_comb begin
        instr = 32'h0000_0000;
        valid = 1'b0;
        is_compressed = (op != 2'b11);
        
        if (!is_compressed) begin
            // Not a compressed instruction
            valid = 1'b0;
        end
        else begin
            case (op)
                // =============================================================
                // Quadrant 0 (C0)
                // =============================================================
                2'b00: begin
                    case (funct3)
                        3'b000: begin
                            // C.ADDI4SPN -> addi rd', x2, imm
                            if (imm_addi4spn != 0) begin
                                valid = 1'b1;
                                instr = {imm_addi4spn_zext, 5'd2, 3'b000, rd_prime, 7'b0010011};
                            end
                        end
                        
                        3'b001: begin
                            // C.FLD -> fld rd', imm(rs1')
                            valid = 1'b1;
                            instr = {imm_cl_d_zext, rs1_prime, 3'b011, rd_prime, 7'b0000111};
                        end
                        
                        3'b010: begin
                            // C.LW -> lw rd', imm(rs1')
                            valid = 1'b1;
                            instr = {imm_cl_w_zext, rs1_prime, 3'b010, rd_prime, 7'b0000011};
                        end
                        
                        3'b011: begin
                            // C.LD -> ld rd', imm(rs1') [RV64]
                            valid = 1'b1;
                            instr = {imm_cl_d_zext, rs1_prime, 3'b011, rd_prime, 7'b0000011};
                        end
                        
                        3'b101: begin
                            // C.FSD -> fsd rs2', imm(rs1')
                            valid = 1'b1;
                            instr = {imm_cs_d[7:5], rs2_prime, rs1_prime, 3'b011, imm_cs_d[4:0], 7'b0100111};
                        end
                        
                        3'b110: begin
                            // C.SW -> sw rs2', imm(rs1')
                            valid = 1'b1;
                            instr = {imm_cs_w[6:5], rs2_prime, rs1_prime, 3'b010, imm_cs_w[4:0], 7'b0100011};
                        end
                        
                        3'b111: begin
                            // C.SD -> sd rs2', imm(rs1') [RV64]
                            valid = 1'b1;
                            instr = {imm_cs_d[7:5], rs2_prime, rs1_prime, 3'b011, imm_cs_d[4:0], 7'b0100011};
                        end
                        
                        default: valid = 1'b0;
                    endcase
                end
                
                // =============================================================
                // Quadrant 1 (C1)
                // =============================================================
                2'b01: begin
                    case (funct3)
                        3'b000: begin
                            // C.NOP (rd=0, imm=0) or C.ADDI
                            valid = 1'b1;
                            instr = {imm_ci_sext, rd_rs1, 3'b000, rd_rs1, 7'b0010011};
                        end
                        
                        3'b001: begin
                            // C.ADDIW -> addiw rd, rd, imm [RV64]
                            if (rd_rs1 != 0) begin
                                valid = 1'b1;
                                instr = {imm_ci_sext, rd_rs1, 3'b000, rd_rs1, 7'b0011011};
                            end
                        end
                        
                        3'b010: begin
                            // C.LI -> addi rd, x0, imm
                            valid = 1'b1;
                            instr = {imm_ci_sext, 5'd0, 3'b000, rd_rs1, 7'b0010011};
                        end
                        
                        3'b011: begin
                            if (rd_rs1 == 5'd2) begin
                                // C.ADDI16SP -> addi x2, x2, imm
                                if (imm_addi16sp != 0) begin
                                    valid = 1'b1;
                                    instr = {imm_addi16sp_sext, 5'd2, 3'b000, 5'd2, 7'b0010011};
                                end
                            end
                            else if (rd_rs1 != 0) begin
                                // C.LUI -> lui rd, imm
                                valid = 1'b1;
                                instr = {imm_lui_sext[31:12], rd_rs1, 7'b0110111};
                            end
                        end
                        
                        3'b100: begin
                            case (cinstr[11:10])
                                2'b00: begin
                                    // C.SRLI -> srli rd', rd', shamt
                                    valid = 1'b1;
                                    instr = {6'b000000, cinstr[12], cinstr[6:2], rs1_prime, 3'b101, rs1_prime, 7'b0010011};
                                end
                                
                                2'b01: begin
                                    // C.SRAI -> srai rd', rd', shamt
                                    valid = 1'b1;
                                    instr = {6'b010000, cinstr[12], cinstr[6:2], rs1_prime, 3'b101, rs1_prime, 7'b0010011};
                                end
                                
                                2'b10: begin
                                    // C.ANDI -> andi rd', rd', imm
                                    valid = 1'b1;
                                    instr = {imm_ci_sext, rs1_prime, 3'b111, rs1_prime, 7'b0010011};
                                end
                                
                                2'b11: begin
                                    case ({cinstr[12], cinstr[6:5]})
                                        3'b000: begin
                                            // C.SUB -> sub rd', rd', rs2'
                                            valid = 1'b1;
                                            instr = {7'b0100000, rs2_prime, rs1_prime, 3'b000, rs1_prime, 7'b0110011};
                                        end
                                        
                                        3'b001: begin
                                            // C.XOR -> xor rd', rd', rs2'
                                            valid = 1'b1;
                                            instr = {7'b0000000, rs2_prime, rs1_prime, 3'b100, rs1_prime, 7'b0110011};
                                        end
                                        
                                        3'b010: begin
                                            // C.OR -> or rd', rd', rs2'
                                            valid = 1'b1;
                                            instr = {7'b0000000, rs2_prime, rs1_prime, 3'b110, rs1_prime, 7'b0110011};
                                        end
                                        
                                        3'b011: begin
                                            // C.AND -> and rd', rd', rs2'
                                            valid = 1'b1;
                                            instr = {7'b0000000, rs2_prime, rs1_prime, 3'b111, rs1_prime, 7'b0110011};
                                        end
                                        
                                        3'b100: begin
                                            // C.SUBW -> subw rd', rd', rs2' [RV64]
                                            valid = 1'b1;
                                            instr = {7'b0100000, rs2_prime, rs1_prime, 3'b000, rs1_prime, 7'b0111011};
                                        end
                                        
                                        3'b101: begin
                                            // C.ADDW -> addw rd', rd', rs2' [RV64]
                                            valid = 1'b1;
                                            instr = {7'b0000000, rs2_prime, rs1_prime, 3'b000, rs1_prime, 7'b0111011};
                                        end
                                        
                                        default: valid = 1'b0;
                                    endcase
                                end
                            endcase
                        end
                        
                        3'b101: begin
                            // C.J -> jal x0, imm
                            valid = 1'b1;
                            instr = {imm_j_sext[20], imm_j_sext[10:1], imm_j_sext[11], 
                                    imm_j_sext[19:12], 5'd0, 7'b1101111};
                        end
                        
                        3'b110: begin
                            // C.BEQZ -> beq rs1', x0, imm
                            valid = 1'b1;
                            instr = {imm_b_sext[12], imm_b_sext[10:5], 5'd0, rs1_prime, 
                                    3'b000, imm_b_sext[4:1], imm_b_sext[11], 7'b1100011};
                        end
                        
                        3'b111: begin
                            // C.BNEZ -> bne rs1', x0, imm
                            valid = 1'b1;
                            instr = {imm_b_sext[12], imm_b_sext[10:5], 5'd0, rs1_prime, 
                                    3'b001, imm_b_sext[4:1], imm_b_sext[11], 7'b1100011};
                        end
                    endcase
                end
                
                // =============================================================
                // Quadrant 2 (C2)
                // =============================================================
                2'b10: begin
                    case (funct3)
                        3'b000: begin
                            // C.SLLI -> slli rd, rd, shamt
                            if (rd_rs1 != 0) begin
                                valid = 1'b1;
                                instr = {6'b000000, cinstr[12], cinstr[6:2], rd_rs1, 3'b001, rd_rs1, 7'b0010011};
                            end
                        end
                        
                        3'b001: begin
                            // C.FLDSP -> fld rd, imm(x2)
                            valid = 1'b1;
                            instr = {imm_ldsp_zext, 5'd2, 3'b011, rd_rs1, 7'b0000111};
                        end
                        
                        3'b010: begin
                            // C.LWSP -> lw rd, imm(x2)
                            if (rd_rs1 != 0) begin
                                valid = 1'b1;
                                instr = {imm_lwsp_zext, 5'd2, 3'b010, rd_rs1, 7'b0000011};
                            end
                        end
                        
                        3'b011: begin
                            // C.LDSP -> ld rd, imm(x2) [RV64]
                            if (rd_rs1 != 0) begin
                                valid = 1'b1;
                                instr = {imm_ldsp_zext, 5'd2, 3'b011, rd_rs1, 7'b0000011};
                            end
                        end
                        
                        3'b100: begin
                            if (cinstr[12] == 1'b0) begin
                                if (rs2 == 0) begin
                                    // C.JR -> jalr x0, rs1, 0
                                    if (rd_rs1 != 0) begin
                                        valid = 1'b1;
                                        instr = {12'b0, rd_rs1, 3'b000, 5'd0, 7'b1100111};
                                    end
                                end
                                else begin
                                    // C.MV -> add rd, x0, rs2
                                    valid = 1'b1;
                                    instr = {7'b0, rs2, 5'd0, 3'b000, rd_rs1, 7'b0110011};
                                end
                            end
                            else begin
                                if (rs2 == 0 && rd_rs1 == 0) begin
                                    // C.EBREAK -> ebreak
                                    valid = 1'b1;
                                    instr = 32'h0010_0073;
                                end
                                else if (rs2 == 0) begin
                                    // C.JALR -> jalr x1, rs1, 0
                                    valid = 1'b1;
                                    instr = {12'b0, rd_rs1, 3'b000, 5'd1, 7'b1100111};
                                end
                                else begin
                                    // C.ADD -> add rd, rd, rs2
                                    valid = 1'b1;
                                    instr = {7'b0, rs2, rd_rs1, 3'b000, rd_rs1, 7'b0110011};
                                end
                            end
                        end
                        
                        3'b101: begin
                            // C.FSDSP -> fsd rs2, imm(x2)
                            valid = 1'b1;
                            instr = {imm_sdsp_zext[11:5], rs2, 5'd2, 3'b011, imm_sdsp_zext[4:0], 7'b0100111};
                        end
                        
                        3'b110: begin
                            // C.SWSP -> sw rs2, imm(x2)
                            valid = 1'b1;
                            instr = {imm_swsp_zext[11:5], rs2, 5'd2, 3'b010, imm_swsp_zext[4:0], 7'b0100011};
                        end
                        
                        3'b111: begin
                            // C.SDSP -> sd rs2, imm(x2) [RV64]
                            valid = 1'b1;
                            instr = {imm_sdsp_zext[11:5], rs2, 5'd2, 3'b011, imm_sdsp_zext[4:0], 7'b0100011};
                        end
                    endcase
                end
                
                default: valid = 1'b0;
            endcase
        end
    end

endmodule
