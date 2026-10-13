`timescale 1ns/1ps
`default_nettype none

/**
 * @module nebula_rvc_expander
 * @brief Expansão de instruções comprimidas RV64C para o formato de 32 bits.
 *
 * Puramente combinacional. 'illegal' sinaliza codificações reservadas /
 * ilegais em RV64C (HINTs são expandidos normalmente e são legais).
 */
module nebula_rvc_expander (
    input  wire [15:0] cinstr,
    output logic [31:0] instr,
    output logic        illegal
);

    // ---------------------------------------------------------------------
    // Montadores de formato
    // ---------------------------------------------------------------------
    function automatic logic [31:0] enc_i(input logic [11:0] imm, input logic [4:0] rs1,
                                          input logic [2:0] f3, input logic [4:0] rd,
                                          input logic [6:0] op);
        return {imm, rs1, f3, rd, op};
    endfunction

    function automatic logic [31:0] enc_s(input logic [11:0] imm, input logic [4:0] rs2,
                                          input logic [4:0] rs1, input logic [2:0] f3,
                                          input logic [6:0] op);
        return {imm[11:5], rs2, rs1, f3, imm[4:0], op};
    endfunction

    function automatic logic [31:0] enc_b(input logic [12:0] imm, input logic [4:0] rs2,
                                          input logic [4:0] rs1, input logic [2:0] f3);
        return {imm[12], imm[10:5], rs2, rs1, f3, imm[4:1], imm[11], 7'b1100011};
    endfunction

    function automatic logic [31:0] enc_j(input logic [20:0] imm, input logic [4:0] rd);
        return {imm[20], imm[10:1], imm[11], imm[19:12], rd, 7'b1101111};
    endfunction

    function automatic logic [31:0] enc_r(input logic [6:0] f7, input logic [4:0] rs2,
                                          input logic [4:0] rs1, input logic [2:0] f3,
                                          input logic [4:0] rd, input logic [6:0] op);
        return {f7, rs2, rs1, f3, rd, op};
    endfunction

    localparam logic [6:0] OP_LOAD   = 7'b0000011;
    localparam logic [6:0] OP_LOADFP = 7'b0000111;
    localparam logic [6:0] OP_STORE  = 7'b0100011;
    localparam logic [6:0] OP_STOREFP= 7'b0100111;
    localparam logic [6:0] OP_IMM    = 7'b0010011;
    localparam logic [6:0] OP_IMM32  = 7'b0011011;
    localparam logic [6:0] OP_OP     = 7'b0110011;
    localparam logic [6:0] OP_OP32   = 7'b0111011;
    localparam logic [6:0] OP_LUI    = 7'b0110111;
    localparam logic [6:0] OP_JALR   = 7'b1100111;

    logic [4:0] rd, rs2, rdp, rs1p, rs2p;
    logic [5:0] imm6;
    assign rd   = cinstr[11:7];
    assign rs2  = cinstr[6:2];
    assign rdp  = {2'b01, cinstr[4:2]};
    assign rs1p = {2'b01, cinstr[9:7]};
    assign rs2p = {2'b01, cinstr[4:2]};
    assign imm6 = {cinstr[12], cinstr[6:2]};

    logic [9:0]  addi4spn_imm;
    logic [7:0]  ld_imm;     // C.LD / C.FLD / C.SD / C.FSD
    logic [6:0]  lw_imm;     // C.LW / C.SW
    logic [9:0]  addi16sp_imm;
    logic [20:0] j_imm;
    logic [12:0] b_imm;
    logic [8:0]  ldsp_imm;
    logic [7:0]  lwsp_imm;
    logic [8:0]  sdsp_imm;
    logic [7:0]  swsp_imm;

    assign addi4spn_imm = {cinstr[10:7], cinstr[12:11], cinstr[5], cinstr[6], 2'b00};
    assign ld_imm       = {cinstr[6:5], cinstr[12:10], 3'b000};
    assign lw_imm       = {cinstr[5], cinstr[12:10], cinstr[6], 2'b00};
    assign addi16sp_imm = {cinstr[12], cinstr[4:3], cinstr[5], cinstr[2], cinstr[6], 4'b0000};
    assign j_imm        = {{10{cinstr[12]}}, cinstr[8], cinstr[10:9], cinstr[6], cinstr[7],
                           cinstr[2], cinstr[11], cinstr[5:3], 1'b0};
    assign b_imm        = {{5{cinstr[12]}}, cinstr[6:5], cinstr[2], cinstr[11:10], cinstr[4:3], 1'b0};
    assign ldsp_imm     = {cinstr[4:2], cinstr[12], cinstr[6:5], 3'b000};
    assign lwsp_imm     = {cinstr[3:2], cinstr[12], cinstr[6:4], 2'b00};
    assign sdsp_imm     = {cinstr[9:7], cinstr[12:10], 3'b000};
    assign swsp_imm     = {cinstr[8:7], cinstr[12:9], 2'b00};

    always_comb begin
        instr   = 32'h0000_0013; // nop
        illegal = 1'b0;

        case (cinstr[1:0])
            // =============================================================
            // Quadrante 0
            // =============================================================
            2'b00: begin
                case (cinstr[15:13])
                    3'b000: begin // C.ADDI4SPN
                        instr   = enc_i({2'b00, addi4spn_imm}, 5'd2, 3'b000, rdp, OP_IMM);
                        illegal = (addi4spn_imm == '0);
                    end
                    3'b001: instr = enc_i({4'b0, ld_imm}, rs1p, 3'b011, rdp, OP_LOADFP); // C.FLD
                    3'b010: instr = enc_i({5'b0, lw_imm}, rs1p, 3'b010, rdp, OP_LOAD);   // C.LW
                    3'b011: instr = enc_i({4'b0, ld_imm}, rs1p, 3'b011, rdp, OP_LOAD);   // C.LD
                    3'b100: illegal = 1'b1;                                              // reservado
                    3'b101: instr = enc_s({4'b0, ld_imm}, rs2p, rs1p, 3'b011, OP_STOREFP); // C.FSD
                    3'b110: instr = enc_s({5'b0, lw_imm}, rs2p, rs1p, 3'b010, OP_STORE);   // C.SW
                    3'b111: instr = enc_s({4'b0, ld_imm}, rs2p, rs1p, 3'b011, OP_STORE);   // C.SD
                endcase
            end

            // =============================================================
            // Quadrante 1
            // =============================================================
            2'b01: begin
                case (cinstr[15:13])
                    3'b000: instr = enc_i({{6{imm6[5]}}, imm6}, rd, 3'b000, rd, OP_IMM);   // C.ADDI
                    3'b001: begin                                                          // C.ADDIW
                        instr   = enc_i({{6{imm6[5]}}, imm6}, rd, 3'b000, rd, OP_IMM32);
                        illegal = (rd == 5'd0);
                    end
                    3'b010: instr = enc_i({{6{imm6[5]}}, imm6}, 5'd0, 3'b000, rd, OP_IMM); // C.LI
                    3'b011: begin
                        if (rd == 5'd2) begin                                              // C.ADDI16SP
                            instr   = enc_i({{2{addi16sp_imm[9]}}, addi16sp_imm}, 5'd2, 3'b000, 5'd2, OP_IMM);
                            illegal = (addi16sp_imm == '0);
                        end else begin                                                     // C.LUI
                            instr   = {{14{imm6[5]}}, imm6, rd, OP_LUI};
                            illegal = (imm6 == '0);
                        end
                    end
                    3'b100: begin
                        case (cinstr[11:10])
                            2'b00: instr = enc_i({6'b000000, imm6}, rs1p, 3'b101, rs1p, OP_IMM); // C.SRLI
                            2'b01: instr = enc_i({6'b010000, imm6}, rs1p, 3'b101, rs1p, OP_IMM); // C.SRAI
                            2'b10: instr = enc_i({{6{imm6[5]}}, imm6}, rs1p, 3'b111, rs1p, OP_IMM); // C.ANDI
                            2'b11: begin
                                if (!cinstr[12]) begin
                                    case (cinstr[6:5])
                                        2'b00: instr = enc_r(7'b0100000, rs2p, rs1p, 3'b000, rs1p, OP_OP); // C.SUB
                                        2'b01: instr = enc_r(7'b0000000, rs2p, rs1p, 3'b100, rs1p, OP_OP); // C.XOR
                                        2'b10: instr = enc_r(7'b0000000, rs2p, rs1p, 3'b110, rs1p, OP_OP); // C.OR
                                        2'b11: instr = enc_r(7'b0000000, rs2p, rs1p, 3'b111, rs1p, OP_OP); // C.AND
                                    endcase
                                end else begin
                                    case (cinstr[6:5])
                                        2'b00: instr = enc_r(7'b0100000, rs2p, rs1p, 3'b000, rs1p, OP_OP32); // C.SUBW
                                        2'b01: instr = enc_r(7'b0000000, rs2p, rs1p, 3'b000, rs1p, OP_OP32); // C.ADDW
                                        default: illegal = 1'b1;
                                    endcase
                                end
                            end
                        endcase
                    end
                    3'b101: instr = enc_j(j_imm, 5'd0);                 // C.J
                    3'b110: instr = enc_b(b_imm, 5'd0, rs1p, 3'b000);   // C.BEQZ
                    3'b111: instr = enc_b(b_imm, 5'd0, rs1p, 3'b001);   // C.BNEZ
                endcase
            end

            // =============================================================
            // Quadrante 2
            // =============================================================
            2'b10: begin
                case (cinstr[15:13])
                    3'b000: instr = enc_i({6'b000000, imm6}, rd, 3'b001, rd, OP_IMM);       // C.SLLI
                    3'b001: instr = enc_i({3'b0, ldsp_imm}, 5'd2, 3'b011, rd, OP_LOADFP);   // C.FLDSP
                    3'b010: begin                                                           // C.LWSP
                        instr   = enc_i({4'b0, lwsp_imm}, 5'd2, 3'b010, rd, OP_LOAD);
                        illegal = (rd == 5'd0);
                    end
                    3'b011: begin                                                           // C.LDSP
                        instr   = enc_i({3'b0, ldsp_imm}, 5'd2, 3'b011, rd, OP_LOAD);
                        illegal = (rd == 5'd0);
                    end
                    3'b100: begin
                        if (!cinstr[12]) begin
                            if (rs2 == 5'd0) begin                                          // C.JR
                                instr   = enc_i(12'd0, rd, 3'b000, 5'd0, OP_JALR);
                                illegal = (rd == 5'd0);
                            end else begin                                                  // C.MV
                                instr = enc_r(7'b0, rs2, 5'd0, 3'b000, rd, OP_OP);
                            end
                        end else begin
                            if (rs2 == 5'd0) begin
                                if (rd == 5'd0) instr = 32'h0010_0073;                      // C.EBREAK
                                else            instr = enc_i(12'd0, rd, 3'b000, 5'd1, OP_JALR); // C.JALR
                            end else begin                                                  // C.ADD
                                instr = enc_r(7'b0, rs2, rd, 3'b000, rd, OP_OP);
                            end
                        end
                    end
                    3'b101: instr = enc_s({3'b0, sdsp_imm}, rs2, 5'd2, 3'b011, OP_STOREFP); // C.FSDSP
                    3'b110: instr = enc_s({4'b0, swsp_imm}, rs2, 5'd2, 3'b010, OP_STORE);   // C.SWSP
                    3'b111: instr = enc_s({3'b0, sdsp_imm}, rs2, 5'd2, 3'b011, OP_STORE);   // C.SDSP
                endcase
            end

            default: begin
                instr   = '0;
                illegal = 1'b1;  // não é instrução comprimida
            end
        endcase
    end

endmodule
