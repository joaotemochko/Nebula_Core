`timescale 1ns/1ps
`default_nettype none

/**
 * @module mdu_rv64
 * @brief Unidade de Multiplicação e Divisão para RV64M
 *
 * CORREÇÕES APLICADAS:
 * 1. Declarações de variáveis locais removidas de dentro de always_ff
 *    (logic temp_remainder, q, r) — movidas para sinais de módulo.
 *    Ferramentas de síntese (Vivado/Quartus) rejeitam declarações
 *    dentro de blocos procedurais sequenciais.
 */
module mdu_rv64 #(
    parameter int XLEN = 64
)(
    input  wire                 clk,
    input  wire                 rst_n,

    // Interface de Requisição
    input  wire                 req_valid,
    output logic                req_ready,
    input  wire [XLEN-1:0]      rs1_data,
    input  wire [XLEN-1:0]      rs2_data,
    input  wire [2:0]           funct3,
    input  wire                 is_word_op,

    // Interface de Resposta
    output logic                resp_valid,
    output logic [XLEN-1:0]     result
);

    // Opcodes da extensão M
    localparam logic [2:0] F3_MUL    = 3'b000;
    localparam logic [2:0] F3_MULH   = 3'b001;
    localparam logic [2:0] F3_MULHSU = 3'b010;
    localparam logic [2:0] F3_MULHU  = 3'b011;
    localparam logic [2:0] F3_DIV    = 3'b100;
    localparam logic [2:0] F3_DIVU   = 3'b101;
    localparam logic [2:0] F3_REM    = 3'b110;
    localparam logic [2:0] F3_REMU   = 3'b111;

    typedef enum logic [2:0] {
        S_IDLE,
        S_MUL_STAGE1,
        S_MUL_STAGE2,
        S_MUL_DONE,
        S_DIV_COMPUTE,
        S_DIV_DONE
    } state_t;

    state_t state, next_state;

    // Registradores internos
    logic [XLEN-1:0]  op1_reg, op2_reg;
    logic [2:0]       funct3_reg;
    logic             is_word_reg;
    logic             op1_signed, op2_signed;
    logic             is_div_op;
    logic             result_negate;
    logic             remainder_negate;

    // Multiplicação
    logic signed [XLEN:0]       mul_op1_signed;
    logic signed [XLEN:0]       mul_op2_signed;
    logic signed [2*XLEN+1:0]   mul_result_full;
    logic [2*XLEN-1:0]          mul_result_reg;

    // Divisão iterativa
    logic [XLEN-1:0]  dividend_reg;
    logic [XLEN-1:0]  divisor_reg;
    logic [XLEN-1:0]  quotient_reg;
    logic [XLEN-1:0]  remainder_reg;
    logic [6:0]       div_counter;
    logic             div_by_zero;
    logic             div_overflow;

    wire [6:0] div_iterations = is_word_reg ? 7'd32 : 7'd64;

    wire [XLEN-1:0] op1_abs = (op1_signed && op1_reg[XLEN-1]) ? -op1_reg : op1_reg;
    wire [XLEN-1:0] op2_abs = (op2_signed && op2_reg[XLEN-1]) ? -op2_reg : op2_reg;

    wire [XLEN-1:0] op1_word = is_word_reg ?
        (op1_signed ? {{32{rs1_data[31]}}, rs1_data[31:0]} : {32'b0, rs1_data[31:0]}) :
        rs1_data;
    wire [XLEN-1:0] op2_word = is_word_reg ?
        (op2_signed ? {{32{rs2_data[31]}}, rs2_data[31:0]} : {32'b0, rs2_data[31:0]}) :
        rs2_data;

    // Sinais combinacionais para divisão — calculados em always_comb
    // para evitar BLKSEQ (blocking assignment em always_ff)
    logic [XLEN:0]   temp_remainder;
    logic [XLEN-1:0] div_q_final;
    logic [XLEN-1:0] div_r_final;

    // temp_remainder: próximo remainder durante iteração de divisão
    always_comb begin
        logic [5:0] bit_idx;
        bit_idx = (is_word_reg ? 6'd31 : 6'd63) - div_counter[5:0];
        
        temp_remainder = {remainder_reg[XLEN-2:0], dividend_reg[bit_idx]};
    end

    // div_q_final / div_r_final: quociente e resto com sinal aplicado
    always_comb begin
        div_q_final = result_negate    ? -quotient_reg  : quotient_reg;
        div_r_final = remainder_negate ? -remainder_reg : remainder_reg;
    end

    // Lógica de controle de sinais
    always_comb begin
        op1_signed = 1'b0;
        op2_signed = 1'b0;
        is_div_op  = 1'b0;

        case (funct3)
            F3_MUL:    begin op1_signed = 1'b1; op2_signed = 1'b1; end
            F3_MULH:   begin op1_signed = 1'b1; op2_signed = 1'b1; end
            F3_MULHSU: begin op1_signed = 1'b1; op2_signed = 1'b0; end
            F3_MULHU:  begin op1_signed = 1'b0; op2_signed = 1'b0; end
            F3_DIV:    begin op1_signed = 1'b1; op2_signed = 1'b1; is_div_op = 1'b1; end
            F3_DIVU:   begin op1_signed = 1'b0; op2_signed = 1'b0; is_div_op = 1'b1; end
            F3_REM:    begin op1_signed = 1'b1; op2_signed = 1'b1; is_div_op = 1'b1; end
            F3_REMU:   begin op1_signed = 1'b0; op2_signed = 1'b0; is_div_op = 1'b1; end
            default: ;
        endcase
    end

    // FSM - Transição de Estados
    always_comb begin
        next_state = state;

        case (state)
            S_IDLE: begin
                if (req_valid && req_ready) begin
                    if (is_div_op)
                        next_state = S_DIV_COMPUTE;
                    else
                        next_state = S_MUL_STAGE1;
                end
            end
            S_MUL_STAGE1:  next_state = S_MUL_STAGE2;
            S_MUL_STAGE2:  next_state = S_MUL_DONE;
            S_MUL_DONE:    next_state = S_IDLE;
            S_DIV_COMPUTE: begin
                if (div_counter >= div_iterations || div_by_zero)
                    next_state = S_DIV_DONE;
            end
            S_DIV_DONE:    next_state = S_IDLE;
            default:       next_state = S_IDLE;
        endcase
    end

    // FSM - Lógica Sequencial
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state            <= S_IDLE;
            req_ready        <= 1'b1;
            resp_valid       <= 1'b0;
            result           <= '0;
            op1_reg          <= '0;
            op2_reg          <= '0;
            funct3_reg       <= '0;
            is_word_reg      <= 1'b0;
            mul_result_reg   <= '0;
            dividend_reg     <= '0;
            divisor_reg      <= '0;
            quotient_reg     <= '0;
            remainder_reg    <= '0;
            div_counter      <= '0;
            div_by_zero      <= 1'b0;
            div_overflow     <= 1'b0;
            result_negate    <= 1'b0;
            remainder_negate <= 1'b0;
        end else begin
            state <= next_state;

            case (state)
                S_IDLE: begin
                    resp_valid <= 1'b0;

                    if (req_valid && req_ready) begin
                        req_ready  <= 1'b0;
                        funct3_reg <= funct3;
                        is_word_reg <= is_word_op;
                        op1_reg    <= op1_word;
                        op2_reg    <= op2_word;

                        if (is_div_op) begin
                            div_counter  <= '0;
                            div_by_zero  <= (op2_word == '0);

                            if (op1_signed && op2_signed) begin
                                if (is_word_op)
                                    div_overflow <= (rs1_data[31:0] == 32'h8000_0000) &&
                                                    (rs2_data[31:0] == 32'hFFFF_FFFF);
                                else
                                    div_overflow <= (rs1_data == 64'h8000_0000_0000_0000) &&
                                                    (rs2_data == 64'hFFFF_FFFF_FFFF_FFFF);
                            end else begin
                                div_overflow <= 1'b0;
                            end

                            if (funct3 == F3_DIV) begin
                                result_negate    <= (op1_word[XLEN-1] ^ op2_word[XLEN-1]) &&
                                                    op1_signed && (op2_word != '0);
                                remainder_negate <= op1_word[XLEN-1] && op1_signed;
                            end else if (funct3 == F3_REM) begin
                                result_negate    <= op1_word[XLEN-1] && op1_signed;
                                remainder_negate <= op1_word[XLEN-1] && op1_signed;
                            end else begin
                                result_negate    <= 1'b0;
                                remainder_negate <= 1'b0;
                            end

                            dividend_reg  <= (op1_signed && op1_word[XLEN-1]) ? -op1_word : op1_word;
                            divisor_reg   <= (op2_signed && op2_word[XLEN-1]) ? -op2_word : op2_word;
                            quotient_reg  <= '0;
                            remainder_reg <= '0;
                        end
                    end
                end

                S_MUL_STAGE1: begin
                    mul_op1_signed <= op1_signed ? $signed({op1_reg[XLEN-1], op1_reg}) :
                                                   $signed({1'b0,            op1_reg});
                    mul_op2_signed <= op2_signed ? $signed({op2_reg[XLEN-1], op2_reg}) :
                                                   $signed({1'b0,            op2_reg});
                end

                S_MUL_STAGE2: begin
                    mul_result_full <= mul_op1_signed * mul_op2_signed;
                end

                S_MUL_DONE: begin
                    resp_valid <= 1'b1;
                    req_ready  <= 1'b1;

                    case (funct3_reg)
                        F3_MUL: begin
                            if (is_word_reg)
                                result <= {{32{mul_result_full[31]}}, mul_result_full[31:0]};
                            else
                                result <= mul_result_full[XLEN-1:0];
                        end
                        F3_MULH, F3_MULHSU, F3_MULHU:
                            result <= mul_result_full[2*XLEN-1:XLEN];
                        default:
                            result <= mul_result_full[XLEN-1:0];
                    endcase
                end

                S_DIV_COMPUTE: begin
                    if (!div_by_zero && !div_overflow) begin
                        // temp_remainder calculado em always_comb acima
                        if (temp_remainder >= {1'b0, divisor_reg}) begin
                            remainder_reg <= temp_remainder[XLEN-1:0] - divisor_reg;
                            quotient_reg  <= {quotient_reg[XLEN-2:0], 1'b1};
                        end else begin
                            remainder_reg <= temp_remainder[XLEN-1:0];
                            quotient_reg  <= {quotient_reg[XLEN-2:0], 1'b0};
                        end

                        div_counter <= div_counter + 1;
                    end
                end

                S_DIV_DONE: begin
                    resp_valid <= 1'b1;
                    req_ready  <= 1'b1;

                    if (div_by_zero) begin
                        case (funct3_reg)
                            F3_DIV, F3_DIVU: result <= '1;
                            F3_REM, F3_REMU: begin
                                if (is_word_reg)
                                    result <= {{32{op1_reg[31]}}, op1_reg[31:0]};
                                else
                                    result <= op1_reg;
                            end
                            default: result <= '0;
                        endcase
                    end else if (div_overflow) begin
                        case (funct3_reg)
                            F3_DIV: begin
                                if (is_word_reg)
                                    result <= {{32{1'b1}}, 32'h8000_0000};
                                else
                                    result <= 64'h8000_0000_0000_0000;
                            end
                            F3_REM: result <= '0;
                            default: result <= '0;
                        endcase
                    end else begin
                        // div_q_final e div_r_final calculados em always_comb acima
                        case (funct3_reg)
                            F3_DIV, F3_DIVU: begin
                                if (is_word_reg)
                                    result <= {{32{div_q_final[31]}}, div_q_final[31:0]};
                                else
                                    result <= div_q_final;
                            end
                            F3_REM, F3_REMU: begin
                                if (is_word_reg)
                                    result <= {{32{div_r_final[31]}}, div_r_final[31:0]};
                                else
                                    result <= div_r_final;
                            end
                            default: result <= '0;
                        endcase
                    end
                end

                default: begin
                    req_ready  <= 1'b1;
                    resp_valid <= 1'b0;
                end
            endcase
        end
    end

endmodule
