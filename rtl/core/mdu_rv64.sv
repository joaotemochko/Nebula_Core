`timescale 1ns/1ps
`default_nettype none

/**
 * @module mdu_rv64
 * @brief Unidade de Multiplicação e Divisão para RV64M
 *
 * @details Implementa todas as instruções da extensão M:
 * - MUL, MULH, MULHSU, MULHU (64-bit)
 * - DIV, DIVU, REM, REMU (64-bit)
 * - MULW, DIVW, DIVUW, REMW, REMUW (32-bit com sign-extension)
 *
 * Pipeline: Multiplicação = 3 ciclos, Divisão = 34 ciclos (iterativa)
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
    input  wire [2:0]           funct3,     // Determina operação
    input  wire                 is_word_op, // 1 = operação de 32-bit (W suffix)
    
    // Interface de Resposta
    output logic                resp_valid,
    output logic [XLEN-1:0]     result
);

    // Opcodes da extensão M (funct3)
    localparam logic [2:0] F3_MUL    = 3'b000;
    localparam logic [2:0] F3_MULH   = 3'b001;
    localparam logic [2:0] F3_MULHSU = 3'b010;
    localparam logic [2:0] F3_MULHU  = 3'b011;
    localparam logic [2:0] F3_DIV    = 3'b100;
    localparam logic [2:0] F3_DIVU   = 3'b101;
    localparam logic [2:0] F3_REM    = 3'b110;
    localparam logic [2:0] F3_REMU   = 3'b111;

    // FSM States
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
    logic [XLEN-1:0] op1_reg, op2_reg;
    logic [2:0]      funct3_reg;
    logic            is_word_reg;
    logic            op1_signed, op2_signed;
    logic            is_div_op;
    logic            result_negate;
    logic            remainder_negate;
    
    // Multiplicação (usa DSP do FPGA se disponível)
    logic signed [XLEN:0]   mul_op1_signed;
    logic signed [XLEN:0]   mul_op2_signed;
    logic signed [2*XLEN+1:0] mul_result_full;
    logic [2*XLEN-1:0]      mul_result_reg;
    
    // Divisão (algoritmo iterativo de restauração)
    logic [XLEN-1:0]   dividend_reg;
    logic [XLEN-1:0]   divisor_reg;
    logic [XLEN-1:0]   quotient_reg;
    logic [XLEN-1:0]   remainder_reg;
    logic [6:0]        div_counter; // Conta até 64 (ou 32 para W)
    logic              div_by_zero;
    logic              div_overflow;
    
    // Número de iterações baseado no tamanho da operação
    wire [6:0] div_iterations = is_word_reg ? 7'd32 : 7'd64;
    
    // Preparação dos operandos
    wire [XLEN-1:0] op1_abs = (op1_signed && op1_reg[XLEN-1]) ? -op1_reg : op1_reg;
    wire [XLEN-1:0] op2_abs = (op2_signed && op2_reg[XLEN-1]) ? -op2_reg : op2_reg;
    
    // Para operações de 32-bit, fazer sign/zero extension
    wire [XLEN-1:0] op1_word = is_word_reg ? 
        (op1_signed ? {{32{rs1_data[31]}}, rs1_data[31:0]} : {32'b0, rs1_data[31:0]}) : 
        rs1_data;
    wire [XLEN-1:0] op2_word = is_word_reg ? 
        (op2_signed ? {{32{rs2_data[31]}}, rs2_data[31:0]} : {32'b0, rs2_data[31:0]}) : 
        rs2_data;

    // Lógica de controle de sinais
    always_comb begin
        op1_signed = 1'b0;
        op2_signed = 1'b0;
        is_div_op = 1'b0;
        
        case (funct3)
            F3_MUL:    begin op1_signed = 1'b1; op2_signed = 1'b1; end
            F3_MULH:   begin op1_signed = 1'b1; op2_signed = 1'b1; end
            F3_MULHSU: begin op1_signed = 1'b1; op2_signed = 1'b0; end
            F3_MULHU:  begin op1_signed = 1'b0; op2_signed = 1'b0; end
            F3_DIV:    begin op1_signed = 1'b1; op2_signed = 1'b1; is_div_op = 1'b1; end
            F3_DIVU:   begin op1_signed = 1'b0; op2_signed = 1'b0; is_div_op = 1'b1; end
            F3_REM:    begin op1_signed = 1'b1; op2_signed = 1'b1; is_div_op = 1'b1; end
            F3_REMU:   begin op1_signed = 1'b0; op2_signed = 1'b0; is_div_op = 1'b1; end
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
            
            S_MUL_STAGE1: next_state = S_MUL_STAGE2;
            S_MUL_STAGE2: next_state = S_MUL_DONE;
            S_MUL_DONE:   next_state = S_IDLE;
            
            S_DIV_COMPUTE: begin
                if (div_counter >= div_iterations || div_by_zero)
                    next_state = S_DIV_DONE;
            end
            
            S_DIV_DONE: next_state = S_IDLE;
            
            default: next_state = S_IDLE;
        endcase
    end

    // FSM - Lógica Sequencial
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            req_ready <= 1'b1;
            resp_valid <= 1'b0;
            result <= '0;
            op1_reg <= '0;
            op2_reg <= '0;
            funct3_reg <= '0;
            is_word_reg <= 1'b0;
            mul_result_reg <= '0;
            dividend_reg <= '0;
            divisor_reg <= '0;
            quotient_reg <= '0;
            remainder_reg <= '0;
            div_counter <= '0;
            div_by_zero <= 1'b0;
            div_overflow <= 1'b0;
            result_negate <= 1'b0;
            remainder_negate <= 1'b0;
        end else begin
            state <= next_state;
            
            case (state)
                S_IDLE: begin
                    resp_valid <= 1'b0;
                    
                    if (req_valid && req_ready) begin
                        req_ready <= 1'b0;
                        funct3_reg <= funct3;
                        is_word_reg <= is_word_op;
                        
                        // Preparar operandos
                        op1_reg <= op1_word;
                        op2_reg <= op2_word;
                        
                        if (is_div_op) begin
                            // Inicializar divisão
                            div_counter <= '0;
                            div_by_zero <= (op2_word == '0);
                            
                            // Detectar overflow: MIN_INT / -1
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
                            
                            // Calcular se resultado deve ser negado
                            if (funct3 == F3_DIV) begin
                                result_negate <= (op1_word[XLEN-1] ^ op2_word[XLEN-1]) && 
                                                op1_signed && (op2_word != '0);
                                remainder_negate <= op1_word[XLEN-1] && op1_signed;
                            end else if (funct3 == F3_REM) begin
                                result_negate <= op1_word[XLEN-1] && op1_signed;
                                remainder_negate <= op1_word[XLEN-1] && op1_signed;
                            end else begin
                                result_negate <= 1'b0;
                                remainder_negate <= 1'b0;
                            end
                            
                            // Usar valores absolutos para divisão signed
                            dividend_reg <= (op1_signed && op1_word[XLEN-1]) ? -op1_word : op1_word;
                            divisor_reg <= (op2_signed && op2_word[XLEN-1]) ? -op2_word : op2_word;
                            quotient_reg <= '0;
                            remainder_reg <= '0;
                        end
                    end
                end
                
                // Pipeline de Multiplicação (3 estágios)
                S_MUL_STAGE1: begin
                    // Estágio 1: Preparar operandos signed/unsigned
                    mul_op1_signed <= op1_signed ? $signed({op1_reg[XLEN-1], op1_reg}) : 
                                                   $signed({1'b0, op1_reg});
                    mul_op2_signed <= op2_signed ? $signed({op2_reg[XLEN-1], op2_reg}) : 
                                                   $signed({1'b0, op2_reg});
                end
                
                S_MUL_STAGE2: begin
                    // Estágio 2: Multiplicação (sintetizador infere DSP)
                    mul_result_full <= mul_op1_signed * mul_op2_signed;
                end
                
                S_MUL_DONE: begin
                    // Estágio 3: Selecionar resultado
                    resp_valid <= 1'b1;
                    req_ready <= 1'b1;
                    
                    case (funct3_reg)
                        F3_MUL: begin
                            // Retorna bits baixos
                            if (is_word_reg)
                                result <= {{32{mul_result_full[31]}}, mul_result_full[31:0]};
                            else
                                result <= mul_result_full[XLEN-1:0];
                        end
                        F3_MULH, F3_MULHSU, F3_MULHU: begin
                            // Retorna bits altos
                            result <= mul_result_full[2*XLEN-1:XLEN];
                        end
                        default: result <= mul_result_full[XLEN-1:0];
                    endcase
                end
                
                // Divisão Iterativa (algoritmo de restauração)
                S_DIV_COMPUTE: begin
                    if (!div_by_zero && !div_overflow) begin
                        // Algoritmo de divisão por restauração
                        logic [XLEN:0] temp_remainder;
                        
                        // Shift left e trazer próximo bit do dividendo
                        temp_remainder = {remainder_reg[XLEN-2:0], dividend_reg[XLEN-1-div_counter]};
                        
                        if (temp_remainder >= {1'b0, divisor_reg}) begin
                            remainder_reg <= temp_remainder - {1'b0, divisor_reg};
                            quotient_reg <= {quotient_reg[XLEN-2:0], 1'b1};
                        end else begin
                            remainder_reg <= temp_remainder[XLEN-1:0];
                            quotient_reg <= {quotient_reg[XLEN-2:0], 1'b0};
                        end
                        
                        div_counter <= div_counter + 1;
                    end
                end
                
                S_DIV_DONE: begin
                    resp_valid <= 1'b1;
                    req_ready <= 1'b1;
                    
                    // Casos especiais
                    if (div_by_zero) begin
                        case (funct3_reg)
                            F3_DIV, F3_DIVU: result <= '1; // -1 (all ones)
                            F3_REM, F3_REMU: result <= op1_reg; // Dividendo original
                            default: result <= '0;
                        endcase
                    end else if (div_overflow) begin
                        case (funct3_reg)
                            F3_DIV: begin
                                // MIN_INT / -1 = MIN_INT (overflow)
                                if (is_word_reg)
                                    result <= {{32{1'b1}}, 32'h8000_0000};
                                else
                                    result <= 64'h8000_0000_0000_0000;
                            end
                            F3_REM: result <= '0; // Remainder = 0
                            default: result <= '0;
                        endcase
                    end else begin
                        // Resultado normal
                        case (funct3_reg)
                            F3_DIV, F3_DIVU: begin
                                logic [XLEN-1:0] q;
                                q = result_negate ? -quotient_reg : quotient_reg;
                                if (is_word_reg)
                                    result <= {{32{q[31]}}, q[31:0]};
                                else
                                    result <= q;
                            end
                            F3_REM, F3_REMU: begin
                                logic [XLEN-1:0] r;
                                r = remainder_negate ? -remainder_reg : remainder_reg;
                                if (is_word_reg)
                                    result <= {{32{r[31]}}, r[31:0]};
                                else
                                    result <= r;
                            end
                            default: result <= '0;
                        endcase
                    end
                end
                
                default: begin
                    req_ready <= 1'b1;
                    resp_valid <= 1'b0;
                end
            endcase
        end
    end

endmodule
