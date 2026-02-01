`timescale 1ns/1ps
`default_nettype none

/**
 * @module amo_unit
 * @brief Unidade de Operações Atômicas para RV64A
 *
 * @details Implementa todas as instruções atômicas:
 * - LR.W, LR.D (Load-Reserved)
 * - SC.W, SC.D (Store-Conditional)
 * - AMOSWAP.W/D, AMOADD.W/D, AMOAND.W/D, AMOOR.W/D, AMOXOR.W/D
 * - AMOMAX.W/D, AMOMAXU.W/D, AMOMIN.W/D, AMOMINU.W/D
 *
 * Interface com reservation_set para LR/SC.
 */
module amo_unit #(
    parameter int XLEN = 64,
    parameter int ADDR_WIDTH = 64
)(
    input  wire                     clk,
    input  wire                     rst_n,
    
    // Interface de Requisição
    input  wire                     req_valid,
    output logic                    req_ready,
    input  wire [4:0]               amo_op,      // funct5 (bits 31:27)
    input  wire                     is_word,     // 1 = .W, 0 = .D
    input  wire [ADDR_WIDTH-1:0]    addr,
    input  wire [XLEN-1:0]          rs2_data,    // Valor para operação
    input  wire                     aq,          // Acquire
    input  wire                     rl,          // Release
    
    // Interface de Resposta
    output logic                    resp_valid,
    output logic [XLEN-1:0]         result,      // Valor lido (antigo)
    output logic                    sc_fail,     // SC falhou (rd = 1)
    
    // Interface de Memória
    output logic                    mem_req,
    output logic                    mem_we,
    output logic [ADDR_WIDTH-1:0]   mem_addr,
    output logic [XLEN-1:0]         mem_wdata,
    output logic [7:0]              mem_wstrb,
    input  wire [XLEN-1:0]          mem_rdata,
    input  wire                     mem_ack,
    input  wire                     mem_error,
    
    // Interface com Reservation Set
    output logic                    lr_valid_out,
    output logic [ADDR_WIDTH-1:0]   lr_addr_out,
    output logic                    lr_is_word_out,
    output logic                    sc_valid_out,
    output logic [ADDR_WIDTH-1:0]   sc_addr_out,
    output logic                    sc_is_word_out,
    input  wire                     sc_success_in
);

    // AMO operations (funct5)
    localparam AMO_LR      = 5'b00010;
    localparam AMO_SC      = 5'b00011;
    localparam AMO_SWAP    = 5'b00001;
    localparam AMO_ADD     = 5'b00000;
    localparam AMO_XOR     = 5'b00100;
    localparam AMO_AND     = 5'b01100;
    localparam AMO_OR      = 5'b01000;
    localparam AMO_MIN     = 5'b10000;
    localparam AMO_MAX     = 5'b10100;
    localparam AMO_MINU    = 5'b11000;
    localparam AMO_MAXU    = 5'b11100;
    
    // FSM States
    typedef enum logic [2:0] {
        S_IDLE,
        S_LR_READ,
        S_SC_CHECK,
        S_SC_WRITE,
        S_AMO_READ,
        S_AMO_COMPUTE,
        S_AMO_WRITE,
        S_DONE
    } state_t;
    
    state_t state, next_state;
    
    // Registradores internos
    logic [4:0]              amo_op_reg;
    logic                    is_word_reg;
    logic [ADDR_WIDTH-1:0]   addr_reg;
    logic [XLEN-1:0]         rs2_reg;
    logic [XLEN-1:0]         loaded_value;
    logic [XLEN-1:0]         computed_value;
    logic                    sc_success_reg;
    
    // Sinais de operação
    logic is_lr, is_sc, is_amo_rmw;
    
    assign is_lr = (amo_op == AMO_LR);
    assign is_sc = (amo_op == AMO_SC);
    assign is_amo_rmw = !is_lr && !is_sc;
    
    // Cálculo do valor para AMO read-modify-write
    function automatic [XLEN-1:0] compute_amo(
        input [4:0] op,
        input [XLEN-1:0] mem_val,
        input [XLEN-1:0] reg_val,
        input is_word
    );
        logic [XLEN-1:0] result;
        logic signed [63:0] mem_signed, reg_signed;
        logic signed [31:0] mem_signed_w, reg_signed_w;
        
        if (is_word) begin
            mem_signed_w = mem_val[31:0];
            reg_signed_w = reg_val[31:0];
        end
        mem_signed = mem_val;
        reg_signed = reg_val;
        
        case (op)
            AMO_SWAP: result = reg_val;
            AMO_ADD:  result = mem_val + reg_val;
            AMO_XOR:  result = mem_val ^ reg_val;
            AMO_AND:  result = mem_val & reg_val;
            AMO_OR:   result = mem_val | reg_val;
            AMO_MIN: begin
                if (is_word)
                    result = (mem_signed_w < reg_signed_w) ? mem_val : reg_val;
                else
                    result = (mem_signed < reg_signed) ? mem_val : reg_val;
            end
            AMO_MAX: begin
                if (is_word)
                    result = (mem_signed_w > reg_signed_w) ? mem_val : reg_val;
                else
                    result = (mem_signed > reg_signed) ? mem_val : reg_val;
            end
            AMO_MINU: begin
                if (is_word)
                    result = (mem_val[31:0] < reg_val[31:0]) ? mem_val : reg_val;
                else
                    result = (mem_val < reg_val) ? mem_val : reg_val;
            end
            AMO_MAXU: begin
                if (is_word)
                    result = (mem_val[31:0] > reg_val[31:0]) ? mem_val : reg_val;
                else
                    result = (mem_val > reg_val) ? mem_val : reg_val;
            end
            default: result = reg_val;
        endcase
        
        return result;
    endfunction
    
    // FSM - Transição de Estados
    always_comb begin
        next_state = state;
        
        case (state)
            S_IDLE: begin
                if (req_valid && req_ready) begin
                    if (is_lr)
                        next_state = S_LR_READ;
                    else if (is_sc)
                        next_state = S_SC_CHECK;
                    else
                        next_state = S_AMO_READ;
                end
            end
            
            S_LR_READ: begin
                if (mem_ack || mem_error)
                    next_state = S_DONE;
            end
            
            S_SC_CHECK: begin
                // Verifica resultado do reservation set
                if (sc_success_in)
                    next_state = S_SC_WRITE;
                else
                    next_state = S_DONE; // SC falhou
            end
            
            S_SC_WRITE: begin
                if (mem_ack || mem_error)
                    next_state = S_DONE;
            end
            
            S_AMO_READ: begin
                if (mem_ack)
                    next_state = S_AMO_COMPUTE;
                else if (mem_error)
                    next_state = S_DONE;
            end
            
            S_AMO_COMPUTE: begin
                next_state = S_AMO_WRITE;
            end
            
            S_AMO_WRITE: begin
                if (mem_ack || mem_error)
                    next_state = S_DONE;
            end
            
            S_DONE: begin
                next_state = S_IDLE;
            end
            
            default: next_state = S_IDLE;
        endcase
    end
    
    // Saídas para Reservation Set
    always_comb begin
        lr_valid_out = 1'b0;
        lr_addr_out = addr_reg;
        lr_is_word_out = is_word_reg;
        sc_valid_out = 1'b0;
        sc_addr_out = addr_reg;
        sc_is_word_out = is_word_reg;
        
        if (state == S_LR_READ && mem_ack) begin
            lr_valid_out = 1'b1;
        end
        
        if (state == S_SC_CHECK) begin
            sc_valid_out = 1'b1;
        end
    end
    
    // Interface de Memória
    always_comb begin
        mem_req = 1'b0;
        mem_we = 1'b0;
        mem_addr = addr_reg;
        mem_wdata = '0;
        mem_wstrb = '0;
        
        case (state)
            S_LR_READ: begin
                mem_req = 1'b1;
                mem_we = 1'b0;
                mem_wstrb = is_word_reg ? 8'h0F : 8'hFF;
            end
            
            S_SC_WRITE: begin
                mem_req = 1'b1;
                mem_we = 1'b1;
                mem_wdata = rs2_reg;
                mem_wstrb = is_word_reg ? 8'h0F : 8'hFF;
            end
            
            S_AMO_READ: begin
                mem_req = 1'b1;
                mem_we = 1'b0;
                mem_wstrb = is_word_reg ? 8'h0F : 8'hFF;
            end
            
            S_AMO_WRITE: begin
                mem_req = 1'b1;
                mem_we = 1'b1;
                mem_wdata = computed_value;
                mem_wstrb = is_word_reg ? 8'h0F : 8'hFF;
            end
            
            default: ;
        endcase
    end
    
    // FSM - Lógica Sequencial
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            req_ready <= 1'b1;
            resp_valid <= 1'b0;
            result <= '0;
            sc_fail <= 1'b0;
            amo_op_reg <= '0;
            is_word_reg <= 1'b0;
            addr_reg <= '0;
            rs2_reg <= '0;
            loaded_value <= '0;
            computed_value <= '0;
            sc_success_reg <= 1'b0;
        end else begin
            state <= next_state;
            
            case (state)
                S_IDLE: begin
                    resp_valid <= 1'b0;
                    sc_fail <= 1'b0;
                    
                    if (req_valid && req_ready) begin
                        req_ready <= 1'b0;
                        amo_op_reg <= amo_op;
                        is_word_reg <= is_word;
                        addr_reg <= addr;
                        rs2_reg <= rs2_data;
                    end
                end
                
                S_LR_READ: begin
                    if (mem_ack) begin
                        // Sign-extend para word
                        if (is_word_reg)
                            loaded_value <= {{32{mem_rdata[31]}}, mem_rdata[31:0]};
                        else
                            loaded_value <= mem_rdata;
                    end
                end
                
                S_SC_CHECK: begin
                    sc_success_reg <= sc_success_in;
                    if (!sc_success_in) begin
                        // SC falhou, resultado = 1
                        result <= 64'd1;
                        sc_fail <= 1'b1;
                    end
                end
                
                S_SC_WRITE: begin
                    if (mem_ack) begin
                        // SC sucesso, resultado = 0
                        result <= 64'd0;
                        sc_fail <= 1'b0;
                    end else if (mem_error) begin
                        result <= 64'd1;
                        sc_fail <= 1'b1;
                    end
                end
                
                S_AMO_READ: begin
                    if (mem_ack) begin
                        // Sign-extend para word
                        if (is_word_reg)
                            loaded_value <= {{32{mem_rdata[31]}}, mem_rdata[31:0]};
                        else
                            loaded_value <= mem_rdata;
                    end
                end
                
                S_AMO_COMPUTE: begin
                    // Calcular novo valor
                    computed_value <= compute_amo(amo_op_reg, loaded_value, rs2_reg, is_word_reg);
                    // Resultado é o valor antigo (sign-extended se word)
                    result <= loaded_value;
                end
                
                S_AMO_WRITE: begin
                    // Resultado já foi definido em S_AMO_COMPUTE
                end
                
                S_DONE: begin
                    resp_valid <= 1'b1;
                    req_ready <= 1'b1;
                    
                    // Para LR, resultado é o valor lido
                    if (amo_op_reg == AMO_LR) begin
                        result <= loaded_value;
                    end
                end
                
                default: begin
                    req_ready <= 1'b1;
                end
            endcase
        end
    end

endmodule
