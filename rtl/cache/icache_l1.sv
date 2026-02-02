`timescale 1ns/1ps
`default_nettype none

import nebula_pkg::*;

/**
 * @module icache_l1
 * @brief Cache de Instruções L1 - 32KB, 4-way set associative
 *
 * @details
 * - Tamanho: 32KB
 * - Associatividade: 4-way
 * - Linha: 64 bytes (512 bits)
 * - Sets: 128
 * - Política de substituição: Pseudo-LRU
 * - Read-only (instruções não são escritas pelo core)
 *
 * Estrutura de endereço (56 bits físico):
 * [55:13] Tag (43 bits)
 * [12:6]  Index (7 bits) -> 128 sets
 * [5:0]   Offset (6 bits) -> 64 bytes
 */
module icache_l1 #(
    parameter int PADDR_WIDTH = 56,
    parameter int LINE_SIZE = 64,           // bytes
    parameter int NUM_WAYS = 4,
    parameter int NUM_SETS = 128
)(
    input  wire                     clk,
    input  wire                     rst_n,
    
    // =========================================================================
    // Interface com Frontend (Fetch)
    // =========================================================================
    input  wire                     req_valid,
    input  wire [PADDR_WIDTH-1:0]   req_addr,
    output logic                    req_ready,
    
    output logic                    resp_valid,
    output logic [63:0]             resp_data,      // 2 instruções (64 bits)
    output logic                    resp_error,
    
    // =========================================================================
    // Interface de Invalidação
    // =========================================================================
    input  wire                     invalidate_all,
    input  wire                     invalidate_addr_valid,
    input  wire [PADDR_WIDTH-1:0]   invalidate_addr,
    
    // =========================================================================
    // Interface com Memória/L2 (Refill)
    // =========================================================================
    output logic                    mem_req,
    output logic [PADDR_WIDTH-1:0]  mem_addr,
    input  wire                     mem_ack,
    input  wire [LINE_SIZE*8-1:0]   mem_data,       // 512 bits = 64 bytes
    input  wire                     mem_error
);

    // =========================================================================
    // Parâmetros Derivados
    // =========================================================================
    
    localparam int OFFSET_BITS = $clog2(LINE_SIZE);     // 6
    localparam int INDEX_BITS = $clog2(NUM_SETS);       // 7
    localparam int TAG_BITS = PADDR_WIDTH - INDEX_BITS - OFFSET_BITS; // 43
    localparam int LINE_BITS = LINE_SIZE * 8;           // 512
    
    // =========================================================================
    // Estruturas de Cache
    // =========================================================================
    
    typedef struct packed {
        logic                   valid;
        logic [TAG_BITS-1:0]    tag;
    } cache_tag_t;
    
    // Storage
    cache_tag_t     tag_array [NUM_SETS][NUM_WAYS];
    logic [LINE_BITS-1:0] data_array [NUM_SETS][NUM_WAYS];
    
    // Pseudo-LRU: 3 bits por set para 4 ways
    logic [2:0]     plru_bits [NUM_SETS];
    
    // =========================================================================
    // Extração de Campos do Endereço
    // =========================================================================
    
    wire [OFFSET_BITS-1:0]  req_offset = req_addr[OFFSET_BITS-1:0];
    wire [INDEX_BITS-1:0]   req_index  = req_addr[INDEX_BITS+OFFSET_BITS-1:OFFSET_BITS];
    wire [TAG_BITS-1:0]     req_tag    = req_addr[PADDR_WIDTH-1:INDEX_BITS+OFFSET_BITS];
    
    // =========================================================================
    // FSM States
    // =========================================================================
    
    typedef enum logic [2:0] {
        S_IDLE,
        S_TAG_CHECK,
        S_REFILL_REQ,
        S_REFILL_WAIT,
        S_REFILL_DONE,
        S_ERROR
    } state_t;
    
    state_t state, next_state;
    
    // =========================================================================
    // Registradores Internos
    // =========================================================================
    
    logic [PADDR_WIDTH-1:0] addr_reg;
    logic [INDEX_BITS-1:0]  index_reg;
    logic [TAG_BITS-1:0]    tag_reg;
    logic [OFFSET_BITS-1:0] offset_reg;
    logic [LINE_BITS-1:0]   refill_data_reg;
    logic [$clog2(NUM_WAYS)-1:0] victim_way;
    
    // =========================================================================
    // Tag Comparison (Combinatorial)
    // =========================================================================
    
    logic [NUM_WAYS-1:0] hit_way;
    logic cache_hit;
    logic [$clog2(NUM_WAYS)-1:0] hit_way_idx;
    
    always_comb begin
        hit_way = '0;
        cache_hit = 1'b0;
        hit_way_idx = '0;
        
        for (int w = 0; w < NUM_WAYS; w++) begin
            if (tag_array[index_reg][w].valid && 
                tag_array[index_reg][w].tag == tag_reg) begin
                hit_way[w] = 1'b1;
                cache_hit = 1'b1;
                hit_way_idx = w[$clog2(NUM_WAYS)-1:0];
            end
        end
    end
    
    // =========================================================================
    // Data Selection
    // =========================================================================
    
    logic [LINE_BITS-1:0] hit_line;
    logic [63:0] selected_data;
    
    always_comb begin
        hit_line = '0;
        
        for (int w = 0; w < NUM_WAYS; w++) begin
            if (hit_way[w])
                hit_line = data_array[index_reg][w];
        end
        
        // Selecionar 64 bits (2 instruções) baseado no offset
        // Offset[5:3] seleciona qual grupo de 64 bits
        case (offset_reg[5:3])
            3'd0: selected_data = hit_line[63:0];
            3'd1: selected_data = hit_line[127:64];
            3'd2: selected_data = hit_line[191:128];
            3'd3: selected_data = hit_line[255:192];
            3'd4: selected_data = hit_line[319:256];
            3'd5: selected_data = hit_line[383:320];
            3'd6: selected_data = hit_line[447:384];
            3'd7: selected_data = hit_line[511:448];
        endcase
    end
    
    // =========================================================================
    // Pseudo-LRU Victim Selection
    // =========================================================================
    
    function automatic logic [$clog2(NUM_WAYS)-1:0] get_plru_victim(
        input logic [2:0] plru
    );
        // Tree-based PLRU for 4 ways
        // Bit 0: left(0,1) vs right(2,3)
        // Bit 1: way 0 vs way 1
        // Bit 2: way 2 vs way 3
        if (!plru[0]) begin
            return plru[1] ? 2'd0 : 2'd1;
        end else begin
            return plru[2] ? 2'd2 : 2'd3;
        end
    endfunction
    
    function automatic logic [2:0] update_plru(
        input logic [2:0] plru,
        input logic [$clog2(NUM_WAYS)-1:0] accessed_way
    );
        logic [2:0] new_plru;
        new_plru = plru;
        
        case (accessed_way)
            2'd0: begin new_plru[0] = 1'b1; new_plru[1] = 1'b1; end
            2'd1: begin new_plru[0] = 1'b1; new_plru[1] = 1'b0; end
            2'd2: begin new_plru[0] = 1'b0; new_plru[2] = 1'b1; end
            2'd3: begin new_plru[0] = 1'b0; new_plru[2] = 1'b0; end
        endcase
        
        return new_plru;
    endfunction
    
    // =========================================================================
    // FSM - Próximo Estado
    // =========================================================================
    
    always_comb begin
        next_state = state;
        
        case (state)
            S_IDLE: begin
                if (req_valid && req_ready)
                    next_state = S_TAG_CHECK;
            end
            
            S_TAG_CHECK: begin
                if (cache_hit)
                    next_state = S_IDLE;
                else
                    next_state = S_REFILL_REQ;
            end
            
            S_REFILL_REQ: begin
                next_state = S_REFILL_WAIT;
            end
            
            S_REFILL_WAIT: begin
                if (mem_ack)
                    next_state = S_REFILL_DONE;
                else if (mem_error)
                    next_state = S_ERROR;
            end
            
            S_REFILL_DONE: begin
                next_state = S_IDLE;
            end
            
            S_ERROR: begin
                next_state = S_IDLE;
            end
            
            default: next_state = S_IDLE;
        endcase
    end
    
    // =========================================================================
    // Saídas
    // =========================================================================
    
    assign req_ready = (state == S_IDLE) && !invalidate_all;
    assign mem_req = (state == S_REFILL_REQ);
    assign mem_addr = {addr_reg[PADDR_WIDTH-1:OFFSET_BITS], {OFFSET_BITS{1'b0}}}; // Alinhar à linha
    
    always_comb begin
        resp_valid = 1'b0;
        resp_data = '0;
        resp_error = 1'b0;
        
        if (state == S_TAG_CHECK && cache_hit) begin
            resp_valid = 1'b1;
            resp_data = selected_data;
        end
        else if (state == S_REFILL_DONE) begin
            resp_valid = 1'b1;
            // Selecionar do dado recém carregado
            case (offset_reg[5:3])
                3'd0: resp_data = refill_data_reg[63:0];
                3'd1: resp_data = refill_data_reg[127:64];
                3'd2: resp_data = refill_data_reg[191:128];
                3'd3: resp_data = refill_data_reg[255:192];
                3'd4: resp_data = refill_data_reg[319:256];
                3'd5: resp_data = refill_data_reg[383:320];
                3'd6: resp_data = refill_data_reg[447:384];
                3'd7: resp_data = refill_data_reg[511:448];
            endcase
        end
        else if (state == S_ERROR) begin
            resp_valid = 1'b1;
            resp_error = 1'b1;
        end
    end
    
    // =========================================================================
    // Lógica Sequencial
    // =========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            addr_reg <= '0;
            index_reg <= '0;
            tag_reg <= '0;
            offset_reg <= '0;
            refill_data_reg <= '0;
            victim_way <= '0;
            
            // Invalidar todas as tags
            for (int s = 0; s < NUM_SETS; s++) begin
                for (int w = 0; w < NUM_WAYS; w++) begin
                    tag_array[s][w].valid <= 1'b0;
                    tag_array[s][w].tag <= '0;
                end
                plru_bits[s] <= '0;
            end
        end
        else begin
            state <= next_state;
            
            // Invalidação global (FENCE.I)
            if (invalidate_all) begin
                for (int s = 0; s < NUM_SETS; s++) begin
                    for (int w = 0; w < NUM_WAYS; w++) begin
                        tag_array[s][w].valid <= 1'b0;
                    end
                end
            end
            // Invalidação por endereço
            else if (invalidate_addr_valid) begin
                logic [INDEX_BITS-1:0] inv_index;
                logic [TAG_BITS-1:0] inv_tag;
                inv_index = invalidate_addr[INDEX_BITS+OFFSET_BITS-1:OFFSET_BITS];
                inv_tag = invalidate_addr[PADDR_WIDTH-1:INDEX_BITS+OFFSET_BITS];
                
                for (int w = 0; w < NUM_WAYS; w++) begin
                    if (tag_array[inv_index][w].valid && 
                        tag_array[inv_index][w].tag == inv_tag) begin
                        tag_array[inv_index][w].valid <= 1'b0;
                    end
                end
            end
            else begin
                case (state)
                    S_IDLE: begin
                        if (req_valid && req_ready) begin
                            addr_reg <= req_addr;
                            index_reg <= req_index;
                            tag_reg <= req_tag;
                            offset_reg <= req_offset;
                        end
                    end
                    
                    S_TAG_CHECK: begin
                        if (cache_hit) begin
                            // Update PLRU
                            plru_bits[index_reg] <= update_plru(plru_bits[index_reg], hit_way_idx);
                        end
                        else begin
                            // Selecionar vítima para refill
                            victim_way <= get_plru_victim(plru_bits[index_reg]);
                        end
                    end
                    
                    S_REFILL_WAIT: begin
                        if (mem_ack) begin
                            refill_data_reg <= mem_data;
                        end
                    end
                    
                    S_REFILL_DONE: begin
                        // Escrever linha no cache
                        tag_array[index_reg][victim_way].valid <= 1'b1;
                        tag_array[index_reg][victim_way].tag <= tag_reg;
                        data_array[index_reg][victim_way] <= refill_data_reg;
                        
                        // Update PLRU
                        plru_bits[index_reg] <= update_plru(plru_bits[index_reg], victim_way);
                    end
                    
                    default: ;
                endcase
            end
        end
    end

endmodule
