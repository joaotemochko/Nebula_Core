`timescale 1ns/1ps
`default_nettype none

import nebula_pkg::*;

/**
 * @module dcache_l1
 * @brief Cache de Dados L1 - 32KB, 4-way set associative, write-back
 *
 * 1. Todas as declarações "logic" removidas de dentro de always_ff e
 * always_comb — movidas para sinais de módulo.
 * 2. found_dirty removido do bloco always_comb da FSM e declarado
 * como sinal de módulo.
 * 3. *_tmp que eram variáveis locais dentro de always_ff com
 * blocking assignment movidos para blocos always_comb separados.
 * Isso elimina os warnings BLKSEQ do Verilator 5.036.
 * 4. BLINDAGEM ANTI-FANTASMA: Proteção no sinal req_ready através do
 * registo prev_resp_valid para evitar que a Cache processe a mesma
 * instrução duas vezes devido a 1 ciclo de "eco" do Backend.
 */
module dcache_l1 #(
    parameter int PADDR_WIDTH = 56,
    parameter int XLEN = 64,
    parameter int LINE_SIZE = 64,
    parameter int NUM_WAYS = 4,
    parameter int NUM_SETS = 128
)(
    input  wire                     clk,
    input  wire                     rst_n,

    input  wire                     req_valid,
    input  wire [PADDR_WIDTH-1:0]   req_addr,
    input  wire [XLEN-1:0]          req_wdata,
    input  wire [7:0]               req_wstrb,
    input  wire                     req_we,
    input  wire                     req_is_amo,
    input  wire [4:0]               req_amo_op,
    output logic                    req_ready,

    output logic                    resp_valid,
    output logic [XLEN-1:0]         resp_rdata,
    output logic                    resp_error,

    input  wire                     invalidate_all,
    input  wire                     flush_all,
    input  wire                     flush_addr_valid,
    input  wire [PADDR_WIDTH-1:0]   flush_addr,
    output logic                    flush_done,

    output logic                    mem_req,
    output logic                    mem_we,
    output logic [PADDR_WIDTH-1:0]  mem_addr,
    output logic [LINE_SIZE*8-1:0]  mem_wdata,
    output logic [LINE_SIZE-1:0]    mem_wstrb,
    output logic                    mem_uncached,
    input  wire                     mem_ack,
    input  wire [LINE_SIZE*8-1:0]   mem_rdata,
    input  wire                     mem_error
);
    localparam int OFFSET_BITS = $clog2(LINE_SIZE);
    localparam int INDEX_BITS  = $clog2(NUM_SETS);
    localparam int TAG_BITS    = PADDR_WIDTH - INDEX_BITS - OFFSET_BITS;
    localparam int LINE_BITS   = LINE_SIZE * 8;
    localparam int WORDS_PER_LINE = LINE_SIZE / (XLEN/8);

    typedef struct packed {
        logic                valid;
        logic                dirty;
        logic [TAG_BITS-1:0] tag;
    } cache_tag_t;

    cache_tag_t           tag_array  [NUM_SETS][NUM_WAYS];
    logic [LINE_BITS-1:0] data_array [NUM_SETS][NUM_WAYS];
    logic [2:0]           plru_bits  [NUM_SETS];

    wire [OFFSET_BITS-1:0] req_offset = req_addr[OFFSET_BITS-1:0];
    wire [INDEX_BITS-1:0]  req_index  = req_addr[INDEX_BITS+OFFSET_BITS-1:OFFSET_BITS];
    wire [TAG_BITS-1:0]    req_tag    = req_addr[PADDR_WIDTH-1:INDEX_BITS+OFFSET_BITS];
    wire [2:0]             word_offset = req_offset[5:3];

    typedef enum logic [3:0] {
        S_IDLE,
        S_TAG_CHECK,
        S_HIT_PROCESS,
        S_WRITEBACK_REQ,
        S_WRITEBACK_WAIT,
        S_REFILL_REQ,
        S_REFILL_WAIT,
        S_REFILL_DONE,
        S_AMO_PROCESS,
        S_FLUSH_SCAN,
        S_FLUSH_WB,
        S_ERROR,
        S_MMIO_REQ,   
        S_MMIO_WAIT,  
        S_MMIO_DONE
    } state_t;

    state_t state, next_state;

    logic [PADDR_WIDTH-1:0]      addr_reg;
    logic [INDEX_BITS-1:0]       index_reg;
    logic [TAG_BITS-1:0]         tag_reg;
    logic [OFFSET_BITS-1:0]      offset_reg;
    logic [2:0]                  word_offset_reg;
    logic [XLEN-1:0]             wdata_reg;
    logic [7:0]                  wstrb_reg;
    logic                        we_reg;
    logic                        is_amo_reg;
    logic [4:0]                  amo_op_reg;
    logic [LINE_BITS-1:0]        line_buffer;
    logic [$clog2(NUM_WAYS)-1:0] victim_way;
    logic [$clog2(NUM_WAYS)-1:0] hit_way_idx_reg;
    logic [INDEX_BITS-1:0]       flush_set_idx;
    logic [$clog2(NUM_WAYS)-1:0] flush_way_idx;

    // Tag comparison signals
    logic [NUM_WAYS-1:0]         hit_way;
    logic                        cache_hit;
    logic [$clog2(NUM_WAYS)-1:0] hit_way_idx;
    logic                        victim_dirty;
    logic [TAG_BITS-1:0]         victim_tag;

    // Sinais de nível de módulo para combinatorial intermediário
    logic                        found_dirty;
    logic [LINE_BITS-1:0]        hit_line;
    logic [XLEN-1:0]             selected_word;

    // =========================================================================
    // FIX BLKSEQ: cálculos intermediários em always_comb separados
    // =========================================================================

    // Para S_HIT_PROCESS
    logic [XLEN-1:0]             hit_new_word;
    logic [LINE_BITS-1:0]        hit_new_line;

    // Para S_REFILL_DONE
    logic [XLEN-1:0]             refill_word_from_buf;
    logic [XLEN-1:0]             refill_new_word;
    logic [LINE_BITS-1:0]        refill_new_line;

    // Para S_AMO_PROCESS
    logic [XLEN-1:0]             amo_current_word;
    logic [XLEN-1:0]             amo_result;
    logic [LINE_BITS-1:0]        amo_new_line;

    // Para S_FLUSH_SCAN
    logic                        flush_found;
    logic [INDEX_BITS-1:0]       flush_set_next;
    logic [$clog2(NUM_WAYS)-1:0] flush_way_next;

    // =========================================================================
    // Detecção de MMIO (Memory Mapped I/O)
    // =========================================================================
    // Qualquer endereço >= 0x1f0000000 é tratado como Periférico (Uncached)
    wire is_mmio = (req_addr >= 56'h1f00_0000);

    // =========================================================================
    // Tag Comparison (combinacional)
    // =========================================================================
    always_comb begin
        hit_way    = '0;
        cache_hit  = 1'b0;
        hit_way_idx = '0;

        for (int w = 0; w < NUM_WAYS; w++) begin
            if (tag_array[index_reg][w].valid &&
                tag_array[index_reg][w].tag == tag_reg) begin
                hit_way[w]  = 1'b1;
                cache_hit   = 1'b1;
                hit_way_idx = w[$clog2(NUM_WAYS)-1:0];
            end
        end

        victim_dirty = tag_array[index_reg][victim_way].dirty;
        victim_tag   = tag_array[index_reg][victim_way].tag;
    end

    // =========================================================================
    // Data Selection (combinacional)
    // =========================================================================
    always_comb begin
        hit_line      = '0;
        selected_word = '0;

        for (int w = 0; w < NUM_WAYS; w++) begin
            if (hit_way[w])
                hit_line = data_array[index_reg][w];
        end

        case (word_offset_reg)
            3'd0: selected_word = hit_line[63:0];
            3'd1: selected_word = hit_line[127:64];
            3'd2: selected_word = hit_line[191:128];
            3'd3: selected_word = hit_line[255:192];
            3'd4: selected_word = hit_line[319:256];
            3'd5: selected_word = hit_line[383:320];
            3'd6: selected_word = hit_line[447:384];
            3'd7: selected_word = hit_line[511:448];
            default: selected_word = '0;
        endcase
    end

    // =========================================================================
    // S_HIT_PROCESS — cálculo combinacional
    // =========================================================================
    always_comb begin
        hit_new_word = apply_wstrb(selected_word, wdata_reg, wstrb_reg);
        hit_new_line = modify_line(hit_line, hit_new_word, word_offset_reg);
    end

    // =========================================================================
    // S_REFILL_DONE — cálculo combinacional
    // =========================================================================
    always_comb begin
        case (word_offset_reg)
            3'd0: refill_word_from_buf = line_buffer[63:0];
            3'd1: refill_word_from_buf = line_buffer[127:64];
            3'd2: refill_word_from_buf = line_buffer[191:128];
            3'd3: refill_word_from_buf = line_buffer[255:192];
            3'd4: refill_word_from_buf = line_buffer[319:256];
            3'd5: refill_word_from_buf = line_buffer[383:320];
            3'd6: refill_word_from_buf = line_buffer[447:384];
            3'd7: refill_word_from_buf = line_buffer[511:448];
            default: refill_word_from_buf = '0;
        endcase

        refill_new_word = apply_wstrb(refill_word_from_buf, wdata_reg, wstrb_reg);
        if (we_reg && !is_amo_reg)
            refill_new_line = modify_line(line_buffer, refill_new_word, word_offset_reg);
        else
            refill_new_line = line_buffer;
    end

    // =========================================================================
    // S_AMO_PROCESS — cálculo combinacional
    // =========================================================================
    always_comb begin
        amo_current_word = selected_word;
        amo_result   = compute_amo(amo_op_reg, amo_current_word, wdata_reg,
                                   (wstrb_reg == 8'h0F));
        amo_new_line = modify_line(data_array[index_reg][hit_way_idx_reg],
                                   amo_result, word_offset_reg);
    end

    // =========================================================================
    // S_FLUSH_SCAN — cálculo combinacional
    // =========================================================================
    always_comb begin
        flush_found    = 1'b0;
        flush_set_next = flush_set_idx;
        flush_way_next = flush_way_idx;

        for (int s = 0; s < NUM_SETS; s++) begin
            for (int w = 0; w < NUM_WAYS; w++) begin
                if (!flush_found &&
                    tag_array[s][w].valid &&
                    tag_array[s][w].dirty) begin
    
                    flush_set_next = s[INDEX_BITS-1:0];
                    flush_way_next = w[$clog2(NUM_WAYS)-1:0];
                    flush_found    = 1'b1;
                end
            end
        end
    end

    // =========================================================================
    // Funções auxiliares
    // =========================================================================
    function automatic [XLEN-1:0] apply_wstrb(
        input [XLEN-1:0] old_data,
        input [XLEN-1:0] new_data,
        input [7:0]      wstrb
    );
        logic [XLEN-1:0] result;
        for (int i = 0; i < 8; i++)
            result[i*8 +: 8] = wstrb[i] ? new_data[i*8 +: 8] : old_data[i*8 +: 8];
        return result;
    endfunction

    function automatic [LINE_BITS-1:0] modify_line(
        input [LINE_BITS-1:0] line,
        input [XLEN-1:0]      new_word,
        input [2:0]           word_idx
    );
        logic [LINE_BITS-1:0] result;
        result = line;
        case (word_idx)
            3'd0: result[63:0]     = new_word;
            3'd1: result[127:64]   = new_word;
            3'd2: result[191:128]  = new_word;
            3'd3: result[255:192]  = new_word;
            3'd4: result[319:256]  = new_word;
            3'd5: result[383:320]  = new_word;
            3'd6: result[447:384]  = new_word;
            3'd7: result[511:448]  = new_word;
            default: ;
        endcase
        return result;
    endfunction

    // AMO opcodes
    localparam AMO_LR   = 5'b00010;
    localparam AMO_SC   = 5'b00011;
    localparam AMO_SWAP = 5'b00001;
    localparam AMO_ADD  = 5'b00000;
    localparam AMO_XOR  = 5'b00100;
    localparam AMO_AND  = 5'b01100;
    localparam AMO_OR   = 5'b01000;
    localparam AMO_MIN  = 5'b10000;
    localparam AMO_MAX  = 5'b10100;
    localparam AMO_MINU = 5'b11000;
    localparam AMO_MAXU = 5'b11100;

    function automatic [XLEN-1:0] compute_amo(
        input [4:0]      op,
        input [XLEN-1:0] mem_val,
        input [XLEN-1:0] reg_val,
        input            is_word
    );
        logic [XLEN-1:0]  result;
        logic signed [63:0] mem_s, reg_s;
        logic signed [31:0] mem_sw, reg_sw;
        mem_s  = $signed(mem_val);
        reg_s  = $signed(reg_val);
        mem_sw = $signed(mem_val[31:0]);
        reg_sw = $signed(reg_val[31:0]);

        case (op)
            AMO_SWAP: result = reg_val;
            AMO_ADD:  result = mem_val + reg_val;
            AMO_XOR:  result = mem_val ^ reg_val;
            AMO_AND:  result = mem_val & reg_val;
            AMO_OR:   result = mem_val | reg_val;
            AMO_MIN:  result = is_word ? ((mem_sw < reg_sw) ? mem_val : reg_val) :
                                         ((mem_s  < reg_s ) ? mem_val : reg_val);
            AMO_MAX:  result = is_word ? ((mem_sw > reg_sw) ? mem_val : reg_val) :
                                         ((mem_s  > reg_s ) ? mem_val : reg_val);
            AMO_MINU: result = is_word ? ((mem_val[31:0] < reg_val[31:0]) ? mem_val : reg_val) :
                                         ((mem_val < reg_val) ? mem_val : reg_val);
            AMO_MAXU: result = is_word ? ((mem_val[31:0] > reg_val[31:0]) ? mem_val : reg_val) :
                                         ((mem_val > reg_val) ? mem_val : reg_val);
            default:  result = reg_val;
        endcase
        return result;
    endfunction

    // Pseudo-LRU
    function automatic logic [$clog2(NUM_WAYS)-1:0] get_plru_victim(
        input logic [2:0] plru
    );
        if (!plru[0])
            return plru[1] ? 2'd0 : 2'd1;
        else
            return plru[2] ? 2'd2 : 2'd3;
    endfunction

    function automatic logic [2:0] update_plru(
        input logic [2:0]                plru,
        input logic [$clog2(NUM_WAYS)-1:0] way
    );
        logic [2:0] new_plru;
        new_plru = plru;
        case (way)
            2'd0: begin new_plru[0] = 1'b1; new_plru[1] = 1'b1; end
            2'd1: begin new_plru[0] = 1'b1; new_plru[1] = 1'b0; end
            2'd2: begin new_plru[0] = 1'b0; new_plru[2] = 1'b1; end
            2'd3: begin new_plru[0] = 1'b0; new_plru[2] = 1'b0; end
            default: ;
        endcase
        return new_plru;
    endfunction

    // =========================================================================
    // Proteção Anti-Fantasma
    // =========================================================================
    logic prev_resp_valid;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) prev_resp_valid <= 1'b0;
        else        prev_resp_valid <= resp_valid;
    end

    // =========================================================================
    // FSM - Próximo Estado
    // =========================================================================
    always_comb begin
        found_dirty = 1'b0;
        next_state  = state;

        case (state)
            S_IDLE: begin
                if (flush_all || flush_addr_valid)
                    next_state = S_FLUSH_SCAN;
                else if (req_valid && req_ready) begin
                    if (is_mmio)
                        next_state = S_MMIO_REQ;
                    else
                        next_state = S_TAG_CHECK;
                end
            end

            S_TAG_CHECK: begin
                if (cache_hit) begin
                    if (is_amo_reg && amo_op_reg != AMO_LR)
                        next_state = S_AMO_PROCESS;
                    else
                        next_state = S_HIT_PROCESS;
                end else begin
                    if (tag_array[index_reg][victim_way].valid && victim_dirty)
                        next_state = S_WRITEBACK_REQ;
                    else
                        next_state = S_REFILL_REQ;
                end
            end

            S_HIT_PROCESS: next_state = S_IDLE;
            S_AMO_PROCESS: next_state = S_IDLE;

            S_WRITEBACK_REQ:  next_state = S_WRITEBACK_WAIT;
            S_WRITEBACK_WAIT: begin
                if (mem_ack)        next_state = S_REFILL_REQ;
                else if (mem_error) next_state = S_ERROR;
            end

            S_REFILL_REQ:  next_state = S_REFILL_WAIT;
            S_REFILL_WAIT: begin
                if (mem_ack)        next_state = S_REFILL_DONE;
                else if (mem_error) next_state = S_ERROR;
            end

            S_REFILL_DONE: begin
                if (is_amo_reg && amo_op_reg != AMO_LR)
                    next_state = S_AMO_PROCESS;
                else
                    next_state = S_HIT_PROCESS;
            end

            S_FLUSH_SCAN: begin
                for (int s = 0; s < NUM_SETS; s++) begin
                    for (int w = 0; w < NUM_WAYS; w++) begin
                        if (tag_array[s][w].valid && tag_array[s][w].dirty)
                            found_dirty = 1'b1;
                    end
                end
                next_state = found_dirty ? S_FLUSH_WB : S_IDLE;
            end

            S_FLUSH_WB: begin
                if (mem_ack)        next_state = S_FLUSH_SCAN;
                else if (mem_error) next_state = S_ERROR;
            end

            S_MMIO_REQ:  next_state = S_MMIO_WAIT;

            S_MMIO_WAIT: begin
                if (mem_ack)        next_state = S_MMIO_DONE;
                else if (mem_error) next_state = S_ERROR;
            end

            S_MMIO_DONE: next_state = S_IDLE;

            S_ERROR: next_state = S_IDLE;
            default: next_state = S_IDLE;
        endcase
    end

    // =========================================================================
    // Saídas de Memória
    // =========================================================================
    always_comb begin
        mem_req      = 1'b0;
        mem_we       = 1'b0;
        mem_addr     = '0;
        mem_wdata    = '0;
        mem_wstrb    = {LINE_SIZE{1'b1}};
        mem_uncached = 1'b0;

        case (state)
            S_WRITEBACK_REQ, S_WRITEBACK_WAIT: begin
                mem_req  = (state == S_WRITEBACK_REQ);
                mem_we   = 1'b1;
                mem_addr = {victim_tag, index_reg, {OFFSET_BITS{1'b0}}};
                mem_wdata = data_array[index_reg][victim_way];
            end
            S_REFILL_REQ, S_REFILL_WAIT: begin
                mem_req  = (state == S_REFILL_REQ);
                mem_we   = 1'b0;
                mem_addr = {tag_reg, index_reg, {OFFSET_BITS{1'b0}}};
            end
            S_FLUSH_WB: begin
                mem_req  = 1'b1;
                mem_we   = 1'b1;
                mem_addr = {tag_array[flush_set_idx][flush_way_idx].tag,
                            flush_set_idx, {OFFSET_BITS{1'b0}}};
                mem_wdata = data_array[flush_set_idx][flush_way_idx];
            end
            S_MMIO_REQ, S_MMIO_WAIT: begin
                mem_req      = (state == S_MMIO_REQ);
                mem_we       = we_reg;
                mem_addr     = addr_reg;
                mem_uncached = 1'b1; 
                
                // Colocar a palavra de 64 bits no offset exato do barramento de 512 bits
                mem_wdata[word_offset_reg * 64 +: 64] = wdata_reg;
                
                // Ligar APENAS os bytes que o CPU quer escrever
                mem_wstrb = '0;
                mem_wstrb[word_offset_reg * 8 +: 8] = we_reg ? wstrb_reg : 8'hFF;
            end
            default: ;
        endcase
    end

    // =========================================================================
    // Saídas para Backend e Controle de Eco (Phantom Request)
    // =========================================================================
    assign req_ready  = (state == S_IDLE) && !flush_all && !flush_addr_valid && !prev_resp_valid;
    assign flush_done = (state == S_FLUSH_SCAN) && (next_state == S_IDLE);

    always_comb begin
        resp_valid = 1'b0;
        resp_rdata = '0;
        resp_error = 1'b0;

        case (state)
            S_HIT_PROCESS: begin
                resp_valid = 1'b1;
                resp_rdata = selected_word;
            end
            S_AMO_PROCESS: begin
                resp_valid = 1'b1;
                resp_rdata = selected_word;
            end
            S_REFILL_DONE: begin
                if (!we_reg && !is_amo_reg) begin
                    resp_valid = 1'b1;
                    case (word_offset_reg)
                        3'd0: resp_rdata = line_buffer[63:0];
                        3'd1: resp_rdata = line_buffer[127:64];
                        3'd2: resp_rdata = line_buffer[191:128];
                        3'd3: resp_rdata = line_buffer[255:192];
                        3'd4: resp_rdata = line_buffer[319:256];
                        3'd5: resp_rdata = line_buffer[383:320];
                        3'd6: resp_rdata = line_buffer[447:384];
                        3'd7: resp_rdata = line_buffer[511:448];
                        default: resp_rdata = '0;
                    endcase
                end
            end
            S_MMIO_DONE: begin
                resp_valid = 1'b1;
                if (!we_reg && !is_amo_reg) begin
                    case (word_offset_reg)
                        3'd0: resp_rdata = line_buffer[63:0];
                        3'd1: resp_rdata = line_buffer[127:64];
                        3'd2: resp_rdata = line_buffer[191:128];
                        3'd3: resp_rdata = line_buffer[255:192];
                        3'd4: resp_rdata = line_buffer[319:256];
                        3'd5: resp_rdata = line_buffer[383:320];
                        3'd6: resp_rdata = line_buffer[447:384];
                        3'd7: resp_rdata = line_buffer[511:448];
                        default: resp_rdata = '0;
                    endcase
                end
            end
            S_ERROR: begin
                resp_valid = 1'b1;
                resp_error = 1'b1;
            end
            default: ;
        endcase
    end

    // =========================================================================
    // Lógica Sequencial
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state           <= S_IDLE;
            addr_reg        <= '0;
            index_reg       <= '0;
            tag_reg         <= '0;
            offset_reg      <= '0;
            word_offset_reg <= '0;
            wdata_reg       <= '0;
            wstrb_reg       <= '0;
            we_reg          <= 1'b0;
            is_amo_reg      <= 1'b0;
            amo_op_reg      <= '0;
            line_buffer     <= '0;
            victim_way      <= '0;
            hit_way_idx_reg <= '0;
            flush_set_idx   <= '0;
            flush_way_idx   <= '0;

            for (int s = 0; s < NUM_SETS; s++) begin
                for (int w = 0; w < NUM_WAYS; w++) begin
                    tag_array[s][w].valid <= 1'b0;
                    tag_array[s][w].dirty <= 1'b0;
                    tag_array[s][w].tag   <= '0;
                end
                plru_bits[s] <= '0;
            end
        end else begin
            state <= next_state;

            if (invalidate_all && state == S_IDLE) begin
                for (int s = 0; s < NUM_SETS; s++) begin
                    for (int w = 0; w < NUM_WAYS; w++) begin
                        tag_array[s][w].valid <= 1'b0;
                        tag_array[s][w].dirty <= 1'b0;
                    end
                end
            end else begin
                case (state)
                    S_IDLE: begin
                        if (flush_all) begin
                             flush_set_idx <= '0;
                             flush_way_idx <= '0;
                        end else if (req_valid && req_ready) begin
                            addr_reg        <= req_addr;
                            index_reg       <= req_index;
                            tag_reg         <= req_tag;
                            offset_reg      <= req_offset;
                            word_offset_reg <= word_offset;
                            wdata_reg       <= req_wdata;
                            wstrb_reg       <= req_wstrb;
                            we_reg          <= req_we;
                            is_amo_reg      <= req_is_amo;
                            amo_op_reg      <= req_amo_op;
                        end
                    end

                    S_TAG_CHECK: begin
                        if (cache_hit)
                            hit_way_idx_reg <= hit_way_idx;
                        else
                            victim_way <= get_plru_victim(plru_bits[index_reg]);
                    end

                    S_HIT_PROCESS: begin
                        if (we_reg) begin
                            data_array[index_reg][hit_way_idx_reg] <= hit_new_line;
                            tag_array[index_reg][hit_way_idx_reg].dirty <= 1'b1;
                        end
                        plru_bits[index_reg] <= update_plru(plru_bits[index_reg], hit_way_idx_reg);
                    end

                    S_AMO_PROCESS: begin
                        data_array[index_reg][hit_way_idx_reg] <= amo_new_line;
                        tag_array[index_reg][hit_way_idx_reg].dirty <= 1'b1;
                        plru_bits[index_reg] <= update_plru(plru_bits[index_reg], hit_way_idx_reg);
                    end

                    S_REFILL_WAIT: begin
                        if (mem_ack)
                            line_buffer <= mem_rdata;
                    end

                    S_REFILL_DONE: begin
                        data_array[index_reg][victim_way]       <= refill_new_line;
                        tag_array[index_reg][victim_way].valid  <= 1'b1;
                        tag_array[index_reg][victim_way].dirty  <= we_reg;
                        tag_array[index_reg][victim_way].tag    <= tag_reg;
                        hit_way_idx_reg                         <= victim_way;
                        plru_bits[index_reg] <= update_plru(plru_bits[index_reg], victim_way);
                    end

                    S_FLUSH_SCAN: begin
                        if (flush_found) begin
                            flush_set_idx <= flush_set_next;
                            flush_way_idx <= flush_way_next;
                        end else begin
                            flush_set_idx <= '0;
                            flush_way_idx <= '0;
                        end
                    end

                    S_FLUSH_WB: begin
                        if (mem_ack) begin
                            tag_array[flush_set_idx][flush_way_idx].dirty <= 1'b0;
                            if (invalidate_all)
                                tag_array[flush_set_idx][flush_way_idx].valid <= 1'b0;

                            if (flush_way_idx == NUM_WAYS - 1) begin
                                flush_way_idx <= '0;
                                flush_set_idx <= flush_set_idx + 1;
                            end else begin
                                flush_way_idx <= flush_way_idx + 1;
                            end
                        end
                    end

                    S_MMIO_WAIT: begin
                        if (mem_ack)
                            line_buffer <= mem_rdata;
                    end

                    default: ;
                endcase
            end
        end
    end

endmodule
