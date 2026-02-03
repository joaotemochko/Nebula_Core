`timescale 1ns/1ps
`default_nettype none

import nebula_pkg::*;

/**
 * @module nebula_frontend_rvc
 * @brief Frontend com suporte completo a RV64C (instruções comprimidas)
 *
 * @details
 * O frontend lida corretamente com mix de instruções de 16 e 32 bits:
 *
 * Problemas resolvidos:
 * 1. Detecção de tamanho: bits[1:0] != 2'b11 indica 16-bit
 * 2. Alinhamento de 2 bytes (não 4)
 * 3. Instrução de 32-bit pode cruzar boundary de halfword
 * 4. PC avança +2 ou +4 dependendo do tamanho
 * 5. Buffer de fetch para lidar com instruções partidas
 *
 * Fetch buffer de 64 bits (4 halfwords) para:
 * - Sempre ter pelo menos 1 instrução completa disponível
 * - Lidar com 32-bit instruction crossing fetch boundary
 *
 * Pipeline:
 * [Fetch] -> [Align/Expand] -> [Decode] -> [Issue]
 */
module nebula_frontend_rvc #(
    parameter int XLEN = 64,
    parameter int VADDR_WIDTH = 39,
    parameter int PADDR_WIDTH = 56,
    parameter int VPN_WIDTH = 27,
    parameter int PPN_WIDTH = 44
)(
    input  wire                     clk,
    input  wire                     rst_n,
    
    // =========================================================================
    // Backend Control
    // =========================================================================
    input  wire                     backend_stall,
    input  wire                     backend_flush,
    input  wire                     backend_redirect,
    input  wire [VADDR_WIDTH-1:0]   backend_redirect_pc,
    
    // =========================================================================
    // Output to Backend
    // =========================================================================
    output frontend_packet_t        frontend_out,
    output logic                    frontend_valid,
    output logic [VADDR_WIDTH-1:0]  fetch_pc_out,
    
    // =========================================================================
    // Branch Prediction
    // =========================================================================
    input  bp_prediction_t          bp_prediction,
    
    // =========================================================================
    // I-Cache Interface
    // =========================================================================
    output logic                    icache_req,
    output logic [PADDR_WIDTH-1:0]  icache_addr,
    input  wire                     icache_ready,
    input  wire                     icache_resp_valid,
    input  wire [63:0]              icache_resp_data,  // 64 bits = 4 halfwords
    input  wire                     icache_resp_error,
    
    // =========================================================================
    // ITLB Interface
    // =========================================================================
    output logic                    itlb_req,
    output logic [VPN_WIDTH-1:0]    itlb_vpn,
    input  wire                     itlb_hit,
    input  wire [PPN_WIDTH-1:0]     itlb_ppn,
    input  wire                     itlb_page_fault,
    input  wire                     itlb_access_fault,
    
    // =========================================================================
    // PTW Interface
    // =========================================================================
    output logic                    ptw_req,
    output logic [VPN_WIDTH-1:0]    ptw_vpn,
    input  wire                     ptw_ready,
    input  wire                     ptw_resp_valid,
    input  wire                     ptw_page_fault,
    input  wire                     ptw_access_fault,
    
    // =========================================================================
    // Control
    // =========================================================================
    input  wire                     mmu_enabled,
    input  wire [1:0]               current_priv,
    
    // =========================================================================
    // Exception Output
    // =========================================================================
    output logic                    fetch_exception,
    output exception_cause_t        fetch_exception_cause,
    output logic [XLEN-1:0]         fetch_exception_value
);

    // =========================================================================
    // Constants
    // =========================================================================
    
    localparam logic [VADDR_WIDTH-1:0] RESET_VECTOR = 39'h00_1000_0000;
    
    //localparam logic [VADDR_WIDTH-1:0] RESET_VECTOR = 39'h00_0000_0000;

    // =========================================================================
    // Fetch Buffer
    // =========================================================================
    // 
    // O fetch buffer mantém dados suficientes para sempre ter pelo menos
    // uma instrução completa disponível, mesmo que seja 32-bit cruzando
    // um boundary.
    //
    // Buffer de 80 bits = 64 bits de fetch + 16 bits de carry-over
    // [79:64] = carry-over do fetch anterior (se houver)
    // [63:0]  = dados do fetch atual
    //
    // fetch_offset indica onde começa a próxima instrução no buffer
    
    logic [79:0]    fetch_buffer;
    logic [3:0]     fetch_buffer_valid;  // Quais halfwords são válidos [3:0]
    logic [2:0]     fetch_offset;        // Offset em halfwords (0-4)
    
    // =========================================================================
    // PC Management
    // =========================================================================
    
    logic [VADDR_WIDTH-1:0] pc_reg;
    logic [VADDR_WIDTH-1:0] fetch_pc;       // PC do fetch atual (alinhado a 8 bytes)
    logic [VADDR_WIDTH-1:0] next_pc;
    logic [VADDR_WIDTH-1:0] instr_pc;       // PC da instrução sendo decodificada
    
    // =========================================================================
    // Instruction Extraction
    // =========================================================================
    
    logic [31:0]    raw_instr;          // Instrução bruta (pode ser 16 ou 32 bits)
    logic [15:0]    first_halfword;     // Primeiro halfword
    logic [15:0]    second_halfword;    // Segundo halfword (para 32-bit)
    logic           is_compressed;      // Instrução é de 16 bits
    logic           need_second_half;   // Precisa do segundo halfword
    logic           have_full_instr;    // Temos instrução completa no buffer
    
    // =========================================================================
    // Compressed Decoder Interface
    // =========================================================================
    
    logic [15:0]    compressed_instr;
    logic [31:0]    expanded_instr;
    logic           compressed_valid;
    logic           compressed_illegal;
    logic           is_compressed_out;
    
    // =========================================================================
    // FSM
    // =========================================================================
    
    typedef enum logic [3:0] {
        S_RESET,
        S_FETCH_REQ,
        S_TLB_LOOKUP,
        S_TLB_WAIT,
        S_ICACHE_REQ,
        S_ICACHE_WAIT,
        S_PROCESS,
        S_DECODE,
        S_STALL,
        S_FLUSH,
        S_EXCEPTION
    } state_t;
    
    state_t state, next_state;
    
    // =========================================================================
    // TLB/PTW State
    // =========================================================================
    
    logic [PPN_WIDTH-1:0] cached_ppn;
    logic                 tlb_done;
    
    // =========================================================================
    // Compressed Decoder Instance
    // =========================================================================
    
    compressed_decoder_rv64 u_decompress (
        .cinstr(compressed_instr),
        .instr(expanded_instr),
        .valid(compressed_valid),
        .illegal(compressed_illegal),
        .is_compressed(is_compressed_out)
    );
    
    // =========================================================================
    // Instruction Alignment and Extraction
    // =========================================================================
    
    // Extrair halfwords do buffer baseado no offset
    always_comb begin
        // Selecionar primeiro halfword baseado no offset
        case (fetch_offset[1:0])
            2'b00: first_halfword = fetch_buffer[15:0];
            2'b01: first_halfword = fetch_buffer[31:16];
            2'b10: first_halfword = fetch_buffer[47:32];
            2'b11: first_halfword = fetch_buffer[63:48];
        endcase
        
        // Selecionar segundo halfword (para instruções de 32 bits)
        case (fetch_offset[1:0])
            2'b00: second_halfword = fetch_buffer[31:16];
            2'b01: second_halfword = fetch_buffer[47:32];
            2'b10: second_halfword = fetch_buffer[63:48];
            2'b11: second_halfword = fetch_buffer[79:64];  // Carry-over
        endcase
        
        // Detectar se é instrução comprimida
        is_compressed = (first_halfword[1:0] != 2'b11);
        
        // Verificar se precisamos do segundo halfword
        need_second_half = !is_compressed;
        
        // Verificar se temos instrução completa
        // Para 16-bit: precisamos apenas do primeiro halfword válido
        // Para 32-bit: precisamos dos dois halfwords válidos
        case (fetch_offset[1:0])
            2'b00: have_full_instr = fetch_buffer_valid[0] && 
                                    (is_compressed || fetch_buffer_valid[1]);
            2'b01: have_full_instr = fetch_buffer_valid[1] && 
                                    (is_compressed || fetch_buffer_valid[2]);
            2'b10: have_full_instr = fetch_buffer_valid[2] && 
                                    (is_compressed || fetch_buffer_valid[3]);
            2'b11: have_full_instr = fetch_buffer_valid[3] && 
                                    (is_compressed || (fetch_offset[2] && fetch_buffer[79:64] != 16'h0));
        endcase
        
        // Montar instrução bruta
        if (is_compressed) begin
            raw_instr = {16'h0, first_halfword};
            compressed_instr = first_halfword;
        end
        else begin
            // Instrução de 32 bits: little-endian
            raw_instr = {second_halfword, first_halfword};
            compressed_instr = 16'h0;
        end
    end
    
    // =========================================================================
    // PC Calculation
    // =========================================================================
    
    always_comb begin
        // PC da instrução atual
        instr_pc = pc_reg;
        
        // PC do próximo fetch (alinhado a 8 bytes)
        fetch_pc = {pc_reg[VADDR_WIDTH-1:3], 3'b000};
        
        // Próximo PC (após consumir instrução atual)
        if (is_compressed)
            next_pc = pc_reg + 2;
        else
            next_pc = pc_reg + 4;
    end
    
    // =========================================================================
    // Decode Logic
    // =========================================================================
    
    decoded_instr_t decoded_instr0;
    decoded_instr_t decoded_instr1;
    logic           can_dual_issue;
    
    // Instrução final (expandida se comprimida)
    logic [31:0] final_instr;
    assign final_instr = is_compressed ? expanded_instr : raw_instr;
    
    // Decodificação da instrução
    always_comb begin
        decoded_instr0 = '0;
        decoded_instr0.pc = instr_pc;
        decoded_instr0.is_compressed = is_compressed;
        decoded_instr0.valid = have_full_instr && (state == S_DECODE);
        
        if (is_compressed && compressed_illegal) begin
            // Instrução comprimida ilegal
            decoded_instr0.valid = 1'b0;
        end
        else if (have_full_instr) begin
            // Decodificar instrução de 32 bits (já expandida se era comprimida)
            decoded_instr0.opcode = final_instr[6:0];
            decoded_instr0.rd = final_instr[11:7];
            decoded_instr0.funct3 = final_instr[14:12];
            decoded_instr0.rs1 = final_instr[19:15];
            decoded_instr0.rs2 = final_instr[24:20];
            decoded_instr0.rs3 = final_instr[31:27];  // Para FMA
            decoded_instr0.funct7 = final_instr[31:25];
            
            // Immediate extraction baseado no opcode
            case (final_instr[6:0])
                7'b0110111, 7'b0010111: begin // LUI, AUIPC
                    decoded_instr0.imm = {{32{final_instr[31]}}, final_instr[31:12], 12'b0};
                    decoded_instr0.is_alu = 1'b1;
                end
                
                7'b1101111: begin // JAL
                    decoded_instr0.imm = {{43{final_instr[31]}}, final_instr[31], 
                                         final_instr[19:12], final_instr[20], 
                                         final_instr[30:21], 1'b0};
                    decoded_instr0.is_branch = 1'b1;
                    decoded_instr0.is_jal = 1'b1;
                end
                
                7'b1100111: begin // JALR
                    decoded_instr0.imm = {{52{final_instr[31]}}, final_instr[31:20]};
                    decoded_instr0.is_branch = 1'b1;
                    decoded_instr0.is_jalr = 1'b1;
                end
                
                7'b1100011: begin // Branch
                    decoded_instr0.imm = {{51{final_instr[31]}}, final_instr[31], 
                                         final_instr[7], final_instr[30:25], 
                                         final_instr[11:8], 1'b0};
                    decoded_instr0.is_branch = 1'b1;
                end
                
                7'b0000011: begin // Load
                    decoded_instr0.imm = {{52{final_instr[31]}}, final_instr[31:20]};
                    decoded_instr0.is_load = 1'b1;
                end
                
                7'b0100011: begin // Store
                    decoded_instr0.imm = {{52{final_instr[31]}}, final_instr[31:25], 
                                         final_instr[11:7]};
                    decoded_instr0.is_store = 1'b1;
                end
                
                7'b0010011, 7'b0011011: begin // ALU-I, ALU-I-W
                    decoded_instr0.imm = {{52{final_instr[31]}}, final_instr[31:20]};
                    decoded_instr0.is_alu = 1'b1;
                    decoded_instr0.is_alu_w = (final_instr[6:0] == 7'b0011011);
                end
                
                7'b0110011, 7'b0111011: begin // ALU-R, ALU-R-W
                    decoded_instr0.is_alu = 1'b1;
                    decoded_instr0.is_alu_w = (final_instr[6:0] == 7'b0111011);
                    // Check for M extension
                    if (final_instr[31:25] == 7'b0000001)
                        decoded_instr0.is_mdu = 1'b1;
                end
                
                7'b0101111: begin // AMO
                    decoded_instr0.is_amo = 1'b1;
                    decoded_instr0.is_lr = (final_instr[31:27] == 5'b00010);
                    decoded_instr0.is_sc = (final_instr[31:27] == 5'b00011);
                end
                
                7'b0000111: begin // FP Load
                    decoded_instr0.imm = {{52{final_instr[31]}}, final_instr[31:20]};
                    decoded_instr0.is_fp_load = 1'b1;
                    decoded_instr0.is_fp = 1'b1;
                end
                
                7'b0100111: begin // FP Store
                    decoded_instr0.imm = {{52{final_instr[31]}}, final_instr[31:25], 
                                         final_instr[11:7]};
                    decoded_instr0.is_fp_store = 1'b1;
                    decoded_instr0.is_fp = 1'b1;
                end
                
                7'b1000011, 7'b1000111, 7'b1001011, 7'b1001111: begin // FMADD, FMSUB, FNMSUB, FNMADD
                    decoded_instr0.is_fp = 1'b1;
                    decoded_instr0.is_fma = 1'b1;
                end
                
                7'b1010011: begin // FP ops
                    decoded_instr0.is_fp = 1'b1;
                    decoded_instr0.is_fp_single = (final_instr[26:25] == 2'b00);
                    decoded_instr0.is_fp_double = (final_instr[26:25] == 2'b01);
                end
                
                7'b1110011: begin // SYSTEM
                    decoded_instr0.csr_addr = final_instr[31:20];
                    if (final_instr[14:12] != 3'b000) begin
                        decoded_instr0.is_csr = 1'b1;
                    end
                    else begin
                        case (final_instr[31:20])
                            12'h000: decoded_instr0.is_ecall = 1'b1;
                            12'h001: decoded_instr0.is_ebreak = 1'b1;
                            12'h302: decoded_instr0.is_mret = 1'b1;
                            12'h102: decoded_instr0.is_sret = 1'b1;
                            12'h105: decoded_instr0.is_wfi = 1'b1;
                            default: ;
                        endcase
                    end
                end
                
                7'b0001111: begin // FENCE
                    if (final_instr[14:12] == 3'b001)
                        decoded_instr0.is_fence_i = 1'b1;
                    else
                        decoded_instr0.is_fence = 1'b1;
                end
                
                default: ;
            endcase
            
            // SFENCE.VMA detection
            if (final_instr[31:25] == 7'b0001001 && final_instr[14:12] == 3'b000 && 
                final_instr[6:0] == 7'b1110011)
                decoded_instr0.is_sfence_vma = 1'b1;
        end
        
        // Segunda instrução (para dual-issue) - simplificado por agora
        decoded_instr1 = '0;
        can_dual_issue = 1'b0;  // TODO: implementar dual-issue com RVC
    end
    
    // =========================================================================
    // FSM Next State
    // =========================================================================
    
    always_comb begin
        next_state = state;
        
        case (state)
            S_RESET: begin
                next_state = S_FETCH_REQ;
            end
            
            S_FETCH_REQ: begin
                if (backend_flush)
                    next_state = S_FLUSH;
                else if (mmu_enabled)
                    next_state = S_TLB_LOOKUP;
                else
                    next_state = S_ICACHE_REQ;
            end
            
            S_TLB_LOOKUP: begin
                if (backend_flush)
                    next_state = S_FLUSH;
                else if (itlb_page_fault)
                    next_state = S_EXCEPTION;
                else if (itlb_hit)
                    next_state = S_ICACHE_REQ;
                else if (ptw_ready)
                    next_state = S_TLB_WAIT;
            end
            
            S_TLB_WAIT: begin
                if (backend_flush)
                    next_state = S_FLUSH;
                else if (ptw_resp_valid) begin
                    if (ptw_page_fault)
                        next_state = S_EXCEPTION;
                    else
                        next_state = S_TLB_LOOKUP;
                end
            end
            
            S_ICACHE_REQ: begin
                if (backend_flush)
                    next_state = S_FLUSH;
                else if (icache_ready)
                    next_state = S_ICACHE_WAIT;
            end
            
            S_ICACHE_WAIT: begin
                if (backend_flush)
                    next_state = S_FLUSH;
                else if (icache_resp_valid) begin
                    if (icache_resp_error)
                        next_state = S_EXCEPTION;
                    else
                        next_state = S_PROCESS;
                end
            end
            
            S_PROCESS: begin
                if (backend_flush)
                    next_state = S_FLUSH;
                else if (have_full_instr)
                    next_state = S_DECODE;
                else
                    next_state = S_FETCH_REQ;  // Precisa de mais dados
            end
            
            S_DECODE: begin
                if (backend_flush)
                    next_state = S_FLUSH;
                else if (backend_stall)
                    next_state = S_STALL;
                else begin
                    // Verificar se ainda temos instruções no buffer
                    if (have_full_instr)
                        next_state = S_DECODE;  // Continuar decodificando
                    else
                        next_state = S_FETCH_REQ;  // Precisa de mais dados
                end
            end
            
            S_STALL: begin
                if (backend_flush)
                    next_state = S_FLUSH;
                else if (!backend_stall)
                    next_state = S_DECODE;
            end
            
            S_FLUSH: begin
                next_state = S_FETCH_REQ;
            end
            
            S_EXCEPTION: begin
                // Aguardar backend tratar exceção
                if (backend_flush)
                    next_state = S_FLUSH;
            end
            
            default: next_state = S_RESET;
        endcase
    end
    
    // =========================================================================
    // Output Signals
    // =========================================================================
    
    assign frontend_out.instr0 = decoded_instr0;
    assign frontend_out.instr1 = decoded_instr1;
    assign frontend_out.instr0_valid = decoded_instr0.valid;
    assign frontend_out.instr1_valid = 1'b0;  // TODO: dual-issue
    assign frontend_out.dual_issue = 1'b0;
    
    assign frontend_valid = (state == S_DECODE) && have_full_instr && !backend_stall;
    assign fetch_pc_out = instr_pc;
    
    // I-Cache interface
    assign icache_req = (state == S_ICACHE_REQ);
    assign icache_addr = mmu_enabled ? {cached_ppn, fetch_pc[11:0]} : 
                                       {{(PADDR_WIDTH-VADDR_WIDTH){1'b0}}, fetch_pc};
    
    // TLB interface
    assign itlb_req = (state == S_TLB_LOOKUP);
    assign itlb_vpn = fetch_pc[VADDR_WIDTH-1:12];
    
    // PTW interface
    assign ptw_req = (state == S_TLB_LOOKUP) && !itlb_hit && !itlb_page_fault;
    assign ptw_vpn = fetch_pc[VADDR_WIDTH-1:12];
    
    // Exception signals
    assign fetch_exception_cause = (itlb_page_fault || ptw_page_fault) ? EXC_INSTR_PAGE_FAULT : EXC_INSTR_ACCESS_FAULT;
    assign fetch_exception_value = {{(XLEN-VADDR_WIDTH){pc_reg[VADDR_WIDTH-1]}}, pc_reg};
    
    // =========================================================================
    // Sequential Logic
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_RESET;
            pc_reg <= RESET_VECTOR;
            fetch_buffer <= '0;
            fetch_buffer_valid <= '0;
            fetch_offset <= '0;
            cached_ppn <= '0;
            tlb_done <= 1'b0;
        end
        else begin
            state <= next_state;
            
            case (state)
                S_RESET: begin
                    pc_reg <= RESET_VECTOR;
                    fetch_buffer <= '0;
                    fetch_buffer_valid <= '0;
                    fetch_offset <= '0;
                end
                
                S_FETCH_REQ: begin
                    // Calcular offset inicial baseado no PC
                    fetch_offset <= {1'b0, pc_reg[2:1]};
                end
                
                S_TLB_LOOKUP: begin
                    if (itlb_hit) begin
                        cached_ppn <= itlb_ppn;
                        tlb_done <= 1'b1;
                    end
                end
                
                S_TLB_WAIT: begin
                    // Aguardar PTW
                end
                
                S_ICACHE_WAIT: begin
                    if (icache_resp_valid && !icache_resp_error) begin
                        // Carregar dados no buffer
                        // Se temos carry-over, manter os bits altos
                        if (fetch_offset[2]) begin
                            // Tinha carry-over do fetch anterior
                            fetch_buffer[79:64] <= fetch_buffer[63:48];
                        end
                        fetch_buffer[63:0] <= icache_resp_data;
                        fetch_buffer_valid <= 4'b1111;
                    end
                end
                
                S_DECODE: begin
                    if (!backend_stall && have_full_instr) begin
                        // Avançar PC
                        if (backend_redirect) begin
                            pc_reg <= backend_redirect_pc;
                            fetch_buffer_valid <= '0;
                            fetch_offset <= '0;
                        end
                        else if (bp_prediction.valid && bp_prediction.taken) begin
                            pc_reg <= bp_prediction.target;
                            fetch_buffer_valid <= '0;
                            fetch_offset <= '0;
                        end
                        else begin
                            pc_reg <= next_pc;
                            
                            // Atualizar offset no buffer
                            if (is_compressed)
                                fetch_offset <= fetch_offset + 1;
                            else
                                fetch_offset <= fetch_offset + 2;
                            
                            // Invalidar halfwords consumidos
                            if (is_compressed) begin
                                case (fetch_offset[1:0])
                                    2'b00: fetch_buffer_valid[0] <= 1'b0;
                                    2'b01: fetch_buffer_valid[1] <= 1'b0;
                                    2'b10: fetch_buffer_valid[2] <= 1'b0;
                                    2'b11: fetch_buffer_valid[3] <= 1'b0;
                                endcase
                            end
                            else begin
                                case (fetch_offset[1:0])
                                    2'b00: fetch_buffer_valid[1:0] <= 2'b00;
                                    2'b01: fetch_buffer_valid[2:1] <= 2'b00;
                                    2'b10: fetch_buffer_valid[3:2] <= 2'b00;
                                    2'b11: begin
                                        fetch_buffer_valid[3] <= 1'b0;
                                        // Carry-over foi consumido
                                    end
                                endcase
                            end
                        end
                    end
                end
                
                S_FLUSH: begin
                    if (backend_redirect) begin
                        pc_reg <= backend_redirect_pc;
                    end
                    fetch_buffer <= '0;
                    fetch_buffer_valid <= '0;
                    fetch_offset <= '0;
                    tlb_done <= 1'b0;
                end
                
                S_EXCEPTION: begin
                    // Manter estado até flush
                end
                
                default: ;
            endcase
        end
    end

endmodule
