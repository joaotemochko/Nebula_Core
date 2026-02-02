`timescale 1ns/1ps
`default_nettype none

import nebula_pkg::*;

/**
 * @module nebula_frontend
 * @brief Frontend do Nebula Core - Fetch, Decode, e Issue
 *
 * @details Responsável por:
 * - Fetch de instruções via I-Cache
 * - Tradução de endereço via ITLB
 * - Decodificação de instruções (dual-issue)
 * - Gerenciamento de PC
 * - Detecção de exceções de fetch
 */
module nebula_frontend #(
    parameter int XLEN = 64,
    parameter int VADDR_WIDTH = 39,
    parameter int PADDR_WIDTH = 56,
    parameter int VPN_WIDTH = 27,
    parameter int PPN_WIDTH = 44,
    parameter int ASID_WIDTH = 16,
    parameter int TLB_ENTRIES = 32
)(
    input  wire                     clk,
    input  wire                     rst_n,
    
    // =========================================================================
    // Controle do Backend
    // =========================================================================
    input  wire                     backend_stall,
    input  wire                     backend_flush,
    input  wire                     backend_redirect,
    input  wire [VADDR_WIDTH-1:0]   backend_redirect_pc,
    
    // =========================================================================
    // Saída para Backend
    // =========================================================================
    output frontend_packet_t        frontend_out,
    output logic                    frontend_valid,
    
    // =========================================================================
    // Interface de I-Cache
    // =========================================================================
    output logic                    icache_req,
    output logic [PADDR_WIDTH-1:0]  icache_addr,
    input  wire                     icache_ready,
    input  wire                     icache_resp_valid,
    input  wire [63:0]              icache_resp_data,
    input  wire                     icache_resp_error,
    
    // =========================================================================
    // Interface de ITLB
    // =========================================================================
    output logic                    itlb_req,
    output logic [VPN_WIDTH-1:0]    itlb_vpn,
    input  wire                     itlb_hit,
    input  wire [PPN_WIDTH-1:0]     itlb_ppn,
    input  wire                     itlb_page_fault,
    
    // =========================================================================
    // Controle de PTW
    // =========================================================================
    output logic                    ptw_req,
    output logic [VPN_WIDTH-1:0]    ptw_vpn,
    output logic                    ptw_for_fetch,
    input  wire                     ptw_ready,
    input  wire                     ptw_resp_valid,
    input  wire                     ptw_page_fault,
    
    // =========================================================================
    // CSRs e Privilégio
    // =========================================================================
    input  wire [XLEN-1:0]          csr_satp,
    input  wire [1:0]               current_priv,
    input  wire                     mmu_enabled,
    
    // =========================================================================
    // Trap Output
    // =========================================================================
    output logic                    fetch_exception,
    output exception_cause_t        fetch_exception_cause,
    output logic [XLEN-1:0]         fetch_exception_value
);

    // =========================================================================
    // Estado do Frontend
    // =========================================================================
    
    typedef enum logic [3:0] {
        F_RESET,
        F_IDLE,
        F_TLB_LOOKUP,
        F_TLB_WAIT,
        F_ICACHE_REQ,
        F_ICACHE_WAIT,
        F_DECODE,
        F_STALL,
        F_EXCEPTION
    } frontend_state_t;
    
    frontend_state_t state, next_state;
    
    // =========================================================================
    // Registradores
    // =========================================================================
    
    logic [VADDR_WIDTH-1:0] pc_reg, next_pc;
    logic [VADDR_WIDTH-1:0] pc_fetch;           // PC sendo buscado
    logic [PADDR_WIDTH-1:0] paddr_reg;          // Endereço físico traduzido
    logic [63:0]            fetch_data;         // Dados do I-Cache (2 instruções)
    logic                   fetch_data_valid;
    
    decoded_instr_t         decode_buf0, decode_buf1;
    logic                   decode_buf_valid;
    
    // Reset vector
    localparam logic [VADDR_WIDTH-1:0] RESET_VECTOR = 39'h00_8000_0000;
    
    // =========================================================================
    // Extração de Campos de Endereço
    // =========================================================================
    
    wire [VPN_WIDTH-1:0]    pc_vpn = pc_reg[VPN_WIDTH+11:12];
    wire [11:0]             pc_offset = pc_reg[11:0];
    wire [PADDR_WIDTH-1:0]  pc_paddr = mmu_enabled ? 
                                       {itlb_ppn, pc_offset} : 
                                       {{(PADDR_WIDTH-VADDR_WIDTH){1'b0}}, pc_reg};
    
    // =========================================================================
    // Função de Decodificação
    // =========================================================================
    
    function automatic decoded_instr_t decode_instruction(
        input [31:0] instr,
        input [VADDR_WIDTH-1:0] pc_val
    );
        decoded_instr_t d;
        
        // Campos básicos
        d.opcode = instr[6:0];
        d.rd = instr[11:7];
        d.funct3 = instr[14:12];
        d.rs1 = instr[19:15];
        d.rs2 = instr[24:20];
        d.funct7 = instr[31:25];
        d.csr_addr = instr[31:20];
        d.pc = pc_val;
        
        // Defaults
        d.valid = 1'b1;
        d.is_alu = 1'b0;
        d.is_alu_w = 1'b0;
        d.is_branch = 1'b0;
        d.is_jal = 1'b0;
        d.is_jalr = 1'b0;
        d.is_load = 1'b0;
        d.is_store = 1'b0;
        d.is_amo = 1'b0;
        d.is_lr = 1'b0;
        d.is_sc = 1'b0;
        d.is_mdu = 1'b0;
        d.is_fpu = 1'b0;
        d.is_csr = 1'b0;
        d.is_fence = 1'b0;
        d.is_fence_i = 1'b0;
        d.is_sfence_vma = 1'b0;
        d.is_ecall = 1'b0;
        d.is_ebreak = 1'b0;
        d.is_mret = 1'b0;
        d.is_sret = 1'b0;
        d.is_wfi = 1'b0;
        d.imm = '0;
        
        case (d.opcode)
            7'b0110111: begin // LUI
                d.is_alu = 1'b1;
                d.imm = {{32{instr[31]}}, instr[31:12], 12'b0};
            end
            
            7'b0010111: begin // AUIPC
                d.is_alu = 1'b1;
                d.imm = {{32{instr[31]}}, instr[31:12], 12'b0};
            end
            
            7'b1101111: begin // JAL
                d.is_branch = 1'b1;
                d.is_jal = 1'b1;
                d.imm = {{43{instr[31]}}, instr[31], instr[19:12], instr[20], instr[30:21], 1'b0};
            end
            
            7'b1100111: begin // JALR
                d.is_branch = 1'b1;
                d.is_jalr = 1'b1;
                d.imm = {{52{instr[31]}}, instr[31:20]};
            end
            
            7'b1100011: begin // Branch
                d.is_branch = 1'b1;
                d.imm = {{51{instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0};
            end
            
            7'b0000011: begin // Load
                d.is_load = 1'b1;
                d.imm = {{52{instr[31]}}, instr[31:20]};
            end
            
            7'b0100011: begin // Store
                d.is_store = 1'b1;
                d.imm = {{52{instr[31]}}, instr[31:25], instr[11:7]};
            end
            
            7'b0010011: begin // ALU-I
                d.is_alu = 1'b1;
                d.imm = {{52{instr[31]}}, instr[31:20]};
            end
            
            7'b0110011: begin // ALU-R / M-ext
                if (d.funct7 == 7'b0000001)
                    d.is_mdu = 1'b1;
                else
                    d.is_alu = 1'b1;
            end
            
            7'b0011011: begin // ALU-I-W
                d.is_alu = 1'b1;
                d.is_alu_w = 1'b1;
                d.imm = {{52{instr[31]}}, instr[31:20]};
            end
            
            7'b0111011: begin // ALU-R-W / M-ext-W
                if (d.funct7 == 7'b0000001)
                    d.is_mdu = 1'b1;
                else begin
                    d.is_alu = 1'b1;
                    d.is_alu_w = 1'b1;
                end
            end
            
            7'b0101111: begin // AMO
                d.is_amo = 1'b1;
                d.is_lr = (d.funct7[6:2] == 5'b00010);
                d.is_sc = (d.funct7[6:2] == 5'b00011);
            end
            
            7'b0001111: begin // FENCE
                d.is_fence = 1'b1;
                d.is_fence_i = (d.funct3 == 3'b001);
            end
            
            7'b1110011: begin // SYSTEM
                if (d.funct3 == 3'b000) begin
                    case (instr[31:20])
                        12'h000: d.is_ecall = 1'b1;
                        12'h001: d.is_ebreak = 1'b1;
                        12'h302: d.is_mret = 1'b1;
                        12'h102: d.is_sret = 1'b1;
                        12'h105: d.is_wfi = 1'b1;
                        default: begin
                            if (d.funct7 == 7'b0001001)
                                d.is_sfence_vma = 1'b1;
                        end
                    endcase
                end else begin
                    d.is_csr = 1'b1;
                    d.imm = {{59{1'b0}}, instr[19:15]};
                end
            end
            
            7'b0000111, 7'b0100111: begin // FP Load/Store
                d.is_load = (d.opcode == 7'b0000111);
                d.is_store = (d.opcode == 7'b0100111);
                d.is_fpu = 1'b1;
                d.imm = (d.opcode == 7'b0000111) ? 
                        {{52{instr[31]}}, instr[31:20]} :
                        {{52{instr[31]}}, instr[31:25], instr[11:7]};
            end
            
            7'b1010011, 7'b1000011, 7'b1000111, 7'b1001011, 7'b1001111: begin
                d.is_fpu = 1'b1;
            end
            
            default: d.valid = 1'b0;
        endcase
        
        return d;
    endfunction
    
    // =========================================================================
    // Verificação de Dual Issue
    // =========================================================================
    
    function automatic logic can_dual_issue(
        input decoded_instr_t i0,
        input decoded_instr_t i1
    );
        if (!i0.valid || !i1.valid)
            return 1'b0;
        
        // RAW dependency
        if (i0.rd != 0 && (i0.rd == i1.rs1 || i0.rd == i1.rs2))
            return 1'b0;
        
        // Structural hazards - unidades compartilhadas
        if ((i0.is_mdu || i0.is_fpu || i0.is_amo || i0.is_csr) &&
            (i1.is_mdu || i1.is_fpu || i1.is_amo || i1.is_csr))
            return 1'b0;
        
        // Single memory port
        if ((i0.is_load || i0.is_store || i0.is_amo) &&
            (i1.is_load || i1.is_store || i1.is_amo))
            return 1'b0;
        
        // No dual branch
        if (i0.is_branch && i1.is_branch)
            return 1'b0;
        
        // System instructions serialize
        if (i0.is_fence || i0.is_fence_i || i0.is_sfence_vma ||
            i0.is_ecall || i0.is_ebreak || i0.is_mret || i0.is_sret || i0.is_wfi)
            return 1'b0;
        
        return 1'b1;
    endfunction
    
    // =========================================================================
    // FSM - Próximo Estado
    // =========================================================================
    
    always_comb begin
        next_state = state;
        
        case (state)
            F_RESET: begin
                if (rst_n)
                    next_state = F_IDLE;
            end
            
            F_IDLE: begin
                if (backend_flush)
                    next_state = F_IDLE;
                else if (!backend_stall)
                    next_state = mmu_enabled ? F_TLB_LOOKUP : F_ICACHE_REQ;
            end
            
            F_TLB_LOOKUP: begin
                if (itlb_page_fault)
                    next_state = F_EXCEPTION;
                else if (itlb_hit)
                    next_state = F_ICACHE_REQ;
                else if (ptw_ready)
                    next_state = F_TLB_WAIT;
            end
            
            F_TLB_WAIT: begin
                if (ptw_resp_valid) begin
                    if (ptw_page_fault)
                        next_state = F_EXCEPTION;
                    else
                        next_state = F_TLB_LOOKUP; // Retry TLB
                end
            end
            
            F_ICACHE_REQ: begin
                if (icache_ready)
                    next_state = F_ICACHE_WAIT;
            end
            
            F_ICACHE_WAIT: begin
                if (icache_resp_valid) begin
                    if (icache_resp_error)
                        next_state = F_EXCEPTION;
                    else
                        next_state = F_DECODE;
                end
            end
            
            F_DECODE: begin
                if (backend_flush)
                    next_state = F_IDLE;
                else if (backend_stall)
                    next_state = F_STALL;
                else
                    next_state = mmu_enabled ? F_TLB_LOOKUP : F_ICACHE_REQ;
            end
            
            F_STALL: begin
                if (backend_flush)
                    next_state = F_IDLE;
                else if (!backend_stall)
                    next_state = mmu_enabled ? F_TLB_LOOKUP : F_ICACHE_REQ;
            end
            
            F_EXCEPTION: begin
                // Wait for backend to handle
                if (backend_flush)
                    next_state = F_IDLE;
            end
            
            default: next_state = F_RESET;
        endcase
    end
    
    // =========================================================================
    // Cálculo de Próximo PC
    // =========================================================================
    
    always_comb begin
        next_pc = pc_reg;
        
        if (backend_redirect)
            next_pc = backend_redirect_pc;
        else if (state == F_DECODE && !backend_stall && !backend_flush) begin
            // Avançar PC
            if (decode_buf0.valid && decode_buf1.valid && 
                can_dual_issue(decode_buf0, decode_buf1))
                next_pc = pc_reg + 8;
            else
                next_pc = pc_reg + 4;
        end
    end
    
    // =========================================================================
    // Saídas de Interface
    // =========================================================================
    
    // ITLB
    assign itlb_req = (state == F_TLB_LOOKUP);
    assign itlb_vpn = pc_vpn;
    
    // PTW
    assign ptw_req = (state == F_TLB_LOOKUP) && !itlb_hit && !itlb_page_fault;
    assign ptw_vpn = pc_vpn;
    assign ptw_for_fetch = 1'b1;
    
    // I-Cache
    assign icache_req = (state == F_ICACHE_REQ);
    assign icache_addr = paddr_reg;
    
    // Frontend output
    always_comb begin
        frontend_out = '0;
        frontend_valid = 1'b0;
        
        if ((state == F_DECODE || state == F_STALL) && decode_buf_valid && !backend_flush) begin
            frontend_valid = 1'b1;
            frontend_out.instr0 = decode_buf0;
            frontend_out.instr1 = decode_buf1;
            frontend_out.instr0_valid = decode_buf0.valid;
            frontend_out.instr1_valid = decode_buf1.valid;
            frontend_out.dual_issue = can_dual_issue(decode_buf0, decode_buf1);
        end
    end
    
    // Exception output
    always_comb begin
        fetch_exception = 1'b0;
        fetch_exception_cause = EXC_INSTR_ACCESS_FAULT;
        fetch_exception_value = '0;
        
        if (state == F_EXCEPTION) begin
            fetch_exception = 1'b1;
            fetch_exception_value = {{(XLEN-VADDR_WIDTH){pc_fetch[VADDR_WIDTH-1]}}, pc_fetch};
            
            if (itlb_page_fault || ptw_page_fault)
                fetch_exception_cause = EXC_INSTR_PAGE_FAULT;
            else
                fetch_exception_cause = EXC_INSTR_ACCESS_FAULT;
        end
    end
    
    // =========================================================================
    // Lógica Sequencial
    // =========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= F_RESET;
            pc_reg <= RESET_VECTOR;
            pc_fetch <= '0;
            paddr_reg <= '0;
            fetch_data <= '0;
            fetch_data_valid <= 1'b0;
            decode_buf0 <= '0;
            decode_buf1 <= '0;
            decode_buf_valid <= 1'b0;
        end
        else begin
            state <= next_state;
            
            // PC update
            if (backend_redirect)
                pc_reg <= backend_redirect_pc;
            else if (state == F_DECODE && !backend_stall && !backend_flush)
                pc_reg <= next_pc;
            
            case (state)
                F_IDLE, F_STALL: begin
                    if (backend_flush) begin
                        decode_buf_valid <= 1'b0;
                        fetch_data_valid <= 1'b0;
                    end
                end
                
                F_TLB_LOOKUP: begin
                    pc_fetch <= pc_reg;
                    if (itlb_hit)
                        paddr_reg <= pc_paddr;
                end
                
                F_ICACHE_REQ: begin
                    if (!mmu_enabled)
                        paddr_reg <= pc_paddr;
                end
                
                F_ICACHE_WAIT: begin
                    if (icache_resp_valid && !icache_resp_error) begin
                        fetch_data <= icache_resp_data;
                        fetch_data_valid <= 1'b1;
                    end
                end
                
                F_DECODE: begin
                    if (fetch_data_valid) begin
                        decode_buf0 <= decode_instruction(fetch_data[31:0], pc_fetch);
                        decode_buf1 <= decode_instruction(fetch_data[63:32], pc_fetch + 4);
                        decode_buf_valid <= 1'b1;
                        fetch_data_valid <= 1'b0;
                    end
                    
                    if (backend_flush) begin
                        decode_buf_valid <= 1'b0;
                    end
                end
                
                F_EXCEPTION: begin
                    decode_buf_valid <= 1'b0;
                end
                
                default: ;
            endcase
        end
    end

endmodule
