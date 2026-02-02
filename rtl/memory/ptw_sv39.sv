`timescale 1ns/1ps
`default_nettype none

/**
 * @module ptw_sv39
 * @brief Page Table Walker para Sv39 com suporte completo
 *
 * @details Implementa o page walk de 3 níveis para Sv39:
 * - Suporte a páginas de 4KB, 2MB (megapage) e 1GB (gigapage)
 * - Extração completa de bits de permissão (R/W/X/U/G/A/D)
 * - Verificação de PTE válida e misaligned superpage
 * - Interface separada para ITLB e DTLB
 */
module ptw_sv39 #(
    parameter int XLEN = 64,
    parameter int PADDR_WIDTH = 56,
    parameter int VPN_WIDTH = 27,
    parameter int PPN_WIDTH = 44,
    parameter int ASID_WIDTH = 16
)(
    input  wire                     clk,
    input  wire                     rst_n,
    
    // =========================================================================
    // Interface de Requisição
    // =========================================================================
    input  wire                     ptw_req_valid,
    input  wire [VPN_WIDTH-1:0]     ptw_req_vpn,
    input  wire [ASID_WIDTH-1:0]    ptw_req_asid,
    input  wire                     ptw_req_is_store,
    input  wire                     ptw_req_is_exec,
    input  wire                     ptw_req_for_itlb,
    output logic                    ptw_req_ready,
    
    // SATP register
    input  wire [XLEN-1:0]          satp,
    
    // =========================================================================
    // Interface de Memória
    // =========================================================================
    output logic                    ptw_mem_req,
    output logic [PADDR_WIDTH-1:0]  ptw_mem_addr,
    input  wire                     ptw_mem_resp_valid,
    input  wire [63:0]              ptw_mem_resp_data,
    input  wire                     ptw_mem_resp_err,
    
    // =========================================================================
    // Interface de Resposta
    // =========================================================================
    output logic                    ptw_resp_valid,
    output logic [VPN_WIDTH-1:0]    ptw_resp_vpn,
    output logic [PPN_WIDTH-1:0]    ptw_resp_ppn,
    output logic [ASID_WIDTH-1:0]   ptw_resp_asid,
    output logic [1:0]              ptw_resp_page_size,
    output logic                    ptw_resp_page_fault,
    output logic                    ptw_resp_access_fault,
    output logic                    ptw_resp_for_itlb,
    
    // Bits de permissão extraídos
    output logic                    ptw_resp_r,
    output logic                    ptw_resp_w,
    output logic                    ptw_resp_x,
    output logic                    ptw_resp_u,
    output logic                    ptw_resp_g,
    output logic                    ptw_resp_a,
    output logic                    ptw_resp_d
);

    // =========================================================================
    // Constantes
    // =========================================================================
    
    localparam int LEVELS = 3;
    localparam int PAGE_OFFSET_BITS = 12;
    localparam int PTE_SIZE_BITS = 3;
    localparam int VPN_BITS_PER_LEVEL = 9;
    
    // SATP modes
    localparam int SATP_MODE_BARE = 0;
    localparam int SATP_MODE_SV39 = 8;
    localparam int SATP_MODE_SV48 = 9;
    
    // PTE bit positions
    localparam int PTE_V = 0;
    localparam int PTE_R = 1;
    localparam int PTE_W = 2;
    localparam int PTE_X = 3;
    localparam int PTE_U = 4;
    localparam int PTE_G = 5;
    localparam int PTE_A = 6;
    localparam int PTE_D = 7;
    
    // =========================================================================
    // FSM States
    // =========================================================================
    
    typedef enum logic [2:0] {
        S_IDLE,
        S_LEVEL2,
        S_LEVEL1,
        S_LEVEL0,
        S_WAIT_MEM,
        S_CHECK_PTE,
        S_RESP
    } state_t;
    
    state_t state, next_state;
    state_t return_state;
    
    // =========================================================================
    // Registradores Internos
    // =========================================================================
    
    logic [VPN_WIDTH-1:0]   vpn_reg;
    logic [ASID_WIDTH-1:0]  asid_reg;
    logic                   is_store_reg;
    logic                   is_exec_reg;
    logic                   for_itlb_reg;
    logic [1:0]             current_level;
    logic [63:0]            pte_reg;
    logic [PPN_WIDTH-1:0]   current_ppn;
    logic                   page_fault;
    logic                   access_fault;
    
    // =========================================================================
    // SATP Parsing
    // =========================================================================
    
    wire [3:0]  satp_mode = satp[63:60];
    wire [15:0] satp_asid_field = satp[59:44];
    wire [43:0] satp_ppn  = satp[43:0];
    
    // =========================================================================
    // VPN Extraction (Sv39: cada nível tem 9 bits)
    // =========================================================================
    
    wire [8:0] vpn2 = vpn_reg[VPN_WIDTH-1:18];
    wire [8:0] vpn1 = vpn_reg[17:9];
    wire [8:0] vpn0 = vpn_reg[8:0];
    
    // =========================================================================
    // PTE Parsing
    // =========================================================================
    
    wire pte_v = pte_reg[PTE_V];
    wire pte_r = pte_reg[PTE_R];
    wire pte_w = pte_reg[PTE_W];
    wire pte_x = pte_reg[PTE_X];
    wire pte_u = pte_reg[PTE_U];
    wire pte_g = pte_reg[PTE_G];
    wire pte_a = pte_reg[PTE_A];
    wire pte_d = pte_reg[PTE_D];
    wire [PPN_WIDTH-1:0] pte_ppn = pte_reg[53:10];
    
    wire pte_is_pointer = !pte_r && !pte_x;
    wire pte_is_leaf = pte_r || pte_x;
    
    wire superpage_misaligned = (current_level == 2 && pte_ppn[17:0] != '0) ||
                                 (current_level == 1 && pte_ppn[8:0] != '0);
    
    // =========================================================================
    // Cálculo de Endereço de PTE
    // =========================================================================
    
    function automatic [PADDR_WIDTH-1:0] make_pte_addr(
        input [PPN_WIDTH-1:0] ppn,
        input [8:0] vpn_index
    );
        return {{(PADDR_WIDTH-PPN_WIDTH-PAGE_OFFSET_BITS){1'b0}}, ppn, 12'b0} + 
               {{(PADDR_WIDTH-9-PTE_SIZE_BITS){1'b0}}, vpn_index, 3'b0};
    endfunction
    
    // =========================================================================
    // FSM - Próximo Estado
    // =========================================================================
    
    always_comb begin
        next_state = state;
        
        case (state)
            S_IDLE: begin
                if (ptw_req_valid && ptw_req_ready) begin
                    if (satp_mode == SATP_MODE_SV39)
                        next_state = S_LEVEL2;
                    else
                        next_state = S_RESP;
                end
            end
            
            S_LEVEL2, S_LEVEL1, S_LEVEL0: begin
                next_state = S_WAIT_MEM;
            end
            
            S_WAIT_MEM: begin
                if (ptw_mem_resp_valid)
                    next_state = S_CHECK_PTE;
                else if (ptw_mem_resp_err)
                    next_state = S_RESP;
            end
            
            S_CHECK_PTE: begin
                if (!pte_v || (!pte_r && pte_w)) begin
                    next_state = S_RESP;
                end
                else if (pte_is_pointer) begin
                    if (current_level == 0)
                        next_state = S_RESP;
                    else if (current_level == 2)
                        next_state = S_LEVEL1;
                    else
                        next_state = S_LEVEL0;
                end
                else begin
                    next_state = S_RESP;
                end
            end
            
            S_RESP: begin
                next_state = S_IDLE;
            end
            
            default: next_state = S_IDLE;
        endcase
    end
    
    // =========================================================================
    // Interface de Memória
    // =========================================================================
    
    always_comb begin
        ptw_mem_req = 1'b0;
        ptw_mem_addr = '0;
        
        case (state)
            S_LEVEL2: begin
                ptw_mem_req = 1'b1;
                ptw_mem_addr = make_pte_addr(satp_ppn, vpn2);
            end
            
            S_LEVEL1: begin
                ptw_mem_req = 1'b1;
                ptw_mem_addr = make_pte_addr(current_ppn, vpn1);
            end
            
            S_LEVEL0: begin
                ptw_mem_req = 1'b1;
                ptw_mem_addr = make_pte_addr(current_ppn, vpn0);
            end
            
            default: ;
        endcase
    end
    
    // =========================================================================
    // Lógica Sequencial
    // =========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            ptw_req_ready <= 1'b1;
            ptw_resp_valid <= 1'b0;
            ptw_resp_page_fault <= 1'b0;
            ptw_resp_access_fault <= 1'b0;
            vpn_reg <= '0;
            asid_reg <= '0;
            is_store_reg <= 1'b0;
            is_exec_reg <= 1'b0;
            for_itlb_reg <= 1'b0;
            current_level <= 2'd2;
            pte_reg <= '0;
            current_ppn <= '0;
            page_fault <= 1'b0;
            access_fault <= 1'b0;
            return_state <= S_IDLE;
            
            ptw_resp_vpn <= '0;
            ptw_resp_ppn <= '0;
            ptw_resp_asid <= '0;
            ptw_resp_page_size <= '0;
            ptw_resp_for_itlb <= 1'b0;
            ptw_resp_r <= 1'b0;
            ptw_resp_w <= 1'b0;
            ptw_resp_x <= 1'b0;
            ptw_resp_u <= 1'b0;
            ptw_resp_g <= 1'b0;
            ptw_resp_a <= 1'b0;
            ptw_resp_d <= 1'b0;
            
        end else begin
            state <= next_state;
            
            case (state)
                S_IDLE: begin
                    ptw_resp_valid <= 1'b0;
                    page_fault <= 1'b0;
                    access_fault <= 1'b0;
                    
                    if (ptw_req_valid && ptw_req_ready) begin
                        ptw_req_ready <= 1'b0;
                        vpn_reg <= ptw_req_vpn;
                        asid_reg <= ptw_req_asid;
                        is_store_reg <= ptw_req_is_store;
                        is_exec_reg <= ptw_req_is_exec;
                        for_itlb_reg <= ptw_req_for_itlb;
                        current_level <= 2'd2;
                    end
                end
                
                S_LEVEL2: begin
                    return_state <= S_LEVEL2;
                end
                
                S_LEVEL1: begin
                    return_state <= S_LEVEL1;
                end
                
                S_LEVEL0: begin
                    return_state <= S_LEVEL0;
                end
                
                S_WAIT_MEM: begin
                    if (ptw_mem_resp_valid) begin
                        pte_reg <= ptw_mem_resp_data;
                        current_ppn <= ptw_mem_resp_data[53:10];
                    end
                    else if (ptw_mem_resp_err) begin
                        access_fault <= 1'b1;
                    end
                end
                
                S_CHECK_PTE: begin
                    if (!pte_v) begin
                        page_fault <= 1'b1;
                    end
                    else if (!pte_r && pte_w) begin
                        page_fault <= 1'b1;
                    end
                    else if (pte_is_pointer && current_level == 0) begin
                        page_fault <= 1'b1;
                    end
                    else if (pte_is_leaf) begin
                        if (superpage_misaligned) begin
                            page_fault <= 1'b1;
                        end
                    end
                    
                    if (pte_is_pointer && current_level > 0) begin
                        current_level <= current_level - 1;
                    end
                end
                
                S_RESP: begin
                    ptw_resp_valid <= 1'b1;
                    ptw_req_ready <= 1'b1;
                    
                    ptw_resp_vpn <= vpn_reg;
                    ptw_resp_asid <= asid_reg;
                    ptw_resp_for_itlb <= for_itlb_reg;
                    ptw_resp_page_fault <= page_fault;
                    ptw_resp_access_fault <= access_fault;
                    
                    if (!page_fault && !access_fault) begin
                        ptw_resp_ppn <= pte_ppn;
                        ptw_resp_page_size <= current_level;
                        ptw_resp_r <= pte_r;
                        ptw_resp_w <= pte_w;
                        ptw_resp_x <= pte_x;
                        ptw_resp_u <= pte_u;
                        ptw_resp_g <= pte_g;
                        ptw_resp_a <= pte_a;
                        ptw_resp_d <= pte_d;
                    end else begin
                        ptw_resp_ppn <= '0;
                        ptw_resp_page_size <= '0;
                        ptw_resp_r <= 1'b0;
                        ptw_resp_w <= 1'b0;
                        ptw_resp_x <= 1'b0;
                        ptw_resp_u <= 1'b0;
                        ptw_resp_g <= 1'b0;
                        ptw_resp_a <= 1'b0;
                        ptw_resp_d <= 1'b0;
                    end
                end
                
                default: ;
            endcase
        end
    end

endmodule
