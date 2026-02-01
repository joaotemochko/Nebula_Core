`timescale 1ns/1ps
`default_nettype none

/**
 * @module tlb_sv39
 * @brief TLB (Translation Lookaside Buffer) para Sv39 com suporte completo a permissões
 *
 * @details TLB totalmente associativa com:
 * - Suporte a VPN de 27 bits (Sv39: 3 níveis x 9 bits)
 * - PPN de 44 bits
 * - ASID de 16 bits
 * - Bits de permissão: R, W, X, U, G, A, D
 * - Suporte a superpages (megapages e gigapages)
 * - Invalidação por ASID e por endereço (SFENCE.VMA)
 */
module tlb_sv39 #(
    parameter int VPN_WIDTH = 27,    // 39 - 12 = 27 bits de VPN
    parameter int PPN_WIDTH = 44,
    parameter int TLB_ENTRIES = 64,
    parameter int ASID_WIDTH = 16
)(
    input  wire clk,
    input  wire rst_n,
    
    // =========================================================================
    // Interface de Lookup
    // =========================================================================
    input  wire                     lookup_valid,
    input  wire [VPN_WIDTH-1:0]     lookup_vpn,
    input  wire [ASID_WIDTH-1:0]    lookup_asid,
    input  wire [1:0]               lookup_priv,      // Privilégio atual
    input  wire                     lookup_is_store,  // Para verificar W
    input  wire                     lookup_is_exec,   // Para verificar X
    input  wire                     mstatus_sum,      // Supervisor pode acessar User
    input  wire                     mstatus_mxr,      // Make eXecutable Readable
    
    output logic                    lookup_hit,
    output logic [PPN_WIDTH-1:0]    lookup_ppn,
    output logic                    lookup_page_fault,
    output logic                    access_fault,     // Violação de permissão
    
    // Bits de página para atualização A/D
    output logic                    need_set_a,       // Precisa setar bit A
    output logic                    need_set_d,       // Precisa setar bit D
    output logic [VPN_WIDTH-1:0]    fault_vpn,        // VPN que causou fault
    
    // =========================================================================
    // Interface de Insert (do PTW)
    // =========================================================================
    input  wire                     insert_valid,
    input  wire [VPN_WIDTH-1:0]     insert_vpn,
    input  wire [PPN_WIDTH-1:0]     insert_ppn,
    input  wire [ASID_WIDTH-1:0]    insert_asid,
    input  wire [1:0]               insert_page_size, // 0=4KB, 1=2MB, 2=1GB
    // Bits de permissão da PTE
    input  wire                     insert_r,
    input  wire                     insert_w,
    input  wire                     insert_x,
    input  wire                     insert_u,
    input  wire                     insert_g,
    input  wire                     insert_a,
    input  wire                     insert_d,
    
    // =========================================================================
    // Interface de Invalidação (SFENCE.VMA)
    // =========================================================================
    input  wire                     invalidate_all,       // rs1=x0, rs2=x0
    input  wire                     invalidate_by_asid,   // rs1=x0, rs2!=x0
    input  wire                     invalidate_by_addr,   // rs1!=x0, rs2=x0
    input  wire                     invalidate_by_both,   // rs1!=x0, rs2!=x0
    input  wire [ASID_WIDTH-1:0]    invalidate_asid,
    input  wire [VPN_WIDTH-1:0]     invalidate_vpn
);

    // Privilege levels
    localparam PRIV_U = 2'b00;
    localparam PRIV_S = 2'b01;
    localparam PRIV_M = 2'b11;

    // Estrutura de entrada TLB
    typedef struct packed {
        logic                   valid;
        logic [VPN_WIDTH-1:0]   vpn;
        logic [PPN_WIDTH-1:0]   ppn;
        logic [ASID_WIDTH-1:0]  asid;
        logic [1:0]             page_size;  // 0=4KB, 1=2MB (megapage), 2=1GB (gigapage)
        // Bits de permissão
        logic                   r;          // Readable
        logic                   w;          // Writable
        logic                   x;          // Executable
        logic                   u;          // User-mode accessible
        logic                   g;          // Global (ignora ASID)
        logic                   a;          // Accessed
        logic                   d;          // Dirty
    } tlb_entry_t;

    tlb_entry_t entries [0:TLB_ENTRIES-1];
    
    // Ponteiro de substituição (pseudo-LRU simplificado como round-robin)
    logic [$clog2(TLB_ENTRIES)-1:0] replace_ptr;
    
    // Máscara de VPN baseada no tamanho da página
    // Sv39: VPN[2] = bits 26:18, VPN[1] = bits 17:9, VPN[0] = bits 8:0
    function automatic logic vpn_match(
        input [VPN_WIDTH-1:0] vpn1,
        input [VPN_WIDTH-1:0] vpn2,
        input [1:0] page_size
    );
        case (page_size)
            2'b00: // 4KB - compara todos os 27 bits
                return (vpn1 == vpn2);
            2'b01: // 2MB (megapage) - ignora VPN[0] (9 bits)
                return (vpn1[VPN_WIDTH-1:9] == vpn2[VPN_WIDTH-1:9]);
            2'b10: // 1GB (gigapage) - ignora VPN[0] e VPN[1] (18 bits)
                return (vpn1[VPN_WIDTH-1:18] == vpn2[VPN_WIDTH-1:18]);
            default:
                return (vpn1 == vpn2);
        endcase
    endfunction
    
    // Construir PPN final para superpages
    function automatic [PPN_WIDTH-1:0] build_ppn(
        input [PPN_WIDTH-1:0] entry_ppn,
        input [VPN_WIDTH-1:0] vpn,
        input [1:0] page_size
    );
        case (page_size)
            2'b00: // 4KB
                return entry_ppn;
            2'b01: // 2MB - PPN[8:0] vem do VPN[0]
                return {entry_ppn[PPN_WIDTH-1:9], vpn[8:0]};
            2'b10: // 1GB - PPN[17:0] vem do VPN[1:0]
                return {entry_ppn[PPN_WIDTH-1:18], vpn[17:0]};
            default:
                return entry_ppn;
        endcase
    endfunction

    // =========================================================================
    // Lógica de Lookup (Combinacional)
    // =========================================================================
    
    logic found;
    logic [$clog2(TLB_ENTRIES)-1:0] found_idx;
    tlb_entry_t found_entry;
    logic perm_ok;
    logic need_a_update, need_d_update;
    
    always_comb begin
        lookup_hit = 1'b0;
        lookup_ppn = '0;
        lookup_page_fault = 1'b0;
        access_fault = 1'b0;
        need_set_a = 1'b0;
        need_set_d = 1'b0;
        fault_vpn = lookup_vpn;
        found = 1'b0;
        found_idx = '0;
        found_entry = '0;
        perm_ok = 1'b0;
        need_a_update = 1'b0;
        need_d_update = 1'b0;
        
        if (lookup_valid) begin
            // Busca na TLB
            for (int i = 0; i < TLB_ENTRIES; i++) begin
                if (entries[i].valid &&
                    vpn_match(entries[i].vpn, lookup_vpn, entries[i].page_size) &&
                    (entries[i].g || entries[i].asid == lookup_asid)) begin
                    
                    found = 1'b1;
                    found_idx = i;
                    found_entry = entries[i];
                end
            end
            
            if (found) begin
                lookup_hit = 1'b1;
                lookup_ppn = build_ppn(found_entry.ppn, lookup_vpn, found_entry.page_size);
                
                // ============================================================
                // Verificação de Permissões
                // ============================================================
                
                // Verificar acesso U-mode
                if (lookup_priv == PRIV_U && !found_entry.u) begin
                    // User tentando acessar página não-User
                    access_fault = 1'b1;
                end
                // Verificar acesso S-mode a página User
                else if (lookup_priv == PRIV_S && found_entry.u && !mstatus_sum) begin
                    // Supervisor tentando acessar página User sem SUM
                    access_fault = 1'b1;
                end
                // Verificar permissão de escrita
                else if (lookup_is_store && !found_entry.w) begin
                    access_fault = 1'b1;
                end
                // Verificar permissão de execução
                else if (lookup_is_exec && !found_entry.x) begin
                    access_fault = 1'b1;
                end
                // Verificar permissão de leitura
                else if (!lookup_is_store && !lookup_is_exec) begin
                    // Load: precisa R, ou (X com MXR)
                    if (!found_entry.r && !(found_entry.x && mstatus_mxr)) begin
                        access_fault = 1'b1;
                    end
                end
                
                // ============================================================
                // Verificar bits A/D
                // ============================================================
                
                if (!access_fault) begin
                    // Bit A (Accessed) deve estar setado
                    if (!found_entry.a) begin
                        need_set_a = 1'b1;
                        lookup_page_fault = 1'b1; // Gera page fault para software setar A
                    end
                    
                    // Bit D (Dirty) deve estar setado para stores
                    if (lookup_is_store && !found_entry.d) begin
                        need_set_d = 1'b1;
                        lookup_page_fault = 1'b1; // Gera page fault para software setar D
                    end
                end
                
                // Se houve violação de permissão, é page fault
                if (access_fault) begin
                    lookup_page_fault = 1'b1;
                end
                
            end else begin
                // TLB miss - não é fault, precisa page walk
                lookup_hit = 1'b0;
            end
        end
    end

    // =========================================================================
    // Lógica Sequencial
    // =========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < TLB_ENTRIES; i++) begin
                entries[i].valid <= 1'b0;
            end
            replace_ptr <= '0;
        end else begin
            // =================================================================
            // Invalidação
            // =================================================================
            
            if (invalidate_all) begin
                // SFENCE.VMA x0, x0 - Invalida tudo
                for (int i = 0; i < TLB_ENTRIES; i++) begin
                    entries[i].valid <= 1'b0;
                end
            end
            else if (invalidate_by_asid) begin
                // SFENCE.VMA x0, rs2 - Invalida por ASID
                for (int i = 0; i < TLB_ENTRIES; i++) begin
                    if (entries[i].valid && 
                        !entries[i].g &&  // Global pages não são afetadas
                        entries[i].asid == invalidate_asid) begin
                        entries[i].valid <= 1'b0;
                    end
                end
            end
            else if (invalidate_by_addr) begin
                // SFENCE.VMA rs1, x0 - Invalida por endereço (todos ASIDs)
                for (int i = 0; i < TLB_ENTRIES; i++) begin
                    if (entries[i].valid &&
                        vpn_match(entries[i].vpn, invalidate_vpn, entries[i].page_size)) begin
                        entries[i].valid <= 1'b0;
                    end
                end
            end
            else if (invalidate_by_both) begin
                // SFENCE.VMA rs1, rs2 - Invalida por endereço e ASID
                for (int i = 0; i < TLB_ENTRIES; i++) begin
                    if (entries[i].valid &&
                        !entries[i].g &&
                        entries[i].asid == invalidate_asid &&
                        vpn_match(entries[i].vpn, invalidate_vpn, entries[i].page_size)) begin
                        entries[i].valid <= 1'b0;
                    end
                end
            end
            
            // =================================================================
            // Inserção
            // =================================================================
            
            else if (insert_valid) begin
                entries[replace_ptr].valid     <= 1'b1;
                entries[replace_ptr].vpn       <= insert_vpn;
                entries[replace_ptr].ppn       <= insert_ppn;
                entries[replace_ptr].asid      <= insert_asid;
                entries[replace_ptr].page_size <= insert_page_size;
                entries[replace_ptr].r         <= insert_r;
                entries[replace_ptr].w         <= insert_w;
                entries[replace_ptr].x         <= insert_x;
                entries[replace_ptr].u         <= insert_u;
                entries[replace_ptr].g         <= insert_g;
                entries[replace_ptr].a         <= insert_a;
                entries[replace_ptr].d         <= insert_d;
                
                // Avançar ponteiro de substituição
                if (replace_ptr == TLB_ENTRIES - 1)
                    replace_ptr <= '0;
                else
                    replace_ptr <= replace_ptr + 1;
            end
        end
    end

endmodule
