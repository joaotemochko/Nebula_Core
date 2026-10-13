`timescale 1ns/1ps
`default_nettype none

/**
 * @module nebula_tlb
 * @brief TLB Sv39 totalmente associativa com suporte a superpáginas (2M/1G).
 *
 * Lookup combinacional. Somente PTEs folha válidas são inseridas (o PTW
 * nunca insere traduções que causariam page fault por PTE inválida).
 * A checagem de permissões (R/W/X/U/A/D, SUM, MXR) é feita pelo usuário.
 * ASIDs não são implementados (satp.ASID é somente-leitura zero), portanto
 * a TLB é inteiramente descartada em SFENCE.VMA / escrita de satp.
 */
module nebula_tlb #(
    parameter int ENTRIES = 16
)(
    input  wire          clk,
    input  wire          rst_n,

    // lookup
    input  wire  [26:0]  lookup_vpn,
    output logic         lookup_hit,
    output logic [43:0]  lookup_ppn,     // PPN já combinado para superpáginas
    output logic [7:0]   lookup_flags,   // D A G U X W R V
    output logic [$clog2(ENTRIES)-1:0] lookup_idx,

    // inserção (resposta do PTW)
    input  wire          insert_valid,
    input  wire  [26:0]  insert_vpn,
    input  wire  [43:0]  insert_ppn,
    input  wire  [1:0]   insert_level,   // 0=4K 1=2M 2=1G
    input  wire  [7:0]   insert_flags,

    // invalidação
    input  wire          flush_all,
    input  wire          inval_valid,    // invalida a entrada que gerou fault
    input  wire  [$clog2(ENTRIES)-1:0] inval_idx
);
    localparam int IW = $clog2(ENTRIES);

    logic [ENTRIES-1:0] valid;
    logic [26:0]        vpn   [ENTRIES];
    logic [43:0]        ppn   [ENTRIES];
    logic [1:0]         level [ENTRIES];
    logic [7:0]         flags [ENTRIES];
    logic [IW-1:0]      repl_ptr;

    function automatic logic match(input logic [26:0] a, input logic [26:0] b, input logic [1:0] lvl);
        case (lvl)
            2'd2:    return a[26:18] == b[26:18];
            2'd1:    return a[26:9]  == b[26:9];
            default: return a == b;
        endcase
    endfunction

    always_comb begin
        lookup_hit   = 1'b0;
        lookup_ppn   = '0;
        lookup_flags = '0;
        lookup_idx   = '0;
        for (int i = 0; i < ENTRIES; i++) begin
            if (valid[i] && match(lookup_vpn, vpn[i], level[i])) begin
                lookup_hit   = 1'b1;
                lookup_idx   = IW'(i);
                lookup_flags = flags[i];
                case (level[i])
                    2'd2:    lookup_ppn = {ppn[i][43:18], lookup_vpn[17:0]};
                    2'd1:    lookup_ppn = {ppn[i][43:9],  lookup_vpn[8:0]};
                    default: lookup_ppn = ppn[i];
                endcase
            end
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid    <= '0;
            repl_ptr <= '0;
        end else begin
            if (flush_all) begin
                valid <= '0;
            end else begin
                if (inval_valid)
                    valid[inval_idx] <= 1'b0;
                if (insert_valid) begin
                    valid[repl_ptr] <= 1'b1;
                    vpn[repl_ptr]   <= insert_vpn;
                    ppn[repl_ptr]   <= insert_ppn;
                    level[repl_ptr] <= insert_level;
                    flags[repl_ptr] <= insert_flags;
                    repl_ptr        <= repl_ptr + 1'b1;
                end
            end
        end
    end

endmodule
