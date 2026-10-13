`timescale 1ns/1ps
`default_nettype none

/**
 * @module nebula_tlb
 * @brief TLB Sv39 totalmente associativa com superpáginas (2M/1G) e ASIDs.
 *
 * Lookup combinacional: casa VPN (conforme o nível) e (G=1 ou ASID igual).
 * Somente PTEs folha válidas são inseridas. A checagem de permissões
 * (R/W/X/U/A/D, SUM, MXR) é feita pelo usuário.
 *
 * SFENCE.VMA seletivo:
 *   flush_va  : invalida entradas que traduzem o VPN dado
 *   flush_asid: restringe à ASID dada e preserva entradas globais
 */
module nebula_tlb #(
    parameter int ENTRIES = 16
)(
    input  wire          clk,
    input  wire          rst_n,

    // lookup
    input  wire  [26:0]  lookup_vpn,
    input  wire  [15:0]  lookup_asid,
    output logic         lookup_hit,
    output logic [43:0]  lookup_ppn,     // PPN já combinado para superpáginas
    output logic [7:0]   lookup_flags,   // D A G U X W R V
    output logic [$clog2(ENTRIES)-1:0] lookup_idx,

    // inserção (resposta do PTW)
    input  wire          insert_valid,
    input  wire  [26:0]  insert_vpn,
    input  wire  [15:0]  insert_asid,
    input  wire  [43:0]  insert_ppn,
    input  wire  [1:0]   insert_level,   // 0=4K 1=2M 2=1G
    input  wire  [7:0]   insert_flags,

    // invalidação
    input  wire          flush,          // SFENCE.VMA (seletivo conforme abaixo)
    input  wire          flush_va,       // rs1 != x0
    input  wire  [26:0]  flush_vpn,
    input  wire          flush_asid,     // rs2 != x0
    input  wire  [15:0]  flush_asid_val,
    input  wire          inval_valid,    // invalida a entrada que gerou fault
    input  wire  [$clog2(ENTRIES)-1:0] inval_idx
);
    localparam int IW = $clog2(ENTRIES);

    logic [ENTRIES-1:0] valid;
    logic [26:0]        vpn   [ENTRIES];
    logic [15:0]        asid  [ENTRIES];
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
            if (valid[i] && match(lookup_vpn, vpn[i], level[i]) &&
                (flags[i][5] || asid[i] == lookup_asid)) begin
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
            if (flush) begin
                for (int i = 0; i < ENTRIES; i++) begin
                    logic hit_va, hit_asid;
                    hit_va   = !flush_va || match(flush_vpn, vpn[i], level[i]);
                    hit_asid = !flush_asid || (!flags[i][5] && asid[i] == flush_asid_val);
                    if (hit_va && hit_asid) valid[i] <= 1'b0;
                end
            end else begin
                if (inval_valid)
                    valid[inval_idx] <= 1'b0;
                if (insert_valid) begin
                    valid[repl_ptr] <= 1'b1;
                    vpn[repl_ptr]   <= insert_vpn;
                    asid[repl_ptr]  <= insert_asid;
                    ppn[repl_ptr]   <= insert_ppn;
                    level[repl_ptr] <= insert_level;
                    flags[repl_ptr] <= insert_flags;
                    repl_ptr        <= repl_ptr + 1'b1;
                end
            end
        end
    end

endmodule
