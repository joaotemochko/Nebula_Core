`timescale 1ns/1ps
`default_nettype none

/**
 * @module nebula_icache
 * @brief L1 I-Cache VIPT, 32KB, 8 vias, linhas de 64B (64 conjuntos).
 *
 * 4KB por via => índice inteiramente dentro do offset de página (sem aliasing).
 * Leitura: índice/bloco derivados do PC registrado (modelo de SRAM síncrona
 * endereçada pelo next-PC), tag física comparada no mesmo ciclo.
 * Entrega um bloco alinhado de 16 bytes (8 halfwords) por ciclo.
 */
module nebula_icache #(
    parameter int WAYS = 8,
    parameter int SETS = 64
)(
    input  wire           clk,
    input  wire           rst_n,

    input  wire  [11:0]   rd_vaddr,     // offset de página (índice + bloco)
    input  wire  [43:0]   rd_ptag,      // PA[55:12]
    output logic          rd_hit,
    output logic [127:0]  rd_data,

    input  wire           refill_valid,
    input  wire  [55:0]   refill_paddr,
    input  wire  [511:0]  refill_data,

    input  wire           invalidate_all
);
    localparam int WW = $clog2(WAYS);

    logic [43:0]  tags  [SETS][WAYS];
    logic [511:0] data  [SETS][WAYS];
    logic [WAYS-1:0] valid [SETS];
    logic [WW-1:0]   victim [SETS];

    logic [5:0] idx;
    assign idx = rd_vaddr[11:6];

    always_comb begin
        rd_hit  = 1'b0;
        rd_data = '0;
        for (int w = 0; w < WAYS; w++) begin
            if (valid[idx][w] && tags[idx][w] == rd_ptag) begin
                rd_hit  = 1'b1;
                rd_data = data[idx][w][rd_vaddr[5:4]*128 +: 128];
            end
        end
    end

    logic [5:0] ridx;
    assign ridx = refill_paddr[11:6];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int s = 0; s < SETS; s++) begin
                valid[s]  <= '0;
                victim[s] <= '0;
            end
        end else begin
            if (invalidate_all) begin
                for (int s = 0; s < SETS; s++) valid[s] <= '0;
            end else if (refill_valid) begin
                valid[ridx][victim[ridx]] <= 1'b1;
                victim[ridx]              <= victim[ridx] + 1'b1;
            end
        end
    end

    always_ff @(posedge clk) begin
        if (refill_valid && !invalidate_all) begin
            tags[ridx][victim[ridx]] <= refill_paddr[55:12];
            data[ridx][victim[ridx]] <= refill_data;
        end
    end

endmodule
