`timescale 1ns/1ps
`default_nettype none

/**
 * @module nebula_pma
 * @brief Physical Memory Attributes: classifica um endereço físico.
 *
 * Até N regiões {base, tamanho, atributos}. Atributos:
 *   bit 0 = acessível, bit 1 = cacheável (memória), bit 2 = executável.
 * Endereços fora de todas as regiões geram access fault precisa
 * (load/store/fetch/PTW), sem emitir transação no barramento.
 *
 * O mapa padrão corresponde ao SoC simulado (tb/sim/nebula_sim.cpp).
 */
module nebula_pma #(
    parameter int                  N    = 8,
    parameter logic [N-1:0][55:0]  BASE = '0,
    parameter logic [N-1:0][55:0]  SIZE = '0,
    parameter logic [N-1:0][2:0]   ATTR = '0
)(
    input  wire  [55:0] addr,
    output logic        ok,
    output logic        cacheable,
    output logic        exec
);
    always_comb begin
        ok = 1'b0; cacheable = 1'b0; exec = 1'b0;
        for (int i = 0; i < N; i++) begin
            if (ATTR[i][0] && addr >= BASE[i] && addr - BASE[i] < SIZE[i]) begin
                ok        = 1'b1;
                cacheable = ATTR[i][1];
                exec      = ATTR[i][2];
            end
        end
    end
endmodule
