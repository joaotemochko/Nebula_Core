`timescale 1ns/1ps
`default_nettype none

/**
 * @module nebula_pmp
 * @brief Verificador PMP (Physical Memory Protection), 16 entradas,
 *        granularidade de 4 bytes (G=0), modos OFF/TOR/NA4/NAPOT e bit L.
 *
 * As regiões chegam pré-decodificadas como intervalos [lo, hi) (calculados
 * no módulo de CSRs quando pmpcfg/pmpaddr mudam), de modo que cada checagem
 * é apenas uma comparação por entrada.
 *
 * Verifica um acesso contido num doubleword alinhado: 'use_lo'/'use_hi'
 * indicam quais setores de 4 bytes (metade baixa/alta) são tocados.
 * Regras (Priv. Spec 3.7):
 *  - a entrada de menor índice que casa com QUALQUER byte decide; se ela
 *    não cobrir todos os bytes, o acesso falha;
 *  - em modo M, entradas sem L não se aplicam;
 *  - sem casamento: M permitido, S/U negado.
 */
module nebula_pmp #(
    parameter int ENTRIES = 16
)(
    input  wire  [ENTRIES-1:0][7:0]  cfg,
    input  wire  [ENTRIES-1:0][57:0] lo,
    input  wire  [ENTRIES-1:0][57:0] hi,
    input  wire                      any_locked,
    input  wire  [55:0]              addr,
    input  wire                      use_lo,      // toca bytes [3:0] do doubleword
    input  wire                      use_hi,      // toca bytes [7:4] do doubleword
    input  wire  [1:0]               acc,         // 0 load, 1 store, 2 fetch
    input  wire  [1:0]               priv,
    output logic                     ok
);
    always_comb begin
        logic [57:0] a0, a1;
        logic        found, m0, m1;
        logic [7:0]  c;
        ok = (priv == 2'b11);
        if (priv != 2'b11 || any_locked) begin
            a0 = {2'b00, addr[55:3], 3'b000};
            a1 = {2'b00, addr[55:3], 3'b100};
            found = 1'b0;
            for (int i = 0; i < ENTRIES; i++) begin
                if (!found && cfg[i][4:3] != 2'b00) begin
                    m0 = use_lo && (a0 >= lo[i]) && (a0 < hi[i]);
                    m1 = use_hi && (a1 >= lo[i]) && (a1 < hi[i]);
                    if (m0 || m1) begin
                        found = 1'b1;
                        c = cfg[i];
                        if ((use_lo && !m0) || (use_hi && !m1)) ok = 1'b0;
                        else if (priv == 2'b11 && !c[7]) ok = 1'b1;
                        else case (acc)
                            2'd0:    ok = c[0];
                            2'd1:    ok = c[1];
                            default: ok = c[2];
                        endcase
                    end
                end
            end
        end
    end
endmodule
