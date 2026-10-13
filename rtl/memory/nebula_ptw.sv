`timescale 1ns/1ps
`default_nettype none

import nebula_pkg::*;

/**
 * @module nebula_ptw
 * @brief Page Table Walker Sv39 compartilhado entre ITLB e DTLB.
 *
 * - Prioridade para requisições da DTLB (instrução mais antiga).
 * - Comportamento Svade: PTE com A=0 (ou D=0 em escrita) gera page fault no
 *   lookup da TLB; o PTW não escreve na memória (ptw_mem_we = 0).
 * - Cada leitura de PTE só é emitida com o store buffer vazio, garantindo
 *   coerência com a D-Cache write-through.
 * - 'flush' durante um walk marca o resultado como descartável (não insere).
 */
module nebula_ptw #(
    parameter logic [PMA_N-1:0][55:0] PMA_BASE = PMA_DEF_BASE,
    parameter logic [PMA_N-1:0][55:0] PMA_SIZE = PMA_DEF_SIZE,
    parameter logic [PMA_N-1:0][2:0]  PMA_ATTR = PMA_DEF_ATTR
)(
    input  wire          clk,
    input  wire          rst_n,

    input  wire  [63:0]  satp,
    input  wire          sb_empty,
    input  wire          flush,
    input  wire  [15:0][7:0]  pmpcfg,
    input  wire  [15:0][57:0] pmp_lo,
    input  wire  [15:0][57:0] pmp_hi,
    input  wire               pmp_any_locked,

    input  wire          req_i,
    input  wire  [26:0]  vpn_i,
    input  wire          req_d,
    input  wire  [26:0]  vpn_d,

    output logic         busy,
    output logic         resp_valid,
    output logic         resp_for_d,
    output logic [26:0]  resp_vpn,
    output logic [15:0]  resp_asid,
    output logic         resp_page_fault,
    output logic         resp_access_fault,
    output logic         resp_insert,      // inserir na TLB correspondente
    output logic [43:0]  resp_ppn,
    output logic [1:0]   resp_level,
    output logic [7:0]   resp_flags,

    output logic         mem_req,
    output logic [55:0]  mem_addr,
    input  wire          mem_ack,
    input  wire  [63:0]  mem_data,
    input  wire          mem_error
);
    typedef enum logic [1:0] { P_IDLE, P_READ, P_RESP } pstate_t;
    pstate_t state;

    logic [1:0]  level;
    logic [26:0] vpn;
    logic [15:0] asid;
    logic        for_d;
    logic [55:0] pte_addr;
    logic        discard;
    logic [63:0] pte;

    // decodificação da PTE recebida
    logic        pte_v, pte_r, pte_w, pte_x, pte_leaf, pte_bad, misaligned;
    always_comb begin
        pte_v    = mem_data[0];
        pte_r    = mem_data[1];
        pte_w    = mem_data[2];
        pte_x    = mem_data[3];
        pte_leaf = pte_r || pte_x;
        // bits reservados / N / PBMT devem ser zero (Svnapot/Svpbmt ausentes)
        pte_bad  = !pte_v || (!pte_r && pte_w) || (mem_data[63:54] != 10'd0);
        misaligned = (level == 2'd2 && mem_data[27:10] != 18'd0) ||
                     (level == 2'd1 && mem_data[18:10] != 9'd0);
    end

    assign busy     = (state != P_IDLE);
    logic pte_ok, pte_mem;
    nebula_pma #(.N(PMA_N), .BASE(PMA_BASE), .SIZE(PMA_SIZE), .ATTR(PMA_ATTR)) u_pma (
        .addr(pte_addr), .ok(pte_ok), .cacheable(pte_mem), .exec()
    );
    logic pte_pmp;
    nebula_pmp u_pmp (.cfg(pmpcfg), .lo(pmp_lo), .hi(pmp_hi), .any_locked(pmp_any_locked), .addr(pte_addr), .use_lo(1'b1), .use_hi(1'b1),
                      .acc(2'd0), .priv(2'b01), .ok(pte_pmp));
    assign mem_req  = (state == P_READ) && sb_empty && pte_ok && pte_mem && pte_pmp;
    assign mem_addr = pte_addr;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state             <= P_IDLE;
            level             <= '0;
            vpn               <= '0;
            for_d             <= 1'b0;
            pte_addr          <= '0;
            discard           <= 1'b0;
            pte               <= '0;
            resp_valid        <= 1'b0;
            resp_for_d        <= 1'b0;
            resp_vpn          <= '0;
            resp_asid         <= '0;
            asid              <= '0;
            resp_page_fault   <= 1'b0;
            resp_access_fault <= 1'b0;
            resp_insert       <= 1'b0;
            resp_ppn          <= '0;
            resp_level        <= '0;
            resp_flags        <= '0;
        end else begin
            resp_valid  <= 1'b0;
            resp_insert <= 1'b0;
            if (flush && state != P_IDLE) discard <= 1'b1;

            case (state)
                P_IDLE: begin
                    discard <= 1'b0;
                    if (req_d || req_i) begin
                        for_d    <= req_d;
                        asid     <= satp[59:44];
                        vpn      <= req_d ? vpn_d : vpn_i;
                        level    <= 2'd2;
                        pte_addr <= {satp[43:0], (req_d ? vpn_d[26:18] : vpn_i[26:18]), 3'b000};
                        state    <= P_READ;
                    end
                end

                P_READ: begin
                    if (!(pte_ok && pte_mem && pte_pmp)) begin
                        // PTE fora da memória: access fault
                        resp_vpn          <= vpn;
                        resp_for_d        <= for_d;
                        resp_page_fault   <= 1'b0;
                        resp_access_fault <= 1'b1;
                        state             <= P_RESP;
                    end else if (mem_req && mem_ack) begin
                        pte <= mem_data;
                        resp_vpn   <= vpn;
                        resp_for_d <= for_d;
                        resp_page_fault   <= 1'b0;
                        resp_access_fault <= 1'b0;
                        if (mem_error) begin
                            resp_access_fault <= 1'b1;
                            state <= P_RESP;
                        end else if (pte_bad) begin
                            resp_page_fault <= 1'b1;
                            state <= P_RESP;
                        end else if (pte_leaf) begin
                            if (misaligned) begin
                                resp_page_fault <= 1'b1;
                            end else begin
                                resp_ppn   <= mem_data[53:10];
                                resp_level <= level;
                                resp_flags <= mem_data[7:0];
                            end
                            state <= P_RESP;
                        end else begin
                            // ponteiro para o próximo nível
                            if (level == 2'd0 || (mem_data[7:4] != 4'd0 && (mem_data[7] || mem_data[6] || mem_data[4]))) begin
                                resp_page_fault <= 1'b1;
                                state <= P_RESP;
                            end else begin
                                level <= level - 2'd1;
                                pte_addr <= {mem_data[53:10],
                                             (level == 2'd2) ? vpn[17:9] : vpn[8:0], 3'b000};
                            end
                        end
                    end
                end

                P_RESP: begin
                    resp_valid  <= 1'b1;
                    resp_asid   <= asid;
                    resp_insert <= !resp_page_fault && !resp_access_fault && !discard && !flush;
                    // walk descartado (SFENCE/troca de satp): não reporta faltas, o requisitante refaz
                    if (discard || flush) begin
                        resp_page_fault   <= 1'b0;
                        resp_access_fault <= 1'b0;
                    end
                    state       <= P_IDLE;
                end

                default: state <= P_IDLE;
            endcase
        end
    end

endmodule
