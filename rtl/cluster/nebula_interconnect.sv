`timescale 1ns/1ps
`default_nettype none

import nebula_pkg::*;

/**
 * @module nebula_interconnect
 * @brief Interconexão coerente do cluster Nebula (N harts).
 *
 * Serializa (round-robin) as portas I (refill de I$), D (refill de D$, store
 * buffer, MMIO, atômicos) e PTW de todos os núcleos numa única porta de
 * memória. Por ser o ponto único de serialização, define a ordem global de
 * memória:
 *
 *  - Coerência das L1D write-through: toda escrita cacheável invalida a linha
 *    nas L1D dos outros núcleos no mesmo ciclo em que é efetivada (no autor,
 *    a linha já foi atualizada pelo próprio store). AMO/SC invalidam a linha
 *    em todos os núcleos, inclusive no autor.
 *  - AMO/LR/SC executados aqui, de forma atômica (leitura + escrita sem
 *    intercalação). Reservas LR por hart (endereço exato); uma escrita de
 *    outro hart na mesma linha cancela a reserva.
 *  - I$ não é coerente por hardware (FENCE.I + IPI, como exige a ISA).
 */
module nebula_interconnect #(
    parameter int NC = 1
)(
    input  wire                     clk,
    input  wire                     rst_n,

    // ---------------- portas dos núcleos ----------------
    input  wire  [NC-1:0]           i_req,
    input  wire  [NC-1:0][55:0]     i_addr,
    output logic [NC-1:0]           i_ack,
    output logic [511:0]            i_data,
    output logic                    i_error,

    input  wire  [NC-1:0]           d_req,
    input  wire  [NC-1:0]           d_we,
    input  wire  [NC-1:0][55:0]     d_addr,
    input  wire  [NC-1:0][511:0]    d_wdata,
    input  wire  [NC-1:0][63:0]     d_wstrb,
    input  wire  [NC-1:0]           d_uncached,
    input  wire  [NC-1:0]           d_is_amo,
    input  wire  [NC-1:0][4:0]      d_amo_op,
    output logic [NC-1:0]           d_ack,
    output logic [511:0]            d_rdata,
    output logic                    d_error,

    input  wire  [NC-1:0]           p_req,
    input  wire  [NC-1:0][55:0]     p_addr,
    output logic [NC-1:0]           p_ack,
    output logic [63:0]             p_data,
    output logic                    p_error,

    // invalidações para as L1D
    output logic [NC-1:0]           inv_valid,
    output logic [49:0]             inv_line,

    // ---------------- memória ----------------
    output logic                    mem_req,
    output logic                    mem_we,
    output logic [55:0]             mem_addr,
    output logic [511:0]            mem_wdata,
    output logic [63:0]             mem_wstrb,
    output logic                    mem_uncached,
    input  wire                     mem_ack,
    input  wire  [511:0]            mem_rdata,
    input  wire                     mem_error
);
    localparam int NP = 3 * NC;                 // portas: [I0..In, D0..Dn, P0..Pn]
    localparam int PW = (NP > 1) ? $clog2(NP) : 1;
    localparam int CW = (NC > 1) ? $clog2(NC) : 1;

    typedef enum logic [2:0] { X_IDLE, X_REQ, X_AWR, X_RESP } xstate_t;
    xstate_t state;

    logic [PW-1:0]  rr;            // próximo a ter prioridade
    logic [PW-1:0]  cur;           // porta em atendimento
    logic [1:0]     cur_kind;      // 0 I, 1 D, 2 P
    logic [CW-1:0]  cur_core;
    logic           c_we, c_unc, c_amo;
    logic [4:0]     c_op;
    logic [55:0]    c_addr;
    logic [511:0]   c_wdata;
    logic [63:0]    c_wstrb;
    logic [511:0]   r_data;
    logic           r_err;
    logic [63:0]    amo_wr;        // doubleword a escrever (AMO/SC)

    // reservas LR
    logic [NC-1:0]       rsv_v;
    logic [NC-1:0][55:0] rsv_a;

    // ---------------- seleção round-robin ----------------
    logic [NP-1:0] reqs;
    always_comb begin
        for (int c = 0; c < NC; c++) begin
            reqs[c]          = i_req[c];
            reqs[NC + c]     = d_req[c];
            reqs[2 * NC + c] = p_req[c];
        end
    end

    logic          sel_v;
    logic [PW-1:0] sel;
    always_comb begin
        sel_v = 1'b0;
        sel   = '0;
        for (int k = 0; k < NP; k++) begin
            int j;
            j = (int'(rr) + k) % NP;
            if (!sel_v && reqs[j]) begin
                sel_v = 1'b1;
                sel   = PW'(j);
            end
        end
    end

    // endereço exato do atômico: doubleword + metade alta (strobes) para .W
    logic        at_w32, at_hi;
    logic [55:0] at_exact;
    always_comb begin
        at_w32   = (c_wstrb[c_addr[5:3]*8 +: 8] != 8'hFF);
        at_hi    = at_w32 && c_wstrb[c_addr[5:3]*8 + 4];
        at_exact = {c_addr[55:3], at_hi, 2'b00};
    end

    // ---------------- portas de saída ----------------
    always_comb begin
        mem_req      = 1'b0;
        mem_we       = 1'b0;
        mem_addr     = c_addr;
        mem_wdata    = c_wdata;
        mem_wstrb    = c_wstrb;
        mem_uncached = c_unc;
        if (state == X_REQ) begin
            mem_req = 1'b1;
            mem_we  = c_we && !c_amo;
            if (c_amo) begin
                mem_uncached = 1'b0;
                mem_wstrb    = '1;
            end
        end else if (state == X_AWR) begin
            mem_req      = 1'b1;
            mem_we       = 1'b1;
            mem_uncached = 1'b0;
            mem_wdata    = {8{amo_wr}};
            mem_wstrb    = c_wstrb;
        end

        i_ack = '0; d_ack = '0; p_ack = '0;
        i_data  = r_data;   i_error = r_err;
        d_rdata = r_data;   d_error = r_err;
        p_data  = r_data[c_addr[5:3]*64 +: 64];
        p_error = r_err;
        if (state == X_RESP) begin
            case (cur_kind)
                2'd0:    i_ack[cur_core] = 1'b1;
                2'd1:    d_ack[cur_core] = 1'b1;
                default: p_ack[cur_core] = 1'b1;
            endcase
        end
    end

    // invalidações (pulso de 1 ciclo, no ciclo em que a escrita é efetivada)
    logic          inv_fire, inv_self;
    always_comb begin
        inv_fire = 1'b0;
        inv_self = 1'b0;
        if (state == X_REQ && mem_ack && !mem_error && c_we && !c_amo && !c_unc) inv_fire = 1'b1;
        if (state == X_AWR && mem_ack && !mem_error) begin inv_fire = 1'b1; inv_self = 1'b1; end
        inv_line = c_addr[55:6];
        for (int c = 0; c < NC; c++)
            inv_valid[c] = inv_fire && (inv_self || CW'(c) != cur_core);
    end

    // ---------------- sequencial ----------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= X_IDLE;
            rr <= '0; cur <= '0; cur_kind <= '0; cur_core <= '0;
            c_we <= 1'b0; c_unc <= 1'b0; c_amo <= 1'b0; c_op <= '0;
            c_addr <= '0; c_wdata <= '0; c_wstrb <= '0;
            r_data <= '0; r_err <= 1'b0; amo_wr <= '0;
            rsv_v <= '0; rsv_a <= '0;
        end else begin
            // escrita efetivada: cancela reservas de outros harts na mesma linha
            if (inv_fire)
                for (int c = 0; c < NC; c++)
                    if (CW'(c) != cur_core && rsv_a[c][55:6] == c_addr[55:6]) rsv_v[c] <= 1'b0;

            case (state)
                X_IDLE: if (sel_v) begin
                    int core;
                    cur <= sel;
                    rr  <= (int'(sel) + 1 == NP) ? '0 : sel + 1'b1;
                    if (int'(sel) < NC) begin
                        core = int'(sel);
                        cur_kind <= 2'd0;
                        c_we <= 1'b0; c_unc <= 1'b0; c_amo <= 1'b0; c_op <= '0;
                        c_addr <= i_addr[core]; c_wdata <= '0; c_wstrb <= '1;
                    end else if (int'(sel) < 2 * NC) begin
                        core = int'(sel) - NC;
                        cur_kind <= 2'd1;
                        c_we  <= d_we[core];   c_unc <= d_uncached[core];
                        c_amo <= d_is_amo[core]; c_op <= d_amo_op[core];
                        c_addr <= d_addr[core]; c_wdata <= d_wdata[core]; c_wstrb <= d_wstrb[core];
                    end else begin
                        core = int'(sel) - 2 * NC;
                        cur_kind <= 2'd2;
                        c_we <= 1'b0; c_unc <= 1'b0; c_amo <= 1'b0; c_op <= '0;
                        c_addr <= p_addr[core]; c_wdata <= '0; c_wstrb <= '1;
                    end
                    cur_core <= CW'(core);
                    state    <= X_REQ;
                end

                X_REQ: if (mem_ack) begin
                    r_data <= mem_rdata;
                    r_err  <= mem_error;
                    if (c_amo && !mem_error) begin
                        // dados lidos: decide escrita do atômico
                        logic [63:0] rd_dw;
                        logic        w32;
                        rd_dw = mem_rdata[c_addr[5:3]*64 +: 64];
                        w32   = at_w32;
                        if (c_op == 5'b00010) begin                       // LR
                            rsv_v[cur_core] <= 1'b1;
                            rsv_a[cur_core] <= at_exact;
                            state <= X_RESP;
                        end else if (c_op == 5'b00011) begin              // SC
                            logic ok;
                            ok = rsv_v[cur_core] && rsv_a[cur_core] == at_exact;
                            rsv_v[cur_core] <= 1'b0;
                            if (ok) begin
                                amo_wr <= c_wdata[c_addr[5:3]*64 +: 64];
                                r_data[c_addr[5:3]*64 +: 64] <= 64'd0;
                                state  <= X_AWR;
                            end else begin
                                r_data[c_addr[5:3]*64 +: 64] <= 64'd1;
                                state  <= X_RESP;
                            end
                        end else begin                                    // AMO
                            logic [63:0] o, sr, n;
                            logic [31:0] o32, s32;
                            if (w32) begin
                                o32 = at_hi ? rd_dw[63:32] : rd_dw[31:0];
                                s32 = at_hi ? c_wdata[c_addr[5:3]*64 + 32 +: 32] : c_wdata[c_addr[5:3]*64 +: 32];
                                o   = {{32{o32[31]}}, o32};
                                sr  = {{32{s32[31]}}, s32};
                            end else begin
                                o  = rd_dw;
                                sr = c_wdata[c_addr[5:3]*64 +: 64];
                            end
                            n = amo_compute(c_op, o, sr);
                            amo_wr <= w32 ? {n[31:0], n[31:0]} : n;
                            state  <= X_AWR;
                        end
                    end else begin
                        state <= X_RESP;
                    end
                end

                X_AWR: if (mem_ack) begin
                    r_err <= mem_error;
                    state <= X_RESP;
                end

                X_RESP: state <= X_IDLE;
                default: state <= X_IDLE;
            endcase
        end
    end

endmodule
