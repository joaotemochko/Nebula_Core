`timescale 1ns/1ps
`default_nettype none

import nebula_pkg::*;

/**
 * @module nebula_fetch
 * @brief Frontend do Nebula Core: geração de PC, predição (BTB + RAS),
 *        ITLB, I-Cache e fila de halfwords (FQ) para o decode dual-issue.
 *
 * - Busca um bloco alinhado de 16 bytes por ciclo (até 8 halfwords).
 * - BTB indexada pelo bloco que contém o ÚLTIMO halfword do desvio; um desvio
 *   predito tomado trunca o bloco e redireciona o próximo fetch no ciclo
 *   seguinte (0 bolhas para desvios tomados previstos).
 * - RAS especulativo atualizado no fetch e recuperado a partir do RAS
 *   arquitetural (atualizado no EX) em redirecionamentos do backend.
 * - Exceções de fetch (page fault / access fault) entram na FQ como uma
 *   entrada marcada; o fetch para até o próximo redirecionamento.
 */
module nebula_fetch #(
    parameter logic [PMA_N-1:0][55:0] PMA_BASE = PMA_DEF_BASE,
    parameter logic [PMA_N-1:0][55:0] PMA_SIZE = PMA_DEF_SIZE,
    parameter logic [PMA_N-1:0][2:0]  PMA_ATTR = PMA_DEF_ATTR,
    parameter int BTB_SETS  = 1024,
    parameter int RAS_DEPTH = 8,
    parameter int FQ_DEPTH  = 16
)(
    input  wire          clk,
    input  wire          rst_n,
    input  wire  [63:0]  reset_vector,

    // redirecionamento
    input  wire          redirect,
    input  wire  [63:0]  redirect_pc,
    input  wire          ras_recover,

    // atualização da BTB (EX)
    input  wire          bp_upd_valid,
    input  wire  [63:0]  bp_upd_pc,
    input  wire          bp_upd_rvc,
    input  wire          bp_upd_taken,
    input  wire  [63:0]  bp_upd_target,
    input  wire  [2:0]   bp_upd_type,
    // RAS arquitetural (EX)
    input  wire          ras_arch_push,
    input  wire  [63:0]  ras_arch_push_addr,
    input  wire          ras_arch_pop,
    // ações do decode
    input  wire          btb_inval,
    input  wire  [63:0]  btb_inval_pc,
    input  wire          dec_ras_push,
    input  wire  [63:0]  dec_ras_push_addr,

    // saída da FQ (4 primeiras entradas)
    output logic [3:0]          fq_v,
    output logic [3:0][15:0]    fq_hw,
    output logic [3:0][63:0]    fq_pc,
    output logic [3:0][1:0]     fq_exc,     // 0 nenhum, 1 page fault, 2 access fault
    output logic [3:0]          fq_pt,      // desvio predito tomado termina aqui
    output logic [3:0][63:0]    fq_ptgt,
    input  wire  [2:0]          fq_pop,

    // estado MMU
    input  wire  [63:0]  satp,
    input  wire  [1:0]   priv,
    input  wire          sf_valid,       // SFENCE.VMA
    input  wire          sf_va,
    input  wire  [26:0]  sf_vpn,
    input  wire          sf_asid,
    input  wire  [15:0]  sf_asid_val,
    input  wire          icache_inval,
    input  wire  [15:0][7:0]  pmpcfg,
    input  wire  [15:0][57:0] pmp_lo,
    input  wire  [15:0][57:0] pmp_hi,
    input  wire               pmp_any_locked,

    // PTW
    output logic         ptw_req,
    output logic [26:0]  ptw_vpn,
    input  wire          ptw_gnt,
    input  wire          ptw_resp_valid,
    input  wire          ptw_resp_for_d,
    input  wire  [26:0]  ptw_resp_vpn,
    input  wire  [15:0]  ptw_resp_asid,
    input  wire          ptw_resp_page_fault,
    input  wire          ptw_resp_access_fault,
    input  wire          ptw_resp_insert,
    input  wire  [43:0]  ptw_resp_ppn,
    input  wire  [1:0]   ptw_resp_level,
    input  wire  [7:0]   ptw_resp_flags,

    // memória de instruções (refill de linha)
    output logic         imem_req,
    output logic [55:0]  imem_addr,
    input  wire          imem_ack,
    input  wire  [511:0] imem_data,
    input  wire          imem_error
);
    localparam int BW = $clog2(BTB_SETS);
    localparam int RW = $clog2(RAS_DEPTH);
    localparam int QW = $clog2(FQ_DEPTH);
    localparam int TAGW = 39 - 4 - BW;

    localparam logic [2:0] BT_COND = 3'd0;
    localparam logic [2:0] BT_JUMP = 3'd1;
    localparam logic [2:0] BT_CALL = 3'd2;
    localparam logic [2:0] BT_RET  = 3'd3;
    localparam logic [2:0] BT_IND  = 3'd4;

    typedef enum logic [1:0] { F_RUN, F_PTW, F_REFILL, F_HALT } fstate_t;
    fstate_t state;

    logic [63:0] fpc;
    logic        booted;
    logic        walk_stale;   // redirecionado durante o walk: resultado não vale para o fpc atual

    // =====================================================================
    // FQ
    // =====================================================================
    logic [15:0] q_hw   [FQ_DEPTH];
    logic [63:0] q_pc   [FQ_DEPTH];
    logic [1:0]  q_exc  [FQ_DEPTH];
    logic        q_pt   [FQ_DEPTH];
    logic [63:0] q_ptgt [FQ_DEPTH];
    logic [QW-1:0] q_head, q_tail;
    logic [QW:0]   q_count;

    always_comb begin
        for (int i = 0; i < 4; i++) begin
            fq_v[i]    = (q_count > (QW+1)'(i));
            fq_hw[i]   = q_hw[q_head + QW'(i)];
            fq_pc[i]   = q_pc[q_head + QW'(i)];
            fq_exc[i]  = q_exc[q_head + QW'(i)];
            fq_pt[i]   = q_pt[q_head + QW'(i)];
            fq_ptgt[i] = q_ptgt[q_head + QW'(i)];
        end
    end

    // =====================================================================
    // ITLB
    // =====================================================================
    logic        vm;
    logic        itlb_hit;
    logic [43:0] itlb_ppn;
    logic [7:0]  itlb_flags;
    logic [3:0]  itlb_idx;
    logic        itlb_perm_fault;

    assign vm = (satp[63:60] == 4'd8) && (priv != 2'b11);

    nebula_tlb #(.ENTRIES(16)) u_itlb (
        .clk, .rst_n,
        .lookup_vpn   (fpc[38:12]),
        .lookup_asid  (satp[59:44]),
        .lookup_hit   (itlb_hit),
        .lookup_ppn   (itlb_ppn),
        .lookup_flags (itlb_flags),
        .lookup_idx   (itlb_idx),
        .insert_valid (ptw_resp_valid && !ptw_resp_for_d && ptw_resp_insert && !sf_valid),
        .insert_asid  (ptw_resp_asid),
        .insert_vpn   (ptw_resp_vpn),
        .insert_ppn   (ptw_resp_ppn),
        .insert_level (ptw_resp_level),
        .insert_flags (ptw_resp_flags),
        .flush        (sf_valid),
        .flush_va     (sf_va),
        .flush_vpn    (sf_vpn),
        .flush_asid   (sf_asid),
        .flush_asid_val(sf_asid_val),
        .inval_valid  (state == F_RUN && vm && itlb_hit && itlb_perm_fault && !redirect),
        .inval_idx    (itlb_idx)
    );

    // permissão de execução: X, A, e U conforme privilégio
    assign itlb_perm_fault = !itlb_flags[3] || !itlb_flags[6] ||
                             (itlb_flags[4] ? (priv != 2'b00) : (priv == 2'b00));

    logic        noncanon;
    logic [43:0] fetch_ppn;
    logic        f_exc;
    logic [1:0]  f_exc_code;
    logic        f_need_walk;
    logic        f_pma_ok, f_pma_x;

    always_comb begin
        noncanon    = fpc[63:39] != {25{fpc[38]}};
        f_exc       = 1'b0;
        f_exc_code  = 2'd0;
        f_need_walk = 1'b0;
        fetch_ppn   = fpc[55:12];
        if (vm) begin
            fetch_ppn = itlb_ppn;
            if (noncanon) begin
                f_exc = 1'b1; f_exc_code = 2'd1;
            end else if (!itlb_hit) begin
                f_need_walk = 1'b1;
            end else if (itlb_perm_fault) begin
                f_exc = 1'b1; f_exc_code = 2'd1;
            end
        end else if (fpc[63:56] != 8'd0) begin
            f_exc = 1'b1; f_exc_code = 2'd2;
        end
        // PMA/PMP: região inexistente, não executável ou negada
        if (!f_exc && !f_need_walk && !(f_pma_ok && f_pma_x && sec_ok[fpc[3:2]])) begin
            f_exc = 1'b1; f_exc_code = 2'd2;
        end
    end

    // PMP por setor de 4 bytes do bloco de 16 B
    logic [3:0] sec_ok;
    for (genvar g = 0; g < 4; g++) begin : g_pmp
        nebula_pmp u_pmp (.cfg(pmpcfg), .lo(pmp_lo), .hi(pmp_hi), .any_locked(pmp_any_locked), .addr({fetch_ppn, fpc[11:4], 2'(g), 2'b00}),
                          .use_lo(g % 2 == 0), .use_hi(g % 2 == 1), .acc(2'd2), .priv(priv), .ok(sec_ok[g]));
    end

    nebula_pma #(.N(PMA_N), .BASE(PMA_BASE), .SIZE(PMA_SIZE), .ATTR(PMA_ATTR)) u_pma (
        .addr({fetch_ppn, fpc[11:0]}), .ok(f_pma_ok), .cacheable(), .exec(f_pma_x)
    );

    // =====================================================================
    // I-Cache
    // =====================================================================
    logic         ic_hit;
    logic [127:0] ic_data;
    logic [55:0]  refill_paddr;
    logic         refill_discard;
    logic         refill_redirected;

    nebula_icache u_icache (
        .clk, .rst_n,
        .rd_vaddr       (fpc[11:0]),
        .rd_ptag        (fetch_ppn),
        .rd_hit         (ic_hit),
        .rd_data        (ic_data),
        .refill_valid   (state == F_REFILL && imem_ack && !imem_error && !refill_discard && !icache_inval),
        .refill_paddr   (refill_paddr),
        .refill_data    (imem_data),
        .invalidate_all (icache_inval)
    );

    assign imem_req  = (state == F_REFILL);
    assign imem_addr = {refill_paddr[55:6], 6'b0};

    // =====================================================================
    // BTB
    // =====================================================================
    logic [BTB_SETS-1:0] btb_v;
    logic [TAGW-1:0] btb_tag [BTB_SETS];
    logic [2:0]      btb_off [BTB_SETS];
    logic [2:0]      btb_typ [BTB_SETS];
    logic [38:0]     btb_tgt [BTB_SETS];
    logic [1:0]      btb_ctr [BTB_SETS];

    logic [BW-1:0]   b_idx;
    logic [TAGW-1:0] b_tag;
    logic [2:0]      start_off;
    logic            b_hit, b_taken;
    logic [63:0]     b_target;
    logic [2:0]      end_off;
    logic [2:0]      pmp_lim;
    logic            pmp_trunc;
    logic [63:0]     seq_next;

    // RAS especulativo
    logic [63:0] ras      [RAS_DEPTH];
    logic [RW-1:0] ras_ptr;
    logic [63:0] ras_arch [RAS_DEPTH];
    logic [RW-1:0] ras_arch_ptr;

    always_comb begin
        b_idx     = fpc[4 +: BW];
        b_tag     = fpc[38 -: TAGW];
        start_off = fpc[3:1];
        b_hit     = btb_v[b_idx] && (btb_tag[b_idx] == b_tag) && (btb_off[b_idx] >= start_off);
        // limite imposto pelo PMP: o bloco termina antes do primeiro setor negado
        pmp_lim   = 3'd7;
        pmp_trunc = 1'b0;
        for (int sct = 3; sct >= 1; sct--)
            if (2'(sct) > fpc[3:2] && !sec_ok[sct]) begin
                pmp_lim   = 3'(2 * sct - 1);
                pmp_trunc = 1'b1;
            end
        b_taken   = b_hit && ((btb_typ[b_idx] != BT_COND) || btb_ctr[b_idx][1]) &&
                    (btb_off[b_idx] <= pmp_lim);
        b_target  = (btb_typ[b_idx] == BT_RET) ? ras[ras_ptr]
                                               : {{25{btb_tgt[b_idx][38]}}, btb_tgt[b_idx]};
        end_off   = b_taken ? btb_off[b_idx] : pmp_lim;
        seq_next  = pmp_trunc ? {fpc[63:4], pmp_lim + 3'd1, 1'b0} : {fpc[63:4] + 60'd1, 4'b0};
    end

    // =====================================================================
    // Push na FQ
    // =====================================================================
    logic        can_push;
    logic        do_push_block;
    logic        do_push_exc;
    logic [3:0]  push_n;

    assign can_push      = (q_count <= (QW+1)'(FQ_DEPTH - 8));
    assign do_push_block = (state == F_RUN) && !redirect && !f_exc && !f_need_walk && ic_hit && can_push;
    assign push_n        = do_push_block ? (4'(end_off) - 4'(start_off) + 4'd1) : 4'd0;

    // exceção de fetch a ser inserida na FQ (do lookup ou do PTW / refill)
    logic        exc_push;
    logic [1:0]  exc_push_code;
    always_comb begin
        exc_push      = 1'b0;
        exc_push_code = 2'd0;
        if (!redirect && q_count < (QW+1)'(FQ_DEPTH)) begin
            if (state == F_RUN && f_exc) begin
                exc_push = 1'b1; exc_push_code = f_exc_code;
            end else if (state == F_PTW && ptw_resp_valid && !ptw_resp_for_d && !walk_stale &&
                         (ptw_resp_page_fault || ptw_resp_access_fault) &&
                         ptw_resp_vpn == fpc[38:12]) begin
                exc_push = 1'b1; exc_push_code = ptw_resp_page_fault ? 2'd1 : 2'd2;
            end else if (state == F_REFILL && imem_ack && imem_error && !refill_redirected) begin
                exc_push = 1'b1; exc_push_code = 2'd2;
            end
        end
    end
    assign do_push_exc = exc_push;

    assign ptw_req = (state == F_RUN) && !redirect && f_need_walk;
    assign ptw_vpn = fpc[38:12];

    // =====================================================================
    // Sequencial
    // =====================================================================
    logic [3:0] pop_n;
    assign pop_n = {1'b0, fq_pop};

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state             <= F_HALT;
            fpc               <= '0;
            booted            <= 1'b0;
            walk_stale        <= 1'b0;
            q_head            <= '0;
            q_tail            <= '0;
            q_count           <= '0;
            refill_paddr      <= '0;
            refill_discard    <= 1'b0;
            refill_redirected <= 1'b0;
            ras_ptr           <= '0;
            ras_arch_ptr      <= '0;
            btb_v <= '0;
            for (int i = 0; i < RAS_DEPTH; i++) begin
                ras[i]      <= '0;
                ras_arch[i] <= '0;
            end
        end else begin
            // ---------------- RAS arquitetural ----------------
            begin
                logic [RW-1:0] ap;
                ap = ras_arch_ptr;
                if (ras_arch_pop)  ap = ap - 1'b1;
                if (ras_arch_push) begin
                    ap = ap + 1'b1;
                    ras_arch[ap] <= ras_arch_push_addr;
                end
                ras_arch_ptr <= ap;

                if (ras_recover) begin
                    for (int i = 0; i < RAS_DEPTH; i++) ras[i] <= ras_arch[i];
                    if (ras_arch_push) ras[ap] <= ras_arch_push_addr;
                    ras_ptr <= ap;
                end else if (dec_ras_push) begin
                    ras[ras_ptr + 1'b1] <= dec_ras_push_addr;
                    ras_ptr <= ras_ptr + 1'b1;
                end else if (do_push_block && b_taken) begin
                    if (btb_typ[b_idx] == BT_CALL) begin
                        ras[ras_ptr + 1'b1] <= {fpc[63:4], btb_off[b_idx], 1'b0} + 64'd2;
                        ras_ptr <= ras_ptr + 1'b1;
                    end else if (btb_typ[b_idx] == BT_RET) begin
                        ras_ptr <= ras_ptr - 1'b1;
                    end
                end
            end

            // ---------------- BTB ----------------
            if (bp_upd_valid) begin
                logic [63:0]     epc;
                logic [BW-1:0]   ui;
                logic [TAGW-1:0] ut;
                logic            uhit;
                epc  = bp_upd_pc + (bp_upd_rvc ? 64'd0 : 64'd2);
                ui   = epc[4 +: BW];
                ut   = epc[38 -: TAGW];
                uhit = btb_v[ui] && btb_tag[ui] == ut && btb_off[ui] == epc[3:1];
                if (uhit) begin
                    btb_typ[ui] <= bp_upd_type;
                    if (bp_upd_taken) btb_tgt[ui] <= bp_upd_target[38:0];
                    if (bp_upd_type == BT_COND) begin
                        if (bp_upd_taken && btb_ctr[ui] != 2'b11) btb_ctr[ui] <= btb_ctr[ui] + 1'b1;
                        if (!bp_upd_taken && btb_ctr[ui] != 2'b00) btb_ctr[ui] <= btb_ctr[ui] - 1'b1;
                    end else begin
                        btb_ctr[ui] <= 2'b11;
                    end
                end else if (bp_upd_taken) begin
                    btb_v[ui]   <= 1'b1;
                    btb_tag[ui] <= ut;
                    btb_off[ui] <= epc[3:1];
                    btb_typ[ui] <= bp_upd_type;
                    btb_tgt[ui] <= bp_upd_target[38:0];
                    btb_ctr[ui] <= 2'b10;
                end
            end else if (btb_inval) begin
                logic [BW-1:0] ii;
                ii = btb_inval_pc[4 +: BW];
                if (btb_tag[ii] == btb_inval_pc[38 -: TAGW]) btb_v[ii] <= 1'b0;
            end

            // ---------------- FQ ----------------
            if (redirect) begin
                q_head  <= '0;
                q_tail  <= '0;
                q_count <= '0;
            end else begin
                logic [3:0] pn;
                pn = do_push_exc ? 4'd1 : push_n;
                for (int i = 0; i < 8; i++) begin
                    if (4'(i) < pn) begin
                        logic [QW-1:0] t;
                        logic [2:0]    o;
                        t = q_tail + QW'(i);
                        o = start_off + 3'(i);
                        if (do_push_exc) begin
                            q_hw[t]   <= 16'h0;
                            q_pc[t]   <= fpc;
                            q_exc[t]  <= exc_push_code;
                            q_pt[t]   <= 1'b0;
                            q_ptgt[t] <= '0;
                        end else begin
                            q_hw[t]   <= ic_data[o*16 +: 16];
                            q_pc[t]   <= {fpc[63:4], o, 1'b0};
                            q_exc[t]  <= 2'd0;
                            q_pt[t]   <= b_taken && (o == end_off);
                            q_ptgt[t] <= b_target;
                        end
                    end
                end
                q_tail  <= q_tail + QW'(pn);
                q_head  <= q_head + QW'(pop_n);
                q_count <= q_count + (QW+1)'(pn) - (QW+1)'(pop_n);
            end

            // ---------------- PC / FSM ----------------
            if (icache_inval && state == F_REFILL) refill_discard <= 1'b1;
            if (redirect && state == F_REFILL) refill_redirected <= 1'b1;

            case (state)
                F_RUN: begin
                    if (redirect) begin
                        fpc <= redirect_pc;
                    end else if (f_exc) begin
                        if (do_push_exc) state <= F_HALT;
                    end else if (f_need_walk) begin
                        if (ptw_gnt) begin
                            state      <= F_PTW;
                            walk_stale <= 1'b0;
                        end
                    end else if (!ic_hit) begin
                        state             <= F_REFILL;
                        refill_paddr      <= {fetch_ppn, fpc[11:0]};
                        refill_discard    <= 1'b0;
                        refill_redirected <= 1'b0;
                    end else if (do_push_block) begin
                        fpc <= b_taken ? b_target : seq_next;
                    end
                end
                F_PTW: begin
                    if (redirect) begin
                        fpc        <= redirect_pc;
                        walk_stale <= 1'b1;
                    end
                    if (ptw_resp_valid && !ptw_resp_for_d) begin
                        state <= do_push_exc ? F_HALT : F_RUN;
                    end
                end
                F_REFILL: begin
                    if (redirect) fpc <= redirect_pc;
                    if (imem_ack) begin
                        state <= do_push_exc ? F_HALT : F_RUN;
                    end
                end
                F_HALT: begin
                    if (!booted) begin
                        fpc    <= reset_vector;
                        booted <= 1'b1;
                        state  <= F_RUN;
                    end else if (redirect) begin
                        fpc   <= redirect_pc;
                        state <= F_RUN;
                    end
                end
                default: state <= F_RUN;
            endcase
        end
    end

endmodule
