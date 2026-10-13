`timescale 1ns/1ps
`default_nettype none

import nebula_pkg::*;

/**
 * @module nebula_lsu
 * @brief Load/Store Unit do estágio MEM.
 *
 * - D-Cache L1 VIPT 32KB, 8 vias, linha 64B, write-through / no-write-allocate,
 *   hit em 1 ciclo.
 * - Store buffer FIFO (SB_DEPTH entradas, 8B cada) drenado para dmem em ordem.
 *   Toda leitura de memória (refill, MMIO, PTW) aguarda o SB vazio.
 * - DTLB + interface com PTW compartilhado (Sv39, Svade).
 * - LR/SC com reserva de endereço físico; AMOs executados como
 *   leitura-modificação-escrita na cache (single hart).
 * - Efeitos colaterais (stores, SC, AMO, leituras MMIO) só ocorrem quando
 *   'side_ok' (nenhum flush do WB e nenhuma interrupção pendente); caso
 *   contrário a operação retorna 'replay' sem efeito.
 */
module nebula_lsu #(
    parameter logic [55:0] CACHE_BASE = 56'h0000_0080_000000,
    parameter logic [55:0] CACHE_SIZE = 56'h0000_0080_000000,
    parameter int SB_DEPTH = 8
)(
    input  wire          clk,
    input  wire          rst_n,

    // requisição (uop no estágio MEM)
    input  wire          req_valid,
    input  wire          req_new,        // primeiro ciclo desta uop no MEM
    input  fu_t          req_fu,         // FU_LOAD / FU_STORE / FU_AMO
    input  wire  [2:0]   req_size,       // funct3
    input  wire          req_fp,
    input  wire  [4:0]   req_amo_op,
    input  wire  [63:0]  req_vaddr,
    input  wire  [63:0]  req_wdata,
    input  wire          side_ok,
    input  wire          kill,           // flush vindo do WB

    output logic         done,
    output logic [63:0]  result,
    output logic         exc,
    output logic [4:0]   exc_cause,
    output logic [63:0]  exc_tval,
    output logic         replay,
    output logic         side_effect,    // efeito colateral realizado (não interrompível)
    output logic         mmio,           // leitura não-cacheável concluída

    // estado de tradução
    input  wire  [63:0]  satp,
    input  wire  [1:0]   priv,
    input  wire          mprv,
    input  wire  [1:0]   mpp,
    input  wire          sum,
    input  wire          mxr,
    input  wire          tlb_flush,

    // PTW
    output logic         ptw_req,
    output logic [26:0]  ptw_vpn,
    input  wire          ptw_gnt,
    input  wire          ptw_resp_valid,
    input  wire          ptw_resp_for_d,
    input  wire  [26:0]  ptw_resp_vpn,
    input  wire          ptw_resp_page_fault,
    input  wire          ptw_resp_access_fault,
    input  wire          ptw_resp_insert,
    input  wire  [43:0]  ptw_resp_ppn,
    input  wire  [1:0]   ptw_resp_level,
    input  wire  [7:0]   ptw_resp_flags,

    output logic         sb_empty,

    // memória de dados
    output logic         dmem_req,
    output logic         dmem_we,
    output logic [55:0]  dmem_addr,
    output logic [511:0] dmem_wdata,
    output logic [63:0]  dmem_wstrb,
    output logic         dmem_uncached,
    input  wire          dmem_ack,
    input  wire  [511:0] dmem_rdata,
    input  wire          dmem_error
);
    localparam int WAYS = 8;
    localparam int SETS = 64;
    localparam int SBW  = $clog2(SB_DEPTH);

    // =====================================================================
    // Arrays da D-Cache
    // =====================================================================
    logic [43:0] dc_tag   [SETS][WAYS];
    logic [WAYS-1:0] dc_valid [SETS];
    logic [63:0] dc_data  [SETS][WAYS][8];
    logic [2:0]  dc_victim [SETS];

    // =====================================================================
    // Store buffer
    // =====================================================================
    // entradas por linha de 64B; stores cacheáveis à mesma linha são fundidos
    logic [49:0]  sb_addr [SB_DEPTH];   // paddr[55:6]
    logic [511:0] sb_data [SB_DEPTH];
    logic [63:0]  sb_strb [SB_DEPTH];
    logic        sb_unc  [SB_DEPTH];
    logic [SBW-1:0] sb_head, sb_tail;
    logic [SBW:0]   sb_count;
    logic           sb_full;

    logic           sb_merge;
    logic [SBW-1:0] sb_merge_idx;
    assign sb_empty = (sb_count == '0);

    // =====================================================================
    // FSM
    // =====================================================================
    typedef enum logic [2:0] {
        L_IDLE, L_PTW_REQ, L_PTW_WAIT, L_DRAIN_REFILL, L_REFILL, L_DRAIN_UNC, L_UNC
    } lstate_t;
    lstate_t state;
    logic [55:0] refill_paddr_q;
    logic [55:0] unc_paddr_q;
    logic [7:0]  unc_mask_q;
    logic        uc_issued;
    logic        orphan;    // transação em voo pertence a uma uop já descartada

    // tipo de acesso
    logic is_load, is_store, is_lr, is_sc, is_amo_rmw;
    assign is_load    = (req_fu == FU_LOAD);
    assign is_store   = (req_fu == FU_STORE);
    assign is_lr      = (req_fu == FU_AMO) && (req_amo_op == 5'b00010);
    assign is_sc      = (req_fu == FU_AMO) && (req_amo_op == 5'b00011);
    assign is_amo_rmw = (req_fu == FU_AMO) && !is_lr && !is_sc;

    logic acc_read_perm;   // usa permissões de leitura (load, LR)
    assign acc_read_perm = is_load || is_lr;

    // ---------------------------------------------------------------------
    // Alinhamento
    // ---------------------------------------------------------------------
    logic misaligned;
    always_comb begin
        case (req_size[1:0])
            2'b00:   misaligned = 1'b0;
            2'b01:   misaligned = req_vaddr[0];
            2'b10:   misaligned = req_vaddr[1:0] != 2'b00;
            default: misaligned = req_vaddr[2:0] != 3'b000;
        endcase
    end

    // ---------------------------------------------------------------------
    // Tradução
    // ---------------------------------------------------------------------
    logic [1:0] priv_eff;
    logic       vm;
    assign priv_eff = (mprv && priv == 2'b11) ? mpp : priv;
    assign vm       = (satp[63:60] == 4'd8) && (priv_eff != 2'b11);

    logic        dtlb_hit;
    logic [43:0] dtlb_ppn;
    logic [7:0]  dtlb_flags;
    logic [3:0]  dtlb_idx;
    logic        perm_fault;
    logic        inval_dtlb;

    nebula_tlb #(.ENTRIES(16)) u_dtlb (
        .clk, .rst_n,
        .lookup_vpn   (req_vaddr[38:12]),
        .lookup_hit   (dtlb_hit),
        .lookup_ppn   (dtlb_ppn),
        .lookup_flags (dtlb_flags),
        .lookup_idx   (dtlb_idx),
        .insert_valid (ptw_resp_valid && ptw_resp_for_d && ptw_resp_insert && !tlb_flush),
        .insert_vpn   (ptw_resp_vpn),
        .insert_ppn   (ptw_resp_ppn),
        .insert_level (ptw_resp_level),
        .insert_flags (ptw_resp_flags),
        .flush_all    (tlb_flush),
        .inval_valid  (inval_dtlb),
        .inval_idx    (dtlb_idx)
    );

    always_comb begin
        logic u, r, w, x, a, d;
        r = dtlb_flags[1]; w = dtlb_flags[2]; x = dtlb_flags[3];
        u = dtlb_flags[4]; a = dtlb_flags[6]; d = dtlb_flags[7];
        perm_fault = 1'b0;
        if (u ? (priv_eff == 2'b01 && !sum) : (priv_eff == 2'b00)) perm_fault = 1'b1;
        if (acc_read_perm) begin
            if (!(r || (mxr && x))) perm_fault = 1'b1;
            if (!a) perm_fault = 1'b1;
        end else begin
            if (!w) perm_fault = 1'b1;
            if (!a || !d) perm_fault = 1'b1;
        end
    end

    logic        noncanon;
    logic [55:0] paddr;
    assign noncanon = req_vaddr[63:39] != {25{req_vaddr[38]}};
    assign paddr    = vm ? {dtlb_ppn, req_vaddr[11:0]} : req_vaddr[55:0];

    logic cacheable;
    assign cacheable = (paddr >= CACHE_BASE) && (paddr < CACHE_BASE + CACHE_SIZE);

    // procura entrada fundível (exceto a cabeça, que está em escrita)
    always_comb begin
        sb_merge     = 1'b0;
        sb_merge_idx = '0;
        for (int k = 1; k < SB_DEPTH; k++) begin
            logic [SBW-1:0] e;
            e = sb_head + SBW'(k);
            if ((SBW+1)'(k) < sb_count && !sb_unc[e] && sb_addr[e] == paddr[55:6] && cacheable) begin
                sb_merge     = 1'b1;
                sb_merge_idx = e;
            end
        end
        sb_full = (sb_count == (SBW+1)'(SB_DEPTH)) && !sb_merge;
    end

    // ---------------------------------------------------------------------
    // Lookup na D-Cache
    // ---------------------------------------------------------------------
    logic [5:0]  idx;
    logic [2:0]  wsel;
    logic        dc_hit;
    logic [2:0]  dc_hit_way;
    logic [63:0] dc_word;
    assign idx  = req_vaddr[11:6];
    assign wsel = req_vaddr[5:3];

    always_comb begin
        dc_hit     = 1'b0;
        dc_hit_way = '0;
        dc_word    = '0;
        for (int w = 0; w < WAYS; w++) begin
            if (dc_valid[idx][w] && dc_tag[idx][w] == paddr[55:12]) begin
                dc_hit     = 1'b1;
                dc_hit_way = 3'(w);
                dc_word    = dc_data[idx][w][wsel];
            end
        end
    end

    // ---------------------------------------------------------------------
    // Dados de escrita / alinhamento de leitura
    // ---------------------------------------------------------------------
    logic [7:0]  bmask;
    logic [63:0] st_data;   // dado posicionado no doubleword
    always_comb begin
        case (req_size[1:0])
            2'b00:   bmask = 8'b0000_0001 << req_vaddr[2:0];
            2'b01:   bmask = 8'b0000_0011 << req_vaddr[2:0];
            2'b10:   bmask = 8'b0000_1111 << req_vaddr[2:0];
            default: bmask = 8'b1111_1111;
        endcase
    end

    function automatic logic [63:0] load_extend(input logic [63:0] dw, input logic [2:0] off,
                                                input logic [2:0] sz, input logic fp);
        logic [63:0] s;
        s = dw >> {off, 3'b000};
        if (fp) begin
            return (sz[1:0] == 2'b10) ? {32'hFFFF_FFFF, s[31:0]} : s;
        end
        case (sz)
            3'b000:  return {{56{s[7]}},  s[7:0]};
            3'b001:  return {{48{s[15]}}, s[15:0]};
            3'b010:  return {{32{s[31]}}, s[31:0]};
            3'b100:  return {56'b0, s[7:0]};
            3'b101:  return {48'b0, s[15:0]};
            3'b110:  return {32'b0, s[31:0]};
            default: return s;
        endcase
    endfunction

    // AMO
    logic [63:0] amo_old, amo_src, amo_new, amo_res;
    logic        amo_w;
    assign amo_w   = (req_size[1:0] == 2'b10);
    assign amo_old = amo_w ? {{32{dc_word[{req_vaddr[2], 5'b11111}]}}, dc_word[req_vaddr[2]*32 +: 32]} : dc_word;
    assign amo_src = amo_w ? {{32{req_wdata[31]}}, req_wdata[31:0]} : req_wdata;

    always_comb begin
        logic signed [63:0] so, ss;
        so = amo_old; ss = amo_src;
        case (req_amo_op)
            5'b00001: amo_res = amo_src;                                   // SWAP
            5'b00000: amo_res = amo_old + amo_src;                         // ADD
            5'b00100: amo_res = amo_old ^ amo_src;                         // XOR
            5'b01100: amo_res = amo_old & amo_src;                         // AND
            5'b01000: amo_res = amo_old | amo_src;                         // OR
            5'b10000: amo_res = (so < ss) ? amo_old : amo_src;             // MIN
            5'b10100: amo_res = (so > ss) ? amo_old : amo_src;             // MAX
            5'b11000: amo_res = (amo_old < amo_src) ? amo_old : amo_src;   // MINU
            default:  amo_res = (amo_old > amo_src) ? amo_old : amo_src;   // MAXU
        endcase
    end

    always_comb begin
        if (is_amo_rmw)
            amo_new = amo_w ? {amo_res[31:0], amo_res[31:0]} : amo_res;
        else
            amo_new = '0;
        if (is_amo_rmw)
            st_data = amo_new;
        else
            st_data = req_wdata << {req_vaddr[2:0], 3'b000};
    end

    // reserva LR/SC
    logic        resv_valid;
    logic [55:0] resv_addr;
    logic        sc_ok;
    assign sc_ok = resv_valid && (resv_addr == paddr);

    // =====================================================================
    // Lógica principal (combinacional) para o estado IDLE
    // =====================================================================
    logic        uc_data_valid;     // dado de leitura não-cacheável disponível
    logic [63:0] uc_data;
    logic        uc_error;

    logic do_cache_write;           // escrever palavra na D-Cache
    logic do_sb_push;
    logic set_resv, clr_resv;
    logic go_ptw, go_refill, go_unc;

    always_comb begin
        done       = 1'b0;
        result     = '0;
        exc        = 1'b0;
        exc_cause  = '0;
        exc_tval   = req_vaddr;
        replay     = 1'b0;
        do_cache_write = 1'b0;
        do_sb_push = 1'b0;
        set_resv   = 1'b0;
        clr_resv   = 1'b0;
        go_ptw     = 1'b0;
        go_refill  = 1'b0;
        go_unc     = 1'b0;
        inval_dtlb = 1'b0;

        if (req_valid && !kill && !orphan) begin
            if (state == L_UNC) begin
                if (uc_data_valid) begin
                    done = 1'b1;
                    if (uc_error) begin
                        exc = 1'b1; exc_cause = EXC_LOAD_ACCESS_FAULT;
                    end else begin
                        result = load_extend(uc_data, req_vaddr[2:0], req_size, req_fp);
                    end
                end
            end else if (state == L_PTW_WAIT) begin
                if (ptw_resp_valid && ptw_resp_for_d &&
                    (ptw_resp_page_fault || ptw_resp_access_fault)) begin
                    done = 1'b1; exc = 1'b1;
                    if (ptw_resp_page_fault)
                        exc_cause = acc_read_perm ? EXC_LOAD_PAGE_FAULT : EXC_STORE_PAGE_FAULT;
                    else
                        exc_cause = acc_read_perm ? EXC_LOAD_ACCESS_FAULT : EXC_STORE_ACCESS_FAULT;
                end
            end else if (state == L_REFILL) begin
                if (dmem_ack && dmem_error) begin
                    done = 1'b1; exc = 1'b1;
                    exc_cause = acc_read_perm ? EXC_LOAD_ACCESS_FAULT : EXC_STORE_ACCESS_FAULT;
                end
            end else if (state == L_IDLE) begin
                if (misaligned) begin
                    done = 1'b1; exc = 1'b1;
                    exc_cause = acc_read_perm ? EXC_LOAD_MISALIGNED : EXC_STORE_MISALIGNED;
                end else if (vm && noncanon) begin
                    done = 1'b1; exc = 1'b1;
                    exc_cause = acc_read_perm ? EXC_LOAD_PAGE_FAULT : EXC_STORE_PAGE_FAULT;
                end else if (vm && !dtlb_hit) begin
                    go_ptw = 1'b1;
                end else if (vm && perm_fault) begin
                    done = 1'b1; exc = 1'b1; inval_dtlb = 1'b1;
                    exc_cause = acc_read_perm ? EXC_LOAD_PAGE_FAULT : EXC_STORE_PAGE_FAULT;
                end else if (!cacheable) begin
                    // ---------------- MMIO / não cacheável ----------------
                    if (is_load) begin
                        if (side_ok) go_unc = 1'b1;
                        else begin done = 1'b1; replay = 1'b1; end
                    end else if (is_store) begin
                        if (!side_ok) begin
                            done = 1'b1; replay = 1'b1;
                        end else if (!sb_full) begin
                            done = 1'b1; do_sb_push = 1'b1;
                        end
                    end else begin
                        // AMO/LR/SC em região não cacheável: access fault
                        done = 1'b1; exc = 1'b1;
                        exc_cause = is_lr ? EXC_LOAD_ACCESS_FAULT : EXC_STORE_ACCESS_FAULT;
                    end
                end else begin
                    // ---------------- cacheável ----------------
                    if (is_load || is_lr) begin
                        if (dc_hit) begin
                            done   = 1'b1;
                            result = load_extend(dc_word, req_vaddr[2:0], req_size, req_fp);
                            set_resv = is_lr;
                        end else begin
                            go_refill = 1'b1;
                        end
                    end else if (is_store) begin
                        if (!side_ok) begin
                            done = 1'b1; replay = 1'b1;
                        end else if (!sb_full) begin
                            done = 1'b1; do_sb_push = 1'b1; do_cache_write = dc_hit;
                        end
                    end else if (is_sc) begin
                        if (!side_ok) begin
                            done = 1'b1; replay = 1'b1;
                        end else if (!sc_ok) begin
                            done = 1'b1; result = 64'd1; clr_resv = 1'b1;
                        end else if (!sb_full) begin
                            done = 1'b1; result = 64'd0; clr_resv = 1'b1;
                            do_sb_push = 1'b1; do_cache_write = dc_hit;
                        end
                    end else begin // AMO RMW
                        if (!side_ok) begin
                            done = 1'b1; replay = 1'b1;
                        end else if (!dc_hit) begin
                            go_refill = 1'b1;
                        end else if (!sb_full) begin
                            done = 1'b1; result = amo_old;
                            do_sb_push = 1'b1; do_cache_write = 1'b1;
                        end
                    end
                end
            end
        end
    end

    assign side_effect = done && !exc && !replay &&
                         (is_store || is_sc || is_amo_rmw || (state == L_UNC));
    assign mmio        = done && !exc && (state == L_UNC);

    // =====================================================================
    // Interface PTW
    // =====================================================================
    assign ptw_req = (state == L_PTW_REQ) && req_valid && !kill;
    assign ptw_vpn = req_vaddr[38:12];

    // =====================================================================
    // Interface dmem: store buffer tem prioridade; leituras só com SB vazio
    // =====================================================================
    logic sb_active;
    assign sb_active = !sb_empty;

    always_comb begin
        dmem_req      = 1'b0;
        dmem_we       = 1'b0;
        dmem_addr     = '0;
        dmem_wdata    = '0;
        dmem_wstrb    = '0;
        dmem_uncached = 1'b0;
        if (sb_active) begin
            dmem_req      = 1'b1;
            dmem_we       = 1'b1;
            dmem_addr     = {sb_addr[sb_head], 6'b0};
            dmem_wdata    = sb_data[sb_head];
            dmem_wstrb    = sb_strb[sb_head];
            dmem_uncached = sb_unc[sb_head];
        end else if (state == L_REFILL) begin
            dmem_req      = 1'b1;
            dmem_addr     = {refill_paddr_q[55:6], 6'b0};
            dmem_wstrb    = '1;
        end else if (state == L_UNC && !uc_issued) begin
            dmem_req      = 1'b1;
            dmem_uncached = 1'b1;
            dmem_addr     = {unc_paddr_q[55:3], 3'b000};
            dmem_wstrb    = 64'(unc_mask_q) << {unc_paddr_q[5:3], 3'b000};
        end
    end


    assign uc_data_valid = (state == L_UNC) && uc_issued;

    // =====================================================================
    // Sequencial
    // =====================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= L_IDLE;
            sb_head        <= '0;
            sb_tail        <= '0;
            sb_count       <= '0;
            resv_valid     <= 1'b0;
            resv_addr      <= '0;
            refill_paddr_q <= '0;
            unc_paddr_q    <= '0;
            unc_mask_q     <= '0;
            uc_issued      <= 1'b0;
            uc_data        <= '0;
            uc_error       <= 1'b0;
            orphan         <= 1'b0;
            for (int s = 0; s < SETS; s++) begin
                dc_valid[s]  <= '0;
                dc_victim[s] <= '0;
            end
        end else begin
            // ---------------- store buffer ----------------
            begin
                logic pop, push;
                pop  = sb_active && dmem_ack;
                push = do_sb_push && !sb_merge;
                if (do_sb_push && sb_merge) begin
                    for (int b = 0; b < 8; b++)
                        if (bmask[b]) begin
                            sb_data[sb_merge_idx][{paddr[5:3], 3'(b)}*8 +: 8] <= st_data[b*8 +: 8];
                            sb_strb[sb_merge_idx][{paddr[5:3], 3'(b)}] <= 1'b1;
                        end
                end
                if (push) begin
                    sb_addr[sb_tail] <= paddr[55:6];
                    sb_data[sb_tail] <= {8{st_data}};
                    sb_strb[sb_tail] <= 64'(bmask) << {paddr[5:3], 3'b000};
                    sb_unc[sb_tail]  <= !cacheable;
                    sb_tail <= sb_tail + 1'b1;
                end
                if (pop) sb_head <= sb_head + 1'b1;
                sb_count <= sb_count + (SBW+1)'(push) - (SBW+1)'(pop);
            end

            // ---------------- reserva ----------------
            if (set_resv) begin
                resv_valid <= 1'b1;
                resv_addr  <= paddr;
            end else if (clr_resv) begin
                resv_valid <= 1'b0;
            end

            // ---------------- escrita na cache (store hit / AMO / SC) ----------------
            if (do_cache_write) begin
                for (int b = 0; b < 8; b++)
                    if (bmask[b]) dc_data[idx][dc_hit_way][wsel][b*8 +: 8] <= st_data[b*8 +: 8];
            end

            // ---------------- FSM ----------------
            if (kill && (state == L_PTW_WAIT || state == L_REFILL || state == L_UNC))
                orphan <= 1'b1;
            if (state == L_IDLE)
                orphan <= 1'b0;

            case (state)
                L_IDLE: begin
                    if (go_ptw)         state <= L_PTW_REQ;
                    else if (go_refill) begin
                        state          <= L_DRAIN_REFILL;
                        refill_paddr_q <= paddr;
                    end else if (go_unc) begin
                        state       <= L_DRAIN_UNC;
                        unc_paddr_q <= paddr;
                        unc_mask_q  <= bmask;
                    end
                end
                L_PTW_REQ: begin
                    if (kill || !req_valid) state <= L_IDLE;
                    else if (ptw_gnt)       state <= L_PTW_WAIT;
                end
                L_PTW_WAIT: begin
                    // walk sempre termina; resultado é inserido na DTLB
                    if (ptw_resp_valid && ptw_resp_for_d) state <= L_IDLE;
                end
                L_DRAIN_REFILL: begin
                    if (kill) state <= L_IDLE;
                    else if (sb_empty) state <= L_REFILL;
                end
                L_REFILL: begin
                    if (dmem_ack && !sb_active) begin
                        if (!dmem_error) begin
                            logic [5:0] ri;
                            ri = refill_paddr_q[11:6];
                            dc_valid[ri][dc_victim[ri]] <= 1'b1;
                            dc_tag[ri][dc_victim[ri]]   <= refill_paddr_q[55:12];
                            for (int k = 0; k < 8; k++)
                                dc_data[ri][dc_victim[ri]][k] <= dmem_rdata[k*64 +: 64];
                            dc_victim[ri] <= dc_victim[ri] + 1'b1;
                        end
                        state <= L_IDLE;
                    end
                end
                L_DRAIN_UNC: begin
                    if (kill) state <= L_IDLE;
                    else if (sb_empty) begin
                        state     <= L_UNC;
                        uc_issued <= 1'b0;
                    end
                end
                L_UNC: begin
                    if (!uc_issued) begin
                        if (dmem_ack) begin
                            uc_issued <= 1'b1;
                            uc_data   <= dmem_rdata[unc_paddr_q[5:3]*64 +: 64];
                            uc_error  <= dmem_error;
                        end
                    end else begin
                        // resultado entregue neste ciclo (done) ou descartado
                        state     <= L_IDLE;
                        uc_issued <= 1'b0;
                    end
                end
                default: state <= L_IDLE;
            endcase

            if (tlb_flush) begin
                // nada extra: DTLB descartada no próprio módulo
            end
        end
    end

endmodule
