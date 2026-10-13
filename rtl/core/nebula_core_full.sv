`timescale 1ns/1ps
`default_nettype none

import nebula_pkg::*;

/**
 * @module nebula_core_full
 * @brief Nebula Core — RV64GC dual-issue in-order, capaz de rodar Linux.
 *
 * Pipeline:
 *   F   : PC + BTB/RAS + ITLB + I-Cache (16B/ciclo) -> FQ (16 halfwords)
 *   D   : extrai, expande (RVC) e decodifica 2 instruções; checa hazards;
 *         emite 0, 1 ou 2 instruções em ordem; lê banco de registradores
 *         (com bypass do WB); corrige predições simples (JAL, BTFN).
 *   EX  : 2 ALUs, resolução de desvios (1 por par), AGU, DIV iterativo e
 *         FPU (multiciclo, slot 0); forwarding de MEM/WB.
 *   MEM : LSU (D-Cache 1 ciclo, store buffer, DTLB/PTW, AMO), multiplicador.
 *   WB  : commit em ordem de até 2 instruções, CSRs, traps e interrupções
 *         precisas, xRET, FENCE.I, SFENCE.VMA, WFI.
 *
 * Regras de pareamento (slot0 = mais antiga):
 *   - no máximo 1 acesso à memória, 1 desvio e 1 MUL por par;
 *   - sem dependência RAW/WAW entre as duas;
 *   - CSR, SYSTEM, FENCE, AMO, DIV, FPU e instruções com exceção emitem
 *     sozinhas no slot 0.
 * Latências: ALU 1 (forwarding total), load/MUL 2 (1 bolha load-use).
 */
module nebula_core_full #(
    parameter int          HART_ID      = 0,
    parameter logic [PMA_N-1:0][55:0] PMA_BASE = PMA_DEF_BASE,
    parameter logic [PMA_N-1:0][55:0] PMA_SIZE = PMA_DEF_SIZE,
    parameter logic [PMA_N-1:0][2:0]  PMA_ATTR = PMA_DEF_ATTR,
    parameter int          XLEN         = 64,
    parameter int          FLEN         = 64,
    parameter int          PADDR_WIDTH  = 56,
    parameter int          VADDR_WIDTH  = 39,
    parameter int          L1I_SIZE_KB  = 32,
    parameter int          L1D_SIZE_KB  = 32,
    parameter int          L1_LINE_SIZE = 64,
    parameter int          L1_WAYS      = 8
)(
    input  wire                       clk,
    input  wire                       rst_n,
    input  wire [63:0]                reset_vector,

    output logic                      imem_req,
    output logic [PADDR_WIDTH-1:0]    imem_addr,
    input  wire                       imem_ack,
    input  wire [L1_LINE_SIZE*8-1:0]  imem_data,
    input  wire                       imem_error,

    output logic                      dmem_req,
    output logic                      dmem_we,
    output logic [PADDR_WIDTH-1:0]    dmem_addr,
    output logic [L1_LINE_SIZE*8-1:0] dmem_wdata,
    output logic [L1_LINE_SIZE-1:0]   dmem_wstrb,
    output logic                      dmem_uncached,
    input  wire                       dmem_ack,
    input  wire [L1_LINE_SIZE*8-1:0]  dmem_rdata,
    input  wire                       dmem_error,
    output logic                      dmem_is_amo,
    output logic [4:0]                dmem_amo_op,
    output logic                      dmem_upgrade,

    output logic                      ptw_mem_req,
    output logic [PADDR_WIDTH-1:0]    ptw_mem_addr,
    output logic                      ptw_mem_we,
    output logic [XLEN-1:0]           ptw_mem_wdata,
    input  wire                       ptw_mem_ack,
    input  wire [XLEN-1:0]            ptw_mem_data,
    input  wire                       ptw_mem_error,

    input  snoop_req_t                snoop_req_in,
    output snoop_resp_t               snoop_resp_out,

    input  wire                       timer_irq,
    input  wire                       external_irq,
    input  wire                       software_irq,
    input  wire                       s_external_irq,
    input  wire [63:0]                mtime,
    input  wire                       debug_req,
    output logic                      debug_halted,

    // ---------------- trace de commit (verificação / co-simulação) ----------------
    output logic [1:0]                trace_valid,
    output logic [1:0][63:0]          trace_pc,
    output logic [1:0][31:0]          trace_instr,
    output logic [1:0]                trace_rd_en,
    output logic [1:0]                trace_rd_fp,
    output logic [1:0][4:0]           trace_rd,
    output logic [1:0][63:0]          trace_rd_val,
    output logic [1:0]                trace_mmio,
    output logic                      trace_trap,
    output logic                      trace_trap_intr,
    output logic [4:0]                trace_trap_cause,
    output logic [63:0]               trace_trap_epc,
    output logic [63:0]               trace_trap_tval,
    output logic [1:0]                trace_priv
);

    assign dmem_upgrade   = 1'b0;
    assign snoop_resp_out = '0;
    assign debug_halted   = 1'b0;
    assign ptw_mem_we     = 1'b0;
    assign ptw_mem_wdata  = '0;

    // =====================================================================
    // Estado CSR
    // =====================================================================
    logic [1:0]  priv;
    logic [63:0] satp;
    logic        mprv, sum, mxr, tvm, tw, tsr;
    logic [1:0]  mpp, fs;
    logic [2:0]  frm;
    logic [15:0][7:0]  pmpcfg;
    trig_t [NTRIG-1:0] trig;
    logic        st_mie, st_sie, deleg_bp;
    logic        any_ex_trig;
    logic        trig_hit_we;
    logic [NTRIG-1:0][1:0] trig_hit_val;
    logic [15:0][53:0] pmpaddr;
    logic [15:0][57:0] pmp_lo, pmp_hi;
    logic              pmp_any_locked;
    logic        irq_pending, wfi_wake, csr_tlb_flush;
    logic [4:0]  irq_cause;

    // =====================================================================
    // Sinais de controle globais
    // =====================================================================
    logic        wb_flush;
    logic [63:0] wb_redirect_pc;
    logic        wb_hold;
    logic        ex_redirect;
    logic [63:0] ex_redirect_pc;
    logic        d_redirect;
    logic [63:0] d_redirect_pc;
    logic        mem_stall, mem_adv, ex_busy, ex_adv;
    logic        tlb_flush_all;
    logic        sf_va, sf_asid;
    logic [26:0] sf_vpn;
    logic [15:0] sf_asid_val;
    logic        icache_inval;
    logic        sb_empty;

    // =====================================================================
    // PTW
    // =====================================================================
    logic        ptw_busy, ptw_resp_valid, ptw_resp_for_d, ptw_resp_page_fault;
    logic        ptw_resp_access_fault, ptw_resp_insert;
    logic [26:0] ptw_resp_vpn;
    logic [15:0] ptw_resp_asid;
    logic [43:0] ptw_resp_ppn;
    logic [1:0]  ptw_resp_level;
    logic [7:0]  ptw_resp_flags;
    logic        ptw_req_i, ptw_req_d;
    logic [26:0] ptw_vpn_i, ptw_vpn_d;

    nebula_ptw #(.PMA_BASE(PMA_BASE), .PMA_SIZE(PMA_SIZE), .PMA_ATTR(PMA_ATTR)) u_ptw (
        .clk, .rst_n,
        .satp, .sb_empty, .flush(tlb_flush_all), .pmpcfg, .pmp_lo, .pmp_hi, .pmp_any_locked,
        .req_i(ptw_req_i), .vpn_i(ptw_vpn_i),
        .req_d(ptw_req_d), .vpn_d(ptw_vpn_d),
        .busy(ptw_busy),
        .resp_valid(ptw_resp_valid), .resp_for_d(ptw_resp_for_d), .resp_vpn(ptw_resp_vpn), .resp_asid(ptw_resp_asid),
        .resp_page_fault(ptw_resp_page_fault), .resp_access_fault(ptw_resp_access_fault),
        .resp_insert(ptw_resp_insert), .resp_ppn(ptw_resp_ppn),
        .resp_level(ptw_resp_level), .resp_flags(ptw_resp_flags),
        .mem_req(ptw_mem_req), .mem_addr(ptw_mem_addr),
        .mem_ack(ptw_mem_ack), .mem_data(ptw_mem_data), .mem_error(ptw_mem_error)
    );

    // =====================================================================
    // FETCH
    // =====================================================================
    logic [3:0]        fq_v;
    logic [3:0][15:0]  fq_hw;
    logic [3:0][63:0]  fq_pc;
    logic [3:0][1:0]   fq_exc;
    logic [3:0]        fq_pt;
    logic [3:0][63:0]  fq_ptgt;
    logic [2:0]        fq_pop;

    logic        f_redirect;
    logic [63:0] f_redirect_pc;
    logic        bp_upd_valid, bp_upd_rvc, bp_upd_taken;
    logic [63:0] bp_upd_pc, bp_upd_target;
    logic [2:0]  bp_upd_type;
    logic        ras_arch_push, ras_arch_pop;
    logic [63:0] ras_arch_push_addr;
    logic        btb_inval, dec_ras_push;
    logic [63:0] btb_inval_pc, dec_ras_push_addr;

    always_comb begin
        f_redirect    = wb_flush || ex_redirect || d_redirect;
        f_redirect_pc = wb_flush ? wb_redirect_pc : ex_redirect ? ex_redirect_pc : d_redirect_pc;
    end

    nebula_fetch #(.PMA_BASE(PMA_BASE), .PMA_SIZE(PMA_SIZE), .PMA_ATTR(PMA_ATTR)) u_fetch (
        .clk, .rst_n, .reset_vector,
        .redirect(f_redirect), .redirect_pc(f_redirect_pc),
        .ras_recover(wb_flush || ex_redirect),
        .bp_upd_valid, .bp_upd_pc, .bp_upd_rvc, .bp_upd_taken, .bp_upd_target, .bp_upd_type,
        .ras_arch_push, .ras_arch_push_addr, .ras_arch_pop,
        .btb_inval, .btb_inval_pc, .dec_ras_push, .dec_ras_push_addr,
        .fq_v, .fq_hw, .fq_pc, .fq_exc, .fq_pt, .fq_ptgt, .fq_pop,
        .satp, .priv, .sf_valid(sfence_commit), .sf_va(sf_va), .sf_vpn(sf_vpn), .sf_asid(sf_asid), .sf_asid_val(sf_asid_val), .icache_inval, .pmpcfg, .pmp_lo, .pmp_hi, .pmp_any_locked,
        .ptw_req(ptw_req_i), .ptw_vpn(ptw_vpn_i), .ptw_gnt(!ptw_busy && !ptw_req_d),
        .ptw_resp_valid, .ptw_resp_for_d, .ptw_resp_vpn, .ptw_resp_asid, .ptw_resp_page_fault,
        .ptw_resp_access_fault, .ptw_resp_insert, .ptw_resp_ppn, .ptw_resp_level, .ptw_resp_flags,
        .imem_req, .imem_addr, .imem_ack, .imem_data, .imem_error
    );

    // =====================================================================
    // DECODE: extração de 2 instruções da FQ
    // =====================================================================
    // instrução 0 (entrada 0)
    logic        i0_avail, i0_is32, i0_fexc, i0_fexc2, i0_alias;
    logic [31:0] i0_raw;
    logic [1:0]  i0_len;
    logic        i0_pt;
    logic [63:0] i0_ptgt;
    // instrução 1 (entrada i0_len)
    logic        i1_avail, i1_is32, i1_fexc, i1_fexc2, i1_alias;
    logic [31:0] i1_raw;
    logic [1:0]  i1_len;
    logic        i1_pt;
    logic [63:0] i1_ptgt;
    logic [63:0] i1_pc;
    logic [1:0]  i1_k;
    logic [1:0]  i0_fcode, i1_fcode;
    logic [63:0] i0_ftval, i1_ftval;

    always_comb begin
        i0_is32  = (fq_hw[0][1:0] == 2'b11);
        i0_fexc  = fq_v[0] && (fq_exc[0] != 2'd0);
        i0_fexc2 = !i0_fexc && i0_is32 && fq_v[1] && (fq_exc[1] != 2'd0);
        i0_avail = fq_v[0] && (i0_fexc || !i0_is32 || fq_v[1]);
        i0_raw   = i0_is32 ? {fq_hw[1], fq_hw[0]} : {16'b0, fq_hw[0]};
        i0_len   = (i0_is32 && !i0_fexc) ? 2'd2 : 2'd1;
        i0_pt    = (i0_is32 && !i0_fexc) ? fq_pt[1] : fq_pt[0];
        i0_ptgt  = (i0_is32 && !i0_fexc) ? fq_ptgt[1] : fq_ptgt[0];
        i0_alias = !i0_fexc && !i0_fexc2 && i0_is32 && fq_pt[0];
        i0_fcode = i0_fexc ? fq_exc[0] : fq_exc[1];
        i0_ftval = i0_fexc ? fq_pc[0] : fq_pc[1];

        i1_k     = i0_len;
        i1_is32  = (fq_hw[i1_k][1:0] == 2'b11);
        i1_fexc  = fq_v[i1_k] && (fq_exc[i1_k] != 2'd0);
        i1_fexc2 = !i1_fexc && i1_is32 && fq_v[i1_k+1] && (fq_exc[i1_k+1] != 2'd0);
        i1_avail = i0_avail && !i0_fexc && fq_v[i1_k] && (i1_fexc || !i1_is32 || fq_v[i1_k+1]);
        i1_raw   = i1_is32 ? {fq_hw[i1_k+1], fq_hw[i1_k]} : {16'b0, fq_hw[i1_k]};
        i1_len   = (i1_is32 && !i1_fexc) ? 2'd2 : 2'd1;
        i1_pt    = (i1_is32 && !i1_fexc) ? fq_pt[i1_k+1] : fq_pt[i1_k];
        i1_ptgt  = (i1_is32 && !i1_fexc) ? fq_ptgt[i1_k+1] : fq_ptgt[i1_k];
        i1_alias = !i1_fexc && !i1_fexc2 && i1_is32 && fq_pt[i1_k];
        i1_pc    = fq_pc[i1_k];
        i1_fcode = i1_fexc ? fq_exc[i1_k] : fq_exc[i1_k+1];
        i1_ftval = i1_fexc ? fq_pc[i1_k] : fq_pc[i1_k+1];
    end

    logic [31:0] i0_exp, i1_exp;
    logic        i0_cill, i1_cill;
    nebula_rvc_expander u_rvc0 (.cinstr(i0_raw[15:0]), .instr(i0_exp), .illegal(i0_cill));
    nebula_rvc_expander u_rvc1 (.cinstr(i1_raw[15:0]), .instr(i1_exp), .illegal(i1_cill));

    uop_t d0_raw, d1_raw, d0, d1;
    nebula_decoder u_dec0 (
        .instr(i0_is32 ? i0_raw : i0_exp), .raw(i0_raw), .is_rvc(!i0_is32), .rvc_illegal(i0_cill),
        .pc(fq_pc[0]), .priv, .fs, .tvm, .tw, .tsr, .frm, .uop(d0_raw)
    );
    nebula_decoder u_dec1 (
        .instr(i1_is32 ? i1_raw : i1_exp), .raw(i1_raw), .is_rvc(!i1_is32), .rvc_illegal(i1_cill),
        .pc(i1_pc), .priv, .fs, .tvm, .tw, .tsr, .frm, .uop(d1_raw)
    );

    // aplica exceções de fetch e informação de predição
    function automatic uop_t fix_uop(input uop_t u, input logic fexc, input logic fexc2,
                                     input logic [1:0] fcode, input logic [63:0] ftval,
                                     input logic pt, input logic [63:0] ptgt);
        uop_t r;
        r = u;
        r.pred_taken  = pt;
        r.pred_target = ptgt;
        if (fexc || fexc2) begin
            r.fu        = FU_NONE;
            r.exc       = 1'b1;
            r.exc_cause = (fcode == 2'd1) ? EXC_INSTR_PAGE_FAULT : EXC_INSTR_ACCESS_FAULT;
            r.exc_tval  = ftval;
            r.rd_en = 1'b0; r.rs1_en = 1'b0; r.rs2_en = 1'b0; r.rs3_en = 1'b0;
            r.serial = 1'b0;
            r.pred_taken = 1'b0;
        end
        return r;
    endfunction

    // ---------------------------------------------------------------------
    // Correções de predição no decode
    // ---------------------------------------------------------------------
    function automatic logic is_cf(input uop_t u);
        return (u.fu == FU_BRANCH) || (u.fu == FU_JAL) || (u.fu == FU_JALR);
    endfunction

    function automatic logic is_link(input logic [4:0] r);
        return (r == 5'd1) || (r == 5'd5);
    endfunction

    logic        d0_redir, d1_redir;
    logic [63:0] d0_redir_pc, d1_redir_pc;
    logic        d0_btbinv, d1_btbinv;
    logic        d0_raspush, d1_raspush;

    task automatic dec_fix(input uop_t ui, input logic [1:0] len, output uop_t uo,
                           output logic redir, output logic [63:0] rpc,
                           output logic binv, output logic rpush);
        logic [63:0] seq, tgt;
        uo    = ui;
        redir = 1'b0;
        rpc   = '0;
        binv  = 1'b0;
        rpush = 1'b0;
        seq   = ui.pc + ((len == 2'd2) ? 64'd4 : 64'd2);
        tgt   = ui.pc + ui.imm;
        if (!ui.exc) begin
            if (ui.pred_taken && !is_cf(ui)) begin
                redir = 1'b1; rpc = seq; binv = 1'b1;
                uo.pred_taken = 1'b0;
            end else if (ui.fu == FU_JAL) begin
                if (!ui.pred_taken || ui.pred_target != tgt) begin
                    redir = 1'b1; rpc = tgt;
                    uo.pred_taken  = 1'b1;
                    uo.pred_target = tgt;
                    rpush = is_link(ui.rd) && !ui.pred_taken;
                end
            end else if (ui.fu == FU_BRANCH) begin
                if (ui.pred_taken && ui.pred_target != tgt) begin
                    redir = 1'b1; rpc = tgt;
                    uo.pred_target = tgt;
                end else if (!ui.pred_taken && ui.imm[63]) begin
                    redir = 1'b1; rpc = tgt;           // BTFN: backward taken
                    uo.pred_taken  = 1'b1;
                    uo.pred_target = tgt;
                end
            end
        end
    endtask

    // triggers de execução: por parcela de 16 bits (endereço antes do fetch,
    // dado depois), na mesma ordem do Spike
    always_comb begin
        any_ex_trig = 1'b0;
        for (int i = 0; i < NTRIG; i++) if (trig[i].valid && trig[i].ex) any_ex_trig = 1'b1;
    end

    function automatic uop_t exec_trig(input uop_t u, input logic is32, input logic [15:0] h0,
                                       input logic [15:0] h1, input logic [63:0] pc,
                                       input logic f0, input logic f1);
        uop_t      r;
        trig_res_t a;
        logic      bp;
        logic [63:0] bpa;
        logic [NTRIG-1:0][1:0] hits;
        r = u; bp = 1'b0; bpa = pc; hits = '0;
        a = trig_check(trig, 2'd0, pc, 4'd2, 1'b0, 64'd0, priv, st_mie, st_sie, deleg_bp);
        hits = hits | a.hit;
        if (a.fire) bp = 1'b1;
        else if (!f0) begin
            a = trig_check(trig, 2'd0, pc, 4'd2, 1'b1, {48'b0, h0}, priv, st_mie, st_sie, deleg_bp);
            hits = hits | a.hit;
            if (a.fire) bp = 1'b1;
            else if (is32) begin
                a = trig_check(trig, 2'd0, pc + 64'd2, 4'd2, 1'b0, 64'd0, priv, st_mie, st_sie, deleg_bp);
                hits = hits | a.hit;
                if (a.fire) begin bp = 1'b1; bpa = pc + 64'd2; end
                else if (!f1) begin
                    a = trig_check(trig, 2'd0, pc + 64'd2, 4'd2, 1'b1, {48'b0, h1}, priv, st_mie, st_sie, deleg_bp);
                    hits = hits | a.hit;
                    if (a.fire) begin bp = 1'b1; bpa = pc + 64'd2; end
                end
            end
        end
        r.trig_hit = hits;
        if (bp) begin
            r.fu = FU_NONE; r.exc = 1'b1; r.exc_cause = EXC_BREAKPOINT; r.exc_tval = bpa;
            r.rd_en = 1'b0; r.rs1_en = 1'b0; r.rs2_en = 1'b0; r.rs3_en = 1'b0;
            r.serial = 1'b0; r.pred_taken = 1'b0;
        end
        return r;
    endfunction

    always_comb begin
        uop_t t0, t1;
        t0 = fix_uop(d0_raw, i0_fexc, i0_fexc2, i0_fcode, i0_ftval, i0_pt, i0_ptgt);
        t1 = fix_uop(d1_raw, i1_fexc, i1_fexc2, i1_fcode, i1_ftval, i1_pt, i1_ptgt);
        t0.trig_hit = '0;
        t1.trig_hit = '0;
        if (any_ex_trig) begin
            t0 = exec_trig(t0, i0_is32 && !i0_fexc, fq_hw[0], fq_hw[1], fq_pc[0], i0_fexc, i0_fexc2);
            t1 = exec_trig(t1, i1_is32 && !i1_fexc, fq_hw[i1_k], fq_hw[i1_k+1], i1_pc, i1_fexc, i1_fexc2);
        end
        dec_fix(t0, i0_len, d0, d0_redir, d0_redir_pc, d0_btbinv, d0_raspush);
        dec_fix(t1, i1_len, d1, d1_redir, d1_redir_pc, d1_btbinv, d1_raspush);
    end

    // =====================================================================
    // Bancos de registradores
    // =====================================================================
    logic [63:0] xrf  [32];
    logic [63:0] fprf [32];

    // portas de escrita (WB)
    logic [1:0]       wr_en, wr_fp;
    logic [1:0][4:0]  wr_idx;
    logic [1:0][63:0] wr_val;

    function automatic logic [63:0] rf_read(input logic [4:0] idx, input logic fp,
                                            input logic [63:0] xv, input logic [63:0] fv,
                                            input logic [1:0] we, input logic [1:0] wfp,
                                            input logic [1:0][4:0] widx, input logic [1:0][63:0] wval);
        logic [63:0] v;
        v = fp ? fv : ((idx == 5'd0) ? 64'd0 : xv);
        if (we[0] && wfp[0] == fp && widx[0] == idx && (fp || idx != 5'd0)) v = wval[0];
        if (we[1] && wfp[1] == fp && widx[1] == idx && (fp || idx != 5'd0)) v = wval[1];
        return v;
    endfunction

    // =====================================================================
    // Estágios EX / MEM / WB — registradores
    // =====================================================================
    logic [1:0]       ex_v;
    uop_t             ex_u   [2];
    logic [1:0][63:0] ex_a, ex_b, ex_c;
    logic             ex_mc_started, ex_mc_done;
    logic [63:0]      ex_mc_res;
    logic [4:0]       ex_mc_flags;

    logic [1:0]       mem_v;
    uop_t             mem_u  [2];
    logic [1:0][63:0] mem_res, mem_a, mem_b;
    logic [1:0][4:0]  mem_flags;

    logic [1:0]       wb_v;
    uop_t             wb_u   [2];
    logic [1:0][63:0] wb_res, wb_b, wb_res_vaddr;
    logic [1:0]       wb_exc;
    logic [1:0][4:0]  wb_cause;
    logic [1:0][63:0] wb_tval;
    logic [1:0]       wb_replay, wb_side, wb_mmio, wb_bp_after;
    logic [1:0][NTRIG-1:0][1:0] wb_trig_hit;
    logic [1:0][4:0]  wb_flags;

    // =====================================================================
    // ISSUE
    // =====================================================================
    // visões compactas de fontes/destino (evita copiar uop_t inteiro em funções)
    typedef struct packed {
        logic [4:0] rs1, rs2, rs3;
        logic       rs1_en, rs2_en, rs3_en, rs1_fp, rs2_fp;
    } srcs_t;
    typedef struct packed {
        logic       en;
        logic [4:0] rd;
        logic       fp;
    } dst_t;

    function automatic srcs_t srcs_of(input uop_t u);
        return '{rs1: u.rs1, rs2: u.rs2, rs3: u.rs3, rs1_en: u.rs1_en, rs2_en: u.rs2_en,
                 rs3_en: u.rs3_en, rs1_fp: u.rs1_fp, rs2_fp: u.rs2_fp};
    endfunction
    function automatic dst_t dst_of(input uop_t u);
        return '{en: u.rd_en, rd: u.rd, fp: u.rd_fp};
    endfunction

    function automatic logic src_match(input logic en, input logic [4:0] idx, input logic fp,
                                       input logic pv, input dst_t p);
        return en && pv && p.en && (p.rd == idx) && (p.fp == fp) && (fp || idx != 5'd0);
    endfunction

    function automatic logic uses_reg(input srcs_t c, input logic pv, input dst_t p);
        return src_match(c.rs1_en, c.rs1, c.rs1_fp, pv, p) ||
               src_match(c.rs2_en, c.rs2, c.rs2_fp, pv, p) ||
               src_match(c.rs3_en, c.rs3, 1'b1,     pv, p);
    endfunction

    srcs_t d0_src, d1_src;
    dst_t  d0_dst, ex_dst0, ex_dst1, mem_dst0, mem_dst1, wb_dst0, wb_dst1;
    logic  ex_late0, ex_late1;
    always_comb begin
        d0_src   = srcs_of(d0);  d1_src = srcs_of(d1);  d0_dst = dst_of(d0);
        ex_dst0  = dst_of(ex_u[0]);  ex_dst1  = dst_of(ex_u[1]);
        mem_dst0 = dst_of(mem_u[0]); mem_dst1 = dst_of(mem_u[1]);
        wb_dst0  = dst_of(wb_u[0]);  wb_dst1  = dst_of(wb_u[1]);
        ex_late0 = ex_v[0] && ((ex_u[0].fu == FU_LOAD) || (ex_u[0].fu == FU_AMO) || (ex_u[0].fu == FU_MUL));
        ex_late1 = ex_v[1] && ((ex_u[1].fu == FU_LOAD) || (ex_u[1].fu == FU_AMO) || (ex_u[1].fu == FU_MUL));
    end

    function automatic logic is_late(input uop_t p);
        return (p.fu == FU_LOAD) || (p.fu == FU_AMO) || (p.fu == FU_MUL);
    endfunction

    function automatic logic alone(input uop_t u);
        return u.serial || u.exc ||
               (u.fu == FU_FPU) || (u.fu == FU_DIV) || (u.fu == FU_AMO) ||
               (u.fu == FU_CSR) || (u.fu == FU_SYS) || (u.fu == FU_NONE);
    endfunction

    function automatic logic is_mem(input uop_t u);
        return (u.fu == FU_LOAD) || (u.fu == FU_STORE) || (u.fu == FU_AMO);
    endfunction

    logic haz0, haz1, pair_ok, issue0, issue1, d_can;

    always_comb begin
        haz0 = uses_reg(d0_src, ex_late0, ex_dst0) || uses_reg(d0_src, ex_late1, ex_dst1);
        haz1 = uses_reg(d1_src, ex_late0, ex_dst0) || uses_reg(d1_src, ex_late1, ex_dst1);

        pair_ok = !alone(d0) && !alone(d1) &&
                  !(is_mem(d0) && is_mem(d1)) &&
                  !(is_cf(d0) && is_cf(d1)) &&
                  !(d0.fu == FU_MUL && d1.fu == FU_MUL) &&
                  !uses_reg(d1_src, 1'b1, d0_dst) &&
                  !(d0.rd_en && d1.rd_en && d0.rd == d1.rd && d0.rd_fp == d1.rd_fp) &&
                  !d0_redir && !i1_alias;

        d_can  = ex_adv && !wb_flush && !ex_redirect;
        issue0 = d_can && i0_avail && !i0_alias && !haz0;
        issue1 = issue0 && i1_avail && pair_ok && !haz1;
    end

    always_comb begin
        d_redirect        = 1'b0;
        d_redirect_pc     = '0;
        btb_inval         = 1'b0;
        btb_inval_pc      = '0;
        dec_ras_push      = 1'b0;
        dec_ras_push_addr = '0;
        fq_pop            = '0;
        if (d_can && i0_avail && i0_alias) begin
            // fragmento de BTB corrompeu uma instrução de 32 bits: rebusca
            d_redirect    = 1'b1;
            d_redirect_pc = fq_pc[0];
            btb_inval     = 1'b1;
            btb_inval_pc  = fq_pc[0];
        end else if (issue0) begin
            fq_pop = 3'(i0_len) + (issue1 ? 3'(i1_len) : 3'd0);
            if (d0_redir) begin
                d_redirect        = 1'b1;
                d_redirect_pc     = d0_redir_pc;
                btb_inval         = d0_btbinv;
                btb_inval_pc      = d0.pc + ((i0_len == 2'd2) ? 64'd2 : 64'd0);
                dec_ras_push      = d0_raspush;
                dec_ras_push_addr = d0.pc + ((i0_len == 2'd2) ? 64'd4 : 64'd2);
            end else if (issue1 && d1_redir) begin
                d_redirect        = 1'b1;
                d_redirect_pc     = d1_redir_pc;
                btb_inval         = d1_btbinv;
                btb_inval_pc      = d1.pc + ((i1_len == 2'd2) ? 64'd2 : 64'd0);
                dec_ras_push      = d1_raspush;
                dec_ras_push_addr = d1.pc + ((i1_len == 2'd2) ? 64'd4 : 64'd2);
            end
        end
    end

    // =====================================================================
    // EX
    // =====================================================================
    // forwarding de MEM (resultados prontos) e WB
    logic [1:0][63:0] fa, fb, fc;

    function automatic logic [63:0] fwd(input logic en, input logic [4:0] idx, input logic fp,
                                        input logic [63:0] dflt);
        logic [63:0] v;
        v = dflt;
        if (en && (fp || idx != 5'd0)) begin
            if      (src_match(1'b1, idx, fp, mem_v[1], mem_dst1)) v = mem_res[1];
            else if (src_match(1'b1, idx, fp, mem_v[0], mem_dst0)) v = mem_res[0];
            else if (src_match(1'b1, idx, fp, wb_v[1],  wb_dst1))  v = wb_res[1];
            else if (src_match(1'b1, idx, fp, wb_v[0],  wb_dst0))  v = wb_res[0];
        end
        return v;
    endfunction

    always_comb begin
        for (int s = 0; s < 2; s++) begin
            fa[s] = fwd(ex_u[s].rs1_en, ex_u[s].rs1, ex_u[s].rs1_fp, ex_a[s]);
            fb[s] = fwd(ex_u[s].rs2_en, ex_u[s].rs2, ex_u[s].rs2_fp, ex_b[s]);
            fc[s] = fwd(ex_u[s].rs3_en, ex_u[s].rs3, 1'b1,           ex_c[s]);
        end
    end

    // ALU
    function automatic logic [63:0] alu(input alu_op_t op, input logic word,
                                        input logic [63:0] a, input logic [63:0] b);
        logic [63:0] r;
        logic [5:0]  sh;
        sh = word ? {1'b0, b[4:0]} : b[5:0];
        case (op)
            ALU_ADD:  r = a + b;
            ALU_SUB:  r = a - b;
            ALU_SLL:  r = word ? {32'b0, a[31:0] << sh[4:0]} : (a << sh);
            ALU_SLT:  r = {63'b0, $signed(a) < $signed(b)};
            ALU_SLTU: r = {63'b0, a < b};
            ALU_XOR:  r = a ^ b;
            ALU_SRL:  r = word ? {32'b0, a[31:0] >> sh[4:0]} : (a >> sh);
            ALU_SRA:  r = word ? {32'b0, 32'($signed(a[31:0]) >>> sh[4:0])} : 64'($signed(a) >>> sh);
            ALU_OR:   r = a | b;
            ALU_AND:  r = a & b;
            default:  r = '0;
        endcase
        if (word) r = {{32{r[31]}}, r[31:0]};
        return r;
    endfunction

    logic [1:0][63:0] ex_res;
    logic [1:0]       br_taken;
    logic [1:0][63:0] br_target, br_seq;
    logic [1:0]       br_mispred;

    always_comb begin
        for (int s = 0; s < 2; s++) begin
            logic [63:0] opa, opb;
            opa = ex_u[s].op_a_zero ? 64'd0 : (ex_u[s].op_a_pc ? ex_u[s].pc : fa[s]);
            opb = ex_u[s].op_b_imm ? ex_u[s].imm : fb[s];
            br_seq[s] = ex_u[s].pc + (ex_u[s].is_rvc ? 64'd2 : 64'd4);

            // desvios
            br_taken[s]  = 1'b0;
            br_target[s] = ex_u[s].pc + ex_u[s].imm;
            case (ex_u[s].fu)
                FU_JAL:  br_taken[s] = 1'b1;
                FU_JALR: begin
                    br_taken[s]  = 1'b1;
                    br_target[s] = (fa[s] + ex_u[s].imm) & ~64'd1;
                end
                FU_BRANCH: begin
                    case (ex_u[s].funct3)
                        3'b000:  br_taken[s] = (fa[s] == fb[s]);
                        3'b001:  br_taken[s] = (fa[s] != fb[s]);
                        3'b100:  br_taken[s] = ($signed(fa[s]) <  $signed(fb[s]));
                        3'b101:  br_taken[s] = ($signed(fa[s]) >= $signed(fb[s]));
                        3'b110:  br_taken[s] = (fa[s] <  fb[s]);
                        default: br_taken[s] = (fa[s] >= fb[s]);
                    endcase
                end
                default: ;
            endcase
            begin
                logic [63:0] act, prd;
                act = br_taken[s] ? br_target[s] : br_seq[s];
                prd = ex_u[s].pred_taken ? ex_u[s].pred_target : br_seq[s];
                br_mispred[s] = ex_v[s] && is_cf(ex_u[s]) && (act != prd);
            end

            // resultado
            case (ex_u[s].fu)
                FU_JAL, FU_JALR:            ex_res[s] = br_seq[s];
                FU_LOAD, FU_STORE, FU_AMO:  ex_res[s] = fa[s] + ex_u[s].imm;
                FU_CSR:                     ex_res[s] = ex_u[s].funct3[2] ? ex_u[s].imm : fa[s];
                FU_SYS:                     ex_res[s] = fa[s];
                default:                    ex_res[s] = alu(ex_u[s].alu_op, ex_u[s].word, opa, opb);
            endcase
        end
    end

    // unidades multiciclo (slot 0)
    logic        ex_is_div, ex_is_fpu;
    logic        div_done, fpu_done;
    logic [63:0] div_res, fpu_res;
    logic [4:0]  fpu_flags;
    logic        mc_start;

    assign ex_is_div = ex_v[0] && ex_u[0].fu == FU_DIV;
    assign ex_is_fpu = ex_v[0] && ex_u[0].fu == FU_FPU;
    assign mc_start  = (ex_is_div || ex_is_fpu) && !ex_mc_started && !wb_flush;

    nebula_div u_div (
        .clk, .rst_n,
        .start(mc_start && ex_is_div), .kill(wb_flush),
        .funct3(ex_u[0].funct3), .word(ex_u[0].word),
        .a(fa[0]), .b(fb[0]),
        .done(div_done), .result(div_res)
    );

    nebula_fpu u_fpu (
        .clk, .rst_n,
        .start(mc_start && ex_is_fpu), .kill(wb_flush),
        .op(ex_u[0].fop), .dbl(ex_u[0].fp_dbl), .src_dbl(ex_u[0].fp_src_dbl),
        .int_sz(ex_u[0].fp_int_sz), .rm(ex_u[0].rm),
        .a(fa[0]), .b(fb[0]), .c(fc[0]),
        .done(fpu_done), .result(fpu_res), .fflags(fpu_flags)
    );

    logic        mc_now_done;
    logic [63:0] mc_res;
    logic [4:0]  mc_flags;
    always_comb begin
        mc_now_done = ex_mc_done || (ex_is_div && div_done) || (ex_is_fpu && fpu_done);
        mc_res      = ex_mc_done ? ex_mc_res   : (ex_is_div ? div_res : fpu_res);
        mc_flags    = ex_mc_done ? ex_mc_flags : (ex_is_fpu ? fpu_flags : 5'd0);
        ex_busy     = (ex_is_div || ex_is_fpu) && !mc_now_done;
    end

    // redirecionamento por erro de predição (resolvido quando EX avança)
    always_comb begin
        ex_redirect    = 1'b0;
        ex_redirect_pc = '0;
        if (ex_adv && !wb_flush) begin
            if (br_mispred[0]) begin
                ex_redirect    = 1'b1;
                ex_redirect_pc = br_taken[0] ? br_target[0] : br_seq[0];
            end else if (br_mispred[1]) begin
                ex_redirect    = 1'b1;
                ex_redirect_pc = br_taken[1] ? br_target[1] : br_seq[1];
            end
        end
    end

    // atualização do preditor
    logic bp_lnk, bp_ret;
    always_comb begin
        int s;
        bp_upd_valid = 1'b0;
        bp_upd_pc = '0; bp_upd_rvc = 1'b0; bp_upd_taken = 1'b0; bp_upd_target = '0; bp_upd_type = '0;
        ras_arch_push = 1'b0; ras_arch_pop = 1'b0; ras_arch_push_addr = '0;
        s = (ex_v[0] && is_cf(ex_u[0])) ? 0 : 1;
        bp_lnk = is_link(ex_u[s].rd);
        bp_ret = (ex_u[s].fu == FU_JALR) && !bp_lnk && is_link(ex_u[s].rs1);
        if (ex_adv && !wb_flush && ex_v[s] && is_cf(ex_u[s])) begin
            bp_upd_valid  = 1'b1;
            bp_upd_pc     = ex_u[s].pc;
            bp_upd_rvc    = ex_u[s].is_rvc;
            bp_upd_taken  = br_taken[s];
            bp_upd_target = br_target[s];
            if (ex_u[s].fu == FU_BRANCH) bp_upd_type = 3'd0;
            else if (bp_lnk)             bp_upd_type = 3'd2;
            else if (bp_ret)             bp_upd_type = 3'd3;
            else if (ex_u[s].fu == FU_JAL) bp_upd_type = 3'd1;
            else                         bp_upd_type = 3'd4;
            ras_arch_push      = (ex_u[s].fu != FU_BRANCH) && bp_lnk;
            ras_arch_push_addr = br_seq[s];
            ras_arch_pop       = bp_ret;
        end
    end

    // =====================================================================
    // MEM
    // =====================================================================
    logic        lsu_done, lsu_exc, lsu_replay;
    logic [63:0] lsu_result, lsu_tval;
    logic [4:0]  lsu_cause;
    logic        lsu_valid;
    logic        ms;            // slot com a operação de memória
    logic        side_ok;
    logic        lsu_side, lsu_mmio, lsu_bp_after;
    logic [NTRIG-1:0][1:0] lsu_trig_hit;

    always_comb begin
        ms        = !(mem_v[0] && is_mem(mem_u[0]));
        lsu_valid = mem_v[ms] && is_mem(mem_u[ms]) && !mem_u[ms].exc;
    end

    assign side_ok = !wb_flush && !irq_pending && !wb_hold;

    nebula_lsu #(.PMA_BASE(PMA_BASE), .PMA_SIZE(PMA_SIZE), .PMA_ATTR(PMA_ATTR)) u_lsu (
        .clk, .rst_n,
        .req_valid(lsu_valid), .req_new(1'b0), .req_fu(mem_u[ms].fu),
        .req_size(mem_u[ms].funct3), .req_fp(mem_u[ms].mem_fp), .req_amo_op(mem_u[ms].amo_op),
        .req_vaddr(mem_res[ms]), .req_wdata(mem_b[ms]),
        .side_ok, .kill(wb_flush),
        .done(lsu_done), .result(lsu_result), .exc(lsu_exc), .exc_cause(lsu_cause),
        .exc_tval(lsu_tval), .replay(lsu_replay), .side_effect(lsu_side), .mmio(lsu_mmio),
        .bp_after(lsu_bp_after), .trig_hit(lsu_trig_hit), .trig,
        .trig_mie(st_mie), .trig_sie(st_sie), .trig_deleg(deleg_bp),
        .satp, .priv, .mprv, .mpp, .sum, .mxr, .sf_valid(sfence_commit), .sf_va(sf_va), .sf_vpn(sf_vpn), .sf_asid(sf_asid), .sf_asid_val(sf_asid_val), .pmpcfg, .pmp_lo, .pmp_hi, .pmp_any_locked,
        .ptw_req(ptw_req_d), .ptw_vpn(ptw_vpn_d), .ptw_gnt(!ptw_busy),
        .ptw_resp_valid, .ptw_resp_for_d, .ptw_resp_vpn, .ptw_resp_asid, .ptw_resp_page_fault,
        .ptw_resp_access_fault, .ptw_resp_insert, .ptw_resp_ppn, .ptw_resp_level, .ptw_resp_flags,
        .sb_empty,
        .snoop_inv_valid(snoop_req_in.valid && snoop_req_in.op == COH_INVALIDATE),
        .snoop_inv_line(snoop_req_in.addr[55:6]),
        .dmem_req, .dmem_we, .dmem_addr, .dmem_wdata, .dmem_wstrb, .dmem_uncached,
        .dmem_is_amo, .dmem_amo_op,
        .dmem_ack, .dmem_rdata, .dmem_error
    );

    // multiplicador (estágio MEM)
    logic [1:0][63:0] mul_res;
    always_comb begin
        for (int s = 0; s < 2; s++) begin
            logic [129:0] p;
            logic [63:0]  a, b;
            logic         sa, sb;
            a = mem_a[s]; b = mem_b[s];
            sa = (mem_u[s].funct3 == 3'b001) || (mem_u[s].funct3 == 3'b010);
            sb = (mem_u[s].funct3 == 3'b001);
            p  = 130'($signed({sa & a[63], a}) * $signed({sb & b[63], b}));
            case (mem_u[s].funct3)
                3'b000:  mul_res[s] = mem_u[s].word ? {{32{p[31]}}, p[31:0]} : p[63:0];
                default: mul_res[s] = p[127:64];
            endcase
        end
    end

    assign mem_stall = lsu_valid && !lsu_done;
    assign mem_adv   = !wb_hold && !mem_stall;
    assign ex_adv    = mem_adv && !ex_busy;

    // =====================================================================
    // WB / COMMIT
    // =====================================================================
    logic        csr_illegal;
    logic [63:0] csr_rdata, trap_target, xret_target;
    logic        trap_valid, trap_intr;
    logic [4:0]  trap_cause;
    logic [63:0] trap_epc, trap_tval;
    logic        do_mret, do_sret;
    logic [1:0]  commit;
    logic        csr_commit;
    logic        sfence_commit;
    logic        fencei_commit;
    logic        serial_flush;
    logic [63:0] serial_pc;
    logic        replay_flush;
    logic [63:0] replay_pc;
    logic [63:0] nxt;
    logic        exc_slot0, exc_slot1;

    always_comb begin
        trap_valid = 1'b0; trap_intr = 1'b0; trap_cause = '0; trap_epc = '0; trap_tval = '0;
        exc_slot0 = 1'b0; exc_slot1 = 1'b0;
        do_mret = 1'b0; do_sret = 1'b0;
        commit = 2'b00;
        csr_commit = 1'b0;
        sfence_commit = 1'b0;
        fencei_commit = 1'b0;
        serial_flush = 1'b0;
        serial_pc = '0;
        replay_flush = 1'b0;
        replay_pc = '0;
        wb_hold = 1'b0;

        nxt = wb_u[0].pc + (wb_u[0].is_rvc ? 64'd2 : 64'd4);
        if (wb_v[0]) begin
            if (irq_pending && !wb_side[0]) begin
                trap_valid = 1'b1; trap_intr = 1'b1; trap_cause = irq_cause;
                trap_epc = wb_u[0].pc; trap_tval = '0;
            end else if (wb_exc[0]) begin
                exc_slot0 = 1'b1;
                trap_valid = 1'b1; trap_cause = wb_cause[0];
                trap_epc = wb_u[0].pc; trap_tval = wb_tval[0];
            end else if (wb_replay[0]) begin
                replay_flush = 1'b1; replay_pc = wb_u[0].pc;
            end else if (wb_u[0].fu == FU_CSR) begin
                if (csr_illegal) begin
                    exc_slot0 = 1'b1;
                    trap_valid = 1'b1; trap_cause = EXC_ILLEGAL_INSTR;
                    trap_epc = wb_u[0].pc; trap_tval = {32'b0, wb_u[0].instr};
                end else begin
                    commit[0] = 1'b1; csr_commit = 1'b1;
                    serial_flush = 1'b1; serial_pc = nxt;
                end
            end else if (wb_u[0].fu == FU_SYS) begin
                case (wb_u[0].sys)
                    SYS_MRET: begin commit[0] = 1'b1; do_mret = 1'b1; end
                    SYS_SRET: begin commit[0] = 1'b1; do_sret = 1'b1; end
                    SYS_WFI: begin
                        if (!wfi_wake) wb_hold = 1'b1;
                        else begin commit[0] = 1'b1; serial_flush = 1'b1; serial_pc = nxt; end
                    end
                    SYS_FENCEI: begin
                        if (!sb_empty) wb_hold = 1'b1;
                        else begin
                            commit[0] = 1'b1; fencei_commit = 1'b1;
                            serial_flush = 1'b1; serial_pc = nxt;
                        end
                    end
                    SYS_SFENCE: begin
                        commit[0] = 1'b1; sfence_commit = 1'b1;
                        serial_flush = 1'b1; serial_pc = nxt;
                    end
                    default: begin   // FENCE: ordena acessos -> drena o store buffer
                        if (!sb_empty) wb_hold = 1'b1;
                        else commit[0] = 1'b1;
                    end
                endcase
            end else begin
                commit[0] = 1'b1;
            end

            // slot 1
            if (commit[0] && !serial_flush && !do_mret && !do_sret && wb_v[1]) begin
                if (irq_pending && !wb_side[1]) begin
                    trap_valid = 1'b1; trap_intr = 1'b1; trap_cause = irq_cause;
                    trap_epc = wb_u[1].pc; trap_tval = '0;
                end else if (wb_exc[1]) begin
                    exc_slot1 = 1'b1;
                    trap_valid = 1'b1; trap_cause = wb_cause[1];
                    trap_epc = wb_u[1].pc; trap_tval = wb_tval[1];
                end else if (wb_replay[1]) begin
                    replay_flush = 1'b1; replay_pc = wb_u[1].pc;
                end else begin
                    commit[1] = 1'b1;
                end
            end
        end

        // trigger com temporização "após": a instrução efetiva e o trap ocorre na seguinte
        if (!trap_valid) begin
            if (commit[0] && wb_bp_after[0]) begin
                commit[1]  = 1'b0;
                trap_valid = 1'b1; trap_intr = 1'b0; trap_cause = EXC_BREAKPOINT;
                trap_epc   = nxt; trap_tval = wb_res_vaddr[0];
                serial_flush = 1'b0;
            end else if (commit[1] && wb_bp_after[1]) begin
                trap_valid = 1'b1; trap_intr = 1'b0; trap_cause = EXC_BREAKPOINT;
                trap_epc   = wb_u[1].pc + (wb_u[1].is_rvc ? 64'd2 : 64'd4);
                trap_tval  = wb_res_vaddr[1];
            end
        end

        // hits de triggers: aplicados quando a instrução efetiva ou gera exceção
        trig_hit_we  = 1'b0;
        trig_hit_val = '0;
        if (commit[0] || exc_slot0) begin
            trig_hit_we = 1'b1; trig_hit_val = trig_hit_val | wb_trig_hit[0];
        end
        if (commit[1] || exc_slot1) begin
            trig_hit_we = 1'b1; trig_hit_val = trig_hit_val | wb_trig_hit[1];
        end

        wb_flush = trap_valid || serial_flush || replay_flush || do_mret || do_sret;
        wb_redirect_pc = trap_valid ? trap_target :
                         (do_mret || do_sret) ? xret_target :
                         serial_flush ? serial_pc : replay_pc;
    end

    // PTW: descarta walks em andamento em SFENCE.VMA ou troca de satp
    assign tlb_flush_all = sfence_commit || csr_tlb_flush;
    // operandos do SFENCE.VMA: rs1 (VA) em wb_res, rs2 (ASID) em wb_b
    assign sf_va       = wb_u[0].instr[19:15] != 5'd0;
    assign sf_vpn      = wb_res[0][38:12];
    assign sf_asid     = wb_u[0].instr[24:20] != 5'd0;
    assign sf_asid_val = wb_b[0][15:0];
    assign icache_inval  = fencei_commit;

    // escrita nos registradores
    always_comb begin
        for (int s = 0; s < 2; s++) begin
            wr_en[s]  = commit[s] && wb_u[s].rd_en;
            wr_fp[s]  = wb_u[s].rd_fp;
            wr_idx[s] = wb_u[s].rd;
            wr_val[s] = (s == 0 && wb_u[0].fu == FU_CSR) ? csr_rdata : wb_res[s];
        end
    end

    logic        fp_dirty;
    logic [4:0]  fflags_set;
    assign fp_dirty   = (wr_en[0] && wr_fp[0]) || (wr_en[1] && wr_fp[1]);
    assign fflags_set = (commit[0] && wb_u[0].fu == FU_FPU) ? wb_flags[0] : 5'd0;

    nebula_csr #(.HART_ID(HART_ID)) u_csr (
        .clk, .rst_n,
        .csr_addr(wb_u[0].csr), .csr_op(wb_u[0].funct3), .csr_wsrc(wb_res[0]),
        .csr_wnz(wb_u[0].instr[19:15] != 5'd0), .csr_commit,
        .csr_rdata, .csr_illegal,
        .trap_valid, .trap_intr, .trap_cause, .trap_epc, .trap_tval, .trap_target,
        .mret(do_mret), .sret(do_sret), .xret_target,
        .instret_inc({1'b0, commit[0]} + {1'b0, commit[1]}),
        .fp_dirty, .fflags_set,
        .irq_mtip(timer_irq), .irq_msip(software_irq), .irq_meip(external_irq),
        .irq_seip(s_external_irq), .mtime,
        .irq_pending, .irq_cause, .wfi_wake,
        .priv, .satp, .mprv, .mpp, .sum, .mxr, .tvm, .tw, .tsr, .fs, .frm,
        .pmpcfg, .pmpaddr, .pmp_lo, .pmp_hi, .pmp_any_locked,
        .trig, .mie_out(st_mie), .sie_out(st_sie), .deleg_bp, .trig_hit_we, .trig_hit_val,
        .tlb_flush(csr_tlb_flush)
    );

    // =====================================================================
    // Sequencial: avanço do pipeline
    // =====================================================================
    always_ff @(posedge clk) begin
        for (int s = 0; s < 2; s++)
            if (wr_en[s]) begin
                if (wr_fp[s]) fprf[wr_idx[s]] <= wr_val[s];
                else if (wr_idx[s] != 5'd0) xrf[wr_idx[s]] <= wr_val[s];
            end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ex_v <= '0; mem_v <= '0; wb_v <= '0;
            ex_mc_started <= 1'b0; ex_mc_done <= 1'b0; ex_mc_res <= '0; ex_mc_flags <= '0;
            for (int s = 0; s < 2; s++) begin
                ex_u[s] <= '0; mem_u[s] <= '0; wb_u[s] <= '0;
            end
            ex_a <= '0; ex_b <= '0; ex_c <= '0;
            mem_res <= '0; mem_a <= '0; mem_b <= '0; mem_flags <= '0;
            wb_res <= '0; wb_b <= '0; wb_res_vaddr <= '0; wb_exc <= '0; wb_cause <= '0; wb_tval <= '0;
            wb_replay <= '0; wb_side <= '0; wb_mmio <= '0; wb_flags <= '0;
            wb_bp_after <= '0; wb_trig_hit <= '0;
        end else begin
            // ---------------- MEM -> WB ----------------
            if (wb_flush || !mem_adv) begin
                if (wb_flush || !wb_hold) wb_v <= '0;
            end else begin
                for (int s = 0; s < 2; s++) begin
                    logic is_ls;
                    is_ls = (logic'(s) == ms) && lsu_valid;
                    wb_v[s]      <= mem_v[s];
                    wb_u[s]      <= mem_u[s];
                    wb_flags[s]  <= mem_flags[s];
                    wb_b[s]      <= mem_b[s];
                    wb_res_vaddr[s] <= mem_res[s];
                    wb_exc[s]    <= mem_u[s].exc || (is_ls && lsu_exc);
                    wb_cause[s]  <= mem_u[s].exc ? mem_u[s].exc_cause : lsu_cause;
                    wb_tval[s]   <= mem_u[s].exc ? mem_u[s].exc_tval  : lsu_tval;
                    wb_replay[s] <= is_ls && lsu_replay;
                    wb_side[s]   <= is_ls && lsu_side;
                    wb_mmio[s]   <= is_ls && lsu_mmio;
                    wb_bp_after[s] <= is_ls && lsu_bp_after && !lsu_exc;
                    wb_trig_hit[s] <= mem_u[s].trig_hit | (is_ls ? lsu_trig_hit : '0);
                    if (is_ls)                     wb_res[s] <= lsu_result;
                    else if (mem_u[s].fu == FU_MUL) wb_res[s] <= mul_res[s];
                    else                           wb_res[s] <= mem_res[s];
                end
            end

            // ---------------- EX -> MEM ----------------
            if (wb_flush) begin
                mem_v <= '0;
            end else if (mem_adv) begin
                if (ex_adv) begin
                    mem_v[0] <= ex_v[0];
                    mem_v[1] <= ex_v[1] && !br_mispred[0];
                    for (int s = 0; s < 2; s++) begin
                        mem_u[s]   <= ex_u[s];
                        mem_a[s]   <= fa[s];
                        mem_b[s]   <= fb[s];
                        mem_res[s] <= ex_res[s];
                        mem_flags[s] <= 5'd0;
                    end
                    if (ex_is_div || ex_is_fpu) begin
                        mem_res[0]   <= mc_res;
                        mem_flags[0] <= mc_flags;
                    end
                end else begin
                    mem_v <= '0;   // EX ocupado (DIV/FPU): bolha
                end
            end

            // ---------------- D -> EX ----------------
            if (wb_flush) begin
                ex_v <= '0;
                ex_mc_started <= 1'b0;
                ex_mc_done    <= 1'b0;
            end else if (ex_adv) begin
                ex_v[0] <= issue0;
                ex_v[1] <= issue1;
                ex_mc_started <= 1'b0;
                ex_mc_done    <= 1'b0;
                if (issue0) begin
                    ex_u[0] <= d0;
                    ex_a[0] <= rf_read(d0.rs1, d0.rs1_fp, xrf[d0.rs1], fprf[d0.rs1], wr_en, wr_fp, wr_idx, wr_val);
                    ex_b[0] <= rf_read(d0.rs2, d0.rs2_fp, xrf[d0.rs2], fprf[d0.rs2], wr_en, wr_fp, wr_idx, wr_val);
                    ex_c[0] <= rf_read(d0.rs3, 1'b1,      xrf[d0.rs3], fprf[d0.rs3], wr_en, wr_fp, wr_idx, wr_val);
                end
                if (issue1) begin
                    ex_u[1] <= d1;
                    ex_a[1] <= rf_read(d1.rs1, d1.rs1_fp, xrf[d1.rs1], fprf[d1.rs1], wr_en, wr_fp, wr_idx, wr_val);
                    ex_b[1] <= rf_read(d1.rs2, d1.rs2_fp, xrf[d1.rs2], fprf[d1.rs2], wr_en, wr_fp, wr_idx, wr_val);
                    ex_c[1] <= rf_read(d1.rs3, 1'b1,      xrf[d1.rs3], fprf[d1.rs3], wr_en, wr_fp, wr_idx, wr_val);
                end
            end else begin
                // EX parado: captura operandos encaminhados (produtores podem sair do WB)
                ex_a <= fa;
                ex_b <= fb;
                ex_c <= fc;
                if (mc_start) ex_mc_started <= 1'b1;
                if (!ex_mc_done && ((ex_is_div && div_done) || (ex_is_fpu && fpu_done))) begin
                    ex_mc_done  <= 1'b1;
                    ex_mc_res   <= mc_res;
                    ex_mc_flags <= mc_flags;
                end
            end
        end
    end

    // =====================================================================
    // Trace
    // =====================================================================
    always_comb begin
        for (int s = 0; s < 2; s++) begin
            trace_valid[s]  = commit[s];
            trace_pc[s]     = wb_u[s].pc;
            trace_instr[s]  = wb_u[s].instr;
            trace_rd_en[s]  = wr_en[s];
            trace_rd_fp[s]  = wr_fp[s];
            trace_rd[s]     = wr_idx[s];
            trace_rd_val[s] = wr_val[s];
            trace_mmio[s]   = wb_mmio[s];
        end
        trace_trap       = trap_valid;
        trace_trap_intr  = trap_intr;
        trace_trap_cause = trap_cause;
        trace_trap_epc   = trap_epc;
        trace_trap_tval  = trap_tval;
        trace_priv       = priv;
    end

endmodule
