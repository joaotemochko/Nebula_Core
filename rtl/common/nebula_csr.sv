`timescale 1ns/1ps
`default_nettype none

import nebula_pkg::*;

/**
 * @module nebula_csr
 * @brief CSRs RV64 privilegiados (M/S/U) + FP CSRs + contadores.
 *
 * Executado no estágio WB (commit), portanto todas as atualizações são
 * precisas. Fornece: leitura/escrita de CSR com checagem de privilégio,
 * entrada em trap (com delegação medeleg/mideleg e mtvec/stvec vetorizado),
 * MRET/SRET, cálculo de interrupção pendente e estado para MMU/decodificador.
 *
 * PMP com 16 regiões (granularidade 4 B). Contadores HPM leem zero.
 */
module nebula_csr #(
    parameter int HART_ID = 0
)(
    input  wire          clk,
    input  wire          rst_n,

    // acesso a CSR (instrução no WB)
    input  wire  [11:0]  csr_addr,
    input  wire  [2:0]   csr_op,       // funct3
    input  wire  [63:0]  csr_wsrc,     // rs1 ou zimm
    input  wire          csr_wnz,      // rs1 != x0 / zimm != 0
    input  wire          csr_commit,   // efetivar a escrita
    output logic [63:0]  csr_rdata,
    output logic         csr_illegal,

    // trap
    input  wire          trap_valid,
    input  wire          trap_intr,
    input  wire  [4:0]   trap_cause,
    input  wire  [63:0]  trap_epc,
    input  wire  [63:0]  trap_tval,
    output logic [63:0]  trap_target,

    input  wire          mret,
    input  wire          sret,
    output logic [63:0]  xret_target,

    // contadores / FP
    input  wire  [1:0]   instret_inc,
    input  wire          fp_dirty,
    input  wire  [4:0]   fflags_set,

    // interrupções
    input  wire          irq_mtip,
    input  wire          irq_msip,
    input  wire          irq_meip,
    input  wire          irq_seip,
    input  wire  [63:0]  mtime,
    output logic         irq_pending,  // interrupção habilitada e pendente
    output logic [4:0]   irq_cause,
    output logic         wfi_wake,

    // estado
    output logic [1:0]   priv,
    output logic [63:0]  satp,
    output logic         mprv,
    output logic [1:0]   mpp,
    output logic         sum,
    output logic         mxr,
    output logic         tvm,
    output logic         tw,
    output logic         tsr,
    output logic [1:0]   fs,
    output logic [2:0]   frm,
    output logic [15:0][7:0]  pmpcfg,
    // triggers de debug
    output trig_t [NTRIG-1:0] trig,
    output logic         mie_out,
    output logic         sie_out,
    output logic         deleg_bp,
    input  wire          trig_hit_we,
    input  wire  [NTRIG-1:0][1:0] trig_hit_val,
    output logic [15:0][53:0] pmpaddr,
    output logic [15:0][57:0] pmp_lo,
    output logic [15:0][57:0] pmp_hi,
    output logic              pmp_any_locked,
    output logic         tlb_flush     // escrita em satp (descarta walks em andamento)
);
    // ---------------------------------------------------------------------
    // Registradores
    // ---------------------------------------------------------------------
    logic [1:0]  priv_q;
    // mstatus em campos
    logic        st_sie, st_mie, st_spie, st_mpie, st_spp;
    logic [1:0]  st_mpp, st_fs;
    logic        st_mprv, st_sum, st_mxr, st_tvm, st_tw, st_tsr;

    logic [63:0] medeleg_q, mideleg_q, mie_q;
    logic        mip_ssip, mip_stip, mip_seip;
    logic [63:0] mtvec_q, stvec_q;
    logic [63:0] mscratch_q, sscratch_q;
    logic [63:0] mepc_q, sepc_q;
    logic [63:0] mcause_q, scause_q;
    logic [63:0] mtval_q, stval_q;
    logic [31:0] mcounteren_q, scounteren_q;
    logic [63:0] satp_q;
    logic        menvcfg_fiom, senvcfg_fiom;
    logic [2:0]  mcountinhibit_q;  // bits 0 (CY) e 2 (IR); bit 1 fixo 0
    logic [63:0] mcycle_q, minstret_q;
    logic [4:0]  fflags_q;
    logic [2:0]  frm_q;
    // triggers
    logic [1:0]               tsel_q;
    logic [NTRIG-1:0][1:0]    t_type;    // 0 desabilitado, 1 mcontrol, 2 mcontrol6
    logic [NTRIG-1:0][1:0]    t_hit;
    trig_t [NTRIG-1:0]        t_cfg;
    always_comb begin
        for (int i = 0; i < NTRIG; i++) begin
            trig[i]       = t_cfg[i];
            trig[i].valid = (t_type[i] != 2'd0);
            trig[i].is6   = (t_type[i] == 2'd2);
        end
    end
    assign mie_out  = st_mie;
    assign sie_out  = st_sie;
    assign deleg_bp = medeleg_q[3];

    logic [63:0] tdata1_val;
    always_comb begin
        trig_t t;
        t = t_cfg[tsel_q];
        tdata1_val = '0;
        case (t_type[tsel_q])
            2'd1: begin
                tdata1_val[63:60] = 4'd2;
                tdata1_val[20]    = t_hit[tsel_q][0];
                tdata1_val[19]    = t.select;
                tdata1_val[18]    = t.timing;
                tdata1_val[15:12] = t.action;
                tdata1_val[11]    = t.chain;
                tdata1_val[10:7]  = t.match;
                tdata1_val[6]     = t.m;
                tdata1_val[4]     = t.s;
                tdata1_val[3]     = t.u;
                tdata1_val[2]     = t.ex;
                tdata1_val[1]     = t.st;
                tdata1_val[0]     = t.ld;
            end
            2'd2: begin
                tdata1_val[63:60] = 4'd6;
                tdata1_val[25]    = t_hit[tsel_q][1];
                tdata1_val[22]    = t_hit[tsel_q][0];
                tdata1_val[21]    = t.select;
                tdata1_val[15:12] = t.action;
                tdata1_val[11]    = t.chain;
                tdata1_val[10:7]  = t.match;
                tdata1_val[6]     = t.m;
                tdata1_val[4]     = t.s;
                tdata1_val[3]     = t.u;
                tdata1_val[2]     = t.ex;
                tdata1_val[1]     = t.st;
                tdata1_val[0]     = t.ld;
            end
            default: tdata1_val[63:60] = 4'd15;
        endcase
    end

    logic [15:0][7:0]  pmpcfg_q;
    logic [15:0][53:0] pmpaddr_q;
    assign pmpcfg  = pmpcfg_q;
    assign pmpaddr = pmpaddr_q;

    // regiões PMP pré-decodificadas em intervalos [lo, hi); recalculadas
    // somente após escrita em CSR de PMP (o flush do pipeline cobre o atraso)
    logic [15:0][57:0] pmp_lo_q, pmp_hi_q;
    logic              pmp_dirty, pmp_locked_q;
    assign pmp_lo = pmp_lo_q;
    assign pmp_hi = pmp_hi_q;
    assign pmp_any_locked = pmp_locked_q;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pmp_lo_q <= '0; pmp_hi_q <= '0; pmp_locked_q <= 1'b0;
        end else if (pmp_dirty) begin
            logic lk;
            lk = 1'b0;
            for (int i = 0; i < 16; i++) begin
                logic [57:0] base, sz, m;
                if (pmpcfg_q[i][7] && pmpcfg_q[i][4:3] != 2'b00) lk = 1'b1;
                case (pmpcfg_q[i][4:3])
                    2'b01: begin
                        pmp_lo_q[i] <= (i == 0) ? 58'd0 : {2'b00, pmpaddr_q[i-1], 2'b00};
                        pmp_hi_q[i] <= {2'b00, pmpaddr_q[i], 2'b00};
                    end
                    2'b10: begin
                        pmp_lo_q[i] <= {2'b00, pmpaddr_q[i], 2'b00};
                        pmp_hi_q[i] <= {2'b00, pmpaddr_q[i], 2'b00} + 58'd4;
                    end
                    2'b11: begin
                        m    = {3'b000, pmpaddr_q[i], 1'b1};
                        sz   = ((m & ~(m + 58'd1)) + 58'd1) << 2;      // 2^(uns finais + 3)
                        base = {2'b00, pmpaddr_q[i], 2'b00} & ~(sz - 58'd1);
                        pmp_lo_q[i] <= base;
                        pmp_hi_q[i] <= base + sz;
                    end
                    default: begin pmp_lo_q[i] <= '0; pmp_hi_q[i] <= '0; end
                endcase
            end
            pmp_locked_q <= lk;
        end
    end

    localparam logic [63:0] MISA = 64'h8000_0000_0014_112D; // RV64 IMAFDC SU
    localparam logic [63:0] MEDELEG_MASK = 64'h0000_0000_000C_B3FE;
    localparam logic [63:0] MIDELEG_MASK = 64'h0000_0000_0000_0222;
    localparam logic [63:0] MIE_MASK     = 64'h0000_0000_0000_0AAA;

    assign priv = priv_q;
    assign satp = satp_q;
    assign mprv = st_mprv;
    assign mpp  = st_mpp;
    assign sum  = st_sum;
    assign mxr  = st_mxr;
    assign tvm  = st_tvm;
    assign tw   = st_tw;
    assign tsr  = st_tsr;
    assign fs   = st_fs;
    assign frm  = frm_q;

    logic sd;
    assign sd = (st_fs == 2'b11);

    logic [63:0] mstatus_val, sstatus_val;
    always_comb begin
        mstatus_val = '0;
        mstatus_val[1]     = st_sie;
        mstatus_val[3]     = st_mie;
        mstatus_val[5]     = st_spie;
        mstatus_val[7]     = st_mpie;
        mstatus_val[8]     = st_spp;
        mstatus_val[12:11] = st_mpp;
        mstatus_val[14:13] = st_fs;
        mstatus_val[17]    = st_mprv;
        mstatus_val[18]    = st_sum;
        mstatus_val[19]    = st_mxr;
        mstatus_val[20]    = st_tvm;
        mstatus_val[21]    = st_tw;
        mstatus_val[22]    = st_tsr;
        mstatus_val[33:32] = 2'b10;   // UXL
        mstatus_val[35:34] = 2'b10;   // SXL
        mstatus_val[63]    = sd;

        sstatus_val = '0;
        sstatus_val[1]     = st_sie;
        sstatus_val[5]     = st_spie;
        sstatus_val[8]     = st_spp;
        sstatus_val[14:13] = st_fs;
        sstatus_val[18]    = st_sum;
        sstatus_val[19]    = st_mxr;
        sstatus_val[33:32] = 2'b10;
        sstatus_val[63]    = sd;
    end

    logic [63:0] mip_val;
    always_comb begin
        mip_val = '0;
        mip_val[1]  = mip_ssip;
        mip_val[3]  = irq_msip;
        mip_val[5]  = mip_stip;
        mip_val[7]  = irq_mtip;
        mip_val[9]  = mip_seip | irq_seip;
        mip_val[11] = irq_meip;
    end

    // ---------------------------------------------------------------------
    // Leitura + checagem de acesso
    // ---------------------------------------------------------------------
    logic        is_write;
    logic        exists;
    logic        priv_ok;
    logic        counter_ok;
    logic [63:0] rdata;

    assign is_write = (csr_op[1:0] == 2'b01) || csr_wnz;
    assign priv_ok  = (priv_q >= csr_addr[9:8]);

    // acesso a contadores de usuário (0xC00-0xC1F)
    always_comb begin
        logic [4:0] ci;
        ci = csr_addr[4:0];
        counter_ok = 1'b1;
        if (priv_q == 2'b01) counter_ok = mcounteren_q[ci];
        else if (priv_q == 2'b00) counter_ok = mcounteren_q[ci] && scounteren_q[ci];
    end

    always_comb begin
        rdata  = '0;
        exists = 1'b1;
        case (csr_addr)
            12'h001: rdata = {59'b0, fflags_q};
            12'h002: rdata = {61'b0, frm_q};
            12'h003: rdata = {56'b0, frm_q, fflags_q};
            12'hC00: rdata = mcycle_q;
            12'hC01: rdata = mtime;
            12'hC02: rdata = minstret_q;
            12'h100: rdata = sstatus_val;
            12'h104: rdata = mie_q & mideleg_q;
            12'h105: rdata = stvec_q;
            12'h106: rdata = {32'b0, scounteren_q};
            12'h10A: rdata = {63'b0, senvcfg_fiom};
            12'h140: rdata = sscratch_q;
            12'h141: rdata = sepc_q;
            12'h142: rdata = scause_q;
            12'h143: rdata = stval_q;
            12'h144: rdata = mip_val & mideleg_q;
            12'h180: rdata = satp_q;
            12'h300: rdata = mstatus_val;
            12'h301: rdata = MISA;
            12'h302: rdata = medeleg_q;
            12'h303: rdata = mideleg_q;
            12'h304: rdata = mie_q;
            12'h305: rdata = mtvec_q;
            12'h306: rdata = {32'b0, mcounteren_q};
            12'h30A: rdata = {63'b0, menvcfg_fiom};
            12'h320: rdata = {61'b0, mcountinhibit_q};
            12'h340: rdata = mscratch_q;
            12'h341: rdata = mepc_q;
            12'h342: rdata = mcause_q;
            12'h343: rdata = mtval_q;
            12'h344: rdata = mip_val;
            12'h3A0: rdata = pmpcfg_q[7:0];
            12'h3A2: rdata = pmpcfg_q[15:8];
            12'hB00: rdata = mcycle_q;
            12'hB02: rdata = minstret_q;
            12'hF11, 12'hF12, 12'hF13, 12'hF15: rdata = '0;
            12'hF14: rdata = 64'(HART_ID);
            12'h7A0: rdata = {62'b0, tsel_q};
            12'h7A1: rdata = tdata1_val;
            12'h7A2: rdata = t_cfg[tsel_q].tdata2;
            12'h7A3: rdata = '0;                                     // tdata3 (textra): sem filtros
            12'h7A4: rdata = 64'h0000_0000_0100_8044;                // tinfo: v1, tipos 2, 6, 15
            default: begin
                if (csr_addr >= 12'hC03 && csr_addr <= 12'hC1F) rdata = '0;       // hpmcounter
                else if (csr_addr >= 12'hB03 && csr_addr <= 12'hB1F) rdata = '0;  // mhpmcounter
                else if (csr_addr >= 12'h323 && csr_addr <= 12'h33F) rdata = '0;  // mhpmevent
                else if (csr_addr >= 12'h3B0 && csr_addr <= 12'h3BF) rdata = {10'b0, pmpaddr_q[csr_addr[3:0]]};
                else if (csr_addr >= 12'h3C0 && csr_addr <= 12'h3EF) rdata = '0;  // pmpaddr16-63
                else if (csr_addr >= 12'h3A4 && csr_addr <= 12'h3AE && !csr_addr[0]) rdata = '0; // pmpcfg4-14
                else exists = 1'b0;
            end
        endcase
    end

    always_comb begin
        csr_illegal = !exists || !priv_ok ||
                      (is_write && csr_addr[11:10] == 2'b11);
        // FP CSRs exigem FS != Off
        if (csr_addr <= 12'h003 && st_fs == 2'b00) csr_illegal = 1'b1;
        // contadores de usuário
        if (csr_addr >= 12'hC00 && csr_addr <= 12'hC1F && !counter_ok) csr_illegal = 1'b1;
        // satp com TVM em modo S
        if (csr_addr == 12'h180 && priv_q == 2'b01 && st_tvm) csr_illegal = 1'b1;
    end

    assign csr_rdata = rdata;

    // valor a escrever
    logic [63:0] wval;
    // Em read-modify-write de mip/sip, o bit SEIP lido inclui o pino externo;
    // a escrita deve usar apenas o bit de software (Priv. Spec 3.1.9).
    logic [63:0] rmw_src;
    always_comb begin
        rmw_src = rdata;
        if (csr_addr == 12'h344) rmw_src[9] = mip_seip;
        case (csr_op[1:0])
            2'b01:   wval = csr_wsrc;
            2'b10:   wval = rmw_src | csr_wsrc;
            default: wval = rmw_src & ~csr_wsrc;
        endcase
    end

    logic do_write;
    assign pmp_dirty = do_write_q;
    logic do_write_q;
    always_ff @(posedge clk or negedge rst_n)
        if (!rst_n) do_write_q <= 1'b0;
        else        do_write_q <= do_write && (csr_addr[11:4] == 8'h3A || csr_addr[11:4] == 8'h3B);
    assign do_write = csr_commit && !csr_illegal && is_write;
    assign tlb_flush = do_write && (csr_addr == 12'h180);

    // ---------------------------------------------------------------------
    // Interrupções
    // ---------------------------------------------------------------------
    logic [63:0] pend, m_pend, s_pend;
    logic        m_en, s_en;
    always_comb begin
        pend   = mip_val & mie_q;
        m_en   = (priv_q != 2'b11) || st_mie;
        s_en   = (priv_q == 2'b00) || (priv_q == 2'b01 && st_sie);
        m_pend = m_en ? (pend & ~mideleg_q) : '0;
        s_pend = s_en ? (pend & mideleg_q)  : '0;
        irq_pending = 1'b1;
        irq_cause   = 5'd0;
        if      (m_pend[11]) irq_cause = 5'd11;
        else if (m_pend[3])  irq_cause = 5'd3;
        else if (m_pend[7])  irq_cause = 5'd7;
        else if (m_pend[9])  irq_cause = 5'd9;
        else if (m_pend[1])  irq_cause = 5'd1;
        else if (m_pend[5])  irq_cause = 5'd5;
        else if (s_pend[9])  irq_cause = 5'd9;
        else if (s_pend[1])  irq_cause = 5'd1;
        else if (s_pend[5])  irq_cause = 5'd5;
        else irq_pending = 1'b0;
        wfi_wake = (pend != '0);
    end

    // ---------------------------------------------------------------------
    // Trap / xRET
    // ---------------------------------------------------------------------
    logic trap_to_s;
    always_comb begin
        trap_to_s = (priv_q != 2'b11) &&
                    (trap_intr ? mideleg_q[trap_cause] : medeleg_q[trap_cause]);
        if (trap_to_s)
            trap_target = (stvec_q[0] && trap_intr) ? ({stvec_q[63:2], 2'b00} + {57'b0, trap_cause, 2'b00})
                                                    : {stvec_q[63:2], 2'b00};
        else
            trap_target = (mtvec_q[0] && trap_intr) ? ({mtvec_q[63:2], 2'b00} + {57'b0, trap_cause, 2'b00})
                                                    : {mtvec_q[63:2], 2'b00};
        xret_target = mret ? mepc_q : sepc_q;
    end

    // ---------------------------------------------------------------------
    // Sequencial
    // ---------------------------------------------------------------------
    logic wr_mcycle, wr_minstret;
    assign wr_mcycle   = do_write && (csr_addr == 12'hB00);
    assign wr_minstret = do_write && (csr_addr == 12'hB02);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            priv_q   <= 2'b11;
            st_sie <= 0; st_mie <= 0; st_spie <= 0; st_mpie <= 0; st_spp <= 0;
            st_mpp <= 2'b00; st_fs <= 2'b00;
            st_mprv <= 0; st_sum <= 0; st_mxr <= 0; st_tvm <= 0; st_tw <= 0; st_tsr <= 0;
            medeleg_q <= '0; mideleg_q <= '0; mie_q <= '0;
            mip_ssip <= 0; mip_stip <= 0; mip_seip <= 0;
            mtvec_q <= '0; stvec_q <= '0;
            mscratch_q <= '0; sscratch_q <= '0;
            mepc_q <= '0; sepc_q <= '0;
            mcause_q <= '0; scause_q <= '0;
            mtval_q <= '0; stval_q <= '0;
            mcounteren_q <= '0; scounteren_q <= '0;
            satp_q <= '0;
            menvcfg_fiom <= 0; senvcfg_fiom <= 0;
            mcountinhibit_q <= '0;
            mcycle_q <= '0; minstret_q <= '0;
            fflags_q <= '0; frm_q <= '0;
            pmpcfg_q <= '0; pmpaddr_q <= '0;
            tsel_q <= '0; t_type <= '0; t_hit <= '0; t_cfg <= '0;
        end else begin
            // contadores
            if (!wr_mcycle && !mcountinhibit_q[0])   mcycle_q   <= mcycle_q + 64'd1;
            if (!wr_minstret && !mcountinhibit_q[2]) minstret_q <= minstret_q + 64'(instret_inc);

            // hits de triggers (efetivados no WB)
            if (trig_hit_we)
                for (int i = 0; i < NTRIG; i++)
                    if (trig_hit_val[i] != 2'd0) t_hit[i] <= (t_type[i] == 2'd1) ? 2'd1 : trig_hit_val[i];

            // FP
            if (|fflags_set) fflags_q <= fflags_q | fflags_set;
            if (fp_dirty || (|fflags_set)) st_fs <= 2'b11;

            if (trap_valid) begin
                if (trap_to_s) begin
                    scause_q <= {trap_intr, 58'b0, trap_cause};
                    sepc_q   <= {trap_epc[63:1], 1'b0};
                    stval_q  <= trap_tval;
                    st_spie  <= st_sie;
                    st_sie   <= 1'b0;
                    st_spp   <= priv_q[0];
                    priv_q   <= 2'b01;
                end else begin
                    mcause_q <= {trap_intr, 58'b0, trap_cause};
                    mepc_q   <= {trap_epc[63:1], 1'b0};
                    mtval_q  <= trap_tval;
                    st_mpie  <= st_mie;
                    st_mie   <= 1'b0;
                    st_mpp   <= priv_q;
                    priv_q   <= 2'b11;
                end
            end else if (mret) begin
                priv_q  <= st_mpp;
                st_mie  <= st_mpie;
                st_mpie <= 1'b1;
                st_mpp  <= 2'b00;
                if (st_mpp != 2'b11) st_mprv <= 1'b0;
            end else if (sret) begin
                priv_q  <= {1'b0, st_spp};
                st_sie  <= st_spie;
                st_spie <= 1'b1;
                st_spp  <= 1'b0;
                st_mprv <= 1'b0;
            end else if (do_write) begin
                case (csr_addr)
                    12'h001: begin fflags_q <= wval[4:0]; st_fs <= 2'b11; end
                    12'h002: begin frm_q <= wval[2:0];    st_fs <= 2'b11; end
                    12'h003: begin fflags_q <= wval[4:0]; frm_q <= wval[7:5]; st_fs <= 2'b11; end
                    12'h100: begin
                        st_sie  <= wval[1];
                        st_spie <= wval[5];
                        st_spp  <= wval[8];
                        st_fs   <= wval[14:13];
                        st_sum  <= wval[18];
                        st_mxr  <= wval[19];
                    end
                    12'h104: mie_q <= (mie_q & ~mideleg_q) | (wval & mideleg_q & MIE_MASK);
                    12'h105: stvec_q <= wval & ~64'h2;
                    12'h106: scounteren_q <= wval[31:0];
                    12'h10A: senvcfg_fiom <= wval[0];
                    12'h140: sscratch_q <= wval;
                    12'h141: sepc_q <= {wval[63:1], 1'b0};
                    12'h142: scause_q <= wval;
                    12'h143: stval_q <= wval;
                    12'h144: if (mideleg_q[1]) mip_ssip <= wval[1];
                    12'h180: begin
                        if (wval[63:60] == 4'd0 || wval[63:60] == 4'd8)
                            satp_q <= wval;
                    end
                    12'h300: begin
                        st_sie  <= wval[1];
                        st_mie  <= wval[3];
                        st_spie <= wval[5];
                        st_mpie <= wval[7];
                        st_spp  <= wval[8];
                        st_mpp  <= (wval[12:11] == 2'b10) ? 2'b00 : wval[12:11];
                        st_fs   <= wval[14:13];
                        st_mprv <= wval[17];
                        st_sum  <= wval[18];
                        st_mxr  <= wval[19];
                        st_tvm  <= wval[20];
                        st_tw   <= wval[21];
                        st_tsr  <= wval[22];
                    end
                    12'h302: medeleg_q <= wval & MEDELEG_MASK;
                    12'h303: mideleg_q <= wval & MIDELEG_MASK;
                    12'h304: mie_q <= wval & MIE_MASK;
                    12'h305: mtvec_q <= wval & ~64'h2;
                    12'h306: mcounteren_q <= wval[31:0];
                    12'h30A: menvcfg_fiom <= wval[0];
                    12'h320: mcountinhibit_q <= {wval[2], 1'b0, wval[0]};
                    12'h340: mscratch_q <= wval;
                    12'h341: mepc_q <= {wval[63:1], 1'b0};
                    12'h342: mcause_q <= wval;
                    12'h343: mtval_q <= wval;
                    12'h344: begin
                        mip_ssip <= wval[1];
                        mip_stip <= wval[5];
                        mip_seip <= wval[9];
                    end
                    12'h3A0, 12'h3A2: begin
                        for (int k = 0; k < 8; k++) begin
                            int e;
                            logic [7:0] nc;
                            e  = (csr_addr == 12'h3A2) ? 8 + k : k;
                            nc = wval[k*8 +: 8] & 8'h9F;
                            if (!nc[0]) nc[1] = 1'b0;          // R=0,W=1 reservado
                            if (!pmpcfg_q[e][7]) pmpcfg_q[e] <= nc;
                        end
                    end
                    12'h7A0: if (wval < 64'(NTRIG)) tsel_q <= wval[1:0];
                    12'h7A1: begin
                        logic [3:0] act, mt;
                        logic       sel, ld, ex;
                        act = (wval[15:12] > 4'd4 || wval[15:12] == 4'd1) ? 4'd0 : wval[15:12];
                        mt  = wval[10:7];
                        ex  = wval[2];
                        ld  = wval[0];
                        t_cfg[tsel_q].action <= act;
                        t_cfg[tsel_q].chain  <= wval[11];
                        t_cfg[tsel_q].m  <= wval[6];
                        t_cfg[tsel_q].s  <= wval[4];
                        t_cfg[tsel_q].u  <= wval[3];
                        t_cfg[tsel_q].ex <= ex;
                        t_cfg[tsel_q].st <= wval[1];
                        t_cfg[tsel_q].ld <= ld;
                        if (wval[63:60] == 4'd2) begin
                            sel = wval[19];
                            t_type[tsel_q] <= 2'd1;
                            t_hit[tsel_q]  <= {1'b0, wval[20]};
                            t_cfg[tsel_q].select <= sel;
                            t_cfg[tsel_q].timing <= (sel && ld) ? 1'b1 : ex ? 1'b0 : wval[18];
                            // mcontrol: maskmax = 0 -> NAPOT legalizado para EQUAL
                            t_cfg[tsel_q].match <= (mt == 4'd0 || mt == 4'd2 || mt == 4'd3 || mt == 4'd4 || mt == 4'd5) ? mt : 4'd0;
                        end else if (wval[63:60] == 4'd6) begin
                            sel = wval[21];
                            t_type[tsel_q] <= 2'd2;
                            t_hit[tsel_q]  <= {wval[25], wval[22]};
                            t_cfg[tsel_q].select <= sel;
                            t_cfg[tsel_q].timing <= sel && ld;
                            t_cfg[tsel_q].match <= (mt <= 4'd5) ? mt : 4'd0;
                        end else begin
                            t_type[tsel_q] <= 2'd0;
                            t_hit[tsel_q]  <= 2'd0;
                        end
                    end
                    12'h7A2: t_cfg[tsel_q].tdata2 <= wval;
                    12'hB00: mcycle_q <= wval;
                    12'hB02: minstret_q <= wval;
                    default: begin
                        if (csr_addr >= 12'h3B0 && csr_addr <= 12'h3BF) begin
                            logic [3:0] e;
                            logic locked_next_tor;
                            e = csr_addr[3:0];
                            locked_next_tor = (e != 4'd15) && pmpcfg_q[e+1][7] && (pmpcfg_q[e+1][4:3] == 2'b01);
                            if (!pmpcfg_q[e][7] && !locked_next_tor) pmpaddr_q[e] <= wval[53:0];
                        end
                    end
                endcase
            end
        end
    end

endmodule
