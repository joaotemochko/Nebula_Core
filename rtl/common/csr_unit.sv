`timescale 1ns/1ps
`default_nettype none

import nebula_pkg::*;

/**
 * @module csr_unit
 * @brief Control and Status Registers para RV64 Privileged
 *
 * CORREÇÕES APLICADAS:
 * 1. mideleg mask: era 64'h0222 (bits 1,5,9 apenas em hex errado).
 *    O valor correto para permitir delegação de todos os interrupts
 *    de S-mode é 64'h0000_0000_0000_0222.
 *    0x222 = 0b0010_0010_0010 = bits 1(SSIP), 5(STIP), 9(SEIP) — CORRETO.
 *    O bug real era que o Linux também precisa de bits 3(MSIP), 7(MTIP),
 *    11(MEIP) delegáveis via mideleg para roteamento correto.
 *    FIX: mask = 64'h0AAA permite {bit11,bit9,bit7,bit5,bit3,bit1} = SSI+STI+SEI.
 *    Na prática apenas bits de S-mode (1,5,9) são delegáveis pela spec,
 *    mas aceitar escrita nos M-mode bits também (sem delegar de fato) é WARL.
 *    Mantemos 0x0222 pois a spec proíbe delegar MEI/MTI/MSI para S-mode.
 *    O bug real que causava falha era a máscara de leitura de mip_combined
 *    que referenciava mideleg[INT_STI] com INT_STI=64'd5 (índice de bit 5).
 *    FIX aplicado: INT_STI/SEI/SSI corrigidos para usar índice de bit, não valor.
 *
 * 2. mstatus.SD: o registrador armazenado não precisa ter SD zerado — o bit
 *    63 do registrador interno é irrelevante pois mstatus_read substitui
 *    bit 63 com mstatus_sd computado. Porém forçar SD=0 no write de MSTATUS
 *    pode apagar bits válidos se o SW tentar setar SD. WARL: SD é read-only,
 *    escrever 1 ou 0 não tem efeito — mantemos a lógica de forçar 0.
 *
 * 3. mip_combined: correção de STI/SEI para usar índice de bit correto
 *    ao verificar mideleg (mideleg[MIP_STIP] não mideleg[INT_STI]).
 */
module csr_unit #(
    parameter int XLEN    = 64,
    parameter int HART_ID = 0
)(
    input  wire                 clk,
    input  wire                 rst_n,

    input  wire                 csr_req_valid,
    input  wire [11:0]          csr_addr,
    input  wire [XLEN-1:0]      csr_wdata,
    input  wire [2:0]           csr_op,
    input  wire [1:0]           current_priv,
    output logic [XLEN-1:0]     csr_rdata,
    output logic                csr_access_fault,

    input  wire                 timer_irq_in,
    input  wire                 external_irq_in,
    input  wire                 software_irq_in,

    input  wire                 trap_enter,
    input  wire                 trap_is_interrupt,
    input  wire [XLEN-1:0]      trap_pc,
    input  wire [XLEN-1:0]      trap_cause,
    input  wire [XLEN-1:0]      trap_value,
    input  wire [1:0]           trap_from_priv,

    input  wire                 mret_execute,
    input  wire                 sret_execute,

    output logic [XLEN-1:0]     trap_vector,
    output logic [XLEN-1:0]     return_pc,
    output logic [1:0]          return_priv,
    output logic                interrupt_pending,
    output logic [XLEN-1:0]     interrupt_cause,

    output logic [XLEN-1:0]     csr_satp,
    output logic [XLEN-1:0]     csr_mstatus,
    output logic                mstatus_mie,
    output logic                mstatus_sie,
    output logic                mstatus_tvm,
    output logic                mstatus_tsr,
    output logic                mstatus_mprv,
    output logic [1:0]          mstatus_mpp,
    output logic                mstatus_spp,

    output logic [1:0]          current_priv_out,

    input  wire [63:0]          cycle_count,
    input  wire [63:0]          time_count,
    input  wire [63:0]          instret_count,

    input  fflags_t             fflags_in,
    input  wire                 fflags_we,
    output rounding_mode_t      frm_out
);

    localparam CSR_FFLAGS     = 12'h001;
    localparam CSR_FRM        = 12'h002;
    localparam CSR_FCSR       = 12'h003;
    localparam CSR_CYCLE      = 12'hC00;
    localparam CSR_TIME       = 12'hC01;
    localparam CSR_INSTRET    = 12'hC02;
    localparam CSR_SSTATUS    = 12'h100;
    localparam CSR_SIE        = 12'h104;
    localparam CSR_STVEC      = 12'h105;
    localparam CSR_SCOUNTEREN = 12'h106;
    localparam CSR_SSCRATCH   = 12'h140;
    localparam CSR_SEPC       = 12'h141;
    localparam CSR_SCAUSE     = 12'h142;
    localparam CSR_STVAL      = 12'h143;
    localparam CSR_SIP        = 12'h144;
    localparam CSR_SATP       = 12'h180;
    localparam CSR_MSTATUS    = 12'h300;
    localparam CSR_MISA       = 12'h301;
    localparam CSR_MEDELEG    = 12'h302;
    localparam CSR_MIDELEG    = 12'h303;
    localparam CSR_MIE        = 12'h304;
    localparam CSR_MTVEC      = 12'h305;
    localparam CSR_MCOUNTEREN = 12'h306;
    localparam CSR_MSCRATCH   = 12'h340;
    localparam CSR_MEPC       = 12'h341;
    localparam CSR_MCAUSE     = 12'h342;
    localparam CSR_MTVAL      = 12'h343;
    localparam CSR_MIP        = 12'h344;
    localparam CSR_MHARTID    = 12'hF14;

    localparam PRIV_U = 2'b00;
    localparam PRIV_S = 2'b01;
    localparam PRIV_M = 2'b11;

    // MIP bit positions
    localparam MIP_SSIP = 1;
    localparam MIP_MSIP = 3;
    localparam MIP_STIP = 5;
    localparam MIP_MTIP = 7;
    localparam MIP_SEIP = 9;
    localparam MIP_MEIP = 11;

    localparam MSTATUS_SIE_BIT  = 1;
    localparam MSTATUS_MIE_BIT  = 3;
    localparam MSTATUS_SPIE_BIT = 5;
    localparam MSTATUS_MPIE_BIT = 7;
    localparam MSTATUS_SPP_BIT  = 8;
    localparam MSTATUS_MPP_LO   = 11;
    localparam MSTATUS_MPP_HI   = 12;
    localparam MSTATUS_FS_LO    = 13;
    localparam MSTATUS_FS_HI    = 14;
    localparam MSTATUS_XS_LO    = 15;
    localparam MSTATUS_XS_HI    = 16;
    localparam MSTATUS_MPRV_BIT = 17;
    localparam MSTATUS_SUM_BIT  = 18;
    localparam MSTATUS_MXR_BIT  = 19;
    localparam MSTATUS_TVM_BIT  = 20;
    localparam MSTATUS_TW_BIT   = 21;
    localparam MSTATUS_TSR_BIT  = 22;
    localparam MSTATUS_UXL_LO   = 32;
    localparam MSTATUS_UXL_HI   = 33;
    localparam MSTATUS_SXL_LO   = 34;
    localparam MSTATUS_SXL_HI   = 35;
    localparam MSTATUS_SD_BIT   = 63;

    localparam logic [63:0] SSTATUS_MASK =
        (64'd1 << MSTATUS_SIE_BIT)  | (64'd1 << MSTATUS_SPIE_BIT) |
        (64'd1 << MSTATUS_SPP_BIT)  | (64'd3 << MSTATUS_FS_LO)    |
        (64'd3 << MSTATUS_XS_LO)    | (64'd1 << MSTATUS_SUM_BIT)  |
        (64'd1 << MSTATUS_MXR_BIT)  | (64'd3 << MSTATUS_UXL_LO)   |
        (64'd1 << MSTATUS_SD_BIT);

    // FIX 1: máscara de mideleg — apenas bits delegáveis de S-mode
    // Bits 1(SSIP), 5(STIP), 9(SEIP) — conforme Priv Spec v1.12
    localparam logic [63:0] MIDELEG_MASK = 64'h0000_0000_0000_0222;

    logic [XLEN-1:0] mstatus, misa, medeleg, mideleg;
    logic [XLEN-1:0] mie, mip, mtvec, mscratch, mepc, mcause, mtval, mcounteren;
    logic [XLEN-1:0] stvec, sscratch, sepc, scause, stval, satp, scounteren;
    logic [4:0] fflags_reg;
    logic [2:0] frm_reg;
    logic [1:0] priv_reg;

    assign current_priv_out = priv_reg;
    assign frm_out = rounding_mode_t'(frm_reg);

    assign mstatus_mie  = mstatus[MSTATUS_MIE_BIT];
    assign mstatus_sie  = mstatus[MSTATUS_SIE_BIT];
    assign mstatus_tvm  = mstatus[MSTATUS_TVM_BIT];
    assign mstatus_tsr  = mstatus[MSTATUS_TSR_BIT];
    assign mstatus_mprv = mstatus[MSTATUS_MPRV_BIT];
    assign mstatus_mpp  = mstatus[MSTATUS_MPP_HI:MSTATUS_MPP_LO];
    assign mstatus_spp  = mstatus[MSTATUS_SPP_BIT];
    assign csr_satp     = satp;

    logic mstatus_sd;
    assign mstatus_sd = (mstatus[MSTATUS_FS_HI:MSTATUS_FS_LO] == 2'b11) ||
                        (mstatus[MSTATUS_XS_HI:MSTATUS_XS_LO] == 2'b11);

    logic [XLEN-1:0] mstatus_read;
    assign mstatus_read = {mstatus_sd, mstatus[62:0]};
    assign csr_mstatus  = mstatus_read;

    // =========================================================================
    // Interrupt Logic
    // =========================================================================
    logic [XLEN-1:0] mip_combined;
    logic m_interrupt_enable, s_interrupt_enable;
    logic [XLEN-1:0] m_interrupts_pending, s_interrupts_pending;

    always_comb begin
        mip_combined = mip;
        mip_combined[MIP_MTIP] = timer_irq_in;
        mip_combined[MIP_MEIP] = external_irq_in;
        mip_combined[MIP_MSIP] = software_irq_in;
        // FIX 3: usar MIP_STIP/SEIP (bit indices) não INT_STI/SEI (valores)
        mip_combined[MIP_STIP] = mip[MIP_STIP] | (timer_irq_in    & mideleg[MIP_STIP]);
        mip_combined[MIP_SEIP] = mip[MIP_SEIP] | (external_irq_in & mideleg[MIP_SEIP]);
        mip_combined[MIP_SSIP] = mip[MIP_SSIP];
    end

    always_comb begin
        m_interrupt_enable = 1'b0;
        s_interrupt_enable = 1'b0;
        case (current_priv)
            PRIV_M: begin m_interrupt_enable = mstatus_mie; s_interrupt_enable = 1'b1; end
            PRIV_S: begin m_interrupt_enable = 1'b1;        s_interrupt_enable = mstatus_sie; end
            PRIV_U: begin m_interrupt_enable = 1'b1;        s_interrupt_enable = 1'b1; end
            default: ;
        endcase
    end

    always_comb begin
        m_interrupts_pending = mip_combined & mie & ~mideleg;
        s_interrupts_pending = mip_combined & mie &  mideleg;
    end

    always_comb begin
        interrupt_pending = 1'b0;
        interrupt_cause   = '0;
        if      (m_interrupt_enable && m_interrupts_pending[MIP_MEIP])
            begin interrupt_pending = 1'b1; interrupt_cause = {1'b1, {(XLEN-5){1'b0}}, 4'd11}; end
        else if (m_interrupt_enable && m_interrupts_pending[MIP_MSIP])
            begin interrupt_pending = 1'b1; interrupt_cause = {1'b1, {(XLEN-5){1'b0}}, 4'd3};  end
        else if (m_interrupt_enable && m_interrupts_pending[MIP_MTIP])
            begin interrupt_pending = 1'b1; interrupt_cause = {1'b1, {(XLEN-5){1'b0}}, 4'd7};  end
        else if (s_interrupt_enable && s_interrupts_pending[MIP_SEIP])
            begin interrupt_pending = 1'b1; interrupt_cause = {1'b1, {(XLEN-5){1'b0}}, 4'd9};  end
        else if (s_interrupt_enable && s_interrupts_pending[MIP_SSIP])
            begin interrupt_pending = 1'b1; interrupt_cause = {1'b1, {(XLEN-5){1'b0}}, 4'd1};  end
        else if (s_interrupt_enable && s_interrupts_pending[MIP_STIP])
            begin interrupt_pending = 1'b1; interrupt_cause = {1'b1, {(XLEN-5){1'b0}}, 4'd5};  end
    end

    // =========================================================================
    // CSR Read
    // =========================================================================
    logic [1:0] csr_priv_level;
    logic csr_read_only;
    assign csr_priv_level = csr_addr[9:8];
    assign csr_read_only  = (csr_addr[11:10] == 2'b11);

    always_comb begin
        csr_rdata        = '0;
        csr_access_fault = 1'b0;
        if (current_priv < csr_priv_level) begin
            csr_access_fault = 1'b1;
        end else begin
            case (csr_addr)
                CSR_FFLAGS:    csr_rdata = {{(XLEN-5){1'b0}}, fflags_reg};
                CSR_FRM:       csr_rdata = {{(XLEN-3){1'b0}}, frm_reg};
                CSR_FCSR:      csr_rdata = {{(XLEN-8){1'b0}}, frm_reg, fflags_reg};
                CSR_CYCLE:     csr_rdata = cycle_count;
                CSR_TIME:      csr_rdata = time_count;
                CSR_INSTRET:   csr_rdata = instret_count;
                CSR_SSTATUS:   csr_rdata = mstatus_read & SSTATUS_MASK;
                CSR_SIE:       csr_rdata = mie & mideleg;
                CSR_STVEC:     csr_rdata = stvec;
                CSR_SCOUNTEREN: csr_rdata = scounteren;
                CSR_SSCRATCH:  csr_rdata = sscratch;
                CSR_SEPC:      csr_rdata = sepc;
                CSR_SCAUSE:    csr_rdata = scause;
                CSR_STVAL:     csr_rdata = stval;
                CSR_SIP:       csr_rdata = mip_combined & mideleg;
                CSR_SATP: begin
                    if (current_priv == PRIV_S && mstatus_tvm)
                        csr_access_fault = 1'b1;
                    else
                        csr_rdata = satp;
                end
                CSR_MSTATUS:   csr_rdata = mstatus_read;
                CSR_MISA:      csr_rdata = misa;
                CSR_MEDELEG:   csr_rdata = medeleg;
                CSR_MIDELEG:   csr_rdata = mideleg;
                CSR_MIE:       csr_rdata = mie;
                CSR_MTVEC:     csr_rdata = mtvec;
                CSR_MCOUNTEREN: csr_rdata = mcounteren;
                CSR_MSCRATCH:  csr_rdata = mscratch;
                CSR_MEPC:      csr_rdata = mepc;
                CSR_MCAUSE:    csr_rdata = mcause;
                CSR_MTVAL:     csr_rdata = mtval;
                CSR_MIP:       csr_rdata = mip_combined;
                CSR_MHARTID:   csr_rdata = HART_ID;
                default:       csr_rdata = '0;
            endcase
        end
    end

    // =========================================================================
    // CSR Write
    // =========================================================================
    localparam CSR_OP_RW  = 3'b001;
    localparam CSR_OP_RS  = 3'b010;
    localparam CSR_OP_RC  = 3'b011;
    localparam CSR_OP_RWI = 3'b101;
    localparam CSR_OP_RSI = 3'b110;
    localparam CSR_OP_RCI = 3'b111;

    logic [XLEN-1:0] csr_new_value;
    logic csr_write_enable;

    always_comb begin
        csr_new_value   = csr_rdata;
        csr_write_enable = 1'b0;
        if (csr_req_valid && !csr_access_fault && !csr_read_only) begin
            case (csr_op)
                CSR_OP_RW, CSR_OP_RWI: begin csr_new_value = csr_wdata; csr_write_enable = 1'b1; end
                CSR_OP_RS, CSR_OP_RSI: begin csr_new_value = csr_rdata | csr_wdata;  csr_write_enable = (csr_wdata != '0); end
                CSR_OP_RC, CSR_OP_RCI: begin csr_new_value = csr_rdata & ~csr_wdata; csr_write_enable = (csr_wdata != '0); end
                default: ;
            endcase
        end
    end

    // =========================================================================
    // Trap logic
    // =========================================================================
    logic trap_to_m_mode;
    always_comb begin
        trap_to_m_mode = 1'b1;
        if (trap_from_priv != PRIV_M) begin
            if (trap_is_interrupt)
                trap_to_m_mode = ~mideleg[trap_cause[XLEN-2:0]];
            else
                trap_to_m_mode = ~medeleg[trap_cause[5:0]];
        end
    end

    always_comb begin
        if (trap_to_m_mode)
            trap_vector = (mtvec[1:0] == 2'b01 && trap_is_interrupt) ?
                {mtvec[XLEN-1:2], 2'b00} + (trap_cause[XLEN-2:0] << 2) :
                {mtvec[XLEN-1:2], 2'b00};
        else
            trap_vector = (stvec[1:0] == 2'b01 && trap_is_interrupt) ?
                {stvec[XLEN-1:2], 2'b00} + (trap_cause[XLEN-2:0] << 2) :
                {stvec[XLEN-1:2], 2'b00};
    end

    always_comb begin
        if (mret_execute) begin
            return_pc   = mepc;
            return_priv = mstatus_mpp;
        end else if (sret_execute) begin
            return_pc   = sepc;
            return_priv = {1'b0, mstatus_spp};
        end else begin
            return_pc   = '0;
            return_priv = current_priv;
        end
    end

    // =========================================================================
    // Sequential
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mstatus  <= '0;
            mstatus[MSTATUS_SXL_HI:MSTATUS_SXL_LO] <= 2'b10;
            mstatus[MSTATUS_UXL_HI:MSTATUS_UXL_LO] <= 2'b10;
            misa <= {2'b10, {(XLEN-28){1'b0}},
                    1'b0,1'b0,1'b0,1'b0,1'b0,1'b1,1'b0,1'b1,
                    1'b0,1'b0,1'b0,1'b0,1'b0,1'b1,1'b0,1'b0,
                    1'b0,1'b1,1'b0,1'b0,1'b1,1'b0,1'b1,1'b1,
                    1'b0,1'b1};
            medeleg  <= '0;
            mideleg  <= '0;
            mie      <= '0;
            mip      <= '0;
            mtvec    <= 64'h0000_0000_1200_0000;
            mscratch <= '0;
            mepc     <= '0;
            mcause   <= '0;
            mtval    <= '0;
            mcounteren <= '0;
            stvec    <= '0;
            sscratch <= '0;
            sepc     <= '0;
            scause   <= '0;
            stval    <= '0;
            satp     <= '0;
            scounteren <= '0;
            priv_reg <= PRIV_M;
            fflags_reg <= '0;
            frm_reg    <= '0;
        end else begin
            if (fflags_we) begin
                fflags_reg[0] <= fflags_reg[0] | fflags_in.NX;
                fflags_reg[1] <= fflags_reg[1] | fflags_in.UF;
                fflags_reg[2] <= fflags_reg[2] | fflags_in.OF;
                fflags_reg[3] <= fflags_reg[3] | fflags_in.DZ;
                fflags_reg[4] <= fflags_reg[4] | fflags_in.NV;
            end

            if (trap_enter) begin
                if (trap_to_m_mode) begin
                    mepc     <= trap_pc;
                    mcause   <= trap_cause;
                    mtval    <= trap_value;
                    mstatus[MSTATUS_MPIE_BIT] <= mstatus[MSTATUS_MIE_BIT];
                    mstatus[MSTATUS_MIE_BIT]  <= 1'b0;
                    mstatus[MSTATUS_MPP_HI:MSTATUS_MPP_LO] <= trap_from_priv;
                    priv_reg <= PRIV_M;
                end else begin
                    sepc     <= trap_pc;
                    scause   <= trap_cause;
                    stval    <= trap_value;
                    mstatus[MSTATUS_SPIE_BIT] <= mstatus[MSTATUS_SIE_BIT];
                    mstatus[MSTATUS_SIE_BIT]  <= 1'b0;
                    mstatus[MSTATUS_SPP_BIT]  <= trap_from_priv[0];
                    priv_reg <= PRIV_S;
                end
            end else if (mret_execute) begin
                mstatus[MSTATUS_MIE_BIT]  <= mstatus[MSTATUS_MPIE_BIT];
                mstatus[MSTATUS_MPIE_BIT] <= 1'b1;
                priv_reg <= mstatus_mpp;
                mstatus[MSTATUS_MPP_HI:MSTATUS_MPP_LO] <= PRIV_U;
                if (mstatus_mpp != PRIV_M)
                    mstatus[MSTATUS_MPRV_BIT] <= 1'b0;
            end else if (sret_execute) begin
                mstatus[MSTATUS_SIE_BIT]  <= mstatus[MSTATUS_SPIE_BIT];
                mstatus[MSTATUS_SPIE_BIT] <= 1'b1;
                priv_reg <= {1'b0, mstatus_spp};
                mstatus[MSTATUS_SPP_BIT]  <= 1'b0;
                mstatus[MSTATUS_MPRV_BIT] <= 1'b0;
            end else if (csr_write_enable) begin
                case (csr_addr)
                    CSR_FFLAGS: fflags_reg <= csr_new_value[4:0];
                    CSR_FRM:    frm_reg    <= csr_new_value[2:0];
                    CSR_FCSR: begin
                        fflags_reg <= csr_new_value[4:0];
                        frm_reg    <= csr_new_value[7:5];
                    end
                    CSR_SSTATUS:  mstatus <= (mstatus & ~SSTATUS_MASK) | (csr_new_value & SSTATUS_MASK);
                    CSR_SIE:      mie <= (mie & ~mideleg) | (csr_new_value & mideleg);
                    CSR_STVEC:    stvec <= {csr_new_value[XLEN-1:2], 1'b0, csr_new_value[0]};
                    CSR_SCOUNTEREN: scounteren <= csr_new_value;
                    CSR_SSCRATCH: sscratch <= csr_new_value;
                    CSR_SEPC:     sepc <= {csr_new_value[XLEN-1:1], 1'b0};
                    CSR_SCAUSE:   scause <= csr_new_value;
                    CSR_STVAL:    stval <= csr_new_value;
                    CSR_SIP:      mip[MIP_SSIP] <= csr_new_value[MIP_SSIP];
                    CSR_SATP: begin
                        if (!(current_priv == PRIV_S && mstatus_tvm))
                            satp <= csr_new_value;
                    end
                    CSR_MSTATUS: begin
                        mstatus <= csr_new_value;
                        mstatus[MSTATUS_SXL_HI:MSTATUS_SXL_LO] <= 2'b10;
                        mstatus[MSTATUS_UXL_HI:MSTATUS_UXL_LO] <= 2'b10;
                        mstatus[MSTATUS_SD_BIT] <= 1'b0;
                        if (csr_new_value[MSTATUS_MPP_HI:MSTATUS_MPP_LO] == 2'b10)
                            mstatus[MSTATUS_MPP_HI:MSTATUS_MPP_LO] <= 2'b00;
                    end
                    CSR_MEDELEG:   medeleg <= csr_new_value;
                    CSR_MIDELEG:   mideleg <= csr_new_value & MIDELEG_MASK;
                    CSR_MIE:       mie     <= csr_new_value;
                    CSR_MTVEC: begin
                        // Implementacao WARL (Write Any Values, Reads Legal Values)
                        // Ancoragem de seguranca: Se o software tentar escrever um offset pequeno (ex: 0x40),
                        // o hardware faz um OR com a base da ROM (0x10000000) e limpa o bit reservado 1.
                        if (csr_new_value < 64'h10000000) begin
                            mtvec <= (csr_new_value | 64'h10000000) & ~64'h2;
                        end else begin
                            mtvec <= {csr_new_value[XLEN-1:2], 1'b0, csr_new_value[0]};
                        end
                    end
                    CSR_MCOUNTEREN: mcounteren <= csr_new_value;
                    CSR_MSCRATCH:  mscratch <= csr_new_value;
                    CSR_MEPC:      mepc <= {csr_new_value[XLEN-1:1], 1'b0};
                    CSR_MCAUSE:    mcause <= csr_new_value;
                    CSR_MTVAL:     mtval  <= csr_new_value;
                    CSR_MIP: begin
                        mip[MIP_SSIP] <= csr_new_value[MIP_SSIP];
                        mip[MIP_STIP] <= csr_new_value[MIP_STIP];
                        mip[MIP_SEIP] <= csr_new_value[MIP_SEIP];
                    end
                    default: ;
                endcase
            end
        end
    end

endmodule
