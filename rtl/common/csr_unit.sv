`timescale 1ns/1ps
`default_nettype none

import nebula_pkg::*;

/**
 * @module csr_unit
 * @brief Unidade de Control and Status Registers para RV64 Privileged
 *
 * @details Implementa CSRs necessários para Linux:
 * - Machine Mode: mstatus, misa, mie, mip, mtvec, mscratch, mepc, mcause, mtval, medeleg, mideleg
 * - Supervisor Mode: sstatus, sie, sip, stvec, sscratch, sepc, scause, stval, satp
 * - Floating Point: fflags, frm, fcsr
 * - Counters: cycle, time, instret (read-only em S/U mode)
 *
 * Suporta todas as instruções CSR: CSRRW, CSRRS, CSRRC, CSRRWI, CSRRSI, CSRRCI
 */
module csr_unit #(
    parameter int XLEN = 64,
    parameter int HART_ID = 0
)(
    input  wire                 clk,
    input  wire                 rst_n,
    
    // Interface de Acesso CSR
    input  wire                 csr_req_valid,
    input  wire [11:0]          csr_addr,
    input  wire [XLEN-1:0]      csr_wdata,
    input  wire [2:0]           csr_op,      // funct3 da instrução
    input  wire [1:0]           current_priv, // Privilégio atual
    output logic [XLEN-1:0]     csr_rdata,
    output logic                csr_access_fault, // Violação de privilégio
    
    // Sinais de Interrupção de Entrada
    input  wire                 timer_irq_in,
    input  wire                 external_irq_in,
    input  wire                 software_irq_in,
    
    // Interface de Trap Entry (do pipeline)
    input  wire                 trap_enter,
    input  wire                 trap_is_interrupt,
    input  wire [XLEN-1:0]      trap_pc,
    input  wire [XLEN-1:0]      trap_cause,
    input  wire [XLEN-1:0]      trap_value,
    input  wire [1:0]           trap_from_priv,
    
    // Interface de Trap Return
    input  wire                 mret_execute,
    input  wire                 sret_execute,
    
    // Saídas para Pipeline
    output logic [XLEN-1:0]     trap_vector,     // mtvec ou stvec
    output logic [XLEN-1:0]     return_pc,       // mepc ou sepc
    output logic [1:0]          return_priv,     // Privilégio após xRET
    output logic                interrupt_pending, // Há interrupção pendente
    output logic [XLEN-1:0]     interrupt_cause,   // Causa da interrupção
    
    // Saídas de CSRs importantes
    output logic [XLEN-1:0]     csr_satp,
    output logic [XLEN-1:0]     csr_mstatus,
    output logic                mstatus_mie,
    output logic                mstatus_sie,
    output logic                mstatus_tvm,     // Trap Virtual Memory
    output logic                mstatus_tsr,     // Trap SRET
    output logic                mstatus_mprv,    // Modify PRiVilege
    output logic [1:0]          mstatus_mpp,     // Previous privilege (M-mode)
    output logic                mstatus_spp,     // Previous privilege (S-mode)
    
    // Counters input
    input  wire [63:0]          cycle_count,
    input  wire [63:0]          time_count,
    input  wire [63:0]          instret_count,

    // Interface FPU
    input  fflags_t             fflags_in,     // Flags vindas da FPU
    input  wire                 fflags_we,     // Write Enable da FPU
    output rounding_mode_t      frm_out        // Rounding mode para a FPU
);

    // =========================================================================
    // Endereços CSR
    // =========================================================================
    
    // User-level CSRs (Floating Point)
    localparam CSR_FFLAGS       = 12'h001;
    localparam CSR_FRM          = 12'h002;
    localparam CSR_FCSR         = 12'h003;

    // User-level CSRs (Counters)
    localparam CSR_CYCLE        = 12'hC00;
    localparam CSR_TIME         = 12'hC01;
    localparam CSR_INSTRET      = 12'hC02;
    localparam CSR_CYCLEH       = 12'hC80; // RV32 only
    localparam CSR_TIMEH        = 12'hC81;
    localparam CSR_INSTRETH     = 12'hC82;
    
    // Supervisor-level CSRs
    localparam CSR_SSTATUS      = 12'h100;
    localparam CSR_SIE          = 12'h104;
    localparam CSR_STVEC        = 12'h105;
    localparam CSR_SCOUNTEREN   = 12'h106;
    localparam CSR_SSCRATCH     = 12'h140;
    localparam CSR_SEPC         = 12'h141;
    localparam CSR_SCAUSE       = 12'h142;
    localparam CSR_STVAL        = 12'h143;
    localparam CSR_SIP          = 12'h144;
    localparam CSR_SATP         = 12'h180;

    // Machine-level CSRs
    localparam CSR_MSTATUS      = 12'h300;
    localparam CSR_MISA         = 12'h301;
    localparam CSR_MEDELEG      = 12'h302;
    localparam CSR_MIDELEG      = 12'h303;
    localparam CSR_MIE          = 12'h304;
    localparam CSR_MTVEC        = 12'h305;
    localparam CSR_MCOUNTEREN   = 12'h306;
    localparam CSR_MSCRATCH     = 12'h340;
    localparam CSR_MEPC         = 12'h341;
    localparam CSR_MCAUSE       = 12'h342;
    localparam CSR_MTVAL        = 12'h343;
    localparam CSR_MIP          = 12'h344;
    localparam CSR_MHARTID      = 12'hF14;
    
    // Privilege levels
    localparam PRIV_U = 2'b00;
    localparam PRIV_S = 2'b01;
    localparam PRIV_M = 2'b11;
    
    // Interrupt causes
    localparam INT_SSI = 64'd1; // Supervisor Software Interrupt
    localparam INT_MSI = 64'd3; // Machine Software Interrupt
    localparam INT_STI = 64'd5; // Supervisor Timer Interrupt
    localparam INT_MTI = 64'd7; // Machine Timer Interrupt
    localparam INT_SEI = 64'd9; // Supervisor External Interrupt
    localparam INT_MEI = 64'd11; // Machine External Interrupt

    // =========================================================================
    // CSR Registers
    // =========================================================================
    
    // Machine Mode
    logic [XLEN-1:0] mstatus;
    logic [XLEN-1:0] misa;
    logic [XLEN-1:0] medeleg;
    logic [XLEN-1:0] mideleg;
    logic [XLEN-1:0] mie;
    logic [XLEN-1:0] mip;
    logic [XLEN-1:0] mtvec;
    logic [XLEN-1:0] mscratch;
    logic [XLEN-1:0] mepc;
    logic [XLEN-1:0] mcause;
    logic [XLEN-1:0] mtval;
    logic [XLEN-1:0] mcounteren;

    // Supervisor Mode
    logic [XLEN-1:0] stvec;
    logic [XLEN-1:0] sscratch;
    logic [XLEN-1:0] sepc;
    logic [XLEN-1:0] scause;
    logic [XLEN-1:0] stval;
    logic [XLEN-1:0] satp;
    logic [XLEN-1:0] scounteren;

    // Floating Point Mode
    logic [4:0] fflags_reg;
    logic [2:0] frm_reg;
    
    assign frm_out = rounding_mode_t'(frm_reg);
    
    // =========================================================================
    // MSTATUS field masks and positions
    // =========================================================================
    
    // mstatus bit positions
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

    // sstatus mask (bits visíveis em S-mode)
    localparam SSTATUS_MASK = (1 << MSTATUS_SIE_BIT) |
                              (1 << MSTATUS_SPIE_BIT) |
                              (1 << MSTATUS_SPP_BIT) | (3 << MSTATUS_FS_LO) |
                              (3 << MSTATUS_XS_LO) | (1 << MSTATUS_SUM_BIT) |
                              (1 << MSTATUS_MXR_BIT) | (64'h3 << MSTATUS_UXL_LO) |
                              (1'b1 << MSTATUS_SD_BIT);

    // MIP/MIE bit positions
    localparam MIP_SSIP = 1;
    localparam MIP_MSIP = 3;
    localparam MIP_STIP = 5;
    localparam MIP_MTIP = 7;
    localparam MIP_SEIP = 9;
    localparam MIP_MEIP = 11;

    // =========================================================================
    // Field extraction
    // =========================================================================
    
    assign mstatus_mie  = mstatus[MSTATUS_MIE_BIT];
    assign mstatus_sie  = mstatus[MSTATUS_SIE_BIT];
    assign mstatus_tvm  = mstatus[MSTATUS_TVM_BIT];
    assign mstatus_tsr  = mstatus[MSTATUS_TSR_BIT];
    assign mstatus_mprv = mstatus[MSTATUS_MPRV_BIT];
    assign mstatus_mpp  = mstatus[MSTATUS_MPP_HI:MSTATUS_MPP_LO];
    assign mstatus_spp  = mstatus[MSTATUS_SPP_BIT];
    
    assign csr_satp = satp;
    assign csr_mstatus = mstatus;

    // =========================================================================
    // Interrupt Pending Logic
    // =========================================================================
    
    logic [XLEN-1:0] mip_combined;
    logic m_interrupt_enable, s_interrupt_enable;
    logic [XLEN-1:0] m_interrupts_pending, s_interrupts_pending;
    
    // Combine hardware IRQs with software-written bits
    always_comb begin
        mip_combined = mip;
        mip_combined[MIP_MTIP] = timer_irq_in;
        mip_combined[MIP_MEIP] = external_irq_in;
        mip_combined[MIP_MSIP] = software_irq_in;
        // S-mode interrupts can be delegated from M-mode or set by software
        mip_combined[MIP_STIP] = mip[MIP_STIP] | (timer_irq_in & mideleg[INT_STI]);
        mip_combined[MIP_SEIP] = mip[MIP_SEIP] | (external_irq_in & mideleg[INT_SEI]);
        mip_combined[MIP_SSIP] = mip[MIP_SSIP];
    end
    
    // Determine if interrupts are enabled based on current privilege
    always_comb begin
        m_interrupt_enable = 1'b0;
        s_interrupt_enable = 1'b0;
        
        case (current_priv)
            PRIV_M: begin
                m_interrupt_enable = mstatus_mie;
                s_interrupt_enable = 1'b1; // S-mode interrupts always enabled in M-mode
            end
            PRIV_S: begin
                m_interrupt_enable = 1'b1; // M-mode interrupts always enabled in S-mode
                s_interrupt_enable = mstatus_sie;
            end
            PRIV_U: begin
                m_interrupt_enable = 1'b1;
                s_interrupt_enable = 1'b1;
            end
            default: begin
                m_interrupt_enable = 1'b0;
                s_interrupt_enable = 1'b0;
            end
        endcase
    end
    
    // Calculate pending interrupts
    always_comb begin
        // M-mode interrupts (not delegated)
        m_interrupts_pending = mip_combined & mie & ~mideleg;
        // S-mode interrupts (delegated)
        s_interrupts_pending = mip_combined & mie & mideleg;
    end
    
    // Final interrupt pending signal
    always_comb begin
        interrupt_pending = 1'b0;
        interrupt_cause = '0;
        
        // Priority: MEI > MSI > MTI > SEI > SSI > STI
        if (m_interrupt_enable && m_interrupts_pending[MIP_MEIP]) begin
            interrupt_pending = 1'b1;
            interrupt_cause = {1'b1, {(XLEN-5){1'b0}}, 4'd11}; // MEI
        end else if (m_interrupt_enable && m_interrupts_pending[MIP_MSIP]) begin
            interrupt_pending = 1'b1;
            interrupt_cause = {1'b1, {(XLEN-5){1'b0}}, 4'd3}; // MSI
        end else if (m_interrupt_enable && m_interrupts_pending[MIP_MTIP]) begin
            interrupt_pending = 1'b1;
            interrupt_cause = {1'b1, {(XLEN-5){1'b0}}, 4'd7}; // MTI
        end else if (s_interrupt_enable && s_interrupts_pending[MIP_SEIP]) begin
            interrupt_pending = 1'b1;
            interrupt_cause = {1'b1, {(XLEN-5){1'b0}}, 4'd9}; // SEI
        end else if (s_interrupt_enable && s_interrupts_pending[MIP_SSIP]) begin
            interrupt_pending = 1'b1;
            interrupt_cause = {1'b1, {(XLEN-5){1'b0}}, 4'd1}; // SSI
        end else if (s_interrupt_enable && s_interrupts_pending[MIP_STIP]) begin
            interrupt_pending = 1'b1;
            interrupt_cause = {1'b1, {(XLEN-5){1'b0}}, 4'd5}; // STI
        end
    end

    // =========================================================================
    // CSR Read Logic
    // =========================================================================
    
    logic [1:0] csr_priv_level;
    logic csr_read_only;
    
    assign csr_priv_level = csr_addr[9:8];
    assign csr_read_only = (csr_addr[11:10] == 2'b11);

    always_comb begin
        csr_rdata = '0;
        csr_access_fault = 1'b0;
        
        // Check privilege level
        if (current_priv < csr_priv_level) begin
            csr_access_fault = 1'b1;
        end else begin
            case (csr_addr)
                // FPU CSRs
                CSR_FFLAGS:  csr_rdata = {{(XLEN-5){1'b0}}, fflags_reg};
                CSR_FRM:     csr_rdata = {{(XLEN-3){1'b0}}, frm_reg};
                CSR_FCSR:    csr_rdata = {{(XLEN-8){1'b0}}, frm_reg, fflags_reg};

                // User-level counters
                CSR_CYCLE:   csr_rdata = cycle_count;
                CSR_TIME:    csr_rdata = time_count;
                CSR_INSTRET: csr_rdata = instret_count;

                // Supervisor CSRs
                CSR_SSTATUS:   csr_rdata = mstatus & SSTATUS_MASK;
                CSR_SIE:       csr_rdata = mie & mideleg;
                CSR_STVEC:     csr_rdata = stvec;
                CSR_SCOUNTEREN: csr_rdata = scounteren;
                CSR_SSCRATCH:  csr_rdata = sscratch;
                CSR_SEPC:      csr_rdata = sepc;
                CSR_SCAUSE:    csr_rdata = scause;
                CSR_STVAL:     csr_rdata = stval;
                CSR_SIP:       csr_rdata = mip_combined & mideleg;
                CSR_SATP: begin
                    // Check TVM
                    if (current_priv == PRIV_S && mstatus_tvm)
                        csr_access_fault = 1'b1;
                    else
                        csr_rdata = satp;
                end
                
                // Machine CSRs
                CSR_MSTATUS:   csr_rdata = mstatus;
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
                
                default: begin
                    csr_rdata = '0; // Don't fault on unknown CSRs, just return 0
                end
            endcase
        end
    end

    // =========================================================================
    // CSR Write Logic
    // =========================================================================
    
    // CSR operation decoding
    localparam CSR_OP_RW  = 3'b001; // CSRRW
    localparam CSR_OP_RS  = 3'b010; // CSRRS
    localparam CSR_OP_RC  = 3'b011; // CSRRC
    localparam CSR_OP_RWI = 3'b101; // CSRRWI
    localparam CSR_OP_RSI = 3'b110; // CSRRSI
    localparam CSR_OP_RCI = 3'b111; // CSRRCI
    
    logic [XLEN-1:0] csr_new_value;
    logic csr_write_enable;
    
    always_comb begin
        csr_new_value = csr_rdata;
        csr_write_enable = 1'b0;
        
        if (csr_req_valid && !csr_access_fault && !csr_read_only) begin
            case (csr_op)
                CSR_OP_RW, CSR_OP_RWI: begin
                    csr_new_value = csr_wdata;
                    csr_write_enable = 1'b1;
                end
                CSR_OP_RS, CSR_OP_RSI: begin
                    csr_new_value = csr_rdata | csr_wdata;
                    csr_write_enable = (csr_wdata != '0);
                end
                CSR_OP_RC, CSR_OP_RCI: begin
                    csr_new_value = csr_rdata & ~csr_wdata;
                    csr_write_enable = (csr_wdata != '0);
                end
                default: csr_write_enable = 1'b0;
            endcase
        end
    end

    // =========================================================================
    // Trap Entry/Return Logic
    // =========================================================================
    
    logic trap_to_m_mode;
    
    // Determine if trap goes to M-mode or S-mode
    always_comb begin
        trap_to_m_mode = 1'b1;
        if (trap_from_priv != PRIV_M) begin
            if (trap_is_interrupt) begin
                // Interrupt delegation
                trap_to_m_mode = ~mideleg[trap_cause[XLEN-2:0]];
            end else begin
                // Exception delegation
                trap_to_m_mode = ~medeleg[trap_cause[5:0]];
            end
        end
    end
    
    // Trap vector calculation
    always_comb begin
        if (trap_to_m_mode) begin
            trap_vector = (mtvec[1:0] == 2'b01 && trap_is_interrupt) ?
                         {mtvec[XLEN-1:2], 2'b00} + (trap_cause[XLEN-2:0] << 2) :
                         {mtvec[XLEN-1:2], 2'b00};
        end else begin
            trap_vector = (stvec[1:0] == 2'b01 && trap_is_interrupt) ?
                         {stvec[XLEN-1:2], 2'b00} + (trap_cause[XLEN-2:0] << 2) :
                         {stvec[XLEN-1:2], 2'b00};
        end
    end
    
    // Return PC and privilege
    always_comb begin
        if (mret_execute) begin
            return_pc = mepc;
            return_priv = mstatus_mpp;
        end else if (sret_execute) begin
            return_pc = sepc;
            return_priv = {1'b0, mstatus_spp};
        end else begin
            return_pc = '0;
            return_priv = current_priv;
        end
    end

    // =========================================================================
    // Sequential Logic
    // =========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Machine Mode CSRs
            mstatus <= '0;
            mstatus[MSTATUS_SXL_HI:MSTATUS_SXL_LO] <= 2'b10; // XLEN=64 for S-mode
            mstatus[MSTATUS_UXL_HI:MSTATUS_UXL_LO] <= 2'b10; // XLEN=64 for U-mode
            
            // MISA: RV64IMAFDS + U + S
            misa <= {2'b10,                  // MXL = 64-bit
                    {(XLEN-28){1'b0}},
                    1'b0,                    // Z (reserved)
                    1'b0,                    // Y
                    1'b0,                    // X
                    1'b0,                    // W
                    1'b0,                    // V
                    1'b1,                    // U (User mode)
                    1'b0,                    // T
                    1'b1,                    // S (Supervisor mode)
                    1'b0,                    // R
                    1'b0,                    // Q
                    1'b0,                    // P
                    1'b0,                    // O
                    1'b0,                    // N
                    1'b1,                    // M (Integer Multiply/Divide)
                    1'b0,                    // L
                    1'b0,                    // K
                    1'b0,                    // J
                    1'b1,                    // I (RV64I Base)
                    1'b0,                    // H
                    1'b0,                    // G
                    1'b1,                    // F (Single-precision FP)
                    1'b0,                    // E
                    1'b1,                    // D (Double-precision FP)
                    1'b0,                    // C (Compressed)
                    1'b0,                    // B
                    1'b1};                   // A (Atomics)
            
            medeleg <= '0;
            mideleg <= '0;
            mie <= '0;
            mip <= '0;
            mtvec <= 64'h0000_0000_1200_0000;
            mscratch <= '0;
            mepc <= '0;
            mcause <= '0;
            mtval <= '0;
            mcounteren <= '0;
            
            // Supervisor Mode CSRs
            stvec <= '0;
            sscratch <= '0;
            sepc <= '0;
            scause <= '0;
            stval <= '0;
            satp <= '0;
            scounteren <= '0;

            // FPU Registers
            fflags_reg <= '0;
            frm_reg <= '0;

        end else begin
            
            // Update FFlags from FPU Pipeline (Acumula flags)
            if (fflags_we) begin
                fflags_reg[0] <= fflags_reg[0] | fflags_in.NX;
                fflags_reg[1] <= fflags_reg[1] | fflags_in.UF;
                fflags_reg[2] <= fflags_reg[2] | fflags_in.OF;
                fflags_reg[3] <= fflags_reg[3] | fflags_in.DZ;
                fflags_reg[4] <= fflags_reg[4] | fflags_in.NV;
            end

            // =================================================================
            // Trap Entry
            // =================================================================
            if (trap_enter) begin
                if (trap_to_m_mode) begin
                    // Save state to M-mode CSRs
                    mepc <= trap_pc;
                    mcause <= trap_cause;
                    mtval <= trap_value;
                    
                    // Update mstatus
                    mstatus[MSTATUS_MPIE_BIT] <= mstatus[MSTATUS_MIE_BIT];
                    mstatus[MSTATUS_MIE_BIT] <= 1'b0;
                    mstatus[MSTATUS_MPP_HI:MSTATUS_MPP_LO] <= trap_from_priv;
                end else begin
                    // Save state to S-mode CSRs
                    sepc <= trap_pc;
                    scause <= trap_cause;
                    stval <= trap_value;
                    
                    // Update mstatus (sstatus view)
                    mstatus[MSTATUS_SPIE_BIT] <= mstatus[MSTATUS_SIE_BIT];
                    mstatus[MSTATUS_SIE_BIT] <= 1'b0;
                    mstatus[MSTATUS_SPP_BIT] <= trap_from_priv[0];
                end
            end
            
            // =================================================================
            // MRET
            // =================================================================
            else if (mret_execute) begin
                mstatus[MSTATUS_MIE_BIT] <= mstatus[MSTATUS_MPIE_BIT];
                mstatus[MSTATUS_MPIE_BIT] <= 1'b1;
                mstatus[MSTATUS_MPP_HI:MSTATUS_MPP_LO] <= PRIV_U; // Set to least privileged
                if (mstatus_mpp != PRIV_M)
                    mstatus[MSTATUS_MPRV_BIT] <= 1'b0;
            end
            
            // =================================================================
            // SRET
            // =================================================================
            else if (sret_execute) begin
                mstatus[MSTATUS_SIE_BIT] <= mstatus[MSTATUS_SPIE_BIT];
                mstatus[MSTATUS_SPIE_BIT] <= 1'b1;
                mstatus[MSTATUS_SPP_BIT] <= 1'b0; // Set to U-mode
                mstatus[MSTATUS_MPRV_BIT] <= 1'b0;
            end
            
            // =================================================================
            // CSR Writes
            // =================================================================
            else if (csr_write_enable) begin
                case (csr_addr)
                    // FPU CSRs
                    CSR_FFLAGS: fflags_reg <= csr_new_value[4:0];
                    CSR_FRM:    frm_reg    <= csr_new_value[2:0];
                    CSR_FCSR: begin
                        fflags_reg <= csr_new_value[4:0];
                        frm_reg    <= csr_new_value[7:5];
                    end

                    // Supervisor CSRs
                    CSR_SSTATUS: begin
                        mstatus <= (mstatus & ~SSTATUS_MASK) | (csr_new_value & SSTATUS_MASK);
                    end
                    CSR_SIE: begin
                        mie <= (mie & ~mideleg) | (csr_new_value & mideleg);
                    end
                    CSR_STVEC:     stvec <= {csr_new_value[XLEN-1:2], 1'b0, csr_new_value[0]};
                    CSR_SCOUNTEREN: scounteren <= csr_new_value;
                    CSR_SSCRATCH:  sscratch <= csr_new_value;
                    CSR_SEPC:      sepc <= {csr_new_value[XLEN-1:1], 1'b0};
                    CSR_SCAUSE:    scause <= csr_new_value;
                    CSR_STVAL:     stval <= csr_new_value;
                    CSR_SIP: begin
                        // Only SSIP is writable
                        mip[MIP_SSIP] <= csr_new_value[MIP_SSIP];
                    end
                    CSR_SATP: begin
                        if (!(current_priv == PRIV_S && mstatus_tvm))
                            satp <= csr_new_value;
                    end
                    
                    // Machine CSRs
                    CSR_MSTATUS: begin
                        // Apply WARL constraints
                        mstatus <= csr_new_value;
                        // Force SXL and UXL to 2 (64-bit)
                        mstatus[MSTATUS_SXL_HI:MSTATUS_SXL_LO] <= 2'b10;
                        mstatus[MSTATUS_UXL_HI:MSTATUS_UXL_LO] <= 2'b10;
                    end
                    CSR_MEDELEG:   medeleg <= csr_new_value;
                    CSR_MIDELEG:   mideleg <= csr_new_value & 64'h0222; // Only S-mode interrupts
                    CSR_MIE:       mie <= csr_new_value;
                    CSR_MTVEC:     mtvec <= {csr_new_value[XLEN-1:2], 1'b0, csr_new_value[0]};
                    CSR_MCOUNTEREN: mcounteren <= csr_new_value;
                    CSR_MSCRATCH:  mscratch <= csr_new_value;
                    CSR_MEPC:      mepc <= {csr_new_value[XLEN-1:1], 1'b0};
                    CSR_MCAUSE:    mcause <= csr_new_value;
                    CSR_MTVAL:     mtval <= csr_new_value;
                    CSR_MIP: begin
                        // Only SSIP, STIP, SEIP are writable
                        mip[MIP_SSIP] <= csr_new_value[MIP_SSIP];
                        mip[MIP_STIP] <= csr_new_value[MIP_STIP];
                        mip[MIP_SEIP] <= csr_new_value[MIP_SEIP];
                    end
                    
                    default: ; // Ignore writes to unknown CSRs
                endcase
            end
        end
    end

endmodule
