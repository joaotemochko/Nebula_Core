`timescale 1ns/1ps
`default_nettype none

import nebula_pkg::*;

/**
 * @module nebula_core_full
 * @brief Núcleo Nebula completo - RV64GC para Linux
 *
 * @details Pipeline in-order com:
 * - Extensões: IMAFDC + Zicsr + Zifencei
 * - Branch Predictor: gshare + BTB + RAS
 * - FPU: IEEE 754 single/double
 * - Compressão: Decodificador RVC
 * - Cache L1: I-Cache + D-Cache (interfaces externas)
 * - MMU: Sv39 com TLB split
 */
module nebula_core #(
    parameter int HART_ID = 0,
    parameter int XLEN = 64,
    parameter int PADDR_WIDTH = 56,
    parameter int VADDR_WIDTH = 39,
    parameter int L1I_SIZE_KB = 32,
    parameter int L1D_SIZE_KB = 32,
    parameter int L1_LINE_SIZE = 64,
    parameter int L1_WAYS = 4
)(
    input  wire                     clk,
    input  wire                     rst_n,
    
    // =========================================================================
    // Interface I-Cache -> L2
    // =========================================================================
    output logic                    imem_req,
    output logic [PADDR_WIDTH-1:0]  imem_addr,
    input  wire                     imem_ack,
    input  wire [L1_LINE_SIZE*8-1:0] imem_data,
    input  wire                     imem_error,
    
    // =========================================================================
    // Interface D-Cache -> L2
    // =========================================================================
    output logic                    dmem_req,
    output logic                    dmem_we,
    output logic [PADDR_WIDTH-1:0]  dmem_addr,
    output logic [L1_LINE_SIZE*8-1:0] dmem_wdata,
    input  wire                     dmem_ack,
    input  wire [L1_LINE_SIZE*8-1:0] dmem_rdata,
    input  wire                     dmem_error,
    output logic                    dmem_is_amo,
    output logic [4:0]              dmem_amo_op,
    output logic                    dmem_upgrade,
    
    // =========================================================================
    // Interface PTW -> L2
    // =========================================================================
    output logic                    ptw_mem_req,
    output logic [PADDR_WIDTH-1:0]  ptw_mem_addr,
    input  wire                     ptw_mem_ack,
    input  wire [XLEN-1:0]          ptw_mem_data,
    input  wire                     ptw_mem_error,
    
    // =========================================================================
    // Snoop Interface
    // =========================================================================
    input  snoop_req_t              snoop_req_in,
    output snoop_resp_t             snoop_resp_out,
    
    // =========================================================================
    // Interrupções
    // =========================================================================
    input  wire                     timer_irq,
    input  wire                     external_irq,
    input  wire                     software_irq,
    
    // =========================================================================
    // Debug
    // =========================================================================
    input  wire                     debug_req,
    output logic                    debug_halted
);

    // =========================================================================
    // Constantes
    // =========================================================================
    
    localparam int VPN_WIDTH = 27;
    localparam int PPN_WIDTH = 44;
    localparam int ASID_WIDTH = 16;
    localparam int TLB_ENTRIES = 32;
    localparam int L1_SETS = (L1I_SIZE_KB * 1024) / (L1_WAYS * L1_LINE_SIZE);
    
    // =========================================================================
    // Sinais Internos
    // =========================================================================
    
    // Frontend <-> Backend
    frontend_packet_t   frontend_packet;
    logic               frontend_valid;
    backend_ctrl_t      backend_ctrl;
    
    // Branch Predictor
    bp_prediction_t     bp_prediction;
    bp_update_t         bp_update;
    logic [VADDR_WIDTH-1:0] fetch_pc;
    
    // I-Cache interface interno
    logic                    icache_req;
    logic [PADDR_WIDTH-1:0]  icache_addr;
    logic                    icache_ready;
    logic                    icache_resp_valid;
    logic [63:0]             icache_resp_data;
    logic                    icache_resp_error;
    
    // D-Cache interface interno
    logic                    dcache_req;
    logic [PADDR_WIDTH-1:0]  dcache_addr;
    logic [XLEN-1:0]         dcache_wdata;
    logic [7:0]              dcache_wstrb;
    logic                    dcache_we;
    logic                    dcache_is_amo;
    logic [4:0]              dcache_amo_op;
    logic                    dcache_ready;
    logic                    dcache_resp_valid;
    logic [XLEN-1:0]         dcache_resp_data;
    logic                    dcache_resp_error;
    
    // TLB interfaces
    logic                    itlb_req, dtlb_req;
    logic [VPN_WIDTH-1:0]    itlb_vpn, dtlb_vpn;
    logic                    itlb_hit, dtlb_hit;
    logic [PPN_WIDTH-1:0]    itlb_ppn, dtlb_ppn;
    logic                    itlb_page_fault, dtlb_page_fault;
    logic                    dtlb_is_store;
    
    // PTW
    logic                    ptw_req_itlb, ptw_req_dtlb;
    logic [VPN_WIDTH-1:0]    ptw_vpn;
    logic                    ptw_ready, ptw_resp_valid;
    logic                    ptw_page_fault;
    logic                    ptw_for_itlb;
    
    // CSR
    logic                    csr_req;
    logic [11:0]             csr_addr;
    logic [XLEN-1:0]         csr_wdata, csr_rdata;
    logic [2:0]              csr_op;
    logic                    csr_fault;
    logic [XLEN-1:0]         csr_satp, csr_mstatus;
    priv_t                   current_priv;
    logic                    mmu_enabled;
    
    // Trap
    logic                    trap_enter, trap_is_interrupt;
    logic [XLEN-1:0]         trap_pc, trap_cause, trap_value;
    logic                    mret_exec, sret_exec;
    logic [XLEN-1:0]         trap_vector, return_pc;
    logic [1:0]              return_priv;
    logic                    interrupt_pending;
    logic [XLEN-1:0]         interrupt_cause;
    
    // MDU
    logic                    mdu_req, mdu_ready, mdu_resp_valid;
    logic [XLEN-1:0]         mdu_rs1, mdu_rs2, mdu_result;
    logic [2:0]              mdu_funct3;
    logic                    mdu_is_word;
    
    // FPU
    logic                    fpu_req, fpu_ready, fpu_resp_valid;
    fpu_op_t                 fpu_op;
    logic [63:0]             fpu_rs1, fpu_rs2, fpu_rs3;
    logic [63:0]             fpu_int_rs1;
    rounding_mode_t          fpu_rm;
    logic                    fpu_is_single;
    logic [63:0]             fpu_result, fpu_int_result;
    fflags_t                 fpu_fflags;
    
    // SFENCE
    logic                    sfence_valid, sfence_all;
    logic [VPN_WIDTH-1:0]    sfence_vpn;
    logic [15:0]             sfence_asid;
    
    // FENCE.I
    logic                    fence_i_valid;
    logic                    dcache_flush, dcache_flush_done;
    
    // Performance counters
    logic [63:0]             cycle_counter, instret_counter, time_counter;
    logic                    instr_retired, instr_retired_2;
    
    // =========================================================================
    // Branch Predictor
    // =========================================================================
    
    branch_predictor #(
        .VADDR_WIDTH(VADDR_WIDTH),
        .BTB_ENTRIES(256),
        .BHT_ENTRIES(1024),
        .RAS_DEPTH(8),
        .GHR_LEN(10)
    ) u_bp (
        .clk(clk),
        .rst_n(rst_n),
        .pc(fetch_pc),
        .predict_valid(icache_req),
        .prediction(bp_prediction),
        .update_valid(backend_ctrl.redirect),
        .update(bp_update)
    );
    
    // =========================================================================
    // I-Cache
    // =========================================================================
    
    icache_l1 #(
        .PADDR_WIDTH(PADDR_WIDTH),
        .LINE_SIZE(L1_LINE_SIZE),
        .NUM_WAYS(L1_WAYS),
        .NUM_SETS(L1_SETS)
    ) u_icache (
        .clk(clk),
        .rst_n(rst_n),
        .req_valid(icache_req),
        .req_addr(icache_addr),
        .req_ready(icache_ready),
        .resp_valid(icache_resp_valid),
        .resp_data(icache_resp_data),
        .resp_error(icache_resp_error),
        .invalidate_all(fence_i_valid),
        .invalidate_addr_valid(1'b0),
        .invalidate_addr('0),
        .mem_req(imem_req),
        .mem_addr(imem_addr),
        .mem_ack(imem_ack),
        .mem_data(imem_data),
        .mem_error(imem_error)
    );
    
    // =========================================================================
    // D-Cache
    // =========================================================================
    
    dcache_l1 #(
        .PADDR_WIDTH(PADDR_WIDTH),
        .XLEN(XLEN),
        .LINE_SIZE(L1_LINE_SIZE),
        .NUM_WAYS(L1_WAYS),
        .NUM_SETS(L1_SETS)
    ) u_dcache (
        .clk(clk),
        .rst_n(rst_n),
        .req_valid(dcache_req),
        .req_addr(dcache_addr),
        .req_wdata(dcache_wdata),
        .req_wstrb(dcache_wstrb),
        .req_we(dcache_we),
        .req_is_amo(dcache_is_amo),
        .req_amo_op(dcache_amo_op),
        .req_ready(dcache_ready),
        .resp_valid(dcache_resp_valid),
        .resp_rdata(dcache_resp_data),
        .resp_error(dcache_resp_error),
        .invalidate_all(1'b0),
        .flush_all(dcache_flush),
        .flush_addr_valid(1'b0),
        .flush_addr('0),
        .flush_done(dcache_flush_done),
        .mem_req(dmem_req),
        .mem_we(dmem_we),
        .mem_addr(dmem_addr),
        .mem_wdata(dmem_wdata),
        .mem_ack(dmem_ack),
        .mem_rdata(dmem_rdata),
        .mem_error(dmem_error)
    );
    
    // D-Cache extra signals
    assign dmem_is_amo = dcache_is_amo;
    assign dmem_amo_op = dcache_amo_op;
    assign dmem_upgrade = 1'b0;  // TODO: implement upgrade
    
    // =========================================================================
    // FPU
    // =========================================================================
    
    fpu_top #(
        .FLEN(64)
    ) u_fpu (
        .clk(clk),
        .rst_n(rst_n),
        .req_valid(fpu_req),
        .req_ready(fpu_ready),
        .op(fpu_op),
        .rs1(fpu_rs1),
        .rs2(fpu_rs2),
        .rs3(fpu_rs3),
        .int_rs1(fpu_int_rs1),
        .rm(fpu_rm),
        .is_single(fpu_is_single),
        .resp_valid(fpu_resp_valid),
        .result(fpu_result),
        .int_result(fpu_int_result),
        .fflags(fpu_fflags)
    );
    
    // =========================================================================
    // MDU
    // =========================================================================
    
    mdu_rv64 #(
        .XLEN(XLEN)
    ) u_mdu (
        .clk(clk),
        .rst_n(rst_n),
        .req_valid(mdu_req),
        .req_ready(mdu_ready),
        .rs1_data(mdu_rs1),
        .rs2_data(mdu_rs2),
        .funct3(mdu_funct3),
        .is_word_op(mdu_is_word),
        .resp_valid(mdu_resp_valid),
        .result(mdu_result)
    );
    
    // =========================================================================
    // CSR Unit
    // =========================================================================
    
    csr_unit #(
        .XLEN(XLEN),
        .HART_ID(HART_ID)
    ) u_csr (
        .clk(clk),
        .rst_n(rst_n),
        .csr_req_valid(csr_req),
        .csr_addr(csr_addr),
        .csr_wdata(csr_wdata),
        .csr_op(csr_op),
        .current_priv(current_priv),
        .csr_rdata(csr_rdata),
        .csr_access_fault(csr_fault),
        .timer_irq_in(timer_irq),
        .external_irq_in(external_irq),
        .software_irq_in(software_irq),
        .trap_enter(trap_enter),
        .trap_is_interrupt(trap_is_interrupt),
        .trap_pc(trap_pc),
        .trap_cause(trap_cause),
        .trap_value(trap_value),
        .trap_from_priv(current_priv),
        .mret_execute(mret_exec),
        .sret_execute(sret_exec),
        .trap_vector(trap_vector),
        .return_pc(return_pc),
        .return_priv(return_priv),
        .interrupt_pending(interrupt_pending),
        .interrupt_cause(interrupt_cause),
        .csr_satp(csr_satp),
        .csr_mstatus(csr_mstatus),
        .mstatus_mie(),
        .mstatus_sie(),
        .mstatus_tvm(),
        .mstatus_tsr(),
        .mstatus_mprv(),
        .mstatus_mpp(),
        .mstatus_spp(),
        .cycle_count(cycle_counter),
        .time_count(time_counter),
        .instret_count(instret_counter),
        // FPU flags
        .fflags_in(fpu_fflags),
        .fflags_we(fpu_resp_valid),
        .frm_out(fpu_rm)
    );
    
    // MMU enable
    assign mmu_enabled = (csr_satp[63:60] != 0) && (current_priv != PRIV_MACHINE);
    
    // =========================================================================
    // TLBs
    // =========================================================================
    
    tlb_sv39 #(
        .TLB_ENTRIES(TLB_ENTRIES)
    ) u_itlb (
        .clk(clk),
        .rst_n(rst_n),
        .lookup_valid(itlb_req),
        .lookup_vpn(itlb_vpn),
        .lookup_asid(csr_satp[59:44]),
        .lookup_priv(current_priv),
        .lookup_is_store(1'b0),
        .lookup_is_exec(1'b1),
        .mstatus_sum(csr_mstatus[18]),
        .mstatus_mxr(csr_mstatus[19]),
        .lookup_hit(itlb_hit),
        .lookup_ppn(itlb_ppn),
        .lookup_page_fault(itlb_page_fault),
        // ... other connections
        .invalidate_all(sfence_valid && sfence_all),
        .invalidate_by_asid(sfence_valid && !sfence_all && (sfence_vpn == '0)),
        .invalidate_by_addr(sfence_valid && !sfence_all && (sfence_asid == '0)),
        .invalidate_by_both(sfence_valid && !sfence_all && (sfence_vpn != '0) && (sfence_asid != '0)),
        .invalidate_asid(sfence_asid),
        .invalidate_vpn(sfence_vpn)
    );
    
    tlb_sv39 #(
        .TLB_ENTRIES(TLB_ENTRIES)
    ) u_dtlb (
        .clk(clk),
        .rst_n(rst_n),
        .lookup_valid(dtlb_req),
        .lookup_vpn(dtlb_vpn),
        .lookup_asid(csr_satp[59:44]),
        .lookup_priv(current_priv),
        .lookup_is_store(dtlb_is_store),
        .lookup_is_exec(1'b0),
        .mstatus_sum(csr_mstatus[18]),
        .mstatus_mxr(csr_mstatus[19]),
        .lookup_hit(dtlb_hit),
        .lookup_ppn(dtlb_ppn),
        .lookup_page_fault(dtlb_page_fault),
        // ... other connections
        .invalidate_all(sfence_valid && sfence_all),
        .invalidate_by_asid(sfence_valid && !sfence_all && (sfence_vpn == '0)),
        .invalidate_by_addr(sfence_valid && !sfence_all && (sfence_asid == '0)),
        .invalidate_by_both(sfence_valid && !sfence_all && (sfence_vpn != '0) && (sfence_asid != '0)),
        .invalidate_asid(sfence_asid),
        .invalidate_vpn(sfence_vpn)
    );
    
    // =========================================================================
    // PTW
    // =========================================================================
    
    ptw_sv39 u_ptw (
        .clk(clk),
        .rst_n(rst_n),
        .ptw_req_valid(ptw_req_itlb || ptw_req_dtlb),
        .ptw_req_vpn(ptw_vpn),
        .ptw_req_asid(csr_satp[59:44]),
        .ptw_req_is_store(dtlb_is_store && ptw_req_dtlb),
        .ptw_req_is_exec(ptw_req_itlb),
        .ptw_req_for_itlb(ptw_req_itlb),
        .ptw_req_ready(ptw_ready),
        .satp(csr_satp),
        .ptw_mem_req(ptw_mem_req),
        .ptw_mem_addr(ptw_mem_addr),
        .ptw_mem_resp_valid(ptw_mem_ack),
        .ptw_mem_resp_data(ptw_mem_data),
        .ptw_mem_resp_err(ptw_mem_error),
        .ptw_resp_valid(ptw_resp_valid),
        .ptw_resp_page_fault(ptw_page_fault)
    );
    
    // =========================================================================
    // Frontend (com suporte a extensão C)
    // =========================================================================
    
    nebula_frontend #(
        .XLEN(XLEN),
        .VADDR_WIDTH(VADDR_WIDTH),
        .PADDR_WIDTH(PADDR_WIDTH),
        .VPN_WIDTH(VPN_WIDTH),
        .PPN_WIDTH(PPN_WIDTH)
    ) u_frontend (
        .clk(clk),
        .rst_n(rst_n),
        
        // Backend control
        .backend_stall(backend_ctrl.stall),
        .backend_flush(backend_ctrl.flush),
        .backend_redirect(backend_ctrl.redirect),
        .backend_redirect_pc(backend_ctrl.redirect_pc),
        
        // Output
        .frontend_out(frontend_packet),
        .frontend_valid(frontend_valid),
        .fetch_pc_out(fetch_pc),
        
        // Branch prediction
        .bp_prediction(bp_prediction),
        
        // I-Cache
        .icache_req(icache_req),
        .icache_addr(icache_addr),
        .icache_ready(icache_ready),
        .icache_resp_valid(icache_resp_valid),
        .icache_resp_data(icache_resp_data),
        .icache_resp_error(icache_resp_error),
        
        // ITLB
        .itlb_req(itlb_req),
        .itlb_vpn(itlb_vpn),
        .itlb_hit(itlb_hit),
        .itlb_ppn(itlb_ppn),
        .itlb_page_fault(itlb_page_fault),
        
        // PTW
        .ptw_req(ptw_req_itlb),
        .ptw_ready(ptw_ready),
        .ptw_resp_valid(ptw_resp_valid && ptw_for_itlb),
        .ptw_page_fault(ptw_page_fault),
        
        // Control
        .mmu_enabled(mmu_enabled),
        .current_priv(current_priv)
    );
    
    // =========================================================================
    // Backend (com FPU)
    // =========================================================================
    
    nebula_backend #(
        .XLEN(XLEN),
        .VADDR_WIDTH(VADDR_WIDTH),
        .PADDR_WIDTH(PADDR_WIDTH),
        .VPN_WIDTH(VPN_WIDTH),
        .PPN_WIDTH(PPN_WIDTH)
    ) u_backend (
        .clk(clk),
        .rst_n(rst_n),
        
        // Frontend
        .frontend_valid(frontend_valid),
        .frontend_in(frontend_packet),
        .backend_ctrl(backend_ctrl),
        .bp_update(bp_update),
        
        // D-Cache
        .dcache_req(dcache_req),
        .dcache_addr(dcache_addr),
        .dcache_wdata(dcache_wdata),
        .dcache_wstrb(dcache_wstrb),
        .dcache_we(dcache_we),
        .dcache_is_amo(dcache_is_amo),
        .dcache_amo_op(dcache_amo_op),
        .dcache_ready(dcache_ready),
        .dcache_resp_valid(dcache_resp_valid),
        .dcache_resp_data(dcache_resp_data),
        .dcache_resp_error(dcache_resp_error),
        
        // DTLB
        .dtlb_req(dtlb_req),
        .dtlb_vpn(dtlb_vpn),
        .dtlb_is_store(dtlb_is_store),
        .dtlb_hit(dtlb_hit),
        .dtlb_ppn(dtlb_ppn),
        .dtlb_page_fault(dtlb_page_fault),
        
        // PTW
        .ptw_req(ptw_req_dtlb),
        .ptw_ready(ptw_ready),
        .ptw_resp_valid(ptw_resp_valid && !ptw_for_itlb),
        .ptw_page_fault(ptw_page_fault),
        
        // CSR
        .csr_req(csr_req),
        .csr_addr(csr_addr),
        .csr_wdata(csr_wdata),
        .csr_op(csr_op),
        .csr_rdata(csr_rdata),
        .csr_fault(csr_fault),
        
        // Trap
        .trap_enter(trap_enter),
        .trap_is_interrupt(trap_is_interrupt),
        .trap_pc(trap_pc),
        .trap_cause(trap_cause),
        .trap_value(trap_value),
        .mret_exec(mret_exec),
        .sret_exec(sret_exec),
        .trap_vector(trap_vector),
        .return_pc(return_pc),
        .return_priv(return_priv),
        .interrupt_pending(interrupt_pending),
        .interrupt_cause(interrupt_cause),
        
        // MDU
        .mdu_req(mdu_req),
        .mdu_rs1(mdu_rs1),
        .mdu_rs2(mdu_rs2),
        .mdu_funct3(mdu_funct3),
        .mdu_is_word(mdu_is_word),
        .mdu_ready(mdu_ready),
        .mdu_resp_valid(mdu_resp_valid),
        .mdu_result(mdu_result),
        
        // FPU
        .fpu_req(fpu_req),
        .fpu_op(fpu_op),
        .fpu_rs1(fpu_rs1),
        .fpu_rs2(fpu_rs2),
        .fpu_rs3(fpu_rs3),
        .fpu_int_rs1(fpu_int_rs1),
        .fpu_rm(fpu_rm),
        .fpu_is_single(fpu_is_single),
        .fpu_ready(fpu_ready),
        .fpu_resp_valid(fpu_resp_valid),
        .fpu_result(fpu_result),
        .fpu_int_result(fpu_int_result),
        .fpu_fflags(fpu_fflags),
        
        // Control
        .current_priv(current_priv),
        .mmu_enabled(mmu_enabled),
        
        // SFENCE
        .sfence_valid(sfence_valid),
        .sfence_all(sfence_all),
        .sfence_vpn(sfence_vpn),
        .sfence_asid(sfence_asid),
        
        // FENCE.I
        .fence_i_valid(fence_i_valid),
        .dcache_flush(dcache_flush),
        .dcache_flush_done(dcache_flush_done),
        
        // Performance
        .instr_retired(instr_retired),
        .instr_retired_2(instr_retired_2)
    );
    
    // =========================================================================
    // Performance Counters
    // =========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cycle_counter <= '0;
            instret_counter <= '0;
            time_counter <= '0;
        end else begin
            cycle_counter <= cycle_counter + 1;
            time_counter <= time_counter + 1;
            if (instr_retired)
                instret_counter <= instret_counter + (instr_retired_2 ? 2 : 1);
        end
    end
    
    // =========================================================================
    // Snoop Response (para coerência L1)
    // =========================================================================
    
    // TODO: Implementar snoop handling no D-Cache
    assign snoop_resp_out = '0;
    
    // Debug
    assign debug_halted = 1'b0;  // TODO: implement debug support

endmodule
