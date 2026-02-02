`timescale 1ns/1ps
`default_nettype none

import nebula_pkg::*;

/**
 * @module nebula_core_full
 * @brief Núcleo Nebula completo - RV64GC para Linux
 *
 * Módulos instanciados:
 * - branch_predictor
 * - icache_l1, dcache_l1
 * - fpu_ieee754 (IEEE 754 compliant)
 * - mdu_rv64
 * - csr_unit
 * - tlb_sv39 (x2), ptw_sv39
 * - nebula_frontend_rvc (inclui compressed_decoder_rv64)
 * - nebula_backend_fpu
 */
module nebula_core #(
    parameter int HART_ID = 0,
    parameter int XLEN = 64,
    parameter int FLEN = 64,
    parameter int PADDR_WIDTH = 56,
    parameter int VADDR_WIDTH = 39,
    parameter int L1I_SIZE_KB = 32,
    parameter int L1D_SIZE_KB = 32,
    parameter int L1_LINE_SIZE = 64,
    parameter int L1_WAYS = 4
)(
    input  wire                     clk,
    input  wire                     rst_n,
    output logic                    imem_req,
    output logic [PADDR_WIDTH-1:0]  imem_addr,
    input  wire                     imem_ack,
    input  wire [L1_LINE_SIZE*8-1:0] imem_data,
    input  wire                     imem_error,
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
    output logic                    ptw_mem_req,
    output logic [PADDR_WIDTH-1:0]  ptw_mem_addr,
    input  wire                     ptw_mem_ack,
    input  wire [XLEN-1:0]          ptw_mem_data,
    input  wire                     ptw_mem_error,
    input  snoop_req_t              snoop_req_in,
    output snoop_resp_t             snoop_resp_out,
    input  wire                     timer_irq,
    input  wire                     external_irq,
    input  wire                     software_irq,
    input  wire                     debug_req,
    output logic                    debug_halted
);
    localparam int VPN_WIDTH = 27;
    localparam int PPN_WIDTH = 44;
    localparam int TLB_ENTRIES = 32;
    localparam int L1_SETS = (L1I_SIZE_KB * 1024) / (L1_WAYS * L1_LINE_SIZE);

    // Internal signals
    frontend_packet_t frontend_packet;
    logic frontend_valid;
    
    // Sinais de Exceção do Frontend (Novos)
    logic frontend_exc;
    logic [5:0] frontend_exc_cause;
    logic [XLEN-1:0] frontend_exc_value;

    backend_ctrl_t backend_ctrl;
    bp_prediction_t bp_prediction;
    bp_update_t bp_update;
    logic [VADDR_WIDTH-1:0] fetch_pc;

    // I-Cache
    logic icache_req;
    logic [PADDR_WIDTH-1:0] icache_addr;
    logic icache_ready, icache_resp_valid, icache_resp_error;
    logic [63:0] icache_resp_data;

    // D-Cache
    logic dcache_req, dcache_we, dcache_is_amo, dcache_ready, dcache_resp_valid, dcache_resp_error;
    logic [PADDR_WIDTH-1:0] dcache_addr;
    logic [XLEN-1:0] dcache_wdata, dcache_resp_data;
    logic [7:0] dcache_wstrb;
    logic [4:0] dcache_amo_op;

    // TLB Signals
    logic itlb_req, itlb_hit, itlb_page_fault, itlb_access_fault;
    logic dtlb_req, dtlb_hit, dtlb_page_fault, dtlb_is_store, dtlb_access_fault;
    logic [VPN_WIDTH-1:0] itlb_vpn, dtlb_vpn;
    logic [PPN_WIDTH-1:0] itlb_ppn, dtlb_ppn;
    // Novos fios de update A/D e Fault VPN
    logic itlb_need_a, itlb_need_d;
    logic dtlb_need_a, dtlb_need_d;
    logic [VPN_WIDTH-1:0] itlb_fault_vpn, dtlb_fault_vpn;

    // PTW Signals (Expandidos)
    logic ptw_req_itlb, ptw_req_dtlb, ptw_ready, ptw_resp_valid, ptw_page_fault, ptw_access_fault, ptw_is_for_itlb;
    logic [VPN_WIDTH-1:0] ptw_vpn_itlb, ptw_vpn_dtlb, ptw_vpn, ptw_resp_vpn;
    logic [PPN_WIDTH-1:0] ptw_resp_ppn;
    logic [15:0] ptw_resp_asid;
    logic [1:0] ptw_resp_page_size;
    logic ptw_resp_for_itlb;
    // Permissões detalhadas do PTW
    logic ptw_resp_r, ptw_resp_w, ptw_resp_x, ptw_resp_u, ptw_resp_g, ptw_resp_a, ptw_resp_d;

    // CSR
    logic csr_req, csr_fault;
    logic [11:0] csr_addr;
    logic [XLEN-1:0] csr_wdata, csr_rdata, csr_satp, csr_mstatus;
    logic [2:0] csr_op;
    priv_t current_priv;
    logic mmu_enabled;
    rounding_mode_t csr_frm;

    // Trap
    logic trap_enter, trap_is_interrupt, mret_exec, sret_exec, interrupt_pending;
    logic [XLEN-1:0] trap_pc, trap_cause, trap_value, trap_vector, return_pc, interrupt_cause;
    logic [1:0] return_priv;

    // MDU
    logic mdu_req, mdu_ready, mdu_resp_valid, mdu_is_word;
    logic [XLEN-1:0] mdu_rs1, mdu_rs2, mdu_result;
    logic [2:0] mdu_funct3;

    // FPU
    logic fpu_req, fpu_ready, fpu_resp_valid, fpu_is_single;
    fpu_op_t fpu_op;
    logic [FLEN-1:0] fpu_rs1, fpu_rs2, fpu_rs3, fpu_result;
    logic [XLEN-1:0] fpu_int_rs1, fpu_int_result;
    rounding_mode_t fpu_rm, fpu_rm_final;
    fflags_t fpu_fflags;

    // SFENCE/FENCE
    logic sfence_valid, sfence_all, fence_i_valid, dcache_flush, dcache_flush_done;
    logic [VPN_WIDTH-1:0] sfence_vpn;
    logic [15:0] sfence_asid;

    // Counters
    logic [63:0] cycle_counter, instret_counter, time_counter;
    logic instr_retired, instr_retired_2;

    // Control logic
    assign mmu_enabled = (csr_satp[63:60] != 4'b0000) && (current_priv != PRIV_MACHINE);
    assign fpu_rm_final = (fpu_rm == RM_DYN) ? csr_frm : fpu_rm;
    assign ptw_is_for_itlb = ptw_req_itlb && !ptw_req_dtlb;
    assign ptw_vpn = ptw_req_dtlb ? ptw_vpn_dtlb : ptw_vpn_itlb;
    assign dmem_is_amo = dcache_is_amo;
    assign dmem_amo_op = dcache_amo_op;
    assign dmem_upgrade = 1'b0;
    assign snoop_resp_out = '0;
    assign debug_halted = 1'b0;

    // Branch Predictor
    branch_predictor #(.VADDR_WIDTH(VADDR_WIDTH), .BTB_ENTRIES(256), .BHT_ENTRIES(1024), .RAS_DEPTH(8), .GHR_LEN(10))
    u_bp (.clk, .rst_n, .pc(fetch_pc), .predict_valid(icache_req), .prediction(bp_prediction), .update_valid(bp_update.valid), .update(bp_update));

    // I-Cache
    icache_l1 #(.PADDR_WIDTH(PADDR_WIDTH), .LINE_SIZE(L1_LINE_SIZE), .NUM_WAYS(L1_WAYS), .NUM_SETS(L1_SETS))
    u_icache (.clk, .rst_n, .req_valid(icache_req), .req_addr(icache_addr), .req_ready(icache_ready),
              .resp_valid(icache_resp_valid), .resp_data(icache_resp_data), .resp_error(icache_resp_error),
              .invalidate_all(fence_i_valid), .invalidate_addr_valid(1'b0), .invalidate_addr('0),
              .mem_req(imem_req), .mem_addr(imem_addr), .mem_ack(imem_ack), .mem_data(imem_data), .mem_error(imem_error));

    // D-Cache
    dcache_l1 #(.PADDR_WIDTH(PADDR_WIDTH), .XLEN(XLEN), .LINE_SIZE(L1_LINE_SIZE), .NUM_WAYS(L1_WAYS), .NUM_SETS(L1_SETS))
    u_dcache (.clk, .rst_n, .req_valid(dcache_req), .req_addr(dcache_addr), .req_wdata(dcache_wdata),
              .req_wstrb(dcache_wstrb), .req_we(dcache_we), .req_is_amo(dcache_is_amo), .req_amo_op(dcache_amo_op),
              .req_ready(dcache_ready), .resp_valid(dcache_resp_valid), .resp_rdata(dcache_resp_data), .resp_error(dcache_resp_error),
              .invalidate_all(1'b0), .flush_all(dcache_flush), .flush_addr_valid(1'b0), .flush_addr('0), .flush_done(dcache_flush_done),
              .mem_req(dmem_req), .mem_we(dmem_we), .mem_addr(dmem_addr), .mem_wdata(dmem_wdata),
              .mem_ack(dmem_ack), .mem_rdata(dmem_rdata), .mem_error(dmem_error));

    // FPU IEEE 754
    fpu_ieee754 #(.FLEN(FLEN))
    u_fpu (.clk, .rst_n, .req_valid(fpu_req), .req_ready(fpu_ready), .op(fpu_op),
           .rs1(fpu_rs1), .rs2(fpu_rs2), .rs3(fpu_rs3), .int_rs1(fpu_int_rs1), .rm(fpu_rm_final), .is_single(fpu_is_single),
           .resp_valid(fpu_resp_valid), .result(fpu_result), .int_result(fpu_int_result), .fflags(fpu_fflags));

    // MDU
    mdu_rv64 #(.XLEN(XLEN))
    u_mdu (.clk, .rst_n, .req_valid(mdu_req), .req_ready(mdu_ready), .rs1_data(mdu_rs1), .rs2_data(mdu_rs2),
           .funct3(mdu_funct3), .is_word_op(mdu_is_word), .resp_valid(mdu_resp_valid), .result(mdu_result));

    // CSR Unit
    csr_unit #(.XLEN(XLEN), .HART_ID(HART_ID))
    u_csr (.clk, .rst_n, .csr_req_valid(csr_req), .csr_addr, .csr_wdata, .csr_op, .current_priv,
           .csr_rdata, .csr_access_fault(csr_fault), .timer_irq_in(timer_irq), .external_irq_in(external_irq),
           .software_irq_in(software_irq), .trap_enter, .trap_is_interrupt, .trap_pc, .trap_cause, .trap_value,
           .trap_from_priv(current_priv), .mret_execute(mret_exec), .sret_execute(sret_exec),
           .trap_vector, .return_pc, .return_priv, .interrupt_pending, .interrupt_cause,
           .csr_satp, .csr_mstatus, .mstatus_mie(), .mstatus_sie(), .mstatus_tvm(), .mstatus_tsr(),
           .mstatus_mprv(), .mstatus_mpp(), .mstatus_spp(), .cycle_count(cycle_counter), .time_count(time_counter),
           .instret_count(instret_counter), .fflags_in(fpu_fflags), .fflags_we(fpu_resp_valid), .frm_out(csr_frm));

    // ITLB (Atualizado)
    tlb_sv39 #(.TLB_ENTRIES(TLB_ENTRIES))
    u_itlb (.clk, .rst_n, .lookup_valid(itlb_req), .lookup_vpn(itlb_vpn), .lookup_asid(csr_satp[59:44]),
            .lookup_priv(current_priv), .lookup_is_store(1'b0), .lookup_is_exec(1'b1),
            .mstatus_sum(csr_mstatus[18]), .mstatus_mxr(csr_mstatus[19]),
            .lookup_hit(itlb_hit), .lookup_ppn(itlb_ppn), .lookup_page_fault(itlb_page_fault),
            .access_fault(itlb_access_fault), .need_set_a(itlb_need_a), .need_set_d(itlb_need_d), .fault_vpn(itlb_fault_vpn),
            // Conexões de Refill do PTW
            .insert_valid(ptw_resp_valid && ptw_is_for_itlb), .insert_vpn(ptw_resp_vpn), .insert_ppn(ptw_resp_ppn),
            .insert_asid(ptw_resp_asid), .insert_page_size(ptw_resp_page_size),
            .insert_r(ptw_resp_r), .insert_w(ptw_resp_w), .insert_x(ptw_resp_x), .insert_u(ptw_resp_u),
            .insert_g(ptw_resp_g), .insert_a(ptw_resp_a), .insert_d(ptw_resp_d),
            // Invalidação
            .invalidate_all(sfence_valid && sfence_all), .invalidate_by_asid(sfence_valid && !sfence_all && (sfence_vpn == '0)),
            .invalidate_by_addr(sfence_valid && !sfence_all && (sfence_asid == '0)),
            .invalidate_by_both(sfence_valid && !sfence_all && (sfence_vpn != '0) && (sfence_asid != '0)),
            .invalidate_asid(sfence_asid), .invalidate_vpn(sfence_vpn));

    // DTLB (Atualizado)
    tlb_sv39 #(.TLB_ENTRIES(TLB_ENTRIES))
    u_dtlb (.clk, .rst_n, .lookup_valid(dtlb_req), .lookup_vpn(dtlb_vpn), .lookup_asid(csr_satp[59:44]),
            .lookup_priv(current_priv), .lookup_is_store(dtlb_is_store), .lookup_is_exec(1'b0),
            .mstatus_sum(csr_mstatus[18]), .mstatus_mxr(csr_mstatus[19]),
            .lookup_hit(dtlb_hit), .lookup_ppn(dtlb_ppn), .lookup_page_fault(dtlb_page_fault),
            .access_fault(dtlb_access_fault), .need_set_a(dtlb_need_a), .need_set_d(dtlb_need_d), .fault_vpn(dtlb_fault_vpn),
            // Conexões de Refill do PTW
            .insert_valid(ptw_resp_valid && !ptw_is_for_itlb), .insert_vpn(ptw_resp_vpn), .insert_ppn(ptw_resp_ppn),
            .insert_asid(ptw_resp_asid), .insert_page_size(ptw_resp_page_size),
            .insert_r(ptw_resp_r), .insert_w(ptw_resp_w), .insert_x(ptw_resp_x), .insert_u(ptw_resp_u),
            .insert_g(ptw_resp_g), .insert_a(ptw_resp_a), .insert_d(ptw_resp_d),
            // Invalidação
            .invalidate_all(sfence_valid && sfence_all), .invalidate_by_asid(sfence_valid && !sfence_all && (sfence_vpn == '0)),
            .invalidate_by_addr(sfence_valid && !sfence_all && (sfence_asid == '0)),
            .invalidate_by_both(sfence_valid && !sfence_all && (sfence_vpn != '0) && (sfence_asid != '0)),
            .invalidate_asid(sfence_asid), .invalidate_vpn(sfence_vpn));

    // PTW (Atualizado)
    ptw_sv39 u_ptw (.clk, .rst_n, .ptw_req_valid(ptw_req_itlb || ptw_req_dtlb), .ptw_req_vpn(ptw_vpn),
                   .ptw_req_asid(csr_satp[59:44]), .ptw_req_is_store(dtlb_is_store && ptw_req_dtlb),
                   .ptw_req_is_exec(ptw_req_itlb && !ptw_req_dtlb), .ptw_req_for_itlb(ptw_is_for_itlb),
                   .ptw_req_ready(ptw_ready), .satp(csr_satp), .ptw_mem_req, .ptw_mem_addr,
                   .ptw_mem_resp_valid(ptw_mem_ack), .ptw_mem_resp_data(ptw_mem_data), .ptw_mem_resp_err(ptw_mem_error),
                   .ptw_resp_valid, .ptw_resp_vpn, .ptw_resp_ppn, .ptw_resp_asid, .ptw_resp_page_size, 
                   .ptw_resp_page_fault(ptw_page_fault), .ptw_resp_access_fault(ptw_access_fault), .ptw_resp_for_itlb,
                   .ptw_resp_r, .ptw_resp_w, .ptw_resp_x, .ptw_resp_u, .ptw_resp_g, .ptw_resp_a, .ptw_resp_d);

    // Frontend (Atualizado)
    nebula_frontend_rvc #(.XLEN(XLEN), .VADDR_WIDTH(VADDR_WIDTH), .PADDR_WIDTH(PADDR_WIDTH), .VPN_WIDTH(VPN_WIDTH), .PPN_WIDTH(PPN_WIDTH))
    u_frontend (.clk, .rst_n, .backend_stall(backend_ctrl.stall), .backend_flush(backend_ctrl.flush),
                .backend_redirect(backend_ctrl.redirect), .backend_redirect_pc(backend_ctrl.redirect_pc),
                .frontend_out(frontend_packet), .frontend_valid, .fetch_pc_out(fetch_pc), .bp_prediction,
                .icache_req, .icache_addr, .icache_ready, .icache_resp_valid, .icache_resp_data, .icache_resp_error,
                .itlb_req, .itlb_vpn, .itlb_hit, .itlb_ppn, .itlb_page_fault, .itlb_access_fault,
                .fetch_exception(frontend_exc), .fetch_exception_cause(frontend_exc_cause), .fetch_exception_value(frontend_exc_value),
                .ptw_req(ptw_req_itlb), .ptw_vpn(ptw_vpn_itlb), .ptw_ready, .ptw_resp_valid(ptw_resp_valid && ptw_is_for_itlb),
                .ptw_page_fault, .ptw_access_fault(ptw_access_fault), .mmu_enabled, .current_priv);

    // Backend (Mantido, mas agora o frontend_packet contém dados corretos)
    nebula_backend_fpu #(.XLEN(XLEN), .FLEN(FLEN), .VADDR_WIDTH(VADDR_WIDTH), .PADDR_WIDTH(PADDR_WIDTH), .VPN_WIDTH(VPN_WIDTH), .PPN_WIDTH(PPN_WIDTH))
    u_backend (.clk, .rst_n, .frontend_valid, .frontend_in(frontend_packet), .backend_ctrl, .bp_update,
               .dcache_req, .dcache_addr, .dcache_wdata, .dcache_wstrb, .dcache_we, .dcache_is_amo, .dcache_amo_op,
               .dcache_ready, .dcache_resp_valid, .dcache_resp_data, .dcache_resp_error,
               .dtlb_req, .dtlb_vpn, .dtlb_is_store, .dtlb_hit, .dtlb_ppn, .dtlb_page_fault,
               .ptw_req(ptw_req_dtlb), .ptw_ready, .ptw_resp_valid(ptw_resp_valid && !ptw_is_for_itlb), .ptw_page_fault,
               .csr_req, .csr_addr, .csr_wdata, .csr_op, .csr_rdata, .csr_fault,
               .trap_enter, .trap_is_interrupt, .trap_pc, .trap_cause, .trap_value, .mret_exec, .sret_exec,
               .trap_vector, .return_pc, .return_priv, .interrupt_pending, .interrupt_cause,
               .mdu_req, .mdu_rs1, .mdu_rs2, .mdu_funct3, .mdu_is_word, .mdu_ready, .mdu_resp_valid, .mdu_result,
               .fpu_req, .fpu_op, .fpu_rs1, .fpu_rs2, .fpu_rs3, .fpu_int_rs1, .fpu_rm, .fpu_is_single,
               .fpu_ready, .fpu_resp_valid, .fpu_result, .fpu_int_result, .fpu_fflags,
               .current_priv, .mmu_enabled, .sfence_valid, .sfence_all, .sfence_vpn, .sfence_asid,
               .fence_i_valid, .dcache_flush, .dcache_flush_done, .instr_retired, .instr_retired_2);

    // Performance Counters
    always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                cycle_counter <= '0;
                instret_counter <= '0;
                time_counter <= '0;
            end else begin
                cycle_counter <= cycle_counter + 1;
                time_counter <= time_counter + 1;
                if (instr_retired) instret_counter <= instret_counter + (instr_retired_2 ? 2 : 1);
            end
        end
endmodule
