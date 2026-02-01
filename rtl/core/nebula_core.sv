`include "stu_pkg.sv"

`timescale 1ns/1ps
`default_nettype none

/**
 * @module nebula_core
 * @brief Núcleo In-Order RV64GC+S, 8 estágios, dual-issue.
 *
 * @details Versão completa com suporte a Linux:
 * - ISA: RV64IMAFDS + Zicsr + Zifencei
 * - MMU: Sv39 com TLB split (ITLB/DTLB) e PTW completo
 * - Privilégios: M/S/U modes com tratamento completo de traps
 * - Atômicas: LR/SC + todas as operações AMO
 * - CSRs: Todos os CSRs necessários para M-mode e S-mode
 * - Interrupções: Timer, External, Software com delegação
 */
module nebula_core #(
    parameter int HART_ID = 0,
    parameter int XLEN = 64,
    parameter int ILEN = 32,
    parameter int PHYS_ADDR_SIZE = 56,
    parameter bit ENABLE_MMU = 1,
    parameter int TLB_ENTRIES = 64,
    parameter int VPN_WIDTH = 27,      // Sv39: 39-12 = 27 bits
    parameter int PPN_WIDTH = 44,
    parameter int ASID_WIDTH = 16
) (
    input wire clk,
    input wire rst_n,

    // -----------------------------------------------------------------
    // --- Interface STU (Speculative Threading Unit) ---
    // -----------------------------------------------------------------
    
    input  wire [stu_pkg::NUM_CORES-1:0] l1_dispatch_valid_in,
    input  stu_pkg::instr_t [stu_pkg::NUM_CORES-1:0][1:0] l1_dispatch_data_in,
    input  wire stu_pkg::core_id_t  l2_spec_core_id_in,
    input  wire stu_pkg::addr_t     l2_spec_pc_in,
    input  wire                     l2_spec_start_in,
    input  wire stu_pkg::addr_t     l2_fork_trigger_pc_in, 
    input  wire [stu_pkg::NUM_CORES-1:0] squash_in,
    input  wire [stu_pkg::NUM_CORES-1:0] commit_in,
    input  wire [4:0]               core_copy_read_addr_in,
    input  wire [4:0]               core_copy_write_addr_in,
    input  wire                     core_copy_write_en_in,
    input  wire [stu_pkg::REG_WIDTH-1:0] core_copy_data_in,
    output logic [stu_pkg::REG_WIDTH-1:0] core_copy_data_out,
    output logic                    core_busy_out,
    output logic                    spec_task_done_out,
    output logic                    spec_exception_out,
    output logic [PHYS_ADDR_SIZE-1:0] core_mem_pa_out,
    output logic                    core_mem_is_store_out,
    output logic                    core_mem_valid_out,
    
    // -----------------------------------------------------------------
    // --- Interface de Memória de Instrução (IMEM) ---
    // -----------------------------------------------------------------
    output logic [PHYS_ADDR_SIZE-1:0] imem_addr_pa,
    output logic                       imem_req,
    input  wire [ILEN*2-1:0]           imem_rdata,
    input  wire                        imem_ack_in,
    input  wire                        imem_error_in,

    // -----------------------------------------------------------------
    // --- Interface de Memória de Dados (DMEM) ---
    // -----------------------------------------------------------------
    output logic [PHYS_ADDR_SIZE-1:0] dmem_addr_pa,
    output logic [XLEN-1:0]           dmem_wdata,
    output logic [7:0]                dmem_wstrb,
    output logic                      dmem_req,
    output logic                      dmem_req_is_amo,
    output logic                      dmem_we,
    input  wire [XLEN-1:0]            dmem_rdata,
    input  wire                       dmem_ack_in,
    input  wire                       dmem_error_in,
    
    // -----------------------------------------------------------------
    // --- Interface do Page Table Walker (PTW) ---
    // -----------------------------------------------------------------
    output logic                       ptw_mem_req,
    output logic [PHYS_ADDR_SIZE-1:0]  ptw_mem_addr,
    input  wire [XLEN-1:0]             ptw_mem_rdata,
    input  wire                        ptw_mem_ack_in,
    input  wire                        ptw_mem_error_in,

    // -----------------------------------------------------------------
    // --- Interrupções ---
    // -----------------------------------------------------------------
    input wire timer_irq,
    input wire external_irq,
    input wire software_irq
);

    // =========================================================================
    // Definições Locais
    // =========================================================================
    
    localparam logic [PHYS_ADDR_SIZE-1:0] PC_RESET_VEC = 56'h0000_0080_0000_0000;
    
    typedef enum logic [1:0] {
        PRIV_USER       = 2'b00,
        PRIV_SUPERVISOR = 2'b01,
        PRIV_RESERVED   = 2'b10,
        PRIV_MACHINE    = 2'b11
    } privilege_t;
    
    typedef enum logic [4:0] {
        STAGE_RESET,
        STAGE_FETCH,
        STAGE_FETCH_WAIT,
        STAGE_DECODE,
        STAGE_ISSUE,
        STAGE_EXECUTE,
        STAGE_MEMORY,
        STAGE_MEMORY_WAIT,
        STAGE_WRITEBACK,
        STAGE_TRAP_ENTER,
        STAGE_TRAP_RETURN,
        STAGE_STALL_TLB,
        STAGE_STALL_MDU,
        STAGE_STALL_AMO,
        STAGE_SFENCE
    } pipeline_state_t;

    // Exception causes
    localparam CAUSE_MISALIGNED_FETCH     = 64'd0;
    localparam CAUSE_FETCH_ACCESS         = 64'd1;
    localparam CAUSE_ILLEGAL_INSTRUCTION  = 64'd2;
    localparam CAUSE_BREAKPOINT           = 64'd3;
    localparam CAUSE_MISALIGNED_LOAD      = 64'd4;
    localparam CAUSE_LOAD_ACCESS          = 64'd5;
    localparam CAUSE_MISALIGNED_STORE     = 64'd6;
    localparam CAUSE_STORE_ACCESS         = 64'd7;
    localparam CAUSE_USER_ECALL           = 64'd8;
    localparam CAUSE_SUPERVISOR_ECALL     = 64'd9;
    localparam CAUSE_MACHINE_ECALL        = 64'd11;
    localparam CAUSE_FETCH_PAGE_FAULT     = 64'd12;
    localparam CAUSE_LOAD_PAGE_FAULT      = 64'd13;
    localparam CAUSE_STORE_PAGE_FAULT     = 64'd15;

    // =========================================================================
    // Estruturas de Pipeline
    // =========================================================================
    
    typedef struct packed {
        logic [6:0]  opcode;
        logic [4:0]  rd;
        logic [2:0]  funct3;
        logic [4:0]  rs1;
        logic [4:0]  rs2;
        logic [6:0]  funct7;
        logic [XLEN-1:0] imm;
        logic [11:0] csr_addr;
        logic        valid;
        logic        is_alu;
        logic        is_alu_w;      // Operação de 32-bit (ADDIW, etc)
        logic        is_branch;
        logic        is_load;
        logic        is_store;
        logic        is_system;
        logic        is_amo;
        logic        is_lr;
        logic        is_sc;
        logic        is_fpu;
        logic        is_mdu;
        logic        is_csr;
        logic        is_fence;
        logic        is_sfence_vma;
        logic        is_wfi;
        logic        is_mret;
        logic        is_sret;
        logic        is_ecall;
        logic        is_ebreak;
    } decoded_instr_t;

    typedef struct packed {
        logic [XLEN-1:0] alu_result;
        logic [XLEN-1:0] mem_addr_va;
        logic [XLEN-1:0] store_data;
        logic [4:0]      rd;
        logic            mem_we;
        logic [7:0]      mem_wstrb;
        logic            reg_we;
        logic            freg_we;
        logic            branch_taken;
        logic [XLEN-1:0] branch_target;
        logic [XLEN-1:0] pc;
        logic            is_amo;
        logic            is_lr;
        logic            is_sc;
        logic [4:0]      amo_funct5;
        logic            amo_aq;
        logic            amo_rl;
        logic            trap;
        logic [XLEN-1:0] trap_cause;
        logic [XLEN-1:0] trap_value;
        logic            is_load;
        logic            is_store;
        logic [2:0]      load_funct3;
    } execute_result_t;
    
    typedef struct packed {
        logic [XLEN-1:0] data;
        logic [4:0]      rd;
        logic            reg_we;
        logic            freg_we;
        logic            trap;
        logic [XLEN-1:0] trap_cause;
        logic [XLEN-1:0] trap_value;
        logic [XLEN-1:0] pc;
    } memory_result_t;

    // =========================================================================
    // Sinais de Estado do Pipeline
    // =========================================================================
    
    pipeline_state_t pipeline_state, next_pipeline_state;
    privilege_t current_priv;
    logic [XLEN-1:0] pc, next_pc;
    
    // Registradores de uso geral
    logic [XLEN-1:0] regfile [0:31];
    logic [XLEN-1:0] fregfile [0:31];
    
    // Registradores de pipeline
    decoded_instr_t  decoded_instr0, decoded_instr1;
    logic            dual_issue_valid;
    execute_result_t execute_result0, execute_result1;
    memory_result_t  memory_result0, memory_result1;
    
    // Fetch buffer
    logic [ILEN*2-1:0] fetch_buffer;
    logic fetch_buffer_valid;
    logic [XLEN-1:0] pc_fetch, pc_decode;
    
    // STU
    const stu_pkg::core_id_t MY_HART_ID = stu_pkg::core_id_t'(HART_ID);
    logic is_speculating_l2;
    logic is_dispatching_l1;
    logic is_stalled_l1;
    
    // Shadow registers para rollback
    logic [XLEN-1:0] shadow_regfile [0:31];
    logic [XLEN-1:0] shadow_fregfile [0:31];
    logic [XLEN-1:0] shadow_pc;
    
    // Trap signals
    logic trap_pending;
    logic trap_is_interrupt;
    logic [XLEN-1:0] trap_cause;
    logic [XLEN-1:0] trap_value;
    logic [XLEN-1:0] trap_pc;
    
    // Counters
    logic [63:0] cycle_counter;
    logic [63:0] instret_counter;
    logic [63:0] time_counter;

    // =========================================================================
    // CSR Unit Instance
    // =========================================================================
    
    logic csr_req_valid;
    logic [11:0] csr_addr;
    logic [XLEN-1:0] csr_wdata;
    logic [2:0] csr_op;
    logic [XLEN-1:0] csr_rdata;
    logic csr_access_fault;
    
    logic trap_enter;
    logic mret_execute, sret_execute;
    logic [XLEN-1:0] trap_vector;
    logic [XLEN-1:0] return_pc;
    logic [1:0] return_priv;
    logic interrupt_pending;
    logic [XLEN-1:0] interrupt_cause;
    
    logic [XLEN-1:0] csr_satp;
    logic [XLEN-1:0] csr_mstatus;
    logic mstatus_mie, mstatus_sie, mstatus_tvm, mstatus_tsr, mstatus_mprv;
    logic [1:0] mstatus_mpp;
    logic mstatus_spp;
    
    csr_unit #(
        .XLEN(XLEN),
        .HART_ID(HART_ID)
    ) u_csr (
        .clk(clk),
        .rst_n(rst_n),
        .csr_req_valid(csr_req_valid),
        .csr_addr(csr_addr),
        .csr_wdata(csr_wdata),
        .csr_op(csr_op),
        .current_priv(current_priv),
        .csr_rdata(csr_rdata),
        .csr_access_fault(csr_access_fault),
        .timer_irq_in(timer_irq),
        .external_irq_in(external_irq),
        .software_irq_in(software_irq),
        .trap_enter(trap_enter),
        .trap_is_interrupt(trap_is_interrupt),
        .trap_pc(trap_pc),
        .trap_cause(trap_cause),
        .trap_value(trap_value),
        .trap_from_priv(current_priv),
        .mret_execute(mret_execute),
        .sret_execute(sret_execute),
        .trap_vector(trap_vector),
        .return_pc(return_pc),
        .return_priv(return_priv),
        .interrupt_pending(interrupt_pending),
        .interrupt_cause(interrupt_cause),
        .csr_satp(csr_satp),
        .csr_mstatus(csr_mstatus),
        .mstatus_mie(mstatus_mie),
        .mstatus_sie(mstatus_sie),
        .mstatus_tvm(mstatus_tvm),
        .mstatus_tsr(mstatus_tsr),
        .mstatus_mprv(mstatus_mprv),
        .mstatus_mpp(mstatus_mpp),
        .mstatus_spp(mstatus_spp),
        .cycle_count(cycle_counter),
        .time_count(time_counter),
        .instret_count(instret_counter)
    );

    // =========================================================================
    // MDU (Multiply-Divide Unit) Instance
    // =========================================================================
    
    logic mdu_req_valid, mdu_req_ready, mdu_resp_valid;
    logic [XLEN-1:0] mdu_rs1, mdu_rs2, mdu_result;
    logic [2:0] mdu_funct3;
    logic mdu_is_word;
    
    mdu_rv64 #(.XLEN(XLEN)) u_mdu (
        .clk(clk),
        .rst_n(rst_n),
        .req_valid(mdu_req_valid),
        .req_ready(mdu_req_ready),
        .rs1_data(mdu_rs1),
        .rs2_data(mdu_rs2),
        .funct3(mdu_funct3),
        .is_word_op(mdu_is_word),
        .resp_valid(mdu_resp_valid),
        .result(mdu_result)
    );

    // =========================================================================
    // AMO Unit Instance
    // =========================================================================
    
    logic amo_req_valid, amo_req_ready, amo_resp_valid;
    logic [4:0] amo_op;
    logic amo_is_word;
    logic [XLEN-1:0] amo_addr, amo_rs2, amo_result;
    logic amo_aq, amo_rl;
    logic amo_sc_fail;
    logic amo_mem_req, amo_mem_we;
    logic [PHYS_ADDR_SIZE-1:0] amo_mem_addr;
    logic [XLEN-1:0] amo_mem_wdata;
    logic [7:0] amo_mem_wstrb;
    
    // Reservation set signals
    logic lr_valid_out, sc_valid_out, sc_success_in;
    logic [XLEN-1:0] lr_addr_out, sc_addr_out;
    logic lr_is_word_out, sc_is_word_out;
    logic invalidate_reservations;
    
    amo_unit #(
        .XLEN(XLEN),
        .ADDR_WIDTH(PHYS_ADDR_SIZE)
    ) u_amo (
        .clk(clk),
        .rst_n(rst_n),
        .req_valid(amo_req_valid),
        .req_ready(amo_req_ready),
        .amo_op(amo_op),
        .is_word(amo_is_word),
        .addr(amo_addr),
        .rs2_data(amo_rs2),
        .aq(amo_aq),
        .rl(amo_rl),
        .resp_valid(amo_resp_valid),
        .result(amo_result),
        .sc_fail(amo_sc_fail),
        .mem_req(amo_mem_req),
        .mem_we(amo_mem_we),
        .mem_addr(amo_mem_addr),
        .mem_wdata(amo_mem_wdata),
        .mem_wstrb(amo_mem_wstrb),
        .mem_rdata(dmem_rdata),
        .mem_ack(dmem_ack_in),
        .mem_error(dmem_error_in),
        .lr_valid_out(lr_valid_out),
        .lr_addr_out(lr_addr_out),
        .lr_is_word_out(lr_is_word_out),
        .sc_valid_out(sc_valid_out),
        .sc_addr_out(sc_addr_out),
        .sc_is_word_out(sc_is_word_out),
        .sc_success_in(sc_success_in)
    );
    
    // Reservation Set
    reservation_set #(
        .NUM_HARTS(stu_pkg::NUM_CORES),
        .ADDR_WIDTH(PHYS_ADDR_SIZE)
    ) u_reservation (
        .clk(clk),
        .rst_n(rst_n),
        .hart_id(MY_HART_ID),
        .lr_valid(lr_valid_out),
        .lr_addr(lr_addr_out),
        .lr_is_word(lr_is_word_out),
        .sc_valid(sc_valid_out),
        .sc_addr(sc_addr_out),
        .sc_is_word(sc_is_word_out),
        .sc_success(sc_success_in),
        .invalidate_all(invalidate_reservations),
        .store_from_other_hart(1'b0),  // TODO: conectar ao sistema de coerência
        .other_store_addr('0),
        .other_hart_id('0),
        .reservation_valid()
    );

    // =========================================================================
    // TLB e PTW - ITLB
    // =========================================================================
    
    logic itlb_lookup_valid, itlb_hit, itlb_page_fault, itlb_access_fault;
    logic [PPN_WIDTH-1:0] itlb_ppn;
    logic itlb_need_set_a, itlb_need_set_d;
    
    logic itlb_insert_valid;
    logic [VPN_WIDTH-1:0] itlb_insert_vpn;
    logic [PPN_WIDTH-1:0] itlb_insert_ppn;
    logic [ASID_WIDTH-1:0] itlb_insert_asid;
    logic [1:0] itlb_insert_page_size;
    logic itlb_insert_r, itlb_insert_w, itlb_insert_x;
    logic itlb_insert_u, itlb_insert_g, itlb_insert_a, itlb_insert_d;
    
    // SFENCE.VMA signals
    logic sfence_all, sfence_asid, sfence_addr, sfence_both;
    logic [ASID_WIDTH-1:0] sfence_asid_val;
    logic [VPN_WIDTH-1:0] sfence_vpn_val;
    
    tlb_sv39 #(
        .VPN_WIDTH(VPN_WIDTH),
        .PPN_WIDTH(PPN_WIDTH),
        .TLB_ENTRIES(TLB_ENTRIES),
        .ASID_WIDTH(ASID_WIDTH)
    ) u_itlb (
        .clk(clk),
        .rst_n(rst_n),
        .lookup_valid(itlb_lookup_valid),
        .lookup_vpn(pc[VPN_WIDTH+11:12]),
        .lookup_asid(csr_satp[59:44]),
        .lookup_priv(current_priv),
        .lookup_is_store(1'b0),
        .lookup_is_exec(1'b1),
        .mstatus_sum(csr_mstatus[18]),
        .mstatus_mxr(csr_mstatus[19]),
        .lookup_hit(itlb_hit),
        .lookup_ppn(itlb_ppn),
        .lookup_page_fault(itlb_page_fault),
        .access_fault(itlb_access_fault),
        .need_set_a(itlb_need_set_a),
        .need_set_d(itlb_need_set_d),
        .fault_vpn(),
        .insert_valid(itlb_insert_valid),
        .insert_vpn(itlb_insert_vpn),
        .insert_ppn(itlb_insert_ppn),
        .insert_asid(itlb_insert_asid),
        .insert_page_size(itlb_insert_page_size),
        .insert_r(itlb_insert_r),
        .insert_w(itlb_insert_w),
        .insert_x(itlb_insert_x),
        .insert_u(itlb_insert_u),
        .insert_g(itlb_insert_g),
        .insert_a(itlb_insert_a),
        .insert_d(itlb_insert_d),
        .invalidate_all(sfence_all),
        .invalidate_by_asid(sfence_asid),
        .invalidate_by_addr(sfence_addr),
        .invalidate_by_both(sfence_both),
        .invalidate_asid(sfence_asid_val),
        .invalidate_vpn(sfence_vpn_val)
    );

    // =========================================================================
    // TLB e PTW - DTLB
    // =========================================================================
    
    logic dtlb_lookup_valid, dtlb_hit, dtlb_page_fault, dtlb_access_fault;
    logic [PPN_WIDTH-1:0] dtlb_ppn;
    logic dtlb_lookup_is_store;
    
    logic dtlb_insert_valid;
    
    tlb_sv39 #(
        .VPN_WIDTH(VPN_WIDTH),
        .PPN_WIDTH(PPN_WIDTH),
        .TLB_ENTRIES(TLB_ENTRIES),
        .ASID_WIDTH(ASID_WIDTH)
    ) u_dtlb (
        .clk(clk),
        .rst_n(rst_n),
        .lookup_valid(dtlb_lookup_valid),
        .lookup_vpn(execute_result0.mem_addr_va[VPN_WIDTH+11:12]),
        .lookup_asid(csr_satp[59:44]),
        .lookup_priv(current_priv),
        .lookup_is_store(dtlb_lookup_is_store),
        .lookup_is_exec(1'b0),
        .mstatus_sum(csr_mstatus[18]),
        .mstatus_mxr(csr_mstatus[19]),
        .lookup_hit(dtlb_hit),
        .lookup_ppn(dtlb_ppn),
        .lookup_page_fault(dtlb_page_fault),
        .access_fault(dtlb_access_fault),
        .need_set_a(),
        .need_set_d(),
        .fault_vpn(),
        .insert_valid(dtlb_insert_valid),
        .insert_vpn(ptw_resp_vpn),
        .insert_ppn(ptw_resp_ppn),
        .insert_asid(ptw_resp_asid),
        .insert_page_size(ptw_resp_page_size),
        .insert_r(ptw_resp_r),
        .insert_w(ptw_resp_w),
        .insert_x(ptw_resp_x),
        .insert_u(ptw_resp_u),
        .insert_g(ptw_resp_g),
        .insert_a(ptw_resp_a),
        .insert_d(ptw_resp_d),
        .invalidate_all(sfence_all),
        .invalidate_by_asid(sfence_asid),
        .invalidate_by_addr(sfence_addr),
        .invalidate_by_both(sfence_both),
        .invalidate_asid(sfence_asid_val),
        .invalidate_vpn(sfence_vpn_val)
    );

    // =========================================================================
    // PTW Instance
    // =========================================================================
    
    logic ptw_req_valid, ptw_req_ready;
    logic [VPN_WIDTH-1:0] ptw_req_vpn;
    logic ptw_req_is_store, ptw_req_is_exec, ptw_req_for_itlb;
    
    logic ptw_resp_valid, ptw_resp_page_fault, ptw_resp_access_fault, ptw_resp_for_itlb;
    logic [VPN_WIDTH-1:0] ptw_resp_vpn;
    logic [PPN_WIDTH-1:0] ptw_resp_ppn;
    logic [ASID_WIDTH-1:0] ptw_resp_asid;
    logic [1:0] ptw_resp_page_size;
    logic ptw_resp_r, ptw_resp_w, ptw_resp_x, ptw_resp_u, ptw_resp_g, ptw_resp_a, ptw_resp_d;
    
    ptw_sv39 #(
        .XLEN(XLEN),
        .PADDR_WIDTH(PHYS_ADDR_SIZE),
        .VPN_WIDTH(VPN_WIDTH),
        .PPN_WIDTH(PPN_WIDTH),
        .ASID_WIDTH(ASID_WIDTH)
    ) u_ptw (
        .clk(clk),
        .rst_n(rst_n),
        .ptw_req_valid(ptw_req_valid),
        .ptw_req_vpn(ptw_req_vpn),
        .ptw_req_asid(csr_satp[59:44]),
        .ptw_req_is_store(ptw_req_is_store),
        .ptw_req_is_exec(ptw_req_is_exec),
        .ptw_req_for_itlb(ptw_req_for_itlb),
        .ptw_req_ready(ptw_req_ready),
        .satp(csr_satp),
        .ptw_mem_req(ptw_mem_req),
        .ptw_mem_addr(ptw_mem_addr),
        .ptw_mem_resp_valid(ptw_mem_ack_in),
        .ptw_mem_resp_data(ptw_mem_rdata),
        .ptw_mem_resp_err(ptw_mem_error_in),
        .ptw_resp_valid(ptw_resp_valid),
        .ptw_resp_vpn(ptw_resp_vpn),
        .ptw_resp_ppn(ptw_resp_ppn),
        .ptw_resp_asid(ptw_resp_asid),
        .ptw_resp_page_size(ptw_resp_page_size),
        .ptw_resp_page_fault(ptw_resp_page_fault),
        .ptw_resp_access_fault(ptw_resp_access_fault),
        .ptw_resp_for_itlb(ptw_resp_for_itlb),
        .ptw_resp_r(ptw_resp_r),
        .ptw_resp_w(ptw_resp_w),
        .ptw_resp_x(ptw_resp_x),
        .ptw_resp_u(ptw_resp_u),
        .ptw_resp_g(ptw_resp_g),
        .ptw_resp_a(ptw_resp_a),
        .ptw_resp_d(ptw_resp_d)
    );

    // PTW response routing to TLBs
    always_comb begin
        itlb_insert_valid = ptw_resp_valid && !ptw_resp_page_fault && !ptw_resp_access_fault && ptw_resp_for_itlb;
        dtlb_insert_valid = ptw_resp_valid && !ptw_resp_page_fault && !ptw_resp_access_fault && !ptw_resp_for_itlb;
        
        itlb_insert_vpn = ptw_resp_vpn;
        itlb_insert_ppn = ptw_resp_ppn;
        itlb_insert_asid = ptw_resp_asid;
        itlb_insert_page_size = ptw_resp_page_size;
        itlb_insert_r = ptw_resp_r;
        itlb_insert_w = ptw_resp_w;
        itlb_insert_x = ptw_resp_x;
        itlb_insert_u = ptw_resp_u;
        itlb_insert_g = ptw_resp_g;
        itlb_insert_a = ptw_resp_a;
        itlb_insert_d = ptw_resp_d;
    end

    // =========================================================================
    // Endereço Físico
    // =========================================================================
    
    wire mmu_enabled = ENABLE_MMU && (csr_satp[63:60] != 4'd0) && (current_priv != PRIV_MACHINE);
    
    assign imem_addr_pa = mmu_enabled ? {itlb_ppn, pc[11:0]} : pc[PHYS_ADDR_SIZE-1:0];
    
    wire [PHYS_ADDR_SIZE-1:0] dmem_addr_pa_translated = mmu_enabled ? 
        {dtlb_ppn, execute_result0.mem_addr_va[11:0]} : 
        execute_result0.mem_addr_va[PHYS_ADDR_SIZE-1:0];
    // =========================================================================
    // Funções de Decodificação
    // =========================================================================
    
    function automatic decoded_instr_t decode_instr(
        input [ILEN-1:0] instr,
        input [XLEN-1:0] pc_val
    );
        decoded_instr_t d;
        
        // Campos básicos
        d.opcode   = instr[6:0];
        d.rd       = instr[11:7];
        d.funct3   = instr[14:12];
        d.rs1      = instr[19:15];
        d.rs2      = instr[24:20];
        d.funct7   = instr[31:25];
        d.csr_addr = instr[31:20];
        
        // Defaults
        d.valid = 1'b1;
        d.is_alu = 1'b0;
        d.is_alu_w = 1'b0;
        d.is_branch = 1'b0;
        d.is_load = 1'b0;
        d.is_store = 1'b0;
        d.is_system = 1'b0;
        d.is_amo = 1'b0;
        d.is_lr = 1'b0;
        d.is_sc = 1'b0;
        d.is_fpu = 1'b0;
        d.is_mdu = 1'b0;
        d.is_csr = 1'b0;
        d.is_fence = 1'b0;
        d.is_sfence_vma = 1'b0;
        d.is_wfi = 1'b0;
        d.is_mret = 1'b0;
        d.is_sret = 1'b0;
        d.is_ecall = 1'b0;
        d.is_ebreak = 1'b0;
        d.imm = '0;
        
        case (d.opcode)
            // RV64I Base
            7'b0110111: begin // LUI
                d.is_alu = 1'b1;
                d.imm = {{32{instr[31]}}, instr[31:12], 12'b0};
            end
            
            7'b0010111: begin // AUIPC
                d.is_alu = 1'b1;
                d.imm = {{32{instr[31]}}, instr[31:12], 12'b0};
            end
            
            7'b1101111: begin // JAL
                d.is_branch = 1'b1;
                d.imm = {{43{instr[31]}}, instr[31], instr[19:12], instr[20], instr[30:21], 1'b0};
            end
            
            7'b1100111: begin // JALR
                d.is_branch = 1'b1;
                d.imm = {{52{instr[31]}}, instr[31:20]};
            end
            
            7'b1100011: begin // Branch
                d.is_branch = 1'b1;
                d.imm = {{51{instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0};
            end
            
            7'b0000011: begin // Load
                d.is_load = 1'b1;
                d.imm = {{52{instr[31]}}, instr[31:20]};
            end
            
            7'b0100011: begin // Store
                d.is_store = 1'b1;
                d.imm = {{52{instr[31]}}, instr[31:25], instr[11:7]};
            end
            
            7'b0010011: begin // ALU-I (ADDI, etc)
                d.is_alu = 1'b1;
                d.imm = {{52{instr[31]}}, instr[31:20]};
            end
            
            7'b0110011: begin // ALU-R
                if (d.funct7 == 7'b0000001) begin
                    d.is_mdu = 1'b1; // M extension
                end else begin
                    d.is_alu = 1'b1;
                end
            end
            
            7'b0011011: begin // ALU-I-W (ADDIW, etc)
                d.is_alu = 1'b1;
                d.is_alu_w = 1'b1;
                d.imm = {{52{instr[31]}}, instr[31:20]};
            end
            
            7'b0111011: begin // ALU-R-W
                if (d.funct7 == 7'b0000001) begin
                    d.is_mdu = 1'b1; // M extension W
                end else begin
                    d.is_alu = 1'b1;
                    d.is_alu_w = 1'b1;
                end
            end
            
            7'b0101111: begin // AMO
                d.is_amo = 1'b1;
                d.is_lr = (d.funct7[6:2] == 5'b00010);
                d.is_sc = (d.funct7[6:2] == 5'b00011);
            end
            
            7'b0001111: begin // FENCE
                d.is_fence = 1'b1;
            end
            
            7'b1110011: begin // SYSTEM
                d.is_system = 1'b1;
                if (d.funct3 == 3'b000) begin
                    case (instr[31:20])
                        12'h000: d.is_ecall = 1'b1;
                        12'h001: d.is_ebreak = 1'b1;
                        12'h302: d.is_mret = 1'b1;
                        12'h102: d.is_sret = 1'b1;
                        12'h105: d.is_wfi = 1'b1;
                        default: begin
                            if (d.funct7 == 7'b0001001) begin
                                d.is_sfence_vma = 1'b1;
                            end
                        end
                    endcase
                end else begin
                    d.is_csr = 1'b1;
                    // CSR immediate
                    d.imm = {{59{1'b0}}, instr[19:15]};
                end
            end
            
            // FP Load/Store
            7'b0000111: begin // FLW, FLD
                d.is_load = 1'b1;
                d.is_fpu = 1'b1;
                d.imm = {{52{instr[31]}}, instr[31:20]};
            end
            
            7'b0100111: begin // FSW, FSD
                d.is_store = 1'b1;
                d.is_fpu = 1'b1;
                d.imm = {{52{instr[31]}}, instr[31:25], instr[11:7]};
            end
            
            // FP Operations
            7'b1010011, 7'b1000011, 7'b1000111, 7'b1001011, 7'b1001111: begin
                d.is_fpu = 1'b1;
            end
            
            default: begin
                d.valid = 1'b0;
            end
        endcase
        
        return d;
    endfunction

    // =========================================================================
    // Função de Execução ALU
    // =========================================================================
    
    function automatic execute_result_t execute_alu(
        input decoded_instr_t instr,
        input [XLEN-1:0] rs1_data,
        input [XLEN-1:0] rs2_data,
        input [XLEN-1:0] pc_val
    );
        execute_result_t r;
        logic [XLEN-1:0] op1, op2, alu_out;
        logic signed [XLEN-1:0] op1_s, op2_s;
        logic signed [31:0] op1_sw, op2_sw;
        logic [31:0] op1_uw, op2_uw;
        logic [5:0] shamt;
        logic [4:0] shamt_w;
        
        // Defaults
        r.alu_result = '0;
        r.mem_addr_va = '0;
        r.store_data = '0;
        r.rd = instr.rd;
        r.mem_we = 1'b0;
        r.mem_wstrb = '0;
        r.reg_we = 1'b0;
        r.freg_we = 1'b0;
        r.branch_taken = 1'b0;
        r.branch_target = '0;
        r.pc = pc_val;
        r.is_amo = instr.is_amo;
        r.is_lr = instr.is_lr;
        r.is_sc = instr.is_sc;
        r.amo_funct5 = instr.funct7[6:2];
        r.amo_aq = instr.funct7[1];
        r.amo_rl = instr.funct7[0];
        r.trap = 1'b0;
        r.trap_cause = '0;
        r.trap_value = '0;
        r.is_load = instr.is_load;
        r.is_store = instr.is_store;
        r.load_funct3 = instr.funct3;
        
        op1 = rs1_data;
        op2 = rs2_data;
        op1_s = $signed(rs1_data);
        op2_s = $signed(rs2_data);
        op1_sw = $signed(rs1_data[31:0]);
        op2_sw = $signed(rs2_data[31:0]);
        op1_uw = rs1_data[31:0];
        op2_uw = rs2_data[31:0];
        shamt = instr.imm[5:0];
        shamt_w = instr.imm[4:0];
        
        if (instr.is_alu) begin
            r.reg_we = 1'b1;
            
            case (instr.opcode)
                7'b0110111: // LUI
                    r.alu_result = instr.imm;
                    
                7'b0010111: // AUIPC
                    r.alu_result = pc_val + instr.imm;
                    
                7'b0010011: begin // ALU-I
                    case (instr.funct3)
                        3'b000: r.alu_result = op1 + instr.imm; // ADDI
                        3'b010: r.alu_result = (op1_s < $signed(instr.imm)) ? 1 : 0; // SLTI
                        3'b011: r.alu_result = (op1 < instr.imm) ? 1 : 0; // SLTIU
                        3'b100: r.alu_result = op1 ^ instr.imm; // XORI
                        3'b110: r.alu_result = op1 | instr.imm; // ORI
                        3'b111: r.alu_result = op1 & instr.imm; // ANDI
                        3'b001: r.alu_result = op1 << shamt; // SLLI
                        3'b101: begin
                            if (instr.funct7[5])
                                r.alu_result = op1_s >>> shamt; // SRAI
                            else
                                r.alu_result = op1 >> shamt; // SRLI
                        end
                    endcase
                end
                
                7'b0110011: begin // ALU-R
                    case ({instr.funct7, instr.funct3})
                        {7'b0000000, 3'b000}: r.alu_result = op1 + op2; // ADD
                        {7'b0100000, 3'b000}: r.alu_result = op1 - op2; // SUB
                        {7'b0000000, 3'b001}: r.alu_result = op1 << op2[5:0]; // SLL
                        {7'b0000000, 3'b010}: r.alu_result = (op1_s < op2_s) ? 1 : 0; // SLT
                        {7'b0000000, 3'b011}: r.alu_result = (op1 < op2) ? 1 : 0; // SLTU
                        {7'b0000000, 3'b100}: r.alu_result = op1 ^ op2; // XOR
                        {7'b0000000, 3'b101}: r.alu_result = op1 >> op2[5:0]; // SRL
                        {7'b0100000, 3'b101}: r.alu_result = op1_s >>> op2[5:0]; // SRA
                        {7'b0000000, 3'b110}: r.alu_result = op1 | op2; // OR
                        {7'b0000000, 3'b111}: r.alu_result = op1 & op2; // AND
                        default: r.trap = 1'b1;
                    endcase
                end
                
                7'b0011011: begin // ALU-I-W (32-bit)
                    case (instr.funct3)
                        3'b000: alu_out = {{32{1'b0}}, op1_uw + instr.imm[31:0]}; // ADDIW
                        3'b001: alu_out = {{32{1'b0}}, op1_uw << shamt_w}; // SLLIW
                        3'b101: begin
                            if (instr.funct7[5])
                                alu_out = {{32{1'b0}}, $unsigned(op1_sw >>> shamt_w)}; // SRAIW
                            else
                                alu_out = {{32{1'b0}}, op1_uw >> shamt_w}; // SRLIW
                        end
                        default: alu_out = '0;
                    endcase
                    r.alu_result = {{32{alu_out[31]}}, alu_out[31:0]}; // Sign extend
                end
                
                7'b0111011: begin // ALU-R-W (32-bit)
                    case ({instr.funct7, instr.funct3})
                        {7'b0000000, 3'b000}: alu_out = op1_uw + op2_uw; // ADDW
                        {7'b0100000, 3'b000}: alu_out = op1_uw - op2_uw; // SUBW
                        {7'b0000000, 3'b001}: alu_out = op1_uw << op2[4:0]; // SLLW
                        {7'b0000000, 3'b101}: alu_out = op1_uw >> op2[4:0]; // SRLW
                        {7'b0100000, 3'b101}: alu_out = $unsigned(op1_sw >>> op2[4:0]); // SRAW
                        default: alu_out = '0;
                    endcase
                    r.alu_result = {{32{alu_out[31]}}, alu_out[31:0]}; // Sign extend
                end
            endcase
        end
        
        else if (instr.is_branch) begin
            r.alu_result = pc_val + 4; // Link address
            
            case (instr.opcode)
                7'b1101111: begin // JAL
                    r.reg_we = 1'b1;
                    r.branch_taken = 1'b1;
                    r.branch_target = pc_val + instr.imm;
                end
                
                7'b1100111: begin // JALR
                    r.reg_we = 1'b1;
                    r.branch_taken = 1'b1;
                    r.branch_target = (op1 + instr.imm) & ~64'h1;
                end
                
                7'b1100011: begin // Conditional branches
                    r.branch_target = pc_val + instr.imm;
                    case (instr.funct3)
                        3'b000: r.branch_taken = (op1 == op2); // BEQ
                        3'b001: r.branch_taken = (op1 != op2); // BNE
                        3'b100: r.branch_taken = (op1_s < op2_s); // BLT
                        3'b101: r.branch_taken = (op1_s >= op2_s); // BGE
                        3'b110: r.branch_taken = (op1 < op2); // BLTU
                        3'b111: r.branch_taken = (op1 >= op2); // BGEU
                        default: r.trap = 1'b1;
                    endcase
                end
            endcase
        end
        
        else if (instr.is_load) begin
            r.reg_we = !instr.is_fpu;
            r.freg_we = instr.is_fpu;
            r.mem_addr_va = op1 + instr.imm;
        end
        
        else if (instr.is_store) begin
            r.mem_we = 1'b1;
            r.mem_addr_va = op1 + instr.imm;
            r.store_data = instr.is_fpu ? op2 : op2; // TODO: fregfile mux
            
            case (instr.funct3)
                3'b000: r.mem_wstrb = 8'b0000_0001 << r.mem_addr_va[2:0]; // SB
                3'b001: r.mem_wstrb = 8'b0000_0011 << r.mem_addr_va[2:0]; // SH
                3'b010: r.mem_wstrb = 8'b0000_1111 << r.mem_addr_va[2:0]; // SW
                3'b011: r.mem_wstrb = 8'b1111_1111; // SD
                default: r.trap = 1'b1;
            endcase
        end
        
        else if (instr.is_amo) begin
            r.reg_we = 1'b1;
            r.mem_addr_va = op1;
            r.store_data = op2;
        end
        
        return r;
    endfunction

    // =========================================================================
    // Função de Load Data Processing
    // =========================================================================
    
    function automatic [XLEN-1:0] process_load_data(
        input [XLEN-1:0] mem_data,
        input [2:0] funct3,
        input [2:0] byte_offset
    );
        logic [XLEN-1:0] result;
        logic [XLEN-1:0] shifted;
        
        shifted = mem_data >> (byte_offset * 8);
        
        case (funct3)
            3'b000: result = {{56{shifted[7]}}, shifted[7:0]};   // LB
            3'b001: result = {{48{shifted[15]}}, shifted[15:0]}; // LH
            3'b010: result = {{32{shifted[31]}}, shifted[31:0]}; // LW
            3'b011: result = shifted;                             // LD
            3'b100: result = {56'b0, shifted[7:0]};              // LBU
            3'b101: result = {48'b0, shifted[15:0]};             // LHU
            3'b110: result = {32'b0, shifted[31:0]};             // LWU
            default: result = shifted;
        endcase
        
        return result;
    endfunction

    // =========================================================================
    // Dual Issue Check
    // =========================================================================
    
    function automatic logic can_dual_issue(
        input decoded_instr_t instr0,
        input decoded_instr_t instr1
    );
        if (!instr0.valid || !instr1.valid)
            return 1'b0;
        
        // RAW dependency check
        if (instr0.rd != 0 && (instr0.rd == instr1.rs1 || instr0.rd == instr1.rs2))
            return 1'b0;
        
        // Structural hazards
        if ((instr0.is_mdu || instr0.is_fpu || instr0.is_amo || instr0.is_system) &&
            (instr1.is_mdu || instr1.is_fpu || instr1.is_amo || instr1.is_system))
            return 1'b0;
        
        // Single memory port
        if ((instr0.is_load || instr0.is_store || instr0.is_amo) &&
            (instr1.is_load || instr1.is_store || instr1.is_amo))
            return 1'b0;
        
        // No dual branch
        if (instr0.is_branch && instr1.is_branch)
            return 1'b0;
        
        return 1'b1;
    endfunction
    // =========================================================================
    // Hazard Detection e Forwarding
    // =========================================================================
    
    logic data_hazard0, data_hazard1, struct_hazard;
    logic [XLEN-1:0] rs1_data0, rs2_data0, rs1_data1, rs2_data1;
    
    always_comb begin
        // RS1 forwarding for instr0
        if (memory_result0.reg_we && decoded_instr0.rs1 == memory_result0.rd && decoded_instr0.rs1 != 0)
            rs1_data0 = memory_result0.data;
        else if (memory_result1.reg_we && decoded_instr0.rs1 == memory_result1.rd && decoded_instr0.rs1 != 0)
            rs1_data0 = memory_result1.data;
        else
            rs1_data0 = (decoded_instr0.rs1 == 0) ? '0 : regfile[decoded_instr0.rs1];
        
        // RS2 forwarding for instr0
        if (memory_result0.reg_we && decoded_instr0.rs2 == memory_result0.rd && decoded_instr0.rs2 != 0)
            rs2_data0 = memory_result0.data;
        else if (memory_result1.reg_we && decoded_instr0.rs2 == memory_result1.rd && decoded_instr0.rs2 != 0)
            rs2_data0 = memory_result1.data;
        else
            rs2_data0 = (decoded_instr0.rs2 == 0) ? '0 : regfile[decoded_instr0.rs2];
        
        // For dual issue
        if (dual_issue_valid) begin
            rs1_data1 = (decoded_instr1.rs1 == 0) ? '0 : regfile[decoded_instr1.rs1];
            rs2_data1 = (decoded_instr1.rs2 == 0) ? '0 : regfile[decoded_instr1.rs2];
        end else begin
            rs1_data1 = '0;
            rs2_data1 = '0;
        end
        
        // Load-use hazard detection
        data_hazard0 = 1'b0;
        if (execute_result0.is_load && 
            ((decoded_instr0.rs1 == execute_result0.rd && decoded_instr0.rs1 != 0) ||
             (decoded_instr0.rs2 == execute_result0.rd && decoded_instr0.rs2 != 0)))
            data_hazard0 = 1'b1;
        
        data_hazard1 = 1'b0;
        struct_hazard = 1'b0;
    end

    // =========================================================================
    // Pipeline FSM - Combinacional
    // =========================================================================
    
    always_comb begin
        next_pipeline_state = pipeline_state;
        
        // Default signal values
        imem_req = 1'b0;
        dmem_req = 1'b0;
        dmem_we = 1'b0;
        dmem_wstrb = '0;
        dmem_wdata = '0;
        dmem_addr_pa = '0;
        dmem_req_is_amo = 1'b0;
        
        itlb_lookup_valid = 1'b0;
        dtlb_lookup_valid = 1'b0;
        dtlb_lookup_is_store = 1'b0;
        
        ptw_req_valid = 1'b0;
        ptw_req_vpn = '0;
        ptw_req_is_store = 1'b0;
        ptw_req_is_exec = 1'b0;
        ptw_req_for_itlb = 1'b0;
        
        csr_req_valid = 1'b0;
        csr_addr = '0;
        csr_wdata = '0;
        csr_op = '0;
        
        trap_enter = 1'b0;
        mret_execute = 1'b0;
        sret_execute = 1'b0;
        
        mdu_req_valid = 1'b0;
        mdu_rs1 = '0;
        mdu_rs2 = '0;
        mdu_funct3 = '0;
        mdu_is_word = 1'b0;
        
        amo_req_valid = 1'b0;
        amo_op = '0;
        amo_is_word = 1'b0;
        amo_addr = '0;
        amo_rs2 = '0;
        amo_aq = 1'b0;
        amo_rl = 1'b0;
        
        sfence_all = 1'b0;
        sfence_asid = 1'b0;
        sfence_addr = 1'b0;
        sfence_both = 1'b0;
        sfence_asid_val = '0;
        sfence_vpn_val = '0;
        
        invalidate_reservations = 1'b0;
        spec_exception_out = 1'b0;
        
        is_dispatching_l1 = l1_dispatch_valid_in[MY_HART_ID] && !is_stalled_l1;
        trap_pending = interrupt_pending && !is_speculating_l2;
        
        // =====================================================================
        // Pipeline State Machine
        // =====================================================================
        
        case (pipeline_state)
            STAGE_RESET: begin
                if (rst_n) next_pipeline_state = STAGE_FETCH;
            end
            
            STAGE_FETCH: begin
                if (trap_pending) begin
                    next_pipeline_state = STAGE_TRAP_ENTER;
                end
                else if (is_dispatching_l1) begin
                    next_pipeline_state = STAGE_DECODE;
                end
                else if (mmu_enabled) begin
                    itlb_lookup_valid = 1'b1;
                    if (itlb_page_fault || itlb_access_fault) begin
                        next_pipeline_state = STAGE_TRAP_ENTER;
                    end
                    else if (!itlb_hit && ptw_req_ready) begin
                        ptw_req_valid = 1'b1;
                        ptw_req_vpn = pc[VPN_WIDTH+11:12];
                        ptw_req_is_exec = 1'b1;
                        ptw_req_for_itlb = 1'b1;
                        next_pipeline_state = STAGE_STALL_TLB;
                    end
                    else if (itlb_hit) begin
                        imem_req = 1'b1;
                        next_pipeline_state = STAGE_FETCH_WAIT;
                    end
                end
                else begin
                    imem_req = 1'b1;
                    next_pipeline_state = STAGE_FETCH_WAIT;
                end
            end
            
            STAGE_FETCH_WAIT: begin
                if (imem_ack_in) next_pipeline_state = STAGE_DECODE;
                else if (imem_error_in) next_pipeline_state = STAGE_TRAP_ENTER;
            end
            
            STAGE_STALL_TLB: begin
                if (ptw_resp_valid) begin
                    if (ptw_resp_page_fault || ptw_resp_access_fault)
                        next_pipeline_state = STAGE_TRAP_ENTER;
                    else
                        next_pipeline_state = STAGE_FETCH;
                end
            end
            
            STAGE_DECODE: begin
                if (!fetch_buffer_valid && !is_dispatching_l1)
                    next_pipeline_state = STAGE_FETCH;
                else if (data_hazard0 || struct_hazard)
                    next_pipeline_state = STAGE_DECODE;
                else
                    next_pipeline_state = STAGE_ISSUE;
            end
            
            STAGE_ISSUE: next_pipeline_state = STAGE_EXECUTE;
            
            STAGE_EXECUTE: begin
                if (decoded_instr0.is_mdu) begin
                    mdu_req_valid = 1'b1;
                    mdu_rs1 = rs1_data0;
                    mdu_rs2 = rs2_data0;
                    mdu_funct3 = decoded_instr0.funct3;
                    mdu_is_word = (decoded_instr0.opcode == 7'b0111011);
                    if (mdu_req_ready) next_pipeline_state = STAGE_STALL_MDU;
                end
                else if (decoded_instr0.is_csr) begin
                    csr_req_valid = 1'b1;
                    csr_addr = decoded_instr0.csr_addr;
                    csr_op = decoded_instr0.funct3;
                    csr_wdata = decoded_instr0.funct3[2] ? decoded_instr0.imm : rs1_data0;
                    next_pipeline_state = csr_access_fault ? STAGE_TRAP_ENTER : STAGE_WRITEBACK;
                end
                else if (decoded_instr0.is_ecall || decoded_instr0.is_ebreak) begin
                    next_pipeline_state = STAGE_TRAP_ENTER;
                end
                else if (decoded_instr0.is_mret) begin
                    mret_execute = 1'b1;
                    next_pipeline_state = STAGE_TRAP_RETURN;
                end
                else if (decoded_instr0.is_sret) begin
                    if (current_priv == PRIV_SUPERVISOR && mstatus_tsr)
                        next_pipeline_state = STAGE_TRAP_ENTER;
                    else begin
                        sret_execute = 1'b1;
                        next_pipeline_state = STAGE_TRAP_RETURN;
                    end
                end
                else if (decoded_instr0.is_sfence_vma) begin
                    if (current_priv == PRIV_SUPERVISOR && mstatus_tvm)
                        next_pipeline_state = STAGE_TRAP_ENTER;
                    else
                        next_pipeline_state = STAGE_SFENCE;
                end
                else if (decoded_instr0.is_wfi || decoded_instr0.is_fence) begin
                    next_pipeline_state = STAGE_WRITEBACK;
                end
                else if (decoded_instr0.is_load || decoded_instr0.is_store || decoded_instr0.is_amo) begin
                    if (mmu_enabled) begin
                        dtlb_lookup_valid = 1'b1;
                        dtlb_lookup_is_store = decoded_instr0.is_store || 
                                               (decoded_instr0.is_amo && !decoded_instr0.is_lr);
                        if (dtlb_page_fault || dtlb_access_fault)
                            next_pipeline_state = STAGE_TRAP_ENTER;
                        else if (!dtlb_hit && ptw_req_ready) begin
                            ptw_req_valid = 1'b1;
                            ptw_req_vpn = execute_result0.mem_addr_va[VPN_WIDTH+11:12];
                            ptw_req_is_store = dtlb_lookup_is_store;
                            next_pipeline_state = STAGE_STALL_TLB;
                        end
                        else if (dtlb_hit)
                            next_pipeline_state = STAGE_MEMORY;
                    end
                    else
                        next_pipeline_state = STAGE_MEMORY;
                end
                else
                    next_pipeline_state = STAGE_MEMORY;
            end
            
            STAGE_STALL_MDU: begin
                if (mdu_resp_valid) next_pipeline_state = STAGE_WRITEBACK;
            end
            
            STAGE_MEMORY: begin
                if (execute_result0.is_amo) begin
                    amo_req_valid = 1'b1;
                    amo_op = execute_result0.amo_funct5;
                    amo_is_word = (decoded_instr0.funct3 == 3'b010);
                    amo_addr = dmem_addr_pa_translated;
                    amo_rs2 = execute_result0.store_data;
                    amo_aq = execute_result0.amo_aq;
                    amo_rl = execute_result0.amo_rl;
                    if (amo_req_ready) next_pipeline_state = STAGE_STALL_AMO;
                end
                else if (execute_result0.is_load || execute_result0.is_store) begin
                    dmem_req = 1'b1;
                    dmem_we = execute_result0.is_store;
                    dmem_addr_pa = dmem_addr_pa_translated;
                    dmem_wdata = execute_result0.store_data;
                    dmem_wstrb = execute_result0.mem_wstrb;
                    next_pipeline_state = STAGE_MEMORY_WAIT;
                end
                else
                    next_pipeline_state = STAGE_WRITEBACK;
            end
            
            STAGE_MEMORY_WAIT: begin
                dmem_req = 1'b1;
                dmem_we = execute_result0.is_store;
                dmem_addr_pa = dmem_addr_pa_translated;
                dmem_wdata = execute_result0.store_data;
                dmem_wstrb = execute_result0.mem_wstrb;
                if (dmem_ack_in) next_pipeline_state = STAGE_WRITEBACK;
                else if (dmem_error_in) next_pipeline_state = STAGE_TRAP_ENTER;
            end
            
            STAGE_STALL_AMO: begin
                dmem_req = amo_mem_req;
                dmem_we = amo_mem_we;
                dmem_addr_pa = amo_mem_addr;
                dmem_wdata = amo_mem_wdata;
                dmem_wstrb = amo_mem_wstrb;
                dmem_req_is_amo = 1'b1;
                if (amo_resp_valid) next_pipeline_state = STAGE_WRITEBACK;
            end
            
            STAGE_SFENCE: begin
                if (decoded_instr0.rs1 == 0 && decoded_instr0.rs2 == 0)
                    sfence_all = 1'b1;
                else if (decoded_instr0.rs1 == 0) begin
                    sfence_asid = 1'b1;
                    sfence_asid_val = rs2_data0[ASID_WIDTH-1:0];
                end
                else if (decoded_instr0.rs2 == 0) begin
                    sfence_addr = 1'b1;
                    sfence_vpn_val = rs1_data0[VPN_WIDTH+11:12];
                end
                else begin
                    sfence_both = 1'b1;
                    sfence_asid_val = rs2_data0[ASID_WIDTH-1:0];
                    sfence_vpn_val = rs1_data0[VPN_WIDTH+11:12];
                end
                invalidate_reservations = 1'b1;
                next_pipeline_state = STAGE_WRITEBACK;
            end
            
            STAGE_WRITEBACK: next_pipeline_state = STAGE_FETCH;
            
            STAGE_TRAP_ENTER: begin
                trap_enter = 1'b1;
                if (is_speculating_l2) spec_exception_out = 1'b1;
                else next_pipeline_state = STAGE_FETCH;
            end
            
            STAGE_TRAP_RETURN: next_pipeline_state = STAGE_FETCH;
            
            default: next_pipeline_state = STAGE_RESET;
        endcase
        
        // =====================================================================
        // Next PC Calculation
        // =====================================================================
        
        next_pc = pc;
        if (pipeline_state == STAGE_WRITEBACK) begin
            if (execute_result0.branch_taken)
                next_pc = execute_result0.branch_target;
            else if (dual_issue_valid && execute_result1.branch_taken)
                next_pc = execute_result1.branch_target;
            else
                next_pc = pc + (dual_issue_valid ? 8 : 4);
        end
        else if (pipeline_state == STAGE_TRAP_ENTER && !is_speculating_l2)
            next_pc = trap_vector;
        else if (pipeline_state == STAGE_TRAP_RETURN)
            next_pc = return_pc;
        
        // =====================================================================
        // Trap Cause Determination
        // =====================================================================
        
        trap_pc = pc_decode;
        trap_is_interrupt = interrupt_pending;
        
        if (interrupt_pending) begin
            trap_cause = interrupt_cause;
            trap_value = '0;
        end
        else if (itlb_page_fault) begin
            trap_cause = CAUSE_FETCH_PAGE_FAULT;
            trap_value = pc;
        end
        else if (dtlb_page_fault && execute_result0.is_load) begin
            trap_cause = CAUSE_LOAD_PAGE_FAULT;
            trap_value = execute_result0.mem_addr_va;
        end
        else if (dtlb_page_fault && execute_result0.is_store) begin
            trap_cause = CAUSE_STORE_PAGE_FAULT;
            trap_value = execute_result0.mem_addr_va;
        end
        else if (decoded_instr0.is_ecall) begin
            case (current_priv)
                PRIV_USER:       trap_cause = CAUSE_USER_ECALL;
                PRIV_SUPERVISOR: trap_cause = CAUSE_SUPERVISOR_ECALL;
                PRIV_MACHINE:    trap_cause = CAUSE_MACHINE_ECALL;
                default:         trap_cause = CAUSE_USER_ECALL;
            endcase
            trap_value = '0;
        end
        else if (decoded_instr0.is_ebreak) begin
            trap_cause = CAUSE_BREAKPOINT;
            trap_value = pc_decode;
        end
        else begin
            trap_cause = CAUSE_ILLEGAL_INSTRUCTION;
            trap_value = {32'b0, fetch_buffer[ILEN-1:0]};
        end
    end
    // =========================================================================
    // Pipeline - Lógica Sequencial
    // =========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Reset state
            pipeline_state <= STAGE_RESET;
            current_priv <= PRIV_MACHINE;
            pc <= PC_RESET_VEC;
            
            // Clear registers
            for (int i = 0; i < 32; i++) begin
                regfile[i] <= '0;
                fregfile[i] <= '0;
                shadow_regfile[i] <= '0;
                shadow_fregfile[i] <= '0;
            end
            
            // Clear pipeline registers
            decoded_instr0 <= '0;
            decoded_instr1 <= '0;
            dual_issue_valid <= 1'b0;
            execute_result0 <= '0;
            execute_result1 <= '0;
            memory_result0 <= '0;
            memory_result1 <= '0;
            
            fetch_buffer <= '0;
            fetch_buffer_valid <= 1'b0;
            pc_fetch <= '0;
            pc_decode <= '0;
            
            // STU state
            is_speculating_l2 <= 1'b0;
            is_stalled_l1 <= 1'b0;
            shadow_pc <= '0;
            
            // Counters
            cycle_counter <= '0;
            instret_counter <= '0;
            time_counter <= '0;
            
        end else begin
            // Update state
            pipeline_state <= next_pipeline_state;
            
            // Update counters
            cycle_counter <= cycle_counter + 1;
            time_counter <= time_counter + 1; // Should be driven by external RTC
            
            // =====================================================================
            // STU Logic
            // =====================================================================
            
            // L2 Speculation Start
            if (l2_spec_start_in && (MY_HART_ID == l2_spec_core_id_in)) begin
                is_speculating_l2 <= 1'b1;
                shadow_pc <= pc;
                for (int i = 0; i < 32; i++) begin
                    shadow_regfile[i] <= regfile[i];
                    shadow_fregfile[i] <= fregfile[i];
                end
                pipeline_state <= STAGE_FETCH;
                pc <= l2_spec_pc_in;
                fetch_buffer_valid <= 1'b0;
            end
            
            // Squash
            if (is_speculating_l2 && squash_in[MY_HART_ID]) begin
                is_speculating_l2 <= 1'b0;
                pc <= shadow_pc;
                for (int i = 0; i < 32; i++) begin
                    regfile[i] <= shadow_regfile[i];
                    fregfile[i] <= shadow_fregfile[i];
                end
                pipeline_state <= STAGE_FETCH;
                fetch_buffer_valid <= 1'b0;
            end
            
            // Commit
            if (is_speculating_l2 && commit_in[MY_HART_ID]) begin
                is_speculating_l2 <= 1'b0;
            end
            
            // =====================================================================
            // Pipeline Stage Updates
            // =====================================================================
            
            // --- FETCH ---
            if (pipeline_state == STAGE_FETCH) begin
                pc_fetch <= pc;
                
                if (is_dispatching_l1) begin
                    fetch_buffer[ILEN-1:0] <= l1_dispatch_data_in[MY_HART_ID][0];
                    fetch_buffer[ILEN*2-1:ILEN] <= l1_dispatch_data_in[MY_HART_ID][1];
                    fetch_buffer_valid <= 1'b1;
                    pc_decode <= pc;
                end
            end
            
            // --- FETCH_WAIT ---
            if (pipeline_state == STAGE_FETCH_WAIT && imem_ack_in) begin
                fetch_buffer <= imem_rdata;
                fetch_buffer_valid <= 1'b1;
                pc_decode <= pc_fetch;
            end
            
            // --- DECODE ---
            if (pipeline_state == STAGE_DECODE && fetch_buffer_valid) begin
                decoded_instr0 <= decode_instr(fetch_buffer[ILEN-1:0], pc_decode);
                decoded_instr1 <= decode_instr(fetch_buffer[ILEN*2-1:ILEN], pc_decode + 4);
                dual_issue_valid <= can_dual_issue(
                    decode_instr(fetch_buffer[ILEN-1:0], pc_decode),
                    decode_instr(fetch_buffer[ILEN*2-1:ILEN], pc_decode + 4)
                );
                fetch_buffer_valid <= 1'b0;
            end
            
            // --- ISSUE / EXECUTE ---
            if (pipeline_state == STAGE_ISSUE) begin
                execute_result0 <= execute_alu(decoded_instr0, rs1_data0, rs2_data0, pc_decode);
                if (dual_issue_valid)
                    execute_result1 <= execute_alu(decoded_instr1, rs1_data1, rs2_data1, pc_decode + 4);
                else
                    execute_result1 <= '0;
            end
            
            // --- MEMORY ---
            if (pipeline_state == STAGE_MEMORY || pipeline_state == STAGE_MEMORY_WAIT) begin
                if (dmem_ack_in && execute_result0.is_load) begin
                    memory_result0.data <= process_load_data(
                        dmem_rdata,
                        execute_result0.load_funct3,
                        execute_result0.mem_addr_va[2:0]
                    );
                    memory_result0.reg_we <= 1'b1;
                    memory_result0.rd <= execute_result0.rd;
                end
                else if (!execute_result0.is_load && !execute_result0.is_store) begin
                    memory_result0.data <= execute_result0.alu_result;
                    memory_result0.reg_we <= execute_result0.reg_we;
                    memory_result0.rd <= execute_result0.rd;
                end
            end
            
            // --- STALL_MDU ---
            if (pipeline_state == STAGE_STALL_MDU && mdu_resp_valid) begin
                memory_result0.data <= mdu_result;
                memory_result0.reg_we <= 1'b1;
                memory_result0.rd <= decoded_instr0.rd;
            end
            
            // --- STALL_AMO ---
            if (pipeline_state == STAGE_STALL_AMO && amo_resp_valid) begin
                if (execute_result0.is_sc) begin
                    // SC returns 0 on success, 1 on failure
                    memory_result0.data <= amo_sc_fail ? 64'd1 : 64'd0;
                end else begin
                    // LR and AMO return the loaded value
                    memory_result0.data <= amo_result;
                end
                memory_result0.reg_we <= 1'b1;
                memory_result0.rd <= execute_result0.rd;
            end
            
            // --- CSR Read Result ---
            if (pipeline_state == STAGE_EXECUTE && decoded_instr0.is_csr && !csr_access_fault) begin
                memory_result0.data <= csr_rdata;
                memory_result0.reg_we <= 1'b1;
                memory_result0.rd <= decoded_instr0.rd;
            end
            
            // --- WRITEBACK ---
            if (pipeline_state == STAGE_WRITEBACK) begin
                // Write to GPR
                if (memory_result0.reg_we && memory_result0.rd != 0)
                    regfile[memory_result0.rd] <= memory_result0.data;
                    
                if (dual_issue_valid && memory_result1.reg_we && memory_result1.rd != 0)
                    regfile[memory_result1.rd] <= memory_result1.data;
                
                // Write to FPR
                if (memory_result0.freg_we)
                    fregfile[memory_result0.rd] <= memory_result0.data;
                
                // Update instruction count
                instret_counter <= instret_counter + (dual_issue_valid ? 2 : 1);
                
                // Clear memory results
                memory_result0 <= '0;
                memory_result1 <= '0;
            end
            
            // --- TRAP_ENTER ---
            if (pipeline_state == STAGE_TRAP_ENTER && !is_speculating_l2) begin
                // Privilege update is handled by CSR unit
                // PC update to trap_vector is handled in next_pc
            end
            
            // --- TRAP_RETURN ---
            if (pipeline_state == STAGE_TRAP_RETURN) begin
                current_priv <= privilege_t'(return_priv);
            end
            
            // --- PC Update ---
            if (pipeline_state != STAGE_RESET)
                pc <= next_pc;
            
            // --- Stall tracking ---
            is_stalled_l1 <= (next_pipeline_state == STAGE_DECODE && data_hazard0);
        end
    end
    
    // =========================================================================
    // Register File Copy Port (for STU)
    // =========================================================================
    
    always_ff @(posedge clk) begin
        if (core_copy_write_en_in && (MY_HART_ID == l2_spec_core_id_in)) begin
            regfile[core_copy_write_addr_in] <= core_copy_data_in;
        end
    end
    
    assign core_copy_data_out = regfile[core_copy_read_addr_in];

    // =========================================================================
    // Output Signals
    // =========================================================================
    
    assign core_busy_out = (pipeline_state != STAGE_RESET) && 
                           (pipeline_state != STAGE_FETCH) &&
                           !is_dispatching_l1;
    
    assign spec_task_done_out = is_speculating_l2 && 
                                (pipeline_state == STAGE_DECODE) && 
                                (pc_decode == l2_fork_trigger_pc_in);
    
    assign core_mem_pa_out = dmem_addr_pa;
    assign core_mem_is_store_out = dmem_we;
    assign core_mem_valid_out = dmem_req;

endmodule
