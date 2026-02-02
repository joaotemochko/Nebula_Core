`timescale 1ns/1ps
`default_nettype none

import nebula_pkg::*;

/**
 * @module nebula_backend
 * @brief Backend do Nebula Core - Execute, Memory, Writeback
 */
module nebula_backend #(
    parameter int XLEN = 64,
    parameter int VADDR_WIDTH = 39,
    parameter int PADDR_WIDTH = 56,
    parameter int VPN_WIDTH = 27,
    parameter int PPN_WIDTH = 44
)(
    input  wire                     clk,
    input  wire                     rst_n,
    
    // Frontend interface
    input  wire                     frontend_valid,
    input  frontend_packet_t        frontend_in,
    output backend_ctrl_t           backend_ctrl,
    
    // D-Cache interface
    output logic                    dcache_req,
    output logic [PADDR_WIDTH-1:0]  dcache_addr,
    output logic [XLEN-1:0]         dcache_wdata,
    output logic [7:0]              dcache_wstrb,
    output logic                    dcache_we,
    output logic                    dcache_is_amo,
    output logic [4:0]              dcache_amo_op,
    input  wire                     dcache_ready,
    input  wire                     dcache_resp_valid,
    input  wire [XLEN-1:0]          dcache_resp_data,
    input  wire                     dcache_resp_error,
    
    // DTLB interface
    output logic                    dtlb_req,
    output logic [VPN_WIDTH-1:0]    dtlb_vpn,
    output logic                    dtlb_is_store,
    input  wire                     dtlb_hit,
    input  wire [PPN_WIDTH-1:0]     dtlb_ppn,
    input  wire                     dtlb_page_fault,
    
    // PTW interface
    output logic                    ptw_req,
    output logic [VPN_WIDTH-1:0]    ptw_vpn,
    output logic                    ptw_for_store,
    input  wire                     ptw_ready,
    input  wire                     ptw_resp_valid,
    input  wire                     ptw_page_fault,
    
    // CSR interface
    output logic                    csr_req,
    output logic [11:0]             csr_addr,
    output logic [XLEN-1:0]         csr_wdata,
    output logic [2:0]              csr_op,
    input  wire [XLEN-1:0]          csr_rdata,
    input  wire                     csr_fault,
    
    // Trap interface
    output logic                    trap_enter,
    output logic                    trap_is_interrupt,
    output logic [XLEN-1:0]         trap_pc,
    output logic [XLEN-1:0]         trap_cause,
    output logic [XLEN-1:0]         trap_value,
    output logic                    mret_exec,
    output logic                    sret_exec,
    input  wire [XLEN-1:0]          trap_vector,
    input  wire [XLEN-1:0]          return_pc,
    input  wire [1:0]               return_priv,
    input  wire                     interrupt_pending,
    input  wire [XLEN-1:0]          interrupt_cause,
    
    // MDU interface
    output logic                    mdu_req,
    output logic [XLEN-1:0]         mdu_rs1,
    output logic [XLEN-1:0]         mdu_rs2,
    output logic [2:0]              mdu_funct3,
    output logic                    mdu_is_word,
    input  wire                     mdu_ready,
    input  wire                     mdu_resp_valid,
    input  wire [XLEN-1:0]          mdu_result,
    
    // Control signals
    input  wire [1:0]               current_priv,
    input  wire                     mmu_enabled,
    input  wire                     mstatus_tvm,
    input  wire                     mstatus_tsr,
    
    // SFENCE.VMA
    output logic                    sfence_valid,
    output logic                    sfence_all,
    output logic [VPN_WIDTH-1:0]    sfence_vpn,
    output logic [15:0]             sfence_asid,
    
    // FENCE.I
    output logic                    fence_i_valid,
    output logic                    dcache_flush,
    input  wire                     dcache_flush_done,
    
    // Performance
    output logic                    instr_retired,
    output logic                    instr_retired_2
);

    // Register file
    logic [XLEN-1:0] regfile [0:31];
    
    // Pipeline registers
    decoded_instr_t issue_instr0, issue_instr1;
    logic issue_valid0, issue_valid1, issue_dual;
    logic [XLEN-1:0] issue_rs1_0, issue_rs2_0, issue_rs1_1, issue_rs2_1;
    
    exec_result_t exec_result0, exec_result1;
    logic exec_valid0, exec_valid1;
    
    mem_result_t mem_result0, mem_result1;
    logic mem_valid0, mem_valid1;
    
    // State machine
    typedef enum logic [3:0] {
        B_IDLE, B_ISSUE, B_EXECUTE, B_MEM_TLB, B_MEM_TLB_WAIT,
        B_MEM_ACCESS, B_MEM_WAIT, B_WRITEBACK, B_MDU_WAIT,
        B_TRAP, B_SFENCE, B_FENCE_I
    } state_t;
    
    state_t state, next_state;
    
    // Forwarding
    logic [XLEN-1:0] fwd_rs1_0, fwd_rs2_0, fwd_rs1_1, fwd_rs2_1;
    logic load_use_hazard;
    
    always_comb begin
        fwd_rs1_0 = (frontend_in.instr0.rs1 == 0) ? '0 : regfile[frontend_in.instr0.rs1];
        fwd_rs2_0 = (frontend_in.instr0.rs2 == 0) ? '0 : regfile[frontend_in.instr0.rs2];
        fwd_rs1_1 = (frontend_in.instr1.rs1 == 0) ? '0 : regfile[frontend_in.instr1.rs1];
        fwd_rs2_1 = (frontend_in.instr1.rs2 == 0) ? '0 : regfile[frontend_in.instr1.rs2];
        
        // Forward from MEM
        if (mem_valid0 && mem_result0.reg_we && mem_result0.rd != 0) begin
            if (frontend_in.instr0.rs1 == mem_result0.rd) fwd_rs1_0 = mem_result0.data;
            if (frontend_in.instr0.rs2 == mem_result0.rd) fwd_rs2_0 = mem_result0.data;
            if (frontend_in.instr1.rs1 == mem_result0.rd) fwd_rs1_1 = mem_result0.data;
            if (frontend_in.instr1.rs2 == mem_result0.rd) fwd_rs2_1 = mem_result0.data;
        end
        
        load_use_hazard = exec_valid0 && exec_result0.mem_re &&
            ((frontend_in.instr0.rs1 == exec_result0.rd && frontend_in.instr0.rs1 != 0) ||
             (frontend_in.instr0.rs2 == exec_result0.rd && frontend_in.instr0.rs2 != 0));
    end
    
    // Execute function
    function automatic exec_result_t execute(
        input decoded_instr_t instr,
        input [XLEN-1:0] rs1, rs2
    );
        exec_result_t r = '0;
        r.rd = instr.rd;
        r.pc = instr.pc;
        r.mem_size = instr.funct3;
        
        if (instr.is_alu) begin
            r.reg_we = 1'b1;
            case (instr.opcode)
                7'b0110111: r.result = instr.imm;
                7'b0010111: r.result = instr.pc + instr.imm;
                7'b0010011: case (instr.funct3)
                    3'b000: r.result = rs1 + instr.imm;
                    3'b010: r.result = ($signed(rs1) < $signed(instr.imm)) ? 1 : 0;
                    3'b011: r.result = (rs1 < instr.imm) ? 1 : 0;
                    3'b100: r.result = rs1 ^ instr.imm;
                    3'b110: r.result = rs1 | instr.imm;
                    3'b111: r.result = rs1 & instr.imm;
                    3'b001: r.result = rs1 << instr.imm[5:0];
                    3'b101: r.result = instr.funct7[5] ? 
                            ($signed(rs1) >>> instr.imm[5:0]) : (rs1 >> instr.imm[5:0]);
                    default: ;
                endcase
                7'b0110011: case ({instr.funct7, instr.funct3})
                    10'b0000000_000: r.result = rs1 + rs2;
                    10'b0100000_000: r.result = rs1 - rs2;
                    10'b0000000_001: r.result = rs1 << rs2[5:0];
                    10'b0000000_010: r.result = ($signed(rs1) < $signed(rs2)) ? 1 : 0;
                    10'b0000000_011: r.result = (rs1 < rs2) ? 1 : 0;
                    10'b0000000_100: r.result = rs1 ^ rs2;
                    10'b0000000_101: r.result = rs1 >> rs2[5:0];
                    10'b0100000_101: r.result = $signed(rs1) >>> rs2[5:0];
                    10'b0000000_110: r.result = rs1 | rs2;
                    10'b0000000_111: r.result = rs1 & rs2;
                    default: ;
                endcase
                default: ;
            endcase
        end
        else if (instr.is_branch) begin
            r.result = instr.pc + 4;
            if (instr.is_jal) begin
                r.reg_we = 1'b1;
                r.branch_taken = 1'b1;
                r.branch_target = instr.pc + instr.imm;
            end
            else if (instr.is_jalr) begin
                r.reg_we = 1'b1;
                r.branch_taken = 1'b1;
                r.branch_target = (rs1 + instr.imm) & ~64'h1;
            end
            else begin
                r.branch_target = instr.pc + instr.imm;
                case (instr.funct3)
                    3'b000: r.branch_taken = (rs1 == rs2);
                    3'b001: r.branch_taken = (rs1 != rs2);
                    3'b100: r.branch_taken = ($signed(rs1) < $signed(rs2));
                    3'b101: r.branch_taken = ($signed(rs1) >= $signed(rs2));
                    3'b110: r.branch_taken = (rs1 < rs2);
                    3'b111: r.branch_taken = (rs1 >= rs2);
                    default: ;
                endcase
            end
        end
        else if (instr.is_load) begin
            r.reg_we = 1'b1;
            r.mem_re = 1'b1;
            r.mem_addr = rs1 + instr.imm;
        end
        else if (instr.is_store) begin
            r.mem_we = 1'b1;
            r.mem_addr = rs1 + instr.imm;
            r.store_data = rs2;
            case (instr.funct3)
                3'b000: r.mem_wstrb = 8'h01 << r.mem_addr[2:0];
                3'b001: r.mem_wstrb = 8'h03 << r.mem_addr[2:0];
                3'b010: r.mem_wstrb = 8'h0F << r.mem_addr[2:0];
                3'b011: r.mem_wstrb = 8'hFF;
                default: ;
            endcase
        end
        else if (instr.is_amo) begin
            r.reg_we = 1'b1;
            r.is_amo = 1'b1;
            r.is_lr = instr.is_lr;
            r.is_sc = instr.is_sc;
            r.amo_op = instr.funct7[6:2];
            r.mem_addr = rs1;
            r.store_data = rs2;
            r.mem_re = 1'b1;
            r.mem_we = !instr.is_lr;
            r.mem_wstrb = (instr.funct3 == 3'b010) ? 8'h0F : 8'hFF;
        end
        
        return r;
    endfunction
    
    // FSM
    always_comb begin
        next_state = state;
        case (state)
            B_IDLE: if (interrupt_pending) next_state = B_TRAP;
                    else if (frontend_valid && !load_use_hazard) next_state = B_ISSUE;
            B_ISSUE: next_state = B_EXECUTE;
            B_EXECUTE: begin
                if (exec_result0.trap) next_state = B_TRAP;
                else if (issue_instr0.is_mdu) next_state = B_MDU_WAIT;
                else if (issue_instr0.is_sfence_vma) next_state = B_SFENCE;
                else if (issue_instr0.is_fence_i) next_state = B_FENCE_I;
                else if (issue_instr0.is_mret || issue_instr0.is_sret ||
                         issue_instr0.is_ecall || issue_instr0.is_ebreak) next_state = B_TRAP;
                else if (exec_result0.mem_re || exec_result0.mem_we)
                    next_state = mmu_enabled ? B_MEM_TLB : B_MEM_ACCESS;
                else next_state = B_WRITEBACK;
            end
            B_MDU_WAIT: if (mdu_resp_valid) next_state = B_WRITEBACK;
            B_MEM_TLB: if (dtlb_page_fault) next_state = B_TRAP;
                       else if (dtlb_hit) next_state = B_MEM_ACCESS;
                       else if (ptw_ready) next_state = B_MEM_TLB_WAIT;
            B_MEM_TLB_WAIT: if (ptw_resp_valid) 
                            next_state = ptw_page_fault ? B_TRAP : B_MEM_TLB;
            B_MEM_ACCESS: if (dcache_ready) next_state = B_MEM_WAIT;
            B_MEM_WAIT: if (dcache_resp_valid)
                        next_state = dcache_resp_error ? B_TRAP : B_WRITEBACK;
            B_WRITEBACK: next_state = B_IDLE;
            B_TRAP: next_state = B_IDLE;
            B_SFENCE: next_state = B_WRITEBACK;
            B_FENCE_I: if (dcache_flush_done) next_state = B_WRITEBACK;
            default: next_state = B_IDLE;
        endcase
    end
    
    // Backend control
    always_comb begin
        backend_ctrl = '0;
        backend_ctrl.stall = (state != B_IDLE) || load_use_hazard;
        backend_ctrl.flush = (state == B_TRAP) || 
                            (state == B_EXECUTE && exec_result0.branch_taken);
        backend_ctrl.redirect = backend_ctrl.flush;
        
        if (state == B_TRAP) begin
            if (issue_instr0.is_mret || issue_instr0.is_sret)
                backend_ctrl.redirect_pc = return_pc[VADDR_WIDTH-1:0];
            else
                backend_ctrl.redirect_pc = trap_vector[VADDR_WIDTH-1:0];
        end
        else if (exec_result0.branch_taken)
            backend_ctrl.redirect_pc = exec_result0.branch_target[VADDR_WIDTH-1:0];
    end
    
    // Physical address
    wire [PADDR_WIDTH-1:0] paddr = mmu_enabled ? 
        {dtlb_ppn, exec_result0.mem_addr[11:0]} : exec_result0.mem_addr[PADDR_WIDTH-1:0];
    
    // Interface assignments
    assign dcache_req = (state == B_MEM_ACCESS);
    assign dcache_addr = paddr;
    assign dcache_wdata = exec_result0.store_data;
    assign dcache_wstrb = exec_result0.mem_wstrb;
    assign dcache_we = exec_result0.mem_we;
    assign dcache_is_amo = exec_result0.is_amo;
    assign dcache_amo_op = exec_result0.amo_op;
    
    assign dtlb_req = (state == B_MEM_TLB);
    assign dtlb_vpn = exec_result0.mem_addr[VPN_WIDTH+11:12];
    assign dtlb_is_store = exec_result0.mem_we;
    
    assign ptw_req = (state == B_MEM_TLB) && !dtlb_hit && !dtlb_page_fault;
    assign ptw_vpn = exec_result0.mem_addr[VPN_WIDTH+11:12];
    assign ptw_for_store = exec_result0.mem_we;
    
    assign mdu_req = (state == B_EXECUTE) && issue_instr0.is_mdu;
    assign mdu_rs1 = issue_rs1_0;
    assign mdu_rs2 = issue_rs2_0;
    assign mdu_funct3 = issue_instr0.funct3;
    assign mdu_is_word = issue_instr0.is_alu_w;
    
    assign csr_req = (state == B_EXECUTE) && issue_instr0.is_csr;
    assign csr_addr = issue_instr0.csr_addr;
    assign csr_op = issue_instr0.funct3;
    assign csr_wdata = issue_instr0.funct3[2] ? issue_instr0.imm : issue_rs1_0;
    
    assign trap_enter = (state == B_TRAP) && !issue_instr0.is_mret && !issue_instr0.is_sret;
    assign trap_is_interrupt = interrupt_pending;
    assign trap_pc = issue_instr0.pc;
    assign mret_exec = (state == B_TRAP) && issue_instr0.is_mret;
    assign sret_exec = (state == B_TRAP) && issue_instr0.is_sret;
    
    always_comb begin
        trap_cause = '0;
        trap_value = '0;
        if (interrupt_pending) trap_cause = interrupt_cause;
        else if (dtlb_page_fault || ptw_page_fault) begin
            trap_cause = exec_result0.mem_we ? {58'b0, EXC_STORE_PAGE_FAULT} : 
                                               {58'b0, EXC_LOAD_PAGE_FAULT};
            trap_value = exec_result0.mem_addr;
        end
        else if (issue_instr0.is_ecall) begin
            case (current_priv)
                PRIV_USER: trap_cause = {58'b0, EXC_ECALL_U};
                PRIV_SUPERVISOR: trap_cause = {58'b0, EXC_ECALL_S};
                PRIV_MACHINE: trap_cause = {58'b0, EXC_ECALL_M};
                default: trap_cause = {58'b0, EXC_ECALL_U};
            endcase
        end
        else if (issue_instr0.is_ebreak) trap_cause = {58'b0, EXC_BREAKPOINT};
    end
    
    assign sfence_valid = (state == B_SFENCE);
    assign sfence_all = (issue_instr0.rs1 == 0) && (issue_instr0.rs2 == 0);
    assign sfence_vpn = issue_rs1_0[VPN_WIDTH+11:12];
    assign sfence_asid = issue_rs2_0[15:0];
    
    assign fence_i_valid = (state == B_FENCE_I);
    assign dcache_flush = fence_i_valid;
    
    assign instr_retired = (state == B_WRITEBACK);
    assign instr_retired_2 = instr_retired && mem_valid1;
    
    // Sequential logic
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= B_IDLE;
            for (int i = 0; i < 32; i++) regfile[i] <= '0;
            issue_instr0 <= '0; issue_instr1 <= '0;
            issue_valid0 <= 0; issue_valid1 <= 0; issue_dual <= 0;
            exec_result0 <= '0; exec_result1 <= '0;
            exec_valid0 <= 0; exec_valid1 <= 0;
            mem_result0 <= '0; mem_result1 <= '0;
            mem_valid0 <= 0; mem_valid1 <= 0;
            issue_rs1_0 <= '0; issue_rs2_0 <= '0;
            issue_rs1_1 <= '0; issue_rs2_1 <= '0;
        end
        else begin
            state <= next_state;
            
            case (state)
                B_IDLE: begin
                    if (frontend_valid && !load_use_hazard && !interrupt_pending) begin
                        issue_instr0 <= frontend_in.instr0;
                        issue_instr1 <= frontend_in.instr1;
                        issue_valid0 <= frontend_in.instr0_valid;
                        issue_valid1 <= frontend_in.dual_issue;
                        issue_dual <= frontend_in.dual_issue;
                        issue_rs1_0 <= fwd_rs1_0;
                        issue_rs2_0 <= fwd_rs2_0;
                        issue_rs1_1 <= fwd_rs1_1;
                        issue_rs2_1 <= fwd_rs2_1;
                    end
                    mem_valid0 <= 0; mem_valid1 <= 0;
                end
                
                B_EXECUTE: begin
                    exec_result0 <= execute(issue_instr0, issue_rs1_0, issue_rs2_0);
                    exec_valid0 <= issue_valid0;
                    if (issue_dual) begin
                        exec_result1 <= execute(issue_instr1, issue_rs1_1, issue_rs2_1);
                        exec_valid1 <= issue_valid1;
                    end else begin
                        exec_result1 <= '0;
                        exec_valid1 <= 0;
                    end
                end
                
                B_MDU_WAIT: if (mdu_resp_valid) begin
                    mem_result0.data <= mdu_result;
                    mem_result0.rd <= issue_instr0.rd;
                    mem_result0.reg_we <= 1;
                    mem_valid0 <= 1;
                end
                
                B_MEM_WAIT: if (dcache_resp_valid && !dcache_resp_error) begin
                    mem_result0.data <= exec_result0.mem_re ? 
                        sign_extend(dcache_resp_data, exec_result0.mem_size) : exec_result0.result;
                    mem_result0.rd <= exec_result0.rd;
                    mem_result0.reg_we <= exec_result0.reg_we;
                    mem_valid0 <= exec_valid0;
                end
                
                B_WRITEBACK: begin
                    if (!mem_valid0 && exec_valid0) begin
                        mem_result0.data <= issue_instr0.is_csr ? csr_rdata : exec_result0.result;
                        mem_result0.rd <= issue_instr0.is_csr ? issue_instr0.rd : exec_result0.rd;
                        mem_result0.reg_we <= issue_instr0.is_csr ? 1 : exec_result0.reg_we;
                        mem_valid0 <= 1;
                    end
                    if (!mem_valid1 && exec_valid1) begin
                        mem_result1.data <= exec_result1.result;
                        mem_result1.rd <= exec_result1.rd;
                        mem_result1.reg_we <= exec_result1.reg_we;
                        mem_valid1 <= 1;
                    end
                    
                    if (mem_valid0 && mem_result0.reg_we && mem_result0.rd != 0)
                        regfile[mem_result0.rd] <= mem_result0.data;
                    if (mem_valid1 && mem_result1.reg_we && mem_result1.rd != 0)
                        regfile[mem_result1.rd] <= mem_result1.data;
                end
                
                B_TRAP: begin
                    exec_valid0 <= 0; exec_valid1 <= 0;
                    issue_valid0 <= 0; issue_valid1 <= 0;
                end
                
                default: ;
            endcase
        end
    end

endmodule
