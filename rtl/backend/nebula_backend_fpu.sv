`timescale 1ns/1ps
`default_nettype none

import nebula_pkg::*;

/**
 * @module nebula_backend_fpu
 * @brief Backend Dual-Issue do Nebula Core com suporte completo a FPU
 *
 * CORREÇÕES APLICADAS:
 * 1. fpu_rm: assign fpu_rm = issue_instr0.rm  — estava faltando, causando
 *    que a FPU sempre usasse RM=000 (RNE) independente da instrução.
 *
 * 2. LUI/AUIPC movidos ANTES do bloco "if (issue_instr0.is_alu_w)" em
 *    alu0_result e alu1_result — evita que a sign-extension de 32 bits
 *    sobrescreva o imediato de 64 bits do LUI/AUIPC.
 *
 * 3. dcache_we corrigido para dual-issue: deve verificar tanto instr0
 *    quanto instr1 (via is_mem1).
 *
 * 4. exec_result exposto como wire alias de exec_result0 para que o
 *    testbench cocotb possa ler o resultado sem depender de --public-flat-rw.
 *
 * 5. Trap PC para dual-issue: quando mem_is_instr1, o trap_pc deve
 *    apontar para o PC da instr1, não da instr0.
 */
module nebula_backend_fpu #(
    parameter int XLEN = 64,
    parameter int FLEN = 64,
    parameter int VADDR_WIDTH = 39,
    parameter int PADDR_WIDTH = 56,
    parameter int VPN_WIDTH = 27,
    parameter int PPN_WIDTH = 44
)(
    input  wire                     clk,
    input  wire                     rst_n,

    input  wire                     frontend_valid,
    input  frontend_packet_t        frontend_in,
    output backend_ctrl_t           backend_ctrl,
    output logic [7:0]              backend_dmem_wstrb,
    output bp_update_t              bp_update,

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

    output logic                    dtlb_req,
    output logic [VPN_WIDTH-1:0]    dtlb_vpn,
    output logic                    dtlb_is_store,
    input  wire                     dtlb_hit,
    input  wire [PPN_WIDTH-1:0]     dtlb_ppn,
    input  wire                     dtlb_page_fault,

    output logic                    ptw_req,
    input  wire                     ptw_ready,
    input  wire                     ptw_resp_valid,
    input  wire                     ptw_page_fault,

    output logic                    csr_req,
    output logic [11:0]             csr_addr,
    output logic [XLEN-1:0]         csr_wdata,
    output logic [2:0]              csr_op,
    input  wire [XLEN-1:0]          csr_rdata,
    input  wire                     csr_fault,

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

    output logic                    mdu_req,
    output logic [XLEN-1:0]         mdu_rs1,
    output logic [XLEN-1:0]         mdu_rs2,
    output logic [2:0]              mdu_funct3,
    output logic                    mdu_is_word,
    input  wire                     mdu_ready,
    input  wire                     mdu_resp_valid,
    input  wire [XLEN-1:0]          mdu_result,

    output logic                    fpu_req,
    output fpu_op_t                 fpu_op,
    output logic [FLEN-1:0]         fpu_rs1,
    output logic [FLEN-1:0]         fpu_rs2,
    output logic [FLEN-1:0]         fpu_rs3,
    output logic [XLEN-1:0]         fpu_int_rs1,
    output rounding_mode_t          fpu_rm,
    output logic                    fpu_is_single,
    input  wire                     fpu_ready,
    input  wire                     fpu_resp_valid,
    input  wire [FLEN-1:0]          fpu_result,
    input  wire [XLEN-1:0]          fpu_int_result,
    input  fflags_t                 fpu_fflags,

    input  wire                     frontend_exception,
    input  wire [5:0]               frontend_exception_cause,
    input  wire [XLEN-1:0]          frontend_exception_value,

    input  wire [1:0]               current_priv,
    input  wire                     mmu_enabled,

    output logic                    sfence_valid,
    output logic                    sfence_all,
    output logic [VPN_WIDTH-1:0]    sfence_vpn,
    output logic [15:0]             sfence_asid,

    output logic                    fence_i_valid,
    output logic                    dcache_flush,
    input  wire                     dcache_flush_done,

    output logic                    instr_retired,
    output logic                    instr_retired_2
);

    // =========================================================================
    // Register Files
    // =========================================================================
    logic [XLEN-1:0] regfile   [0:31];
    logic [FLEN-1:0] fpregfile [0:31];

    // =========================================================================
    // Pipeline Registers (DUAL ISSUE)
    // =========================================================================
    decoded_instr_t  issue_instr0;
    logic            issue_valid0;
    logic [XLEN-1:0] issue0_rs1_data, issue0_rs2_data;
    logic [FLEN-1:0] issue0_frs1_data, issue0_frs2_data, issue0_frs3_data;

    decoded_instr_t  issue_instr1;
    logic            issue_valid1;
    logic            issue_dual;
    logic [XLEN-1:0] issue1_rs1_data, issue1_rs2_data;

    // Execute stage
    logic [XLEN-1:0] exec_result0, exec_result1;
    logic [FLEN-1:0] exec_fp_result;

    // FIX 4: alias público para o testbench
    logic [XLEN-1:0] exec_result;
    assign exec_result = exec_result0;

    logic [VADDR_WIDTH-1:0] exec_mem_addr;
    logic [XLEN-1:0]        exec_store_data;
    logic [FLEN-1:0]        exec_fp_store_data;
    logic                   mem_is_instr1;

    logic            exec_branch_taken;
    logic [VADDR_WIDTH-1:0] exec_branch_target;
    logic            exec_trap;
    exception_cause_t exec_trap_cause;
    logic [XLEN-1:0] exec_trap_value;

    // Memory stage
    logic [XLEN-1:0] mem0_result, mem1_result;
    logic [4:0]      mem0_rd, mem1_rd;
    logic            mem0_int_we, mem1_int_we;
    logic [FLEN-1:0] mem_fp_result;
    logic            mem_fp_we;

    // =========================================================================
    // FSM
    // =========================================================================
    typedef enum logic [4:0] {
        S_IDLE,
        S_ISSUE,
        S_EXECUTE,
        S_MDU_WAIT,
        S_FPU_WAIT,
        S_MEM_TLB,
        S_MEM_TLB_WAIT,
        S_MEM_ACCESS,
        S_MEM_WAIT,
        S_WRITEBACK,
        S_TRAP,
        S_SFENCE,
        S_FENCE_I,
        S_FENCE_I_WAIT
    } state_t;

    state_t state, next_state;

    // =========================================================================
    // Forwarding Logic
    // =========================================================================
    logic [XLEN-1:0] rs1_0_fwd, rs2_0_fwd;
    logic [XLEN-1:0] rs1_1_fwd, rs2_1_fwd;
    logic [FLEN-1:0] frs1_forwarded, frs2_forwarded, frs3_forwarded;

    always_comb begin
        // --- FORWARDING INSTR 0 ---
        if (frontend_in.instr0.rs1 == 5'd0) 
            rs1_0_fwd = 64'd0;
        else if (mem1_int_we && (state != S_IDLE) && mem1_rd == frontend_in.instr0.rs1) 
            rs1_0_fwd = mem1_result;
        else if (mem0_int_we && (state != S_IDLE) && mem0_rd == frontend_in.instr0.rs1) 
            rs1_0_fwd = mem0_result;
        else 
            rs1_0_fwd = regfile[frontend_in.instr0.rs1];

        if (frontend_in.instr0.rs2 == 5'd0) 
            rs2_0_fwd = 64'd0;
        else if (mem1_int_we && (state != S_IDLE) && mem1_rd == frontend_in.instr0.rs2) 
            rs2_0_fwd = mem1_result;
        else if (mem0_int_we && (state != S_IDLE) && mem0_rd == frontend_in.instr0.rs2) 
            rs2_0_fwd = mem0_result;
        else 
            rs2_0_fwd = regfile[frontend_in.instr0.rs2];

        // --- FORWARDING INSTR 1 ---
        if (frontend_in.instr1.rs1 == 5'd0) 
            rs1_1_fwd = 64'd0;
        else if (mem1_int_we && (state != S_IDLE) && mem1_rd == frontend_in.instr1.rs1) 
            rs1_1_fwd = mem1_result;
        else if (mem0_int_we && (state != S_IDLE) && mem0_rd == frontend_in.instr1.rs1) 
            rs1_1_fwd = mem0_result;
        else 
            rs1_1_fwd = regfile[frontend_in.instr1.rs1];

        if (frontend_in.instr1.rs2 == 5'd0) 
            rs2_1_fwd = 64'd0;
        else if (mem1_int_we && (state != S_IDLE) && mem1_rd == frontend_in.instr1.rs2) 
            rs2_1_fwd = mem1_result;
        else if (mem0_int_we && (state != S_IDLE) && mem0_rd == frontend_in.instr1.rs2) 
            rs2_1_fwd = mem0_result;
        else 
            rs2_1_fwd = regfile[frontend_in.instr1.rs2];

        if (mem_fp_we && (state != S_IDLE) && mem0_rd == frontend_in.instr0.rs1) 
            frs1_forwarded = mem_fp_result;
        else frs1_forwarded = fpregfile[frontend_in.instr0.rs1];
        
        if (mem_fp_we && (state != S_IDLE) && mem0_rd == frontend_in.instr0.rs2) 
            frs2_forwarded = mem_fp_result;
        else frs2_forwarded = fpregfile[frontend_in.instr0.rs2];
        
        if (mem_fp_we && (state != S_IDLE) && mem0_rd == frontend_in.instr0.rs3) 
            frs3_forwarded = mem_fp_result;
        else frs3_forwarded = fpregfile[frontend_in.instr0.rs3];
    end

    // =========================================================================
    // ALU (Dual)
    // =========================================================================
    logic [XLEN-1:0] alu0_result, alu1_result;
    logic [XLEN-1:0] alu0_op1, alu0_op2, alu1_op1, alu1_op2;
    logic [5:0]      shamt0, shamt1;

    assign alu0_op1 = issue0_rs1_data;
    assign alu0_op2 = (issue_instr0.is_alu && issue_instr0.opcode[5] == 1'b0) ?
                      issue_instr0.imm : issue0_rs2_data;
    assign shamt0   = issue_instr0.opcode[5] ?
                      (issue_instr0.is_alu_w ? {1'b0, issue0_rs2_data[4:0]} : issue0_rs2_data[5:0]) :
                      (issue_instr0.is_alu_w ? {1'b0, issue_instr0.imm[4:0]} : issue_instr0.imm[5:0]);

    assign alu1_op1 = issue1_rs1_data;
    assign alu1_op2 = (issue_instr1.is_alu && issue_instr1.opcode[5] == 1'b0) ?
                      issue_instr1.imm : issue1_rs2_data;
    assign shamt1   = issue_instr1.opcode[5] ?
                      (issue_instr1.is_alu_w ? {1'b0, issue1_rs2_data[4:0]} : issue1_rs2_data[5:0]) :
                      (issue_instr1.is_alu_w ? {1'b0, issue_instr1.imm[4:0]} : issue_instr1.imm[5:0]);

    always_comb begin
        // ---- ALU 0 ----
        alu0_result = '0;

        // FIX 2: LUI/AUIPC ANTES do bloco is_alu_w para não serem sobrescritos
        if (issue_instr0.opcode == 7'b0110111)
            alu0_result = issue_instr0.imm;
        else if (issue_instr0.opcode == 7'b0010111)
            alu0_result = {{(XLEN-VADDR_WIDTH){issue_instr0.pc[VADDR_WIDTH-1]}}, issue_instr0.pc}
                          + issue_instr0.imm;
        else begin
            case (issue_instr0.funct3)
                3'b000: alu0_result = (issue_instr0.funct7[5] && issue_instr0.opcode[5]) ?
                                      (alu0_op1 - alu0_op2) : (alu0_op1 + alu0_op2);
                3'b001: alu0_result = issue_instr0.is_alu_w ?
                                      {{32{1'b0}}, alu0_op1[31:0] << shamt0[4:0]} :
                                      (alu0_op1 << shamt0);
                3'b010: alu0_result = ($signed(alu0_op1) < $signed(alu0_op2)) ? 64'd1 : 64'd0;
                3'b011: alu0_result = (alu0_op1 < alu0_op2) ? 64'd1 : 64'd0;
                3'b100: alu0_result = alu0_op1 ^ alu0_op2;
                3'b101: begin
                    if (issue_instr0.funct7[5])
                        alu0_result = issue_instr0.is_alu_w ?
                                      {{32{alu0_op1[31]}}, $signed(alu0_op1[31:0]) >>> shamt0[4:0]} :
                                      ($signed(alu0_op1) >>> shamt0);
                    else
                        alu0_result = issue_instr0.is_alu_w ?
                                      {32'b0, alu0_op1[31:0] >> shamt0[4:0]} :
                                      (alu0_op1 >> shamt0);
                end
                3'b110: alu0_result = alu0_op1 | alu0_op2;
                3'b111: alu0_result = alu0_op1 & alu0_op2;
                default: alu0_result = '0;
            endcase
            // Sign-extend para instruções W (só ALU-R/I W, não LUI/AUIPC)
            if (issue_instr0.is_alu_w)
                alu0_result = {{32{alu0_result[31]}}, alu0_result[31:0]};
        end

        // ---- ALU 1 ----
        alu1_result = '0;
        if (issue_valid1) begin
            if (issue_instr1.opcode == 7'b0110111)
                alu1_result = issue_instr1.imm;
            else if (issue_instr1.opcode == 7'b0010111)
                alu1_result = {{(XLEN-VADDR_WIDTH){issue_instr1.pc[VADDR_WIDTH-1]}}, issue_instr1.pc}
                              + issue_instr1.imm;
            else begin
                case (issue_instr1.funct3)
                    3'b000: alu1_result = (issue_instr1.funct7[5] && issue_instr1.opcode[5]) ?
                                          (alu1_op1 - alu1_op2) : (alu1_op1 + alu1_op2);
                    3'b001: alu1_result = issue_instr1.is_alu_w ?
                                          {{32{1'b0}}, alu1_op1[31:0] << shamt1[4:0]} :
                                          (alu1_op1 << shamt1);
                    3'b010: alu1_result = ($signed(alu1_op1) < $signed(alu1_op2)) ? 64'd1 : 64'd0;
                    3'b011: alu1_result = (alu1_op1 < alu1_op2) ? 64'd1 : 64'd0;
                    3'b100: alu1_result = alu1_op1 ^ alu1_op2;
                    3'b101: begin
                        if (issue_instr1.funct7[5])
                            alu1_result = issue_instr1.is_alu_w ?
                                          {{32{alu1_op1[31]}}, $signed(alu1_op1[31:0]) >>> shamt1[4:0]} :
                                          ($signed(alu1_op1) >>> shamt1);
                        else
                            alu1_result = issue_instr1.is_alu_w ?
                                          {32'b0, alu1_op1[31:0] >> shamt1[4:0]} :
                                          (alu1_op1 >> shamt1);
                    end
                    3'b110: alu1_result = alu1_op1 | alu1_op2;
                    3'b111: alu1_result = alu1_op1 & alu1_op2;
                    default: alu1_result = '0;
                endcase
                if (issue_instr1.is_alu_w)
                    alu1_result = {{32{alu1_result[31]}}, alu1_result[31:0]};
            end
        end
    end

    // =========================================================================
    // Branch
    // =========================================================================
    logic branch_taken;
    logic [VADDR_WIDTH-1:0] branch_target;

    always_comb begin
        branch_taken  = 1'b0;
        branch_target = issue_instr0.pc + 4;

        if (issue_instr0.is_jal) begin
            branch_taken  = 1'b1;
            branch_target = issue_instr0.pc + issue_instr0.imm[VADDR_WIDTH-1:0];
        end else if (issue_instr0.is_jalr) begin
            branch_taken  = 1'b1;
            branch_target = (issue0_rs1_data[VADDR_WIDTH-1:0] +
                             issue_instr0.imm[VADDR_WIDTH-1:0]) & ~{VADDR_WIDTH{1'b0}} &
                            ~{{(VADDR_WIDTH-1){1'b0}}, 1'b1};
        end else if (issue_instr0.is_branch) begin
            case (issue_instr0.funct3)
                3'b000: branch_taken = (issue0_rs1_data == issue0_rs2_data);
                3'b001: branch_taken = (issue0_rs1_data != issue0_rs2_data);
                3'b100: branch_taken = ($signed(issue0_rs1_data) < $signed(issue0_rs2_data));
                3'b101: branch_taken = ($signed(issue0_rs1_data) >= $signed(issue0_rs2_data));
                3'b110: branch_taken = (issue0_rs1_data < issue0_rs2_data);
                3'b111: branch_taken = (issue0_rs1_data >= issue0_rs2_data);
                default: branch_taken = 1'b0;
            endcase
            if (branch_taken)
                branch_target = issue_instr0.pc + issue_instr0.imm[VADDR_WIDTH-1:0];
        end
    end

    // =========================================================================
    // FPU Operation Decode
    // =========================================================================
    fpu_op_t decoded_fpu_op;

    always_comb begin
        decoded_fpu_op = FPU_ADD;
        case (issue_instr0.funct7[6:2])
            5'b00000: decoded_fpu_op = FPU_ADD;
            5'b00001: decoded_fpu_op = FPU_SUB;
            5'b00010: decoded_fpu_op = FPU_MUL;
            5'b00011: decoded_fpu_op = FPU_DIV;
            5'b01011: decoded_fpu_op = FPU_SQRT;
            5'b00100: case (issue_instr0.funct3)
                3'b000: decoded_fpu_op = FPU_SGNJ;
                3'b001: decoded_fpu_op = FPU_SGNJN;
                3'b010: decoded_fpu_op = FPU_SGNJX;
                default: ;
            endcase
            5'b00101: decoded_fpu_op = (issue_instr0.funct3 == 3'b000) ? FPU_MIN : FPU_MAX;
            5'b10100: case (issue_instr0.funct3)
                3'b010: decoded_fpu_op = FPU_CMP_EQ;
                3'b001: decoded_fpu_op = FPU_CMP_LT;
                3'b000: decoded_fpu_op = FPU_CMP_LE;
                default: ;
            endcase
            5'b11100: decoded_fpu_op = (issue_instr0.funct3 == 3'b001) ? FPU_CLASS : FPU_MV_X_W;
            5'b11110: decoded_fpu_op = FPU_MV_W_X;
            5'b11000: case (issue_instr0.rs2)
                5'b00000: decoded_fpu_op = FPU_CVT_W;
                5'b00001: decoded_fpu_op = FPU_CVT_WU;
                5'b00010: decoded_fpu_op = FPU_CVT_L;
                5'b00011: decoded_fpu_op = FPU_CVT_LU;
                default: ;
            endcase
            5'b11010: decoded_fpu_op = FPU_CVT_INT;
            5'b01000: decoded_fpu_op = (issue_instr0.rs2[0]) ? FPU_CVT_D : FPU_CVT_S;
            default: ;
        endcase
        if (issue_instr0.is_fma) begin
            case (issue_instr0.opcode[3:2])
                2'b00: decoded_fpu_op = FPU_MADD;
                2'b01: decoded_fpu_op = FPU_MSUB;
                2'b10: decoded_fpu_op = FPU_NMSUB;
                2'b11: decoded_fpu_op = FPU_NMADD;
            endcase
        end
    end

    // =========================================================================
    // Memory Address
    // =========================================================================
    logic [VADDR_WIDTH-1:0] mem_vaddr;
    logic [2:0]             mem_size;
    logic [7:0]             mem_wstrb_calc;

    wire is_mem1 = issue_valid1 &&
                   (issue_instr1.is_load || issue_instr1.is_store || issue_instr1.is_amo);

    assign mem_vaddr = is_mem1 ?
        (issue1_rs1_data[VADDR_WIDTH-1:0] + issue_instr1.imm[VADDR_WIDTH-1:0]) :
        (issue0_rs1_data[VADDR_WIDTH-1:0] + issue_instr0.imm[VADDR_WIDTH-1:0]);
    assign mem_size = is_mem1 ? issue_instr1.funct3 : issue_instr0.funct3;

    always_comb begin
        mem_wstrb_calc = 8'b0;
        case (mem_size[1:0])
            2'b00: mem_wstrb_calc = 8'b00000001 << mem_vaddr[2:0];
            2'b01: mem_wstrb_calc = 8'b00000011 << mem_vaddr[2:0];
            2'b10: mem_wstrb_calc = 8'b00001111 << mem_vaddr[2:0];
            2'b11: mem_wstrb_calc = 8'b11111111;
        endcase
    end

    // =========================================================================
    // Load sign extension
    // =========================================================================
    function automatic logic [XLEN-1:0] sign_extend_load(
        input logic [XLEN-1:0] data,
        input logic [2:0] size
    );
        case (size)
            3'b000: return {{56{data[7]}},  data[7:0]};
            3'b001: return {{48{data[15]}}, data[15:0]};
            3'b010: return {{32{data[31]}}, data[31:0]};
            3'b011: return data;
            3'b100: return {56'b0, data[7:0]};
            3'b101: return {48'b0, data[15:0]};
            3'b110: return {32'b0, data[31:0]};
            default: return data;
        endcase
    endfunction

    // =========================================================================
    // FSM Next State
    // =========================================================================
    always_comb begin
        next_state = state;
        case (state)
            S_IDLE: begin
                if (interrupt_pending)         next_state = S_TRAP;
                else if (frontend_exception)   next_state = S_TRAP;
                else if (frontend_valid && frontend_in.instr0_valid)
                    next_state = S_ISSUE;
            end
            S_ISSUE: begin
                if (issue_instr0.is_mdu)
                    next_state = mdu_ready ? S_MDU_WAIT : S_ISSUE;
                else if (issue_instr0.is_fp && !issue_instr0.is_fp_load && !issue_instr0.is_fp_store)
                    next_state = fpu_ready ? S_FPU_WAIT : S_ISSUE;
                else
                    next_state = S_EXECUTE;
            end
            S_EXECUTE: begin
                if (issue_instr0.is_ecall || issue_instr0.is_ebreak || (issue_instr0.is_csr && csr_fault)) 
                    next_state = S_TRAP;
                else if (issue_instr0.is_load || issue_instr0.is_store ||
                         issue_instr0.is_fp_load || issue_instr0.is_fp_store ||
                         issue_instr0.is_amo ||
                         (issue_valid1 && (issue_instr1.is_load || issue_instr1.is_store || issue_instr1.is_amo)))
                    next_state = mmu_enabled ? S_MEM_TLB : S_MEM_ACCESS;
                else if (issue_instr0.is_sfence_vma) next_state = S_SFENCE;
                else if (issue_instr0.is_fence_i)    next_state = S_FENCE_I;
                else                                 next_state = S_WRITEBACK;
            end
            S_MDU_WAIT:     if (mdu_resp_valid)  next_state = S_WRITEBACK;
            S_FPU_WAIT:     if (fpu_resp_valid)  next_state = S_WRITEBACK;
            S_MEM_TLB: begin
                if (dtlb_page_fault) next_state = S_TRAP;
                else if (dtlb_hit)   next_state = S_MEM_ACCESS;
                else if (ptw_ready)  next_state = S_MEM_TLB_WAIT;
            end
            S_MEM_TLB_WAIT: begin
                if (ptw_resp_valid) begin
                    if (ptw_page_fault) next_state = S_TRAP;
                    else                next_state = S_MEM_TLB;
                end
            end
            S_MEM_ACCESS:   if (dcache_ready)       next_state = S_MEM_WAIT;
            S_MEM_WAIT: begin
                if (dcache_resp_valid) begin
                    if (dcache_resp_error) next_state = S_TRAP;
                    else                   next_state = S_WRITEBACK;
                end
            end
            S_WRITEBACK:    next_state = S_IDLE;
            S_TRAP:         next_state = S_IDLE;
            S_SFENCE:       next_state = S_WRITEBACK;
            S_FENCE_I:      next_state = S_FENCE_I_WAIT;
            S_FENCE_I_WAIT: if (dcache_flush_done) next_state = S_WRITEBACK;
            default:        next_state = S_IDLE;
        endcase
    end

    // ------------------------------------------------------------------------
    // Geração de Write Strobe (Máscara de Bytes)
    // ------------------------------------------------------------------------
    logic [7:0] dmem_wstrb;
    logic [2:0] mem_funct3;
    
    // Descobre qual instrução está indo para a memória
    assign mem_funct3 = mem_is_instr1 ? issue_instr1.funct3 : issue_instr0.funct3;

    always_comb begin
        dmem_wstrb = 8'h00;
        
        // Se for um Store, geramos a máscara baseada no funct3 (tamanho)
        if ((!mem_is_instr1 && issue_instr0.is_store) || 
            ( mem_is_instr1 && issue_instr1.is_store)) begin
            
            case (mem_funct3[1:0])
                2'b00: dmem_wstrb = 8'h01; // SB (1 byte)
                2'b01: dmem_wstrb = 8'h03; // SH (2 bytes)
                2'b10: dmem_wstrb = 8'h0F; // SW (4 bytes)
                2'b11: dmem_wstrb = 8'hFF; // SD (8 bytes)
            endcase
            
            dmem_wstrb = dmem_wstrb << exec_mem_addr[2:0];
        end
    end

    // =========================================================================
    // Output Signals
    // =========================================================================
    logic mret_pending, sret_pending;

    assign backend_ctrl.stall = (state != S_IDLE);
    assign backend_ctrl.flush = (state == S_TRAP) ||
                                (state == S_WRITEBACK && (exec_branch_taken || mret_pending || sret_pending));
    assign backend_ctrl.redirect = backend_ctrl.flush;
    assign backend_ctrl.redirect_pc = (state == S_TRAP)     ? trap_vector[VADDR_WIDTH-1:0] :
                                      (mret_pending)         ? return_pc[VADDR_WIDTH-1:0] :
                                      (sret_pending)         ? return_pc[VADDR_WIDTH-1:0] :
                                                               exec_branch_target;

    assign bp_update.valid       = (state == S_WRITEBACK) && issue_instr0.is_branch;
    assign bp_update.taken       = exec_branch_taken;
    assign bp_update.mispredicted = exec_branch_taken != issue_instr0.bp_pred.taken;
    assign bp_update.pc          = issue_instr0.pc;
    assign bp_update.target      = exec_branch_target;
    assign bp_update.is_call     = issue_instr0.is_jal &&
                                   (issue_instr0.rd == 5'd1 || issue_instr0.rd == 5'd5);
    assign bp_update.is_ret      = issue_instr0.is_jalr &&
                                   (issue_instr0.rs1 == 5'd1 || issue_instr0.rs1 == 5'd5);

    // FIX 3: dcache_we considera dual-issue
    assign dcache_req    = (state == S_MEM_ACCESS);
    assign dcache_we     = is_mem1 ? (issue_instr1.is_store || issue_instr1.is_fp_store) :
                                     (issue_instr0.is_store || issue_instr0.is_fp_store);
    assign dcache_is_amo = is_mem1 ? issue_instr1.is_amo : issue_instr0.is_amo;
    assign dcache_amo_op = is_mem1 ? issue_instr1.funct7[6:2] : issue_instr0.funct7[6:2];
    assign dcache_wstrb  = mem_wstrb_calc;
    assign dcache_wdata  = issue_instr0.is_fp_store ? exec_fp_store_data : exec_store_data;

    assign dtlb_req      = (state == S_MEM_TLB);
    assign dtlb_vpn      = exec_mem_addr[VADDR_WIDTH-1:12];
    assign dtlb_is_store = is_mem1 ?
        (issue_instr1.is_store || issue_instr1.is_fp_store ||
         (issue_instr1.is_amo && !issue_instr1.is_lr)) :
        (issue_instr0.is_store || issue_instr0.is_fp_store ||
         (issue_instr0.is_amo && !issue_instr0.is_lr));

    assign ptw_req = (state == S_MEM_TLB) && !dtlb_hit && !dtlb_page_fault;

    assign mdu_req    = (state == S_ISSUE) && issue_instr0.is_mdu;
    assign mdu_rs1    = issue0_rs1_data;
    assign mdu_rs2    = issue0_rs2_data;
    assign mdu_funct3 = issue_instr0.funct3;
    assign mdu_is_word = issue_instr0.is_alu_w;

    assign fpu_req      = (state == S_ISSUE) && issue_instr0.is_fp &&
                          !issue_instr0.is_fp_load && !issue_instr0.is_fp_store;
    assign fpu_op       = decoded_fpu_op;
    assign fpu_rs1      = issue0_frs1_data;
    assign fpu_rs2      = issue0_frs2_data;
    assign fpu_rs3      = issue0_frs3_data;
    assign fpu_int_rs1  = issue0_rs1_data;
    assign fpu_is_single = issue_instr0.is_fp_single;
    // FIX 1: fpu_rm conectado ao campo rm da instrução decodificada
    assign fpu_rm       = issue_instr0.rm;

    assign csr_req   = (state == S_EXECUTE) && issue_instr0.is_csr;
    assign csr_addr  = issue_instr0.csr_addr;
    assign csr_wdata = (issue_instr0.funct3[2]) ? {59'b0, issue_instr0.rs1} : issue0_rs1_data;
    assign csr_op    = issue_instr0.funct3;

    assign trap_enter = (state == S_TRAP);
    assign mret_exec  = (state == S_WRITEBACK) && mret_pending;
    assign sret_exec  = (state == S_WRITEBACK) && sret_pending;

    assign sfence_valid = (state == S_SFENCE);
    assign sfence_all   = (issue_instr0.rs1 == 5'd0) && (issue_instr0.rs2 == 5'd0);
    assign sfence_vpn   = issue0_rs1_data[VPN_WIDTH-1:0];
    assign sfence_asid  = issue0_rs2_data[15:0];

    assign fence_i_valid = (state == S_FENCE_I) || (state == S_FENCE_I_WAIT);
    assign dcache_flush  = (state == S_FENCE_I);

    assign instr_retired  = (state == S_WRITEBACK);
    assign instr_retired_2 = (state == S_WRITEBACK) && issue_dual;

    // =========================================================================
    // Sequential Logic
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state              <= S_IDLE;
            issue_instr0       <= '0;
            issue_valid0       <= 1'b0;
            issue0_rs1_data    <= '0;
            issue0_rs2_data    <= '0;
            issue0_frs1_data   <= '0;
            issue0_frs2_data   <= '0;
            issue0_frs3_data   <= '0;
            issue_instr1       <= '0;
            issue_valid1       <= 1'b0;
            issue_dual         <= 1'b0;
            issue1_rs1_data    <= '0;
            issue1_rs2_data    <= '0;
            exec_result0       <= '0;
            exec_result1       <= '0;
            exec_fp_result     <= '0;
            exec_mem_addr      <= '0;
            exec_store_data    <= '0;
            exec_fp_store_data <= '0;
            mem_is_instr1      <= 1'b0;
            exec_branch_taken  <= 1'b0;
            exec_branch_target <= '0;
            exec_trap          <= 1'b0;
            exec_trap_cause    <= EXC_ILLEGAL_INSTR;
            exec_trap_value    <= '0;
            mem0_result        <= '0;
            mem1_result        <= '0;
            mem_fp_result      <= '0;
            mem0_rd            <= '0;
            mem1_rd            <= '0;
            mem0_int_we        <= 1'b0;
            mem1_int_we        <= 1'b0;
            mem_fp_we          <= 1'b0;
            trap_cause         <= '0;
            trap_value         <= '0;
            trap_pc            <= '0;
            trap_is_interrupt  <= 1'b0;
            mret_pending       <= 1'b0;
            sret_pending       <= 1'b0;
            dcache_addr        <= '0;
            for (int i = 0; i < 32; i++) begin
                regfile[i]   <= '0;
                fpregfile[i] <= '0;
            end
        end else begin
            state <= next_state;

            case (state)
                S_IDLE: begin
                    mem0_int_we       <= 1'b0;
                    mem1_int_we       <= 1'b0;
                    mem_fp_we         <= 1'b0;
                    exec_trap         <= 1'b0;
                    trap_is_interrupt <= 1'b0;
                    mret_pending      <= 1'b0;
                    sret_pending      <= 1'b0;

                    if (interrupt_pending) begin
                        trap_cause <= interrupt_cause;
                        trap_value <= '0;
                        trap_pc    <= frontend_valid ?
                                      {{(XLEN-VADDR_WIDTH){frontend_in.instr0.pc[VADDR_WIDTH-1]}},
                                       frontend_in.instr0.pc} : trap_pc;
                        trap_is_interrupt <= 1'b1;
                    end else if (frontend_exception) begin
                        trap_cause <= {58'b0, frontend_exception_cause};
                        trap_value <= frontend_exception_value;
                        trap_pc    <= frontend_exception_value;
                    end else if (frontend_valid && frontend_in.instr0_valid) begin
                        issue_instr0     <= frontend_in.instr0;
                        issue_valid0     <= 1'b1;
                        issue0_rs1_data  <= rs1_0_fwd;
                        issue0_rs2_data  <= rs2_0_fwd;
                        issue0_frs1_data <= frs1_forwarded;
                        issue0_frs2_data <= frs2_forwarded;
                        issue0_frs3_data <= frs3_forwarded;
                        issue_instr1     <= frontend_in.instr1;
                        issue_valid1     <= frontend_in.instr1_valid;
                        issue_dual       <= frontend_in.dual_issue;
                        issue1_rs1_data  <= rs1_1_fwd;
                        issue1_rs2_data  <= rs2_1_fwd;
                    end
                end

                S_ISSUE: begin
                    // MDU/FPU requests são combinacionais
                end

                S_EXECUTE: begin
                    if (issue_instr0.is_alu)
                        exec_result0 <= alu0_result;
                    else if (issue_instr0.is_jal || issue_instr0.is_jalr)
                        exec_result0 <= {{(XLEN-VADDR_WIDTH){issue_instr0.pc[VADDR_WIDTH-1]}},
                                         issue_instr0.pc} +
                                        (issue_instr0.is_compressed ? 64'd2 : 64'd4);
                    else if (issue_instr0.is_csr)
                        exec_result0 <= csr_rdata;

                    if (issue_valid1) begin
                        if (issue_instr1.is_alu)
                            exec_result1 <= alu1_result;
                        else if (issue_instr1.is_jal || issue_instr1.is_jalr)
                            exec_result1 <= {{(XLEN-VADDR_WIDTH){issue_instr1.pc[VADDR_WIDTH-1]}},
                                             issue_instr1.pc} +
                                            (issue_instr1.is_compressed ? 64'd2 : 64'd4);
                    end

                    exec_branch_taken  <= branch_taken;
                    exec_branch_target <= branch_target;

                    if (issue_valid1 && is_mem1) begin
                        exec_mem_addr   <= mem_vaddr;
                        exec_store_data <= issue1_rs2_data;
                        mem_is_instr1   <= 1'b1;
                    end else begin
                        exec_mem_addr      <= mem_vaddr;
                        exec_store_data    <= issue0_rs2_data;
                        exec_fp_store_data <= issue0_frs2_data;
                        mem_is_instr1      <= 1'b0;
                    end

                    if (!mmu_enabled)
                        dcache_addr <= {{(PADDR_WIDTH-VADDR_WIDTH){1'b0}}, mem_vaddr};

                    // Traps (instr0 apenas — instr1 só é ALU, sem traps)
                    if (issue_instr0.is_ecall) begin
                        exec_trap <= 1'b1;
                        trap_is_interrupt <= 1'b0;
                        trap_pc   <= {{(XLEN-VADDR_WIDTH){issue_instr0.pc[VADDR_WIDTH-1]}},
                                      issue_instr0.pc};
                        trap_value <= '0;
                        case (current_priv)
                            PRIV_USER:       begin exec_trap_cause <= EXC_ECALL_U; trap_cause <= {58'b0, 6'(EXC_ECALL_U)}; end
                            PRIV_SUPERVISOR: begin exec_trap_cause <= EXC_ECALL_S; trap_cause <= {58'b0, 6'(EXC_ECALL_S)}; end
                            PRIV_MACHINE:    begin exec_trap_cause <= EXC_ECALL_M; trap_cause <= {58'b0, 6'(EXC_ECALL_M)}; end
                            default:         begin exec_trap_cause <= EXC_ECALL_M; trap_cause <= {58'b0, 6'(EXC_ECALL_M)}; end
                        endcase
                    end else if (issue_instr0.is_ebreak) begin
                        exec_trap         <= 1'b1;
                        exec_trap_cause   <= EXC_BREAKPOINT;
                        trap_is_interrupt <= 1'b0;
                        trap_pc    <= {{(XLEN-VADDR_WIDTH){issue_instr0.pc[VADDR_WIDTH-1]}}, issue_instr0.pc};
                        trap_cause <= {58'b0, 6'(EXC_BREAKPOINT)};
                        trap_value <= {{(XLEN-VADDR_WIDTH){issue_instr0.pc[VADDR_WIDTH-1]}}, issue_instr0.pc};
                    end else if (csr_fault && issue_instr0.is_csr) begin
                        exec_trap       <= 1'b1;
                        exec_trap_cause <= EXC_ILLEGAL_INSTR;
                        trap_is_interrupt <= 1'b0;
                        trap_pc    <= {{(XLEN-VADDR_WIDTH){issue_instr0.pc[VADDR_WIDTH-1]}}, issue_instr0.pc};
                        trap_cause <= {58'b0, 6'(EXC_ILLEGAL_INSTR)};
                        trap_value <= {{32{1'b0}}, issue_instr0.opcode, issue_instr0.rd,
                                       issue_instr0.funct3, issue_instr0.rs1, issue_instr0.csr_addr};
                    end

                    if (issue_instr0.is_mret) mret_pending <= 1'b1;
                    if (issue_instr0.is_sret) sret_pending <= 1'b1;
                end

                S_MDU_WAIT: begin
                    if (mdu_resp_valid) exec_result0 <= mdu_result;
                end

                S_FPU_WAIT: begin
                    if (fpu_resp_valid) begin
                        exec_fp_result <= fpu_result;
                        exec_result0   <= fpu_int_result;
                    end
                end

                S_MEM_TLB: begin
                    if (dtlb_hit) begin
                        dcache_addr <= {dtlb_ppn, exec_mem_addr[11:0]};
                    end else if (dtlb_page_fault) begin
                        exec_trap <= 1'b1;
                        trap_is_interrupt <= 1'b0;
                        // FIX 5: trap_pc aponta para a instrução que gerou a fault
                        trap_pc <= mem_is_instr1 ?
                                   {{(XLEN-VADDR_WIDTH){issue_instr1.pc[VADDR_WIDTH-1]}}, issue_instr1.pc} :
                                   {{(XLEN-VADDR_WIDTH){issue_instr0.pc[VADDR_WIDTH-1]}}, issue_instr0.pc};
                        exec_trap_cause <= dcache_we ? EXC_STORE_PAGE_FAULT : EXC_LOAD_PAGE_FAULT;
                        trap_cause <= {58'b0, dcache_we ? 6'(EXC_STORE_PAGE_FAULT) : 6'(EXC_LOAD_PAGE_FAULT)};
                        trap_value <= {{(XLEN-VADDR_WIDTH){exec_mem_addr[VADDR_WIDTH-1]}}, exec_mem_addr};
                    end
                end

                S_MEM_TLB_WAIT: begin
                    if (ptw_resp_valid && ptw_page_fault) begin
                        exec_trap <= 1'b1;
                        trap_is_interrupt <= 1'b0;
                        trap_pc <= mem_is_instr1 ?
                                   {{(XLEN-VADDR_WIDTH){issue_instr1.pc[VADDR_WIDTH-1]}}, issue_instr1.pc} :
                                   {{(XLEN-VADDR_WIDTH){issue_instr0.pc[VADDR_WIDTH-1]}}, issue_instr0.pc};
                        exec_trap_cause <= dcache_we ? EXC_STORE_PAGE_FAULT : EXC_LOAD_PAGE_FAULT;
                        trap_cause <= {58'b0, dcache_we ? 6'(EXC_STORE_PAGE_FAULT) : 6'(EXC_LOAD_PAGE_FAULT)};
                        trap_value <= {{(XLEN-VADDR_WIDTH){exec_mem_addr[VADDR_WIDTH-1]}}, exec_mem_addr};
                    end
                end

                S_MEM_ACCESS: begin end

                S_MEM_WAIT: begin
                    if (dcache_resp_valid) begin
                        if (dcache_resp_error) begin
                            exec_trap <= 1'b1;
                            trap_is_interrupt <= 1'b0;
                            trap_pc <= mem_is_instr1 ?
                                       {{(XLEN-VADDR_WIDTH){issue_instr1.pc[VADDR_WIDTH-1]}}, issue_instr1.pc} :
                                       {{(XLEN-VADDR_WIDTH){issue_instr0.pc[VADDR_WIDTH-1]}}, issue_instr0.pc};
                            exec_trap_cause <= dcache_we ? EXC_STORE_ACCESS_FAULT : EXC_LOAD_ACCESS_FAULT;
                            trap_cause <= {58'b0, dcache_we ? 6'(EXC_STORE_ACCESS_FAULT) : 6'(EXC_LOAD_ACCESS_FAULT)};
                            trap_value <= {{(XLEN-VADDR_WIDTH){exec_mem_addr[VADDR_WIDTH-1]}}, exec_mem_addr};
                        end else if (issue_instr0.is_load || (issue_valid1 && issue_instr1.is_load)) begin
                            mem0_result <= sign_extend_load(dcache_resp_data,
                                          mem_is_instr1 ? issue_instr1.funct3 : issue_instr0.funct3);
                        end else if (issue_instr0.is_fp_load) begin
                            mem_fp_result <= dcache_resp_data;
                        end else if (issue_instr0.is_amo || (issue_valid1 && issue_instr1.is_amo)) begin
                            mem0_result <= dcache_resp_data;
                        end
                    end
                end

                S_WRITEBACK: begin
                    if (!mem_is_instr1) begin
                        
                        // 1. Gravar a Instr 0 (Load/AMO ou ALU)
                        if (issue_instr0.is_load || issue_instr0.is_amo) begin
                            regfile[issue_instr0.rd] <= mem0_result;
                            mem0_rd     <= issue_instr0.rd;
                            mem0_int_we <= 1'b1;
                        // CORREÇÃO: Bloquear a FPU de escrever "lixo" no x1 e x2!
                        // Só escreve se não for FPU, OU se for uma conversão específica de Float->Int
                        end else if (issue_instr0.rd != 5'd0 && !issue_instr0.is_store && !issue_instr0.is_branch &&
                                     (!issue_instr0.is_fp || (decoded_fpu_op inside {FPU_CMP_EQ, FPU_CMP_LT, FPU_CMP_LE, FPU_CVT_W, FPU_CVT_WU, FPU_CVT_L, FPU_CVT_LU, FPU_CLASS, FPU_MV_X_W}))) begin
                            regfile[issue_instr0.rd] <= exec_result0;
                            mem0_rd     <= issue_instr0.rd;
                            mem0_int_we <= 1'b1;
                        end
                        
                        // 2. Gravar a Instr 1 (ALU, etc)
                        if (issue_valid1) begin
                            if (issue_instr1.rd != 5'd0 && !issue_instr1.is_store && !issue_instr1.is_branch && !issue_instr1.is_fp) begin
                                regfile[issue_instr1.rd] <= exec_result1;
                                mem1_rd     <= issue_instr1.rd;
                                mem1_int_we <= 1'b1;
                            end
                        end
                    end else begin
                        
                        // 1. Gravar a Instr 0 (ALU, etc)
                        if (issue_instr0.rd != 5'd0 && !issue_instr0.is_store && !issue_instr0.is_branch &&
                            (!issue_instr0.is_fp || (decoded_fpu_op inside {FPU_CMP_EQ, FPU_CMP_LT, FPU_CMP_LE, FPU_CVT_W, FPU_CVT_WU, FPU_CVT_L, FPU_CVT_LU, FPU_CLASS, FPU_MV_X_W}))) begin
                            regfile[issue_instr0.rd] <= exec_result0;
                            mem0_rd     <= issue_instr0.rd;
                            mem0_int_we <= 1'b1;
                        end
                        
                        // 2. Gravar a Instr 1 (Load/AMO ou ALU falhada)
                        if (issue_instr1.is_load || issue_instr1.is_amo) begin
                            regfile[issue_instr1.rd] <= mem1_result;
                            mem1_rd     <= issue_instr1.rd;
                            mem1_int_we <= 1'b1;
                        end else if (issue_instr1.rd != 5'd0 && !issue_instr1.is_store && !issue_instr1.is_branch && !issue_instr1.is_fp) begin
                            regfile[issue_instr1.rd] <= exec_result1;
                            mem1_rd     <= issue_instr1.rd;
                            mem1_int_we <= 1'b1;
                        end
                    end

                    // FP writeback
                    if (issue_instr0.is_fp_load) begin
                        fpregfile[issue_instr0.rd] <= mem_fp_result;
                        mem0_rd    <= issue_instr0.rd;
                        mem_fp_we  <= 1'b1;
                    end else if (issue_instr0.is_fp &&
                                !(decoded_fpu_op inside {FPU_CMP_EQ, FPU_CMP_LT, FPU_CMP_LE,
                                                         FPU_CVT_W, FPU_CVT_WU, FPU_CVT_L, FPU_CVT_LU,
                                                         FPU_CLASS, FPU_MV_X_W})) begin
                        fpregfile[issue_instr0.rd] <= exec_fp_result;
                        mem_fp_result <= exec_fp_result;
                        mem0_rd       <= issue_instr0.rd;
                        mem_fp_we     <= 1'b1;
                    end
                end

                S_TRAP: begin end

                default: ;
            endcase
        end
    end

endmodule
