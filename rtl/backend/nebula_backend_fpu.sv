`timescale 1ns/1ps
`default_nettype none

import nebula_pkg::*;

/**
 * @module nebula_backend_fpu
 * @brief Backend do Nebula Core com suporte completo a FPU
 *
 * @details
 * Responsabilidades:
 * - Leitura de registradores inteiros e FP
 * - Forwarding de dados
 * - Execução de ALU, Branch, MDU, FPU
 * - Acesso à memória via D-Cache (incluindo FP loads/stores)
 * - Tratamento de CSRs (incluindo FPU CSRs)
 * - Tratamento de Traps/Interrupções
 * - Writeback para registradores inteiros e FP
 *
 * Registradores:
 * - x0-x31: Registradores inteiros (x0 = 0)
 * - f0-f31: Registradores de ponto flutuante
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
    
    // =========================================================================
    // Frontend Interface
    // =========================================================================
    input  wire                     frontend_valid,
    input  frontend_packet_t        frontend_in,
    output backend_ctrl_t           backend_ctrl,
    output bp_update_t              bp_update,
    
    // =========================================================================
    // D-Cache Interface
    // =========================================================================
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
    
    // =========================================================================
    // DTLB Interface
    // =========================================================================
    output logic                    dtlb_req,
    output logic [VPN_WIDTH-1:0]    dtlb_vpn,
    output logic                    dtlb_is_store,
    input  wire                     dtlb_hit,
    input  wire [PPN_WIDTH-1:0]     dtlb_ppn,
    input  wire                     dtlb_page_fault,
    
    // =========================================================================
    // PTW Interface
    // =========================================================================
    output logic                    ptw_req,
    input  wire                     ptw_ready,
    input  wire                     ptw_resp_valid,
    input  wire                     ptw_page_fault,
    
    // =========================================================================
    // CSR Interface
    // =========================================================================
    output logic                    csr_req,
    output logic [11:0]             csr_addr,
    output logic [XLEN-1:0]         csr_wdata,
    output logic [2:0]              csr_op,
    input  wire [XLEN-1:0]          csr_rdata,
    input  wire                     csr_fault,
    
    // =========================================================================
    // Trap Interface
    // =========================================================================
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
    
    // =========================================================================
    // MDU Interface
    // =========================================================================
    output logic                    mdu_req,
    output logic [XLEN-1:0]         mdu_rs1,
    output logic [XLEN-1:0]         mdu_rs2,
    output logic [2:0]              mdu_funct3,
    output logic                    mdu_is_word,
    input  wire                     mdu_ready,
    input  wire                     mdu_resp_valid,
    input  wire [XLEN-1:0]          mdu_result,
    
    // =========================================================================
    // FPU Interface
    // =========================================================================
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
    
    // =========================================================================
    // Frontend Exception Interface
    // =========================================================================
    input  wire                     frontend_exception,
    input  wire [5:0]               frontend_exception_cause,
    input  wire [XLEN-1:0]          frontend_exception_value,

    // =========================================================================
    // Control
    // =========================================================================
    input  wire [1:0]               current_priv,
    input  wire                     mmu_enabled,
    
    // =========================================================================
    // SFENCE.VMA
    // =========================================================================
    output logic                    sfence_valid,
    output logic                    sfence_all,
    output logic [VPN_WIDTH-1:0]    sfence_vpn,
    output logic [15:0]             sfence_asid,
    
    // =========================================================================
    // FENCE.I
    // =========================================================================
    output logic                    fence_i_valid,
    output logic                    dcache_flush,
    input  wire                     dcache_flush_done,
    
    // =========================================================================
    // Performance Counters
    // =========================================================================
    output logic                    instr_retired,
    output logic                    instr_retired_2
);

    // =========================================================================
    // Register Files
    // =========================================================================
    
    logic [XLEN-1:0] regfile [0:31];     // Integer registers
    logic [FLEN-1:0] fpregfile [0:31];   // FP registers
    
    // =========================================================================
    // Pipeline Registers (DUAL ISSUE)
    // =========================================================================
    
    // Issue stage - Instrução 0
    decoded_instr_t  issue_instr0;
    logic            issue_valid0;
    logic [XLEN-1:0] issue0_rs1_data, issue0_rs2_data;
    logic [FLEN-1:0] issue0_frs1_data, issue0_frs2_data, issue0_frs3_data;

    // Issue stage - Instrução 1
    decoded_instr_t  issue_instr1;
    logic            issue_valid1;
    logic            issue_dual; // Indica se estamos executando em modo Dual Issue neste ciclo
    logic [XLEN-1:0] issue1_rs1_data, issue1_rs2_data;
    
    // Execute stage
    logic [XLEN-1:0] exec_result0, exec_result1;
    logic [FLEN-1:0] exec_fp_result; // FPU ainda é single-issue (1 por ciclo)
    
    // Roteamento de Memória (Quem vai usar a memória: Instr0 ou Instr1?)
    logic [VADDR_WIDTH-1:0] exec_mem_addr;
    logic [XLEN-1:0]        exec_store_data;
    logic [FLEN-1:0]        exec_fp_store_data;
    logic                   mem_is_instr1; // Flag para saber quem fez o load/store
    
    logic            exec_branch_taken;
    logic [VADDR_WIDTH-1:0] exec_branch_target;
    logic            exec_trap;
    exception_cause_t exec_trap_cause;
    logic [XLEN-1:0] exec_trap_value;

    // Memory stage
    logic [XLEN-1:0] mem_result;
    logic [FLEN-1:0] mem_fp_result;
    logic [4:0]      mem_rd;
    logic            mem_int_we;
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
    // Forwarding Logic (Multi-ported)
    // =========================================================================
    
    logic [XLEN-1:0] rs1_0_fwd, rs2_0_fwd;
    logic [XLEN-1:0] rs1_1_fwd, rs2_1_fwd;
    logic [FLEN-1:0] frs1_forwarded, frs2_forwarded, frs3_forwarded;
    
    always_comb begin
        // --- Forwarding Instr 0 ---
        if (frontend_in.instr0.rs1 == 5'd0) rs1_0_fwd = 64'd0;
        else if (mem_int_we && mem_rd == frontend_in.instr0.rs1) rs1_0_fwd = mem_result;
        else rs1_0_fwd = regfile[frontend_in.instr0.rs1];

        if (frontend_in.instr0.rs2 == 5'd0) rs2_0_fwd = 64'd0;
        else if (mem_int_we && mem_rd == frontend_in.instr0.rs2) rs2_0_fwd = mem_result;
        else rs2_0_fwd = regfile[frontend_in.instr0.rs2];

        // --- Forwarding Instr 1 ---
        if (frontend_in.instr1.rs1 == 5'd0) rs1_1_fwd = 64'd0;
        else if (mem_int_we && mem_rd == frontend_in.instr1.rs1) rs1_1_fwd = mem_result;
        else rs1_1_fwd = regfile[frontend_in.instr1.rs1];

        if (frontend_in.instr1.rs2 == 5'd0) rs2_1_fwd = 64'd0;
        else if (mem_int_we && mem_rd == frontend_in.instr1.rs2) rs2_1_fwd = mem_result;
        else rs2_1_fwd = regfile[frontend_in.instr1.rs2];

        // FP forwarding (mantido focado na instr0 por enquanto, pois FPU é 1 por ciclo)
        if (mem_fp_we && mem_rd == frontend_in.instr0.rs1) frs1_forwarded = mem_fp_result;
        else frs1_forwarded = fpregfile[frontend_in.instr0.rs1];
        
        if (mem_fp_we && mem_rd == frontend_in.instr0.rs2) frs2_forwarded = mem_fp_result;
        else frs2_forwarded = fpregfile[frontend_in.instr0.rs2];
        
        if (mem_fp_we && mem_rd == frontend_in.instr0.rs3) frs3_forwarded = mem_fp_result;
        else frs3_forwarded = fpregfile[frontend_in.instr0.rs3];
    end
    
    // =========================================================================
    // ALU Execution (DUAL ALU)
    // =========================================================================
    
    logic [XLEN-1:0] alu0_result, alu1_result;
    logic [XLEN-1:0] alu0_op1, alu0_op2, alu1_op1, alu1_op2;
    logic [5:0]      shamt0, shamt1;

    // --- Configuração ALU 0 ---
    assign alu0_op1 = issue0_rs1_data;
    assign alu0_op2 = (issue_instr0.is_alu && issue_instr0.opcode[5] == 1'b0) ? issue_instr0.imm : issue0_rs2_data;
    assign shamt0   = issue_instr0.opcode[5] ? (issue_instr0.is_alu_w ? {1'b0, issue0_rs2_data[4:0]} : issue0_rs2_data[5:0]) :
                                               (issue_instr0.is_alu_w ? {1'b0, issue_instr0.imm[4:0]} : issue_instr0.imm[5:0]);

    // --- Configuração ALU 1 ---
    assign alu1_op1 = issue1_rs1_data;
    assign alu1_op2 = (issue_instr1.is_alu && issue_instr1.opcode[5] == 1'b0) ? issue_instr1.imm : issue1_rs2_data;
    assign shamt1   = issue_instr1.opcode[5] ? (issue_instr1.is_alu_w ? {1'b0, issue1_rs2_data[4:0]} : issue1_rs2_data[5:0]) :
                                               (issue_instr1.is_alu_w ? {1'b0, issue_instr1.imm[4:0]} : issue_instr1.imm[5:0]);

    always_comb begin
        // --- Cálculo ALU 0 ---
        alu0_result = '0;
        case (issue_instr0.funct3)
            3'b000: alu0_result = (issue_instr0.funct7[5] && issue_instr0.opcode[5]) ? (alu0_op1 - alu0_op2) : (alu0_op1 + alu0_op2);
            3'b001: alu0_result = issue_instr0.is_alu_w ? $signed(alu0_op1[31:0] << shamt0[4:0]) : (alu0_op1 << shamt0);
            3'b010: alu0_result = ($signed(alu0_op1) < $signed(alu0_op2)) ? 64'd1 : 64'd0;
            3'b011: alu0_result = (alu0_op1 < alu0_op2) ? 64'd1 : 64'd0;
            3'b100: alu0_result = alu0_op1 ^ alu0_op2;
            3'b101: begin
                if (issue_instr0.funct7[5]) alu0_result = issue_instr0.is_alu_w ? {{32{alu0_op1[31]}}, $signed(alu0_op1[31:0]) >>> shamt0} : ($signed(alu0_op1) >>> shamt0);
                else                        alu0_result = issue_instr0.is_alu_w ? {32'b0, alu0_op1[31:0] >> shamt0} : (alu0_op1 >> shamt0);
            end
            3'b110: alu0_result = alu0_op1 | alu0_op2;
            3'b111: alu0_result = alu0_op1 & alu0_op2;
        endcase
        if (issue_instr0.is_alu_w) alu0_result = {{32{alu0_result[31]}}, alu0_result[31:0]};
        if (issue_instr0.opcode == 7'b0110111) alu0_result = issue_instr0.imm; // LUI
        else if (issue_instr0.opcode == 7'b0010111) alu0_result = issue_instr0.pc + issue_instr0.imm; // AUIPC

        // --- Cálculo ALU 1 ---
        alu1_result = '0;
        if (issue_valid1) begin
            case (issue_instr1.funct3)
                3'b000: alu1_result = (issue_instr1.funct7[5] && issue_instr1.opcode[5]) ? (alu1_op1 - alu1_op2) : (alu1_op1 + alu1_op2);
                3'b001: alu1_result = issue_instr1.is_alu_w ? $signed(alu1_op1[31:0] << shamt1[4:0]) : (alu1_op1 << shamt1);
                3'b010: alu1_result = ($signed(alu1_op1) < $signed(alu1_op2)) ? 64'd1 : 64'd0;
                3'b011: alu1_result = (alu1_op1 < alu1_op2) ? 64'd1 : 64'd0;
                3'b100: alu1_result = alu1_op1 ^ alu1_op2;
                3'b101: begin
                    if (issue_instr1.funct7[5]) alu1_result = issue_instr1.is_alu_w ? {{32{alu1_op1[31]}}, $signed(alu1_op1[31:0]) >>> shamt1} : ($signed(alu1_op1) >>> shamt1);
                    else                        alu1_result = issue_instr1.is_alu_w ? {32'b0, alu1_op1[31:0] >> shamt1} : (alu1_op1 >> shamt1);
                end
                3'b110: alu1_result = alu1_op1 | alu1_op2;
                3'b111: alu1_result = alu1_op1 & alu1_op2;
            endcase
            if (issue_instr1.is_alu_w) alu1_result = {{32{alu1_result[31]}}, alu1_result[31:0]};
            if (issue_instr1.opcode == 7'b0110111) alu1_result = issue_instr1.imm; // LUI
            else if (issue_instr1.opcode == 7'b0010111) alu1_result = issue_instr1.pc + issue_instr1.imm; // AUIPC
        end
    end
    
    // =========================================================================
    // Branch Execution
    // =========================================================================
    
    logic branch_taken;
    logic [VADDR_WIDTH-1:0] branch_target;
    
    always_comb begin
        branch_taken = 1'b0;
        branch_target = issue_instr0.pc + 4;
        if (issue_instr0.is_jal) begin
            branch_taken = 1'b1;
            branch_target = issue_instr0.pc + issue_instr0.imm;
        end
        else if (issue_instr0.is_jalr) begin
            branch_taken = 1'b1;
            branch_target = (issue0_rs1_data + issue_instr0.imm) & ~64'd1;
        end
        else if (issue_instr0.is_branch) begin
            case (issue_instr0.funct3)
                3'b000: branch_taken = (issue0_rs1_data == issue0_rs2_data); // BEQ
                3'b001: branch_taken = (issue0_rs1_data != issue0_rs2_data); // BNE
                3'b100: branch_taken = ($signed(issue0_rs1_data) < $signed(issue0_rs2_data)); // BLT
                3'b101: branch_taken = ($signed(issue0_rs1_data) >= $signed(issue0_rs2_data)); // BGE
                3'b110: branch_taken = (issue0_rs1_data < issue0_rs2_data); // BLTU
                3'b111: branch_taken = (issue0_rs1_data >= issue0_rs2_data); // BGEU
                default: branch_taken = 1'b0;
            endcase
            if (branch_taken)
                branch_target = issue_instr0.pc + issue_instr0.imm;
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
            5'b00100: begin
                case (issue_instr0.funct3)
                    3'b000: decoded_fpu_op = FPU_SGNJ;
                    3'b001: decoded_fpu_op = FPU_SGNJN;
                    3'b010: decoded_fpu_op = FPU_SGNJX;
                    default: ;
                endcase
            end
            5'b00101: decoded_fpu_op = (issue_instr0.funct3 == 3'b000) ? FPU_MIN : FPU_MAX;
            5'b10100: begin
                case (issue_instr0.funct3)
                    3'b010: decoded_fpu_op = FPU_CMP_EQ;
                    3'b001: decoded_fpu_op = FPU_CMP_LT;
                    3'b000: decoded_fpu_op = FPU_CMP_LE;
                    default: ;
                endcase
            end
            5'b11100: decoded_fpu_op = (issue_instr0.funct3 == 3'b001) ? FPU_CLASS : FPU_MV_X_W;
            5'b11110: decoded_fpu_op = FPU_MV_W_X;
            5'b11000: begin
                case (issue_instr0.rs2)
                    5'b00000: decoded_fpu_op = FPU_CVT_W;
                    5'b00001: decoded_fpu_op = FPU_CVT_WU;
                    5'b00010: decoded_fpu_op = FPU_CVT_L;
                    5'b00011: decoded_fpu_op = FPU_CVT_LU;
                    default: ;
                endcase
            end
            5'b11010: decoded_fpu_op = FPU_CVT_INT;
            5'b01000: decoded_fpu_op = (issue_instr0.rs2[0]) ? FPU_CVT_D : FPU_CVT_S;
            default: ;
        endcase
        
        // FMA operations
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
    // Memory Address Calculation
    // =========================================================================
    
    logic [VADDR_WIDTH-1:0] mem_vaddr;
    logic [2:0]             mem_size;
    logic [7:0]             mem_wstrb_calc;
    
    wire is_mem1 = issue_valid1 && (issue_instr1.is_load || issue_instr1.is_store || issue_instr1.is_amo);
    
    assign mem_vaddr = is_mem1 ? (issue1_rs1_data[VADDR_WIDTH-1:0] + issue_instr1.imm[VADDR_WIDTH-1:0]) :
                                 (issue0_rs1_data[VADDR_WIDTH-1:0] + issue_instr0.imm[VADDR_WIDTH-1:0]);
    assign mem_size = is_mem1 ? issue_instr1.funct3 : issue_instr0.funct3;
    
    always_comb begin
        mem_wstrb_calc = 8'b0;
        case (mem_size[1:0])
            2'b00: mem_wstrb_calc = 8'b00000001 << mem_vaddr[2:0]; // Byte
            2'b01: mem_wstrb_calc = 8'b00000011 << mem_vaddr[2:0]; // Halfword
            2'b10: mem_wstrb_calc = 8'b00001111 << mem_vaddr[2:0]; // Word
            2'b11: mem_wstrb_calc = 8'b11111111;                    // Doubleword
        endcase
    end
    
    // =========================================================================
    // Load Data Sign Extension
    // =========================================================================
    
    function automatic logic [XLEN-1:0] sign_extend_load(
        input logic [XLEN-1:0] data,
        input logic [2:0] size
    );
        case (size)
            3'b000: return {{56{data[7]}}, data[7:0]};   // LB
            3'b001: return {{48{data[15]}}, data[15:0]}; // LH
            3'b010: return {{32{data[31]}}, data[31:0]}; // LW
            3'b011: return data;                          // LD
            3'b100: return {56'b0, data[7:0]};           // LBU
            3'b101: return {48'b0, data[15:0]};          // LHU
            3'b110: return {32'b0, data[31:0]};          // LWU
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
                if (interrupt_pending)
                    next_state = S_TRAP;
                else if (frontend_exception)
                    next_state = S_TRAP;
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
                if (exec_trap)
                    next_state = S_TRAP;
                else if (issue_instr0.is_load || issue_instr0.is_store || 
                         issue_instr0.is_fp_load || issue_instr0.is_fp_store ||
                         issue_instr0.is_amo || 
                         (issue_valid1 && (issue_instr1.is_load || issue_instr1.is_store || issue_instr1.is_amo)))
                    next_state = mmu_enabled ? S_MEM_TLB : S_MEM_ACCESS;
                else if (issue_instr0.is_sfence_vma)
                    next_state = S_SFENCE;
                else if (issue_instr0.is_fence_i)
                    next_state = S_FENCE_I;
                else
                    next_state = S_WRITEBACK;
            end
            
            S_MDU_WAIT: begin
                if (mdu_resp_valid)
                    next_state = S_WRITEBACK;
            end
            
            S_FPU_WAIT: begin
                if (fpu_resp_valid)
                    next_state = S_WRITEBACK;
            end
            
            S_MEM_TLB: begin
                if (dtlb_page_fault)
                    next_state = S_TRAP;
                else if (dtlb_hit)
                    next_state = S_MEM_ACCESS;
                else if (ptw_ready)
                    next_state = S_MEM_TLB_WAIT;
            end
            
            S_MEM_TLB_WAIT: begin
                if (ptw_resp_valid) begin
                    if (ptw_page_fault)
                        next_state = S_TRAP;
                    else
                        next_state = S_MEM_TLB;
                end
            end
            
            S_MEM_ACCESS: begin
                if (dcache_ready)
                    next_state = S_MEM_WAIT;
            end
            
            S_MEM_WAIT: begin
                if (dcache_resp_valid) begin
                    if (dcache_resp_error)
                        next_state = S_TRAP;
                    else
                        next_state = S_WRITEBACK;
                end
            end
            
            S_WRITEBACK: begin
                next_state = S_IDLE;
            end
            
            S_TRAP: begin
                next_state = S_IDLE;
            end
            
            S_SFENCE: begin
                next_state = S_WRITEBACK;
            end
            
            S_FENCE_I: begin
                next_state = S_FENCE_I_WAIT;
            end
            
            S_FENCE_I_WAIT: begin
                if (dcache_flush_done)
                    next_state = S_WRITEBACK;
            end
            
            default: next_state = S_IDLE;
        endcase
    end
    
    // =========================================================================
    // Output Signals
    // =========================================================================
    
    // MRET/SRET flags latched from S_EXECUTE for use during S_WRITEBACK
    logic mret_pending, sret_pending;

    // Backend control
    assign backend_ctrl.stall = (state != S_IDLE);
    assign backend_ctrl.flush = (state == S_TRAP) ||
                                (state == S_WRITEBACK && (exec_branch_taken || mret_pending || sret_pending));
    assign backend_ctrl.redirect = backend_ctrl.flush;
    assign backend_ctrl.redirect_pc = (state == S_TRAP) ? trap_vector[VADDR_WIDTH-1:0] :
                                      (mret_pending) ? return_pc[VADDR_WIDTH-1:0] :
                                      (sret_pending) ? return_pc[VADDR_WIDTH-1:0] :
                                      exec_branch_target;

    // Atualização do Branch Predictor (Apenas instr0 pode ser branch)
    assign bp_update.valid = (state == S_WRITEBACK) && issue_instr0.is_branch;
    assign bp_update.taken = exec_branch_taken;
    assign bp_update.mispredicted = exec_branch_taken != issue_instr0.bp_pred.taken;
    assign bp_update.pc = issue_instr0.pc;
    assign bp_update.target = exec_branch_target;
    assign bp_update.is_call = issue_instr0.is_jal && (issue_instr0.rd == 5'd1 || issue_instr0.rd == 5'd5);
    assign bp_update.is_ret = issue_instr0.is_jalr && (issue_instr0.rs1 == 5'd1 || issue_instr0.rs1 == 5'd5);

    // Interface D-Cache (Apenas instr0 pode ser Load/Store)
    assign dcache_req = (state == S_MEM_ACCESS);
    assign dcache_we = issue_instr0.is_store || issue_instr0.is_fp_store;
    assign dcache_is_amo = issue_instr0.is_amo;
    assign dcache_amo_op = issue_instr0.funct7[6:2];
    assign dcache_wstrb = mem_wstrb_calc;
    assign dcache_wdata = issue_instr0.is_fp_store ? exec_fp_store_data : exec_store_data;

    // Interface MDU e FPU (Apenas instr0)
    assign mdu_req = (state == S_ISSUE) && issue_instr0.is_mdu;
    assign mdu_rs1 = issue0_rs1_data;
    assign mdu_rs2 = issue0_rs2_data;
    assign mdu_funct3 = issue_instr0.funct3;
    assign mdu_is_word = issue_instr0.is_alu_w;

    assign fpu_req = (state == S_ISSUE) && issue_instr0.is_fp && 
                     !issue_instr0.is_fp_load && !issue_instr0.is_fp_store;
    assign fpu_op = decoded_fpu_op;
    assign fpu_rs1 = issue0_frs1_data;
    assign fpu_rs2 = issue0_frs2_data;
    assign fpu_rs3 = issue0_frs3_data;
    assign fpu_int_rs1 = issue0_rs1_data;
    assign fpu_is_single = issue_instr0.is_fp_single;

    // Interface CSR
    assign csr_req = (state == S_EXECUTE) && issue_instr0.is_csr;
    assign csr_addr = issue_instr0.csr_addr;
    assign csr_wdata = (issue_instr0.funct3[2]) ? {59'b0, issue_instr0.rs1} : issue0_rs1_data;
    assign csr_op = issue_instr0.funct3;

    // Sinais de Trap
    assign trap_enter = (state == S_TRAP);
    assign mret_exec = (state == S_EXECUTE) && issue_instr0.is_mret;
    assign sret_exec = (state == S_EXECUTE) && issue_instr0.is_sret;

    // Contadores de Performance
    assign instr_retired = (state == S_WRITEBACK);
    assign instr_retired_2 = (state == S_WRITEBACK) && issue_dual;
    
    // =========================================================================
    // Sequential Logic
    // =========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            // Reset Pipeline Regs - Instr 0
            issue_instr0 <= '0;
            issue_valid0 <= 1'b0;
            issue0_rs1_data <= '0;
            issue0_rs2_data <= '0;
            issue0_frs1_data <= '0;
            issue0_frs2_data <= '0;
            issue0_frs3_data <= '0;
            exec_result0 <= '0;
            
            // Reset Pipeline Regs - Instr 1
            issue_instr1 <= '0;
            issue_valid1 <= 1'b0;
            issue_dual   <= 1'b0;
            issue1_rs1_data <= '0;
            issue1_rs2_data <= '0;
            exec_result1 <= '0;

            // Shared Exec & Mem Regs
            exec_fp_result <= '0;
            exec_mem_addr <= '0;
            exec_store_data <= '0;
            exec_fp_store_data <= '0;
            mem_is_instr1 <= 1'b0;
            exec_branch_taken <= 1'b0;
            exec_branch_target <= '0;
            exec_trap <= 1'b0;
            exec_trap_cause <= EXC_ILLEGAL_INSTR;
            exec_trap_value <= '0;
            mem_result <= '0;
            mem_fp_result <= '0;
            mem_rd <= '0;
            mem_int_we <= 1'b0;
            mem_fp_we <= 1'b0;
            
            // System Regs
            trap_cause <= '0;
            trap_value <= '0;
            trap_pc <= '0;
            trap_is_interrupt <= 1'b0;
            mret_pending <= 1'b0;
            sret_pending <= 1'b0;
            dcache_addr <= '0;
            
            // Initialize register files
            for (int i = 0; i < 32; i++) begin
                regfile[i] <= '0;
                fpregfile[i] <= '0;
            end
        end
        else begin
            state <= next_state;
            case (state)
                S_IDLE: begin
                    mem_int_we <= 1'b0;
                    mem_fp_we <= 1'b0;
                    exec_trap <= 1'b0;
                    trap_is_interrupt <= 1'b0;
                    mret_pending <= 1'b0;
                    sret_pending <= 1'b0;
                    
                    if (interrupt_pending) begin
                        // Set trap signals BEFORE entering S_TRAP so CSR sees them
                        trap_cause <= interrupt_cause;
                        trap_value <= '0;
                        trap_pc <= frontend_valid ? {{(XLEN-VADDR_WIDTH){frontend_in.instr0.pc[VADDR_WIDTH-1]}}, frontend_in.instr0.pc} : trap_pc;
                        trap_is_interrupt <= 1'b1;
                    end
                    else if (frontend_exception) begin
                        // Frontend exception (instruction page fault, access fault)
                        trap_cause <= {58'b0, frontend_exception_cause};
                        trap_value <= frontend_exception_value;
                        trap_pc <= frontend_exception_value; // PC that caused the fault
                        trap_is_interrupt <= 1'b0;
                    end
                    else if (frontend_valid && frontend_in.instr0_valid) begin
                        // LATCH INSTR 0
                        issue_instr0 <= frontend_in.instr0;
                        issue_valid0 <= 1'b1;
                        issue0_rs1_data <= rs1_0_fwd;
                        issue0_rs2_data <= rs2_0_fwd;
                        issue0_frs1_data <= frs1_forwarded;
                        issue0_frs2_data <= frs2_forwarded;
                        issue0_frs3_data <= frs3_forwarded;

                        // LATCH INSTR 1 (Apenas se o Frontend liberou via can_dual_issue)
                        issue_instr1 <= frontend_in.instr1;
                        issue_valid1 <= frontend_in.instr1_valid;
                        issue_dual   <= frontend_in.dual_issue;
                        issue1_rs1_data <= rs1_1_fwd;
                        issue1_rs2_data <= rs2_1_fwd;
                    end
                end
                
                S_ISSUE: begin
                    // MDU and FPU requests are combinational (gerados pelas flags da issue_instr0)
                end
                
                S_EXECUTE: begin
                    // ==========================================
                    // 1. ALU e Endereços (Instr 0 e Instr 1)
                    // ==========================================
                    if (issue_instr0.is_alu) 
                        exec_result0 <= alu0_result;
                    else if (issue_instr0.is_jal || issue_instr0.is_jalr) 
                        exec_result0 <= {{(XLEN-VADDR_WIDTH){issue_instr0.pc[VADDR_WIDTH-1]}}, issue_instr0.pc} + (issue_instr0.is_compressed ? 64'd2 : 64'd4);
                    else if (issue_instr0.is_csr) 
                        exec_result0 <= csr_rdata;

                    if (issue_valid1) begin
                        if (issue_instr1.is_alu) 
                            exec_result1 <= alu1_result;
                        else if (issue_instr1.is_jal || issue_instr1.is_jalr) 
                            exec_result1 <= {{(XLEN-VADDR_WIDTH){issue_instr1.pc[VADDR_WIDTH-1]}}, issue_instr1.pc} + (issue_instr1.is_compressed ? 64'd2 : 64'd4);
                    end

                    // ==========================================
                    // 2. Roteamento de Memória (Quem faz Load/Store?)
                    // ==========================================
                    // O hazard unit garantiu que ou a instr0 faz acesso, ou a instr1 faz, ou nenhuma.
                    if (issue_valid1 && (issue_instr1.is_load || issue_instr1.is_store || issue_instr1.is_amo)) begin
                        exec_mem_addr <= issue1_rs1_data[VADDR_WIDTH-1:0] + issue_instr1.imm[VADDR_WIDTH-1:0];
                        exec_store_data <= issue1_rs2_data;
                        mem_is_instr1 <= 1'b1;
                    end else begin
                        exec_mem_addr <= issue0_rs1_data[VADDR_WIDTH-1:0] + issue_instr0.imm[VADDR_WIDTH-1:0];
                        exec_store_data <= issue0_rs2_data;
                        exec_fp_store_data <= issue0_frs2_data;
                        mem_is_instr1 <= 1'b0;
                    end

                    // Physical address (when MMU disabled)
                    if (!mmu_enabled)
                        dcache_addr <= {{(PADDR_WIDTH-VADDR_WIDTH){1'b0}}, exec_mem_addr};

                    // ==========================================
                    // 3. Branches (Apenas Instr0)
                    // ==========================================
                    exec_branch_taken <= branch_taken;
                    exec_branch_target <= branch_target;

                    // ==========================================
                    // 4. Traps, Exceções e CSRs (Apenas Instr0)
                    // ==========================================
                    if (issue_instr0.is_ecall) begin
                        exec_trap <= 1'b1;
                        trap_is_interrupt <= 1'b0;
                        trap_pc <= {{(XLEN-VADDR_WIDTH){issue_instr0.pc[VADDR_WIDTH-1]}}, issue_instr0.pc};
                        trap_value <= '0;
                        case (current_priv)
                            PRIV_USER:       begin exec_trap_cause <= EXC_ECALL_U; trap_cause <= {58'b0, 6'(EXC_ECALL_U)}; end
                            PRIV_SUPERVISOR: begin exec_trap_cause <= EXC_ECALL_S; trap_cause <= {58'b0, 6'(EXC_ECALL_S)}; end
                            PRIV_MACHINE:    begin exec_trap_cause <= EXC_ECALL_M; trap_cause <= {58'b0, 6'(EXC_ECALL_M)}; end
                            default:         begin exec_trap_cause <= EXC_ECALL_M; trap_cause <= {58'b0, 6'(EXC_ECALL_M)}; end
                        endcase
                    end
                    else if (issue_instr0.is_ebreak) begin
                        exec_trap <= 1'b1;
                        exec_trap_cause <= EXC_BREAKPOINT;
                        exec_trap_value <= {{(XLEN-VADDR_WIDTH){issue_instr0.pc[VADDR_WIDTH-1]}}, issue_instr0.pc};
                        trap_is_interrupt <= 1'b0;
                        trap_pc <= {{(XLEN-VADDR_WIDTH){issue_instr0.pc[VADDR_WIDTH-1]}}, issue_instr0.pc};
                        trap_cause <= {58'b0, 6'(EXC_BREAKPOINT)};
                        trap_value <= {{(XLEN-VADDR_WIDTH){issue_instr0.pc[VADDR_WIDTH-1]}}, issue_instr0.pc};
                    end
                    else if (csr_fault && issue_instr0.is_csr) begin
                        exec_trap <= 1'b1;
                        exec_trap_cause <= EXC_ILLEGAL_INSTR;
                        exec_trap_value <= {{32{1'b0}}, issue_instr0.opcode, issue_instr0.rd, issue_instr0.funct3, issue_instr0.rs1, issue_instr0.csr_addr};
                        trap_is_interrupt <= 1'b0;
                        trap_pc <= {{(XLEN-VADDR_WIDTH){issue_instr0.pc[VADDR_WIDTH-1]}}, issue_instr0.pc};
                        trap_cause <= {58'b0, 6'(EXC_ILLEGAL_INSTR)};
                        trap_value <= {{32{1'b0}}, issue_instr0.opcode, issue_instr0.rd, issue_instr0.funct3, issue_instr0.rs1, issue_instr0.csr_addr};
                    end

                    // MRET/SRET
                    if (issue_instr0.is_mret) mret_pending <= 1'b1;
                    if (issue_instr0.is_sret) sret_pending <= 1'b1;
                end

                S_MDU_WAIT: begin
                    if (mdu_resp_valid) exec_result0 <= mdu_result;
                end
                
                S_FPU_WAIT: begin
                    if (fpu_resp_valid) begin
                        exec_fp_result <= fpu_result;
                        exec_result0 <= fpu_int_result; // Conversoes FP->Int e comps
                    end
                end
                
                S_MEM_TLB: begin
                    if (dtlb_hit) begin
                        dcache_addr <= {dtlb_ppn, exec_mem_addr[11:0]};
                    end else if (dtlb_page_fault) begin
                        exec_trap <= 1'b1;
                        exec_trap_cause <= (issue_instr0.is_store || issue_instr0.is_fp_store || issue_instr1.is_store) ? EXC_STORE_PAGE_FAULT : EXC_LOAD_PAGE_FAULT;
                        exec_trap_value <= {{(XLEN-VADDR_WIDTH){exec_mem_addr[VADDR_WIDTH-1]}}, exec_mem_addr};
                        trap_is_interrupt <= 1'b0;
                        trap_pc <= {{(XLEN-VADDR_WIDTH){issue_instr0.pc[VADDR_WIDTH-1]}}, issue_instr0.pc};
                        trap_cause <= {58'b0, (issue_instr0.is_store || issue_instr0.is_fp_store || issue_instr1.is_store) ? 6'(EXC_STORE_PAGE_FAULT) : 6'(EXC_LOAD_PAGE_FAULT)};
                        trap_value <= {{(XLEN-VADDR_WIDTH){exec_mem_addr[VADDR_WIDTH-1]}}, exec_mem_addr};
                    end
                end

                S_MEM_TLB_WAIT: begin
                    if (ptw_resp_valid && ptw_page_fault) begin
                        exec_trap <= 1'b1;
                        exec_trap_cause <= (issue_instr0.is_store || issue_instr0.is_fp_store || issue_instr1.is_store) ? EXC_STORE_PAGE_FAULT : EXC_LOAD_PAGE_FAULT;
                        exec_trap_value <= {{(XLEN-VADDR_WIDTH){exec_mem_addr[VADDR_WIDTH-1]}}, exec_mem_addr};
                        trap_is_interrupt <= 1'b0;
                        trap_pc <= {{(XLEN-VADDR_WIDTH){issue_instr0.pc[VADDR_WIDTH-1]}}, issue_instr0.pc};
                        trap_cause <= {58'b0, (issue_instr0.is_store || issue_instr0.is_fp_store || issue_instr1.is_store) ? 6'(EXC_STORE_PAGE_FAULT) : 6'(EXC_LOAD_PAGE_FAULT)};
                        trap_value <= {{(XLEN-VADDR_WIDTH){exec_mem_addr[VADDR_WIDTH-1]}}, exec_mem_addr};
                    end
                end
                
                S_MEM_ACCESS: begin
                    // Wait for dcache_ready combinational
                end
                
                S_MEM_WAIT: begin
                    if (dcache_resp_valid) begin
                        if (dcache_resp_error) begin
                            exec_trap <= 1'b1;
                            exec_trap_cause <= (issue_instr0.is_store || issue_instr0.is_fp_store || issue_instr1.is_store) ? EXC_STORE_ACCESS_FAULT : EXC_LOAD_ACCESS_FAULT;
                            exec_trap_value <= {{(XLEN-VADDR_WIDTH){exec_mem_addr[VADDR_WIDTH-1]}}, exec_mem_addr};
                            trap_is_interrupt <= 1'b0;
                            trap_pc <= {{(XLEN-VADDR_WIDTH){issue_instr0.pc[VADDR_WIDTH-1]}}, issue_instr0.pc};
                            trap_cause <= {58'b0, (issue_instr0.is_store || issue_instr0.is_fp_store || issue_instr1.is_store) ? 6'(EXC_STORE_ACCESS_FAULT) : 6'(EXC_LOAD_ACCESS_FAULT)};
                            trap_value <= {{(XLEN-VADDR_WIDTH){exec_mem_addr[VADDR_WIDTH-1]}}, exec_mem_addr};
                        end
                        else if (issue_instr0.is_load || issue_instr1.is_load) begin
                            // Apenas uma das instruções chegou até aqui com is_load verdadeiro
                            mem_result <= sign_extend_load(dcache_resp_data, mem_is_instr1 ? issue_instr1.funct3 : issue_instr0.funct3);
                        end
                        else if (issue_instr0.is_fp_load) begin
                            mem_fp_result <= dcache_resp_data;
                        end
                        else if (issue_instr0.is_amo || issue_instr1.is_amo) begin
                            mem_result <= dcache_resp_data;
                        end
                    end
                end
                
                S_WRITEBACK: begin
                    // ==========================================
                    // 1. Writeback - Int Register Instr 0
                    // ==========================================
                    if (issue_instr0.rd != 5'd0) begin
                        if (issue_instr0.is_alu || issue_instr0.is_jal || issue_instr0.is_jalr || issue_instr0.is_csr || issue_instr0.is_mdu) begin
                            regfile[issue_instr0.rd] <= exec_result0;
                            mem_result <= exec_result0;
                            mem_rd <= issue_instr0.rd;
                            mem_int_we <= 1'b1;
                        end
                        else if (!mem_is_instr1 && (issue_instr0.is_load || issue_instr0.is_amo)) begin
                            regfile[issue_instr0.rd] <= mem_result;
                            mem_rd <= issue_instr0.rd;
                            mem_int_we <= 1'b1;
                        end
                        // FP->Int resultados
                        else if (issue_instr0.is_fp && (decoded_fpu_op inside {FPU_CMP_EQ, FPU_CMP_LT, FPU_CMP_LE, FPU_CVT_W, FPU_CVT_WU, FPU_CVT_L, FPU_CVT_LU, FPU_CLASS, FPU_MV_X_W})) begin
                            regfile[issue_instr0.rd] <= exec_result0;
                            mem_result <= exec_result0;
                            mem_rd <= issue_instr0.rd;
                            mem_int_we <= 1'b1;
                        end
                    end
                    
                    // ==========================================
                    // 2. Writeback - Int Register Instr 1 (Simultâneo)
                    // ==========================================
                    if (issue_valid1 && issue_instr1.rd != 5'd0) begin
                        if (issue_instr1.is_alu || issue_instr1.is_jal || issue_instr1.is_jalr) begin
                            regfile[issue_instr1.rd] <= exec_result1;
                            // Se as duas gravam na memória de forwarding (mem_rd/mem_result), instr1 sobrescreve. 
                            // O Hazard Unit já garantiu WAW seguro.
                            mem_result <= exec_result1; 
                            mem_rd <= issue_instr1.rd;
                            mem_int_we <= 1'b1;
                        end 
                        else if (mem_is_instr1 && (issue_instr1.is_load || issue_instr1.is_amo)) begin
                            regfile[issue_instr1.rd] <= mem_result;
                            mem_rd <= issue_instr1.rd;
                            mem_int_we <= 1'b1;
                        end
                    end
                    
                    // ==========================================
                    // 3. Writeback - FP Register (Apenas Instr0)
                    // ==========================================
                    if (issue_instr0.is_fp_load) begin
                        fpregfile[issue_instr0.rd] <= mem_fp_result;
                        mem_rd <= issue_instr0.rd;
                        mem_fp_we <= 1'b1;
                    end
                    else if (issue_instr0.is_fp && !(decoded_fpu_op inside {FPU_CMP_EQ, FPU_CMP_LT, FPU_CMP_LE, FPU_CVT_W, FPU_CVT_WU, FPU_CVT_L, FPU_CVT_LU, FPU_CLASS, FPU_MV_X_W})) begin
                        fpregfile[issue_instr0.rd] <= exec_fp_result;
                        mem_fp_result <= exec_fp_result;
                        mem_rd <= issue_instr0.rd;
                        mem_fp_we <= 1'b1;
                    end
                end
                
                S_TRAP: begin
                    // Tratado nos estados prévios, CSR assume daqui.
                end
                
                default: ;
            endcase
        end
    end

endmodule
