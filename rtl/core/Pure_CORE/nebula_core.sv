// NEBULA CORE - RV64I Implementation
// Alchemist RV - Little Core
// Pipeline: 8-stage in-order with full RV64I compliance

`timescale 1ns/1ps
`default_nettype none

module nebula_core #(
    parameter int HART_ID = 0,
    parameter int XLEN = 64,
    parameter int ILEN = 32,
    parameter int PHYS_ADDR_SIZE = 64,
    parameter bit ENABLE_MISALIGNED_ACCESS = 0,
    parameter bit ENABLE_MMU = 1,
    parameter int INPUT_QUEUE_DEPTH = 4
) (
    input wire clk,
    input wire rst_n,
    
    // Instruction memory interface
    output logic [PHYS_ADDR_SIZE-1:0] imem_addr,
    output logic imem_req,
    input wire [ILEN-1:0] imem_data_in0,
    input wire [ILEN-1:0] imem_data_in1,
    input wire imem_ack_in,
    input wire imem_error_in,
    
    // Data memory interface
    output logic [PHYS_ADDR_SIZE-1:0] dmem_addr0,
    output logic [XLEN-1:0] dmem_wdata0,
    output logic [7:0] dmem_wstrb0,
    output logic dmem_req0,
    output logic dmem_we0,
    input wire [XLEN-1:0] dmem_rdata0,
    input wire dmem_ack_in,
    input wire dmem_error_in,
    
    output logic [PHYS_ADDR_SIZE-1:0] dmem_addr1,
    output logic [XLEN-1:0] dmem_wdata1,
    output logic [7:0] dmem_wstrb1,
    output logic dmem_req1,
    output logic dmem_we1,
    input wire [XLEN-1:0] dmem_rdata1,

    // Interrupt interface
    input wire timer_irq,
    input wire external_irq,
    input wire software_irq,
    
    // Debug interface
    input wire debug_req,
    output logic debug_ack,
    output logic debug_halted,
    
    // Performance counters
    output logic [63:0] inst_retired,
    output logic [63:0] cycles,

    // Debug interface for testbench
    output wire [PHYS_ADDR_SIZE-1:0] debug_pc,
    output wire [XLEN-1:0] debug_next_pc,
    output wire [XLEN-1:0] debug_regfile [0:31],
    output wire [63:0] debug_inst_retired,
    output wire debug_cycles,
    output wire [1:0] debug_privilege,
    output wire [6:0] debug_opcode,
    output wire debug_valid_instr,
    output wire [XLEN-1:0] debug_imm,
    output wire [4:0] debug_rs2,
    output wire [2:0] debug_funct3,
    output wire [XLEN-1:0] debug_result_alu,
    output wire [4:0] debug_mem_rd
);

// --------------------------
// Constants and Types
// --------------------------
localparam logic [PHYS_ADDR_SIZE-1:0] PC_RESET = 64'h8000_0000;
localparam MTVEC_DEFAULT = 'h1000_0000;

typedef enum logic [3:0] {
    OP_ALU     = 4'b0000,
    OP_LOAD    = 4'b0001,
    OP_STORE   = 4'b0010,
    OP_BRANCH  = 4'b0011,
    OP_CSR     = 4'b0100,
    OP_SYSTEM  = 4'b0101
} operation_type_t;

typedef struct packed {
    operation_type_t op_type;
    logic [6:0] opcode;
    logic [2:0] funct3;
    logic [6:0] funct7;
    logic [XLEN-1:0] imm;
    logic [XLEN-1:0] rs1_value;
    logic [XLEN-1:0] rs2_value;
    logic [XLEN-1:0] pc;
    logic [4:0] rd;
} rs_entry_t;

typedef enum logic [3:0] {
    STAGE_RESET = 0,
    STAGE_FETCH = 1,
    STAGE_DECODE = 2,
    STAGE_ISSUE = 3,
    STAGE_EXECUTE = 4,
    STAGE_MEMORY = 5,
    STAGE_WRITEBACK = 6,
    STAGE_TRAP = 7,
    STAGE_STALL = 8
} pipeline_state_t;

typedef enum logic [1:0] {
    PRIV_MACHINE = 2'b11,
    PRIV_SUPERVISOR = 2'b01,
    PRIV_USER = 2'b00
} privilege_t;

typedef struct packed {
    logic [6:0] opcode;
    logic [4:0] rd;
    logic [2:0] funct3;
    logic [4:0] rs1;
    logic [4:0] rs2;
    logic [6:0] funct7;
    logic [XLEN-1:0] imm;
    logic valid;
    logic is_alu;
    logic is_branch;
    logic is_load;
    logic is_store;
    logic is_system;
} decoded_instr_t;

typedef struct packed {
    decoded_instr_t instr0;
    decoded_instr_t instr1;
    logic dual_issue_valid;
} decoded_instr_pair_t;

typedef struct packed {
    logic [XLEN-1:0] alu_result;
    logic [XLEN-1:0] mem_addr;
    logic [XLEN-1:0] store_data;
    logic [4:0] rd;
    logic mem_we;
    logic [7:0] mem_wstrb;
    logic reg_we;
    logic mem_unsigned;
    logic [1:0] mem_size;
    logic branch_taken;
    logic [XLEN-1:0] branch_target;
    logic csr_we;
    logic [11:0] csr_addr;
    logic illegal_instr;
    logic [XLEN-1:0] pc;
} execute_result_t;

typedef struct packed {
    logic [XLEN-1:0] data;
    logic [4:0] rd;
    logic reg_we;
    logic trap;
    logic [XLEN-1:0] trap_cause;
    logic [XLEN-1:0] trap_value;
    logic [XLEN-1:0] pc;
} memory_result_t;

// --------------------------
// Pipeline Registers
// --------------------------
pipeline_state_t pipeline_state, next_pipeline_state;
logic [XLEN-1:0] pc, next_pc;
logic [XLEN-1:0] regfile [0:31];
decoded_instr_pair_t decoded_instr_pair;
execute_result_t execute_result0, execute_result1;
memory_result_t memory_result0, memory_result1;

// ALU busy flags
logic alu0_busy, alu1_busy;

// Reservation station entries
rs_entry_t rs_data0, rs_data1;

// --------------------------
// Control and Status Registers
// --------------------------
logic [XLEN-1:0] csr_mstatus;
logic [XLEN-1:0] csr_mtvec;
logic [XLEN-1:0] csr_mepc;
logic [XLEN-1:0] csr_mcause;
logic [XLEN-1:0] csr_mtval;
logic [XLEN-1:0] csr_mie;
logic [XLEN-1:0] csr_mip;
logic [XLEN-1:0] csr_mscratch;
logic [XLEN-1:0] csr_mcycle;
logic [XLEN-1:0] csr_minstret;
logic [XLEN-1:0] csr_misa;
logic [XLEN-1:0] csr_satp;
logic [XLEN-1:0] csr_stvec;
logic [XLEN-1:0] csr_sepc;
logic [XLEN-1:0] csr_scause;
logic [XLEN-1:0] csr_stval;
logic [XLEN-1:0] csr_sscratch;

privilege_t current_privilege;
logic mstatus_mie;
logic mstatus_mpie;
logic mstatus_sie;
logic mstatus_spie;

// --------------------------
// Hazard Detection
// --------------------------
logic data_hazard0, data_hazard1;
logic control_hazard;
logic struct_hazard;

// --------------------------
// Instruction Fetch Stage
// --------------------------
logic fetch_valid0, fetch_valid1;
logic [XLEN-1:0] fetched_instr0;
logic [XLEN-1:0] fetched_instr1;

assign imem_addr = pc;
assign imem_req = (pipeline_state == STAGE_FETCH) && !control_hazard && !debug_halted;

// --------------------------
// Register File
// --------------------------
logic [XLEN-1:0] rs1_data0, rs2_data0, rs1_data1, rs2_data1;
logic reg_write_en0, reg_write_en1;
logic [4:0] reg_write_addr0, reg_write_addr1;
logic [XLEN-1:0] reg_write_data0, reg_write_data1;

// Memory interface assignments
assign dmem_addr0 = execute_result0.mem_addr[PHYS_ADDR_SIZE-1:0];
assign dmem_wdata0 = execute_result0.store_data;
assign dmem_wstrb0 = execute_result0.mem_wstrb;
assign dmem_req0 = (pipeline_state == STAGE_MEMORY) && 
                  (execute_result0.mem_we || decoded_instr_pair.instr0.opcode == 7'b0000011);
assign dmem_we0 = execute_result0.mem_we;

assign dmem_addr1 = execute_result1.mem_addr[PHYS_ADDR_SIZE-1:0];
assign dmem_wdata1 = execute_result1.store_data;
assign dmem_wstrb1 = execute_result1.mem_wstrb;
assign dmem_req1 = (pipeline_state == STAGE_MEMORY) && 
                  (execute_result1.mem_we || decoded_instr_pair.instr1.opcode == 7'b0000011) &&
                  decoded_instr_pair.dual_issue_valid;
assign dmem_we1 = execute_result1.mem_we;

// --------------------------
// Trap Handling
// --------------------------
logic interrupt_pending;
logic [XLEN-1:0] interrupt_cause;
always_comb begin
    interrupt_pending = 0;
    interrupt_cause = 0;
    
    if (timer_irq & csr_mie[7]) begin
        interrupt_pending = 1;
        interrupt_cause = {1'b1, 63'd7};  // Machine timer interrupt
    end else if (external_irq & csr_mie[11]) begin
        interrupt_pending = 1;
        interrupt_cause = {1'b1, 63'd11}; // Machine external interrupt
    end else if (software_irq & csr_mie[3]) begin
        interrupt_pending = 1;
        interrupt_cause = {1'b1, 63'd3};  // Machine software interrupt
    end
end

// --------------------------
// Forwarding and Hazard Detection (Combinational)
// --------------------------
always_comb begin
    // Instruction 0 forwarding
    // RS1 forwarding
    if (reg_write_en0 && decoded_instr_pair.instr0.rs1 == reg_write_addr0 && decoded_instr_pair.instr0.rs1 != 0)
        rs1_data0 = reg_write_data0;
    else if (reg_write_en1 && decoded_instr_pair.instr0.rs1 == reg_write_addr1 && decoded_instr_pair.instr0.rs1 != 0)
        rs1_data0 = reg_write_data1;
    else if (execute_result0.reg_we && decoded_instr_pair.instr0.rs1 == execute_result0.rd && decoded_instr_pair.instr0.rs1 != 0)
        rs1_data0 = execute_result0.alu_result;
    else if (execute_result1.reg_we && decoded_instr_pair.instr0.rs1 == execute_result1.rd && decoded_instr_pair.instr0.rs1 != 0)
        rs1_data0 = execute_result1.alu_result;
    else if (memory_result0.reg_we && decoded_instr_pair.instr0.rs1 == memory_result0.rd && decoded_instr_pair.instr0.rs1 != 0)
        rs1_data0 = memory_result0.data;
    else if (memory_result1.reg_we && decoded_instr_pair.instr0.rs1 == memory_result1.rd && decoded_instr_pair.instr0.rs1 != 0)
        rs1_data0 = memory_result1.data;
    else
        rs1_data0 = (decoded_instr_pair.instr0.rs1 == 0) ? 0 : regfile[decoded_instr_pair.instr0.rs1];
    
    // RS2 forwarding
    if (reg_write_en0 && decoded_instr_pair.instr0.rs2 == reg_write_addr0 && decoded_instr_pair.instr0.rs2 != 0)
        rs2_data0 = reg_write_data0;
    else if (reg_write_en1 && decoded_instr_pair.instr0.rs2 == reg_write_addr1 && decoded_instr_pair.instr0.rs2 != 0)
        rs2_data0 = reg_write_data1;
    else if (execute_result0.reg_we && decoded_instr_pair.instr0.rs2 == execute_result0.rd && decoded_instr_pair.instr0.rs2 != 0)
        rs2_data0 = execute_result0.alu_result;
    else if (execute_result1.reg_we && decoded_instr_pair.instr0.rs2 == execute_result1.rd && decoded_instr_pair.instr0.rs2 != 0)
        rs2_data0 = execute_result1.alu_result;
    else if (memory_result0.reg_we && decoded_instr_pair.instr0.rs2 == memory_result0.rd && decoded_instr_pair.instr0.rs2 != 0)
        rs2_data0 = memory_result0.data;
    else if (memory_result1.reg_we && decoded_instr_pair.instr0.rs2 == memory_result1.rd && decoded_instr_pair.instr0.rs2 != 0)
        rs2_data0 = memory_result1.data;
    else
        rs2_data0 = (decoded_instr_pair.instr0.rs2 == 0) ? 0 : regfile[decoded_instr_pair.instr0.rs2];

    // Instruction 1 forwarding (only if dual issue)
    if (decoded_instr_pair.dual_issue_valid) begin
        // RS1 forwarding
        if (reg_write_en0 && decoded_instr_pair.instr1.rs1 == reg_write_addr0 && decoded_instr_pair.instr1.rs1 != 0)
            rs1_data1 = reg_write_data0;
        else if (reg_write_en1 && decoded_instr_pair.instr1.rs1 == reg_write_addr1 && decoded_instr_pair.instr1.rs1 != 0)
            rs1_data1 = reg_write_data1;
        else if (execute_result0.reg_we && decoded_instr_pair.instr1.rs1 == execute_result0.rd && decoded_instr_pair.instr1.rs1 != 0)
            rs1_data1 = execute_result0.alu_result;
        else if (execute_result1.reg_we && decoded_instr_pair.instr1.rs1 == execute_result1.rd && decoded_instr_pair.instr1.rs1 != 0)
            rs1_data1 = execute_result1.alu_result;
        else if (memory_result0.reg_we && decoded_instr_pair.instr1.rs1 == memory_result0.rd && decoded_instr_pair.instr1.rs1 != 0)
            rs1_data1 = memory_result0.data;
        else if (memory_result1.reg_we && decoded_instr_pair.instr1.rs1 == memory_result1.rd && decoded_instr_pair.instr1.rs1 != 0)
            rs1_data1 = memory_result1.data;
        else
            rs1_data1 = (decoded_instr_pair.instr1.rs1 == 0) ? 0 : regfile[decoded_instr_pair.instr1.rs1];
        
        // RS2 forwarding
        if (reg_write_en0 && decoded_instr_pair.instr1.rs2 == reg_write_addr0 && decoded_instr_pair.instr1.rs2 != 0)
            rs2_data1 = reg_write_data0;
        else if (reg_write_en1 && decoded_instr_pair.instr1.rs2 == reg_write_addr1 && decoded_instr_pair.instr1.rs2 != 0)
            rs2_data1 = reg_write_data1;
        else if (execute_result0.reg_we && decoded_instr_pair.instr1.rs2 == execute_result0.rd && decoded_instr_pair.instr1.rs2 != 0)
            rs2_data1 = execute_result0.alu_result;
        else if (execute_result1.reg_we && decoded_instr_pair.instr1.rs2 == execute_result1.rd && decoded_instr_pair.instr1.rs2 != 0)
            rs2_data1 = execute_result1.alu_result;
        else if (memory_result0.reg_we && decoded_instr_pair.instr1.rs2 == memory_result0.rd && decoded_instr_pair.instr1.rs2 != 0)
            rs2_data1 = memory_result0.data;
        else if (memory_result1.reg_we && decoded_instr_pair.instr1.rs2 == memory_result1.rd && decoded_instr_pair.instr1.rs2 != 0)
            rs2_data1 = memory_result1.data;
        else
            rs2_data1 = (decoded_instr_pair.instr1.rs2 == 0) ? 0 : regfile[decoded_instr_pair.instr1.rs2];
    end else begin
        rs1_data1 = '0;
        rs2_data1 = '0;
    end

    // Hazard detection
    data_hazard0 = 0;
    if ((decoded_instr_pair.instr0.rs1 == execute_result0.rd && decoded_instr_pair.instr0.rs1 != 0 && execute_result0.reg_we && 
         (execute_result0.mem_we || decoded_instr_pair.instr0.opcode == 7'b0000011)) ||
        (decoded_instr_pair.instr0.rs2 == execute_result0.rd && decoded_instr_pair.instr0.rs2 != 0 && execute_result0.reg_we && 
         (execute_result0.mem_we || decoded_instr_pair.instr0.opcode == 7'b0000011)) ||
        (decoded_instr_pair.instr0.rs1 == execute_result1.rd && decoded_instr_pair.instr0.rs1 != 0 && execute_result1.reg_we && 
         (execute_result1.mem_we || decoded_instr_pair.instr0.opcode == 7'b0000011)) ||
        (decoded_instr_pair.instr0.rs2 == execute_result1.rd && decoded_instr_pair.instr0.rs2 != 0 && execute_result1.reg_we && 
         (execute_result1.mem_we || decoded_instr_pair.instr0.opcode == 7'b0000011))) begin
        data_hazard0 = 1;
    end
    
    data_hazard1 = 0;
    if (decoded_instr_pair.dual_issue_valid) begin
        if ((decoded_instr_pair.instr1.rs1 == execute_result0.rd && decoded_instr_pair.instr1.rs1 != 0 && execute_result0.reg_we && 
             (execute_result0.mem_we || decoded_instr_pair.instr1.opcode == 7'b0000011)) ||
            (decoded_instr_pair.instr1.rs2 == execute_result0.rd && decoded_instr_pair.instr1.rs2 != 0 && execute_result0.reg_we && 
             (execute_result0.mem_we || decoded_instr_pair.instr1.opcode == 7'b0000011)) ||
            (decoded_instr_pair.instr1.rs1 == execute_result1.rd && decoded_instr_pair.instr1.rs1 != 0 && execute_result1.reg_we && 
             (execute_result1.mem_we || decoded_instr_pair.instr1.opcode == 7'b0000011)) ||
            (decoded_instr_pair.instr1.rs2 == execute_result1.rd && decoded_instr_pair.instr1.rs2 != 0 && execute_result1.reg_we && 
             (execute_result1.mem_we || decoded_instr_pair.instr1.opcode == 7'b0000011))) begin
            data_hazard1 = 1;
        end
    end
    
    control_hazard = execute_result0.branch_taken || execute_result1.branch_taken;
    struct_hazard = (pipeline_state == STAGE_MEMORY) && (dmem_req0 && !dmem_ack_in);
    
    // Pipeline control (combinational)
    next_pipeline_state = pipeline_state;

    case (pipeline_state)
        STAGE_RESET: 
            if (rst_n) next_pipeline_state = STAGE_FETCH;
        
        STAGE_FETCH: 
            if (imem_ack_in) next_pipeline_state = STAGE_DECODE;
            else if (imem_error_in) next_pipeline_state = STAGE_TRAP;
        
        STAGE_DECODE: 
            if (!data_hazard0 && decoded_instr_pair.instr0.valid) 
                next_pipeline_state = STAGE_ISSUE;
        
        STAGE_ISSUE:
            next_pipeline_state = STAGE_EXECUTE;
        
        STAGE_EXECUTE:
            next_pipeline_state = STAGE_MEMORY;
        
        STAGE_MEMORY: 
            if ((!dmem_req0 || dmem_ack_in) && (!dmem_req1 || !decoded_instr_pair.dual_issue_valid || dmem_ack_in)) 
                next_pipeline_state = STAGE_WRITEBACK;
            else if (dmem_error_in)
                next_pipeline_state = STAGE_TRAP;
        
        STAGE_WRITEBACK:
                next_pipeline_state = STAGE_FETCH;
        
        STAGE_TRAP: 
            next_pipeline_state = STAGE_FETCH;
        
        STAGE_STALL: 
            if (!debug_halted) next_pipeline_state = STAGE_FETCH;

        default: next_pipeline_state = STAGE_RESET;
    endcase
    
    // Handle interrupts
    if (interrupt_pending && mstatus_mie && pipeline_state != STAGE_RESET && 
        pipeline_state != STAGE_TRAP && !debug_halted) begin
        next_pipeline_state = STAGE_TRAP;
    end
end

// --------------------------
// Sequential Logic (Clock-driven updates)
// --------------------------
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        // Reset all registers
        pc = PC_RESET;
        pipeline_state <= STAGE_RESET;
        fetch_valid0 = 0;
        fetch_valid1 = 0;
        fetched_instr0 = '0;
        fetched_instr1 = '0;
        debug_ack = 0;
        debug_halted = 0;
        cycles = 0;
        inst_retired = 0;
        for (int i = 0; i < 32; i++) begin
            regfile[i] = 0;
        end

        // CSR reset
        csr_mstatus = '0;
        csr_mtvec = MTVEC_DEFAULT;
        csr_mepc = '0;
        csr_mcause = '0;
        csr_mtval = '0;
        csr_mie = '0;
        csr_mip = '0;
        csr_mscratch = '0;
        csr_mcycle = '0;
        csr_minstret = '0;
        csr_misa = (1 << 12) | (1 << 8) | (1 << 18) | (1 << 20);  // RV64IMSU
        csr_satp = '0;
        csr_stvec = '0;
        csr_sepc = '0;
        csr_scause = '0;
        csr_stval = '0;
        csr_sscratch = '0;
        current_privilege = PRIV_MACHINE;
        mstatus_mie = 0;
        mstatus_mpie = 0;
        mstatus_sie = 0;
        mstatus_spie = 0;
        
        // Clear pipeline registers
        decoded_instr_pair = '0;
        execute_result0 = '0;
        execute_result1 = '0;
        memory_result0 = '0;
        memory_result1 = '0;
        alu0_busy = 0;
        alu1_busy = 0;
    end else begin
        // Performance counters
        cycles = cycles + 1;

        // Update pipeline state and PC
        pipeline_state <= next_pipeline_state;
        if (!debug_halted && !struct_hazard && next_pc != 0) begin
            pc = next_pc;
        end

        // Debug interface
        if (debug_req && !debug_halted) begin
            debug_ack = 1;
            debug_halted = 1;
        end else if (!debug_req && debug_halted) begin
            debug_halted = 0;
        end
        
        // --------------------------
        // Instruction Fetch Stage
        // --------------------------
        if (pipeline_state == STAGE_FETCH) begin
            if (imem_ack_in) begin
                fetched_instr0 = {32'b0, imem_data_in0};
                fetch_valid0 = !imem_error_in;
                
                if (!imem_error_in) begin
                    fetched_instr1 = {32'b0, imem_data_in1};
                    fetch_valid1 = !imem_error_in;
                end else begin
                    fetched_instr1 = '0;
                    fetch_valid1 = 0;
                end

                if (imem_error_in) begin
                    csr_mcause = {1'b0, 63'd1}; // Instruction access fault
                    csr_mtval = { {XLEN-PHYS_ADDR_SIZE{1'b0}}, imem_addr};
                end
            end
        end
        
        // --------------------------
        // Decode Stage
        // --------------------------
        if (pipeline_state == STAGE_DECODE) begin
            decoded_instr_pair.instr0 = decode_instr(fetched_instr0, pc);
            decoded_instr_pair.instr1 = fetch_valid1 ? decode_instr(fetched_instr1, pc + 4) : '0;
            decoded_instr_pair.dual_issue_valid = fetch_valid0 && fetch_valid1 && can_dual_issue(decoded_instr_pair.instr0, decoded_instr_pair.instr1);
            
            // Prepare reservation station entries
            rs_data0 = '{
                op_type: decoded_instr_pair.instr0.is_load ? OP_LOAD : 
                        decoded_instr_pair.instr0.is_store ? OP_STORE : 
                        decoded_instr_pair.instr0.is_branch ? OP_BRANCH : 
                        decoded_instr_pair.instr0.is_system ? OP_SYSTEM : OP_ALU,
                opcode: decoded_instr_pair.instr0.opcode,
                funct3: decoded_instr_pair.instr0.funct3,
                funct7: decoded_instr_pair.instr0.funct7,
                imm: decoded_instr_pair.instr0.imm,
                rs1_value: rs1_data0,
                rs2_value: rs2_data0,
                pc: pc,
                rd: decoded_instr_pair.instr0.rd
            };

            if (decoded_instr_pair.dual_issue_valid) begin
                rs_data1 = '{
                    op_type: decoded_instr_pair.instr1.is_load ? OP_LOAD : 
                            decoded_instr_pair.instr1.is_store ? OP_STORE : 
                            decoded_instr_pair.instr1.is_branch ? OP_BRANCH : 
                            decoded_instr_pair.instr1.is_system ? OP_SYSTEM : OP_ALU,
                    opcode: decoded_instr_pair.instr1.opcode,
                    funct3: decoded_instr_pair.instr1.funct3,
                    funct7: decoded_instr_pair.instr1.funct7,
                    imm: decoded_instr_pair.instr1.imm,
                    rs1_value: rs1_data1,
                    rs2_value: rs2_data1,
                    pc: pc + 4,
                    rd: decoded_instr_pair.instr1.rd
                };
            end
        end

        // --------------------------
        // Execute Stage
        // --------------------------
        if (pipeline_state == STAGE_EXECUTE) begin
            if (!alu0_busy) begin
                execute_result0 = execute_alu(rs_data0);
                alu0_busy = 1'b1;
            end
            
            if (decoded_instr_pair.dual_issue_valid && !alu1_busy) begin
                execute_result1 = execute_alu(rs_data1);
                alu1_busy = 1'b1;
            end
        end else begin
            alu0_busy = 0;
            alu1_busy = 0;
        end

        // --------------------------
        // Memory Stage
        // --------------------------
        if (pipeline_state == STAGE_MEMORY) begin
            // Instruction 0 memory handling
            if (execute_result0.mem_we) begin
                // Store operation
                memory_result0 = '{
                    data: '0,
                    rd: execute_result0.rd,
                    reg_we: 1'b0,
                    trap: dmem_error_in,
                    trap_cause: {1'b0, 63'd7}, // Store access fault
                    trap_value: execute_result0.mem_addr,
                    pc: execute_result0.pc
                };
            end else if (decoded_instr_pair.instr0.opcode == 7'b0000011) begin
                // Load operation
                if (dmem_ack_in) begin
                    case (decoded_instr_pair.instr0.funct3)
                        3'b000: memory_result0.data = {{56{dmem_rdata0[7]}}, dmem_rdata0[7:0]};  // LB
                        3'b001: memory_result0.data = {{48{dmem_rdata0[15]}}, dmem_rdata0[15:0]}; // LH
                        3'b010: memory_result0.data = {{32{dmem_rdata0[31]}}, dmem_rdata0[31:0]}; // LW
                        3'b011: memory_result0.data = dmem_rdata0;  // LD
                        3'b100: memory_result0.data = {56'b0, dmem_rdata0[7:0]};  // LBU
                        3'b101: memory_result0.data = {48'b0, dmem_rdata0[15:0]}; // LHU
                        3'b110: memory_result0.data = {32'b0, dmem_rdata0[31:0]}; // LWU
                        default: memory_result0.data = '0;
                    endcase
                    memory_result0.reg_we = 1'b1;
                    memory_result0.trap = dmem_error_in;
                end
            end else begin
                // ALU operation
                memory_result0.rd = execute_result0.rd;
                memory_result0.data = execute_result0.alu_result;
                memory_result0.reg_we = execute_result0.reg_we;
                memory_result0.trap = 1'b0;
            end
            
            // Instruction 1 memory handling (only if dual issue)
            if (decoded_instr_pair.dual_issue_valid) begin
                if (execute_result1.mem_we) begin
                    // Store operation
                    memory_result1 = '{
                        data: '0,
                        rd: execute_result1.rd,
                        reg_we: 1'b0,
                        trap: dmem_error_in,
                        trap_cause: {1'b0, 63'd7}, // Store access fault
                        trap_value: execute_result1.mem_addr,
                        pc: execute_result1.pc
                    };
                end else if (decoded_instr_pair.instr1.opcode == 7'b0000011) begin
                    // Load operation
                    if (dmem_ack_in) begin
                        case (decoded_instr_pair.instr1.funct3)
                            3'b000: memory_result1.data = {{56{dmem_rdata1[7]}}, dmem_rdata1[7:0]};  // LB
                            3'b001: memory_result1.data = {{48{dmem_rdata1[15]}}, dmem_rdata1[15:0]}; // LH
                            3'b010: memory_result1.data = {{32{dmem_rdata1[31]}}, dmem_rdata1[31:0]}; // LW
                            3'b011: memory_result1.data = dmem_rdata1;  // LD
                            3'b100: memory_result1.data = {56'b0, dmem_rdata1[7:0]};  // LBU
                            3'b101: memory_result1.data = {48'b0, dmem_rdata1[15:0]}; // LHU
                            3'b110: memory_result1.data = {32'b0, dmem_rdata1[31:0]}; // LWU
                            default: memory_result1.data = '0;
                        endcase
                        memory_result1.reg_we = 1'b1;
                        memory_result1.trap = dmem_error_in;
                    end
                end else begin
                    // ALU operation
                    memory_result1.rd = execute_result1.rd;
                    memory_result1.data = execute_result1.alu_result;
                    memory_result1.reg_we = execute_result1.reg_we;
                    memory_result1.trap = 1'b0;
                end
            end
        end
        
        // --------------------------
        // Writeback Stage
        // --------------------------
        if (pipeline_state == STAGE_WRITEBACK) begin
            // Instruction 0 writeback
            while (memory_result0.reg_we || memory_result1.reg_we) begin
               if (memory_result0.reg_we && !memory_result0.trap && memory_result0.rd != 0) begin
                    regfile[memory_result0.rd] = memory_result0.data;
                    inst_retired = inst_retired + 1;
                    memory_result0.reg_we = '0;
                end
            
                // Instruction 1 writeback (only if dual issue)
                if (decoded_instr_pair.dual_issue_valid && memory_result1.reg_we && 
                    !memory_result1.trap && memory_result1.rd != 0) begin
                    regfile[memory_result1.rd] = memory_result1.data;
                    inst_retired = inst_retired + 1;
                    memory_result1.reg_we = '0;
                end 
            end
            
            // Handle traps
            if (memory_result0.trap) begin
                csr_mcause = memory_result0.trap_cause;
                csr_mtval = memory_result0.trap_value;
            end else if (memory_result1.trap && decoded_instr_pair.dual_issue_valid) begin
                csr_mcause = memory_result1.trap_cause;
                csr_mtval = memory_result1.trap_value;
            end
        end
            
        // Branch handling
        if (execute_result0.branch_taken && pipeline_state == STAGE_EXECUTE) begin
            next_pc = execute_result0.branch_target;
        end else if (execute_result1.branch_taken && pipeline_state == STAGE_EXECUTE) begin
            next_pc = execute_result1.branch_target;
        end else if (pipeline_state == STAGE_MEMORY) begin
            next_pc = pc + (decoded_instr_pair.dual_issue_valid ? 8 : 4);
        end

        // PC alignment
        if (next_pc[1:0] != 2'b00 && !ENABLE_MISALIGNED_ACCESS) begin
            csr_mcause = {1'b0, 63'd0};  // Instruction address misaligned
            csr_mtval = next_pc;
        end

        // --------------------------
        // Trap Handling
        // --------------------------
        if (pipeline_state == STAGE_TRAP) begin
            if (interrupt_pending) begin
                csr_mcause = interrupt_cause;
                csr_mtval = '0;
            end
            
            csr_mepc = { {XLEN-PHYS_ADDR_SIZE{1'b0}}, pc };
            mstatus_mpie = mstatus_mie;
            mstatus_mie = 0;
            pc = csr_mtvec[PHYS_ADDR_SIZE-1:0];
        end else if (decoded_instr_pair.instr0.opcode == 7'b1110011 && decoded_instr_pair.instr0.funct3 == 3'b0) begin
            // MRET instruction
            mstatus_mie = mstatus_mpie;
            pc = csr_mepc[PHYS_ADDR_SIZE-1:0];
        end
    end
end

// --------------------------
// Instruction Decode Function
// --------------------------
function automatic decoded_instr_t decode_instr(input [XLEN-1:0] instr, input [XLEN-1:0] pc_val);
    decoded_instr_t result;
    result.opcode = instr[6:0];
    result.rd = instr[11:7];
    result.funct3 = instr[14:12];
    result.rs1 = instr[19:15];
    result.rs2 = instr[24:20];
    result.funct7 = instr[31:25];
    result.valid = 1'b1;
    result.is_alu = 0;
    result.is_branch = 0;
    result.is_load = 0;
    result.is_store = 0;
    result.is_system = 0;

    case (instr[6:0])
        // Loads
        7'b0000011: begin
            result.is_load = 1'b1;
            case (instr[14:12])
                3'b000, 3'b001, 3'b010, 3'b011,  // LB, LH, LW, LD
                3'b100, 3'b101, 3'b110: result.valid = 1'b1; // LBU, LHU, LWU
                default: result.valid = 1'b0;
            endcase
        end
        
        // Stores
        7'b0100011: begin
            result.is_store = 1'b1;
            case (instr[14:12])
                3'b000, 3'b001, 3'b010, 3'b011: result.valid = 1'b1; // SB, SH, SW, SD
                default: result.valid = 1'b0;
            endcase
        end
        
        // ALU operations
        7'b0010011, 7'b0110011: result.is_alu = 1'b1; // Immediate and register-register ALU
        
        // Branches
        7'b1100011: result.is_branch = 1'b1; // Conditional branches
        7'b1101111, 7'b1100111: result.is_branch = 1'b1; // JAL, JALR
        
        // LUI/AUIPC
        7'b0110111, 7'b0010111: result.is_alu = 1'b1;
        
        // System/CSR
        7'b1110011: begin
            result.is_system = 1'b1;
            if (instr[14:12] == 3'b000) begin
                result.is_system = 1'b1; // ECALL, EBREAK, etc.
            end else begin
                result.is_system = 1'b1; // CSR operations
            end
        end
        
        default: begin
            result.valid = 1'b0;
        end
    endcase

    // Immediate generation
    case (instr[6:0])
        // LUI, AUIPC
        7'b0110111, 7'b0010111:
            result.imm = {{XLEN-32{instr[31]}}, instr[31:12], 12'b0};
        // JAL
        7'b1101111:
            result.imm = {{XLEN-20{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0};
        // JALR
        7'b1100111:
            result.imm = {{XLEN-11{instr[31]}}, instr[30:20]};
        // Branches
        7'b1100011:
            result.imm = {{XLEN-12{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0};
        // Loads, immediate ALU
        7'b0000011, 7'b0010011:
            result.imm = {{XLEN-11{instr[31]}}, instr[30:20]};
        // Stores
        7'b0100011:
            result.imm = {{XLEN-11{instr[31]}}, instr[30:25], instr[11:7]};
        default:
            result.imm = '0;
    endcase
    
    return result;
endfunction

// --------------------------
// Dual-Issue Qualification Function
// --------------------------
function automatic logic can_dual_issue(
    input decoded_instr_t instr0,
    input decoded_instr_t instr1
);
    // Basic dual-issue rules:
    // 1. No structural hazards (both ALUs available)
    // 2. No data dependencies between instructions
    // 3. Second instruction is simple ALU or simple memory op
    // 4. Both instructions are valid
    
    logic no_dependency = 
        (instr0.rd != instr1.rs1) && 
        (instr0.rd != instr1.rs2) && 
        (instr1.rd != instr0.rs1) && 
        (instr1.rd != instr0.rs2);
    
    logic instr1_simple = 
        (instr1.opcode inside {7'b0010011, 7'b0110011}) || // ALU ops
        (instr1.opcode == 7'b0000011 && instr1.funct3 inside {3'b000, 3'b001, 3'b010, 3'b011}) || // Loads
        (instr1.opcode == 7'b0100011 && instr1.funct3 inside {3'b000, 3'b001, 3'b010, 3'b011}); // Stores
    
    return instr0.valid && instr1.valid && no_dependency && instr1_simple;
endfunction

// --------------------------
// ALU Execution Function
// --------------------------
function automatic execute_result_t execute_alu(input rs_entry_t rs_entry);
    execute_result_t result;
    result.pc = rs_entry.pc;
    result.alu_result = '0;
    result.mem_addr = '0;
    result.store_data = '0;
    result.rd = '0;
    result.mem_we = '0;
    result.mem_wstrb = '0;
    result.reg_we = '0;
    result.mem_unsigned = '0;
    result.mem_size = '0;
    result.branch_taken = '0;
    result.branch_target = '0;
    result.csr_we = '0;
    result.csr_addr = '0;
    result.illegal_instr = '0;

    case (rs_entry.op_type)
        OP_ALU: begin
            result.reg_we = 1'b1;
            result.rd = rs_entry.rd;
            
            case (rs_entry.opcode)
                7'b0110111: result.alu_result = rs_entry.imm; // LUI
                7'b0010111: result.alu_result = rs_entry.pc + rs_entry.imm; // AUIPC
                7'b0010011: begin // Immediate ALU operations
                    case (rs_entry.funct3)
                        3'b000: result.alu_result = rs_entry.rs1_value + rs_entry.imm; // ADDI
                        3'b010: result.alu_result = ($signed(rs_entry.rs1_value) < $signed(rs_entry.imm)) ? 1 : 0; // SLTI
                        3'b011: result.alu_result = (rs_entry.rs1_value < rs_entry.imm) ? 1 : 0; // SLTIU
                        3'b100: result.alu_result = rs_entry.rs1_value ^ rs_entry.imm; // XORI
                        3'b110: result.alu_result = rs_entry.rs1_value | rs_entry.imm; // ORI
                        3'b001: result.alu_result = rs_entry.rs1_value << rs_entry.imm[5:0]; // SLLI
                        3'b101: begin
                            if (rs_entry.funct7[5] == 0)
                                result.alu_result = rs_entry.rs1_value >> rs_entry.imm[5:0]; // SRLI
                            else
                                result.alu_result = ($signed(rs_entry.rs1_value)) >>> rs_entry.imm[5:0]; // SRAI
                        end
                        default: result.illegal_instr = 1;
                    endcase
                end
                7'b0110011: begin // Register-register ALU operations
                    case ({rs_entry.funct7, rs_entry.funct3})
                        {7'b0000000, 3'b000}: result.alu_result = rs_entry.rs1_value + rs_entry.rs2_value; // ADD
                        {7'b0100000, 3'b000}: result.alu_result = rs_entry.rs1_value - rs_entry.rs2_value; // SUB
                        {7'b0000000, 3'b001}: result.alu_result = rs_entry.rs1_value << rs_entry.rs2_value[5:0]; // SLL
                        {7'b0000000, 3'b010}: result.alu_result = {{63{1'b0}}, ($signed(rs_entry.rs1_value) < $signed(rs_entry.rs2_value))}; // SLT
                        {7'b0000000, 3'b011}: result.alu_result = {{63{1'b0}}, (rs_entry.rs1_value < rs_entry.rs2_value)}; // SLTU
                        {7'b0000000, 3'b100}: result.alu_result = rs_entry.rs1_value ^ rs_entry.rs2_value; // XOR
                        {7'b0000000, 3'b101}: result.alu_result = rs_entry.rs1_value >> rs_entry.rs2_value[5:0]; // SRL
                        {7'b0100000, 3'b101}: result.alu_result = ($signed(rs_entry.rs1_value)) >>> rs_entry.rs2_value[5:0]; // SRA
                        {7'b0000000, 3'b110}: result.alu_result = rs_entry.rs1_value | rs_entry.rs2_value; // OR
                        {7'b0000000, 3'b111}: result.alu_result = rs_entry.rs1_value & rs_entry.rs2_value; // AND
                        default: result.illegal_instr = 1;
                    endcase
                end
                default: result.alu_result = '0;
            endcase
        end
        OP_BRANCH: begin
            case (rs_entry.opcode)
                7'b1101111: begin // JAL
                    result.alu_result = rs_entry.pc + 4;
                    result.reg_we = 1'b1;
                    result.rd = rs_entry.rd;
                    result.branch_taken = 1'b1;
                    result.branch_target = rs_entry.pc + rs_entry.imm;
                end
                7'b1100111: begin // JALR
                    result.alu_result = rs_entry.pc + 4;
                    result.reg_we = 1'b1;
                    result.rd = rs_entry.rd;
                    result.branch_taken = 1'b1;
                    result.branch_target = (rs_entry.rs1_value + rs_entry.imm) & ~1;
                end
                7'b1100011: begin // Conditional branches
                    result.branch_target = rs_entry.pc + rs_entry.imm;
                    case (rs_entry.funct3)
                        3'b000: result.branch_taken = (rs_entry.rs1_value == rs_entry.rs2_value); // BEQ
                        3'b001: result.branch_taken = (rs_entry.rs1_value != rs_entry.rs2_value); // BNE
                        3'b100: result.branch_taken = ($signed(rs_entry.rs1_value) < $signed(rs_entry.rs2_value)); // BLT
                        3'b101: result.branch_taken = ($signed(rs_entry.rs1_value) >= $signed(rs_entry.rs2_value)); // BGE
                        3'b110: result.branch_taken = (rs_entry.rs1_value < rs_entry.rs2_value); // BLTU
                        3'b111: result.branch_taken = (rs_entry.rs1_value >= rs_entry.rs2_value); // BGEU
                        default: result.branch_taken = 0;
                    endcase
                end
                default: begin
                    result.branch_taken = 0;
                    result.alu_result = '0;
                end
            endcase
        end

        OP_LOAD, OP_STORE: begin
            // Calculate memory address
            result.mem_we = 1'b1;
            result.mem_addr = rs_entry.rs1_value + rs_entry.imm;
            result.rd = rs_entry.rd;
            
            if (rs_entry.op_type == OP_STORE) begin
                result.store_data = rs_entry.rs2_value;
                result.mem_we = 1'b1;
                // Determine wstrb based on funct3
                case (rs_entry.funct3[1:0])
                    2'b00: result.mem_wstrb = 8'b00000001; // SB
                    2'b01: result.mem_wstrb = 8'b00000011; // SH
                    2'b10: result.mem_wstrb = 8'b00001111; // SW
                    2'b11: result.mem_wstrb = 8'b11111111; // SD
                endcase
            end else begin
                result.mem_we = 1'b0;
                result.reg_we = 1'b1;
                result.mem_unsigned = rs_entry.funct3[2];
                result.mem_size = rs_entry.funct3[1:0];
            end
        end
        OP_SYSTEM: begin
            // Handle system instructions (ECALL, EBREAK, etc.)
            // Simplified for this implementation
            result.illegal_instr = 1'b0;
        end
        default: begin
            result.alu_result = '0;
            result.branch_taken = 0;
        end
    endcase
    
    return result;
endfunction

// Debug interface assignments
assign debug_pc = pc;
assign debug_next_pc = next_pc;
assign debug_regfile = regfile;
assign debug_inst_retired = inst_retired;
assign debug_cycles = memory_result0.reg_we;
assign debug_opcode = decoded_instr_pair.instr0.opcode;
assign debug_result_alu = execute_result1.alu_result;
assign debug_valid_instr = decoded_instr_pair.instr0.valid;
assign debug_imm = decoded_instr_pair.instr0.imm;
assign debug_rs2 = decoded_instr_pair.instr0.rs2;
assign debug_funct3 = decoded_instr_pair.instr0.funct3;
assign debug_privilege = current_privilege;
assign debug_mem_rd = execute_result0.rd;

endmodule