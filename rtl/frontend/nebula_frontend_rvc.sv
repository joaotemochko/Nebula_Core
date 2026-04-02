`timescale 1ns/1ps
`default_nettype none

import nebula_pkg::*;

/**
 * @module nebula_frontend_rvc
 * @brief Frontend Dual Issue com suporte completo a RV64C
 *
 */
module nebula_frontend_rvc #(
    parameter int XLEN = 64,
    parameter int VADDR_WIDTH = 39,
    parameter int PADDR_WIDTH = 56,
    parameter int VPN_WIDTH = 27,
    parameter int PPN_WIDTH = 44
)(
    input  wire                     clk,
    input  wire                     rst_n,

    input  wire                     backend_stall,
    input  wire                     backend_flush,
    input  wire                     backend_redirect,
    input  wire [VADDR_WIDTH-1:0]   backend_redirect_pc,

    output frontend_packet_t        frontend_out,
    output logic                    frontend_valid,
    output logic [VADDR_WIDTH-1:0]  fetch_pc_out,

    input  bp_prediction_t          bp_prediction,

    output logic                    icache_req,
    output logic [PADDR_WIDTH-1:0]  icache_addr,
    input  wire                     icache_ready,
    input  wire                     icache_resp_valid,
    input  wire [63:0]              icache_resp_data,
    input  wire                     icache_resp_error,

    output logic                    itlb_req,
    output logic [VPN_WIDTH-1:0]    itlb_vpn,
    input  wire                     itlb_hit,
    input  wire [PPN_WIDTH-1:0]     itlb_ppn,
    input  wire                     itlb_page_fault,
    input  wire                     itlb_access_fault,

    output logic                    ptw_req,
    output logic [VPN_WIDTH-1:0]    ptw_vpn,
    input  wire                     ptw_ready,
    input  wire                     ptw_resp_valid,
    input  wire                     ptw_page_fault,
    input  wire                     ptw_access_fault,

    input  wire                     mmu_enabled,
    input  wire [1:0]               current_priv,

    output logic                    fetch_exception,
    output logic [5:0]              fetch_exception_cause,
    output logic [XLEN-1:0]         fetch_exception_value
);

    // Endereço base de reset (Ajuste para seu OpenSBI)
    localparam logic [VADDR_WIDTH-1:0] RESET_VECTOR = 39'h00_1000_0000;

    // =========================================================================
    // Fetch Buffer e Estado
    // =========================================================================
    logic [79:0] fetch_buffer;
    logic [3:0]  fetch_buffer_valid;
    logic [2:0]  fetch_offset;

    logic [VADDR_WIDTH-1:0] pc_reg;
    logic [VADDR_WIDTH-1:0] fetch_pc;
    logic [VADDR_WIDTH-1:0] next_pc;
    logic [VADDR_WIDTH-1:0] instr0_pc, instr1_pc;

    // =========================================================================
    // Extração Paralela de Instruções (Até 4 Halfwords)
    // =========================================================================
    logic [15:0] hw0, hw1, hw2, hw3;
    logic val0, val1, val2, val3;

    always_comb begin
        hw0 = '0; hw1 = '0; hw2 = '0; hw3 = '0;
        val0 = 0; val1 = 0; val2 = 0; val3 = 0;

        case (fetch_offset[1:0])
            2'b00: begin
                hw0 = fetch_buffer[15:0];  val0 = fetch_buffer_valid[0];
                hw1 = fetch_buffer[31:16]; val1 = fetch_buffer_valid[1];
                hw2 = fetch_buffer[47:32]; val2 = fetch_buffer_valid[2];
                hw3 = fetch_buffer[63:48]; val3 = fetch_buffer_valid[3];
            end
            2'b01: begin
                hw0 = fetch_buffer[31:16]; val0 = fetch_buffer_valid[1];
                hw1 = fetch_buffer[47:32]; val1 = fetch_buffer_valid[2];
                hw2 = fetch_buffer[63:48]; val2 = fetch_buffer_valid[3];
                hw3 = fetch_buffer[79:64]; val3 = fetch_offset[2]; 
            end
            2'b10: begin
                hw0 = fetch_buffer[47:32]; val0 = fetch_buffer_valid[2];
                hw1 = fetch_buffer[63:48]; val1 = fetch_buffer_valid[3];
                hw2 = fetch_buffer[79:64]; val2 = fetch_offset[2];
                hw3 = 16'h0;               val3 = 1'b0;
            end
            2'b11: begin
                hw0 = fetch_buffer[63:48]; val0 = fetch_buffer_valid[3];
                hw1 = fetch_buffer[79:64]; val1 = fetch_offset[2];
                hw2 = 16'h0;               val2 = 1'b0;
                hw3 = 16'h0;               val3 = 1'b0;
            end
        endcase
    end

    // --- Lógica Instr 0 ---
    logic i0_is_compressed;
    logic i0_have_full;
    logic [31:0] i0_raw;
    
    assign i0_is_compressed = (hw0[1:0] != 2'b11);
    assign i0_have_full = val0 && (i0_is_compressed || val1);
    assign i0_raw = i0_is_compressed ? {16'h0, hw0} : {hw1, hw0};

    // --- Lógica Instr 1 ---
    logic [15:0] i1_hw0, i1_hw1;
    logic i1_val0, i1_val1;
    logic i1_is_compressed;
    logic i1_have_full;
    logic [31:0] i1_raw;

    always_comb begin
        if (i0_is_compressed) begin
            i1_hw0 = hw1; i1_hw1 = hw2;
            i1_val0 = val1; i1_val1 = val2;
        end else begin
            i1_hw0 = hw2; i1_hw1 = hw3;
            i1_val0 = val2; i1_val1 = val3;
        end
        i1_is_compressed = (i1_hw0[1:0] != 2'b11);
        i1_have_full = i1_val0 && (i1_is_compressed || i1_val1);
        i1_raw = i1_is_compressed ? {16'h0, i1_hw0} : {i1_hw1, i1_hw0};
    end

    // =========================================================================
    // Decompressores Duplos
    // =========================================================================
    logic [31:0] i0_expanded, i1_expanded;
    logic i0_c_valid, i1_c_valid, i0_illegal, i1_illegal;

    compressed_decoder_rv64 u_decomp0 (
        .cinstr(i0_raw[15:0]), .instr(i0_expanded), .valid(i0_c_valid), .illegal(i0_illegal), .is_compressed()
    );
    compressed_decoder_rv64 u_decomp1 (
        .cinstr(i1_raw[15:0]), .instr(i1_expanded), .valid(i1_c_valid), .illegal(i1_illegal), .is_compressed()
    );

    logic [31:0] final_instr0, final_instr1;
    assign final_instr0 = i0_is_compressed ? i0_expanded : i0_raw;
    assign final_instr1 = i1_is_compressed ? i1_expanded : i1_raw;

    // =========================================================================
    // Decodificação Centralizada (Macro Function)
    // =========================================================================
    function automatic decoded_instr_t decode_instr(
        input logic [31:0] instr_bits, 
        input logic is_comp, 
        input logic illegal_comp,
        input vaddr_t pc_val
    );
        decoded_instr_t dec = '0;
        dec.pc = pc_val;
        dec.is_compressed = is_comp;
        
        if (is_comp && illegal_comp) begin
            dec.valid = 1'b0;
            return dec;
        end
        
        dec.valid  = 1'b1;
        dec.opcode = instr_bits[6:0];
        dec.rd     = instr_bits[11:7];
        dec.funct3 = instr_bits[14:12];
        dec.rs1    = instr_bits[19:15];
        dec.rs2    = instr_bits[24:20];
        dec.rs3    = instr_bits[31:27];
        dec.funct7 = instr_bits[31:25];

        case (instr_bits[6:0])
            7'b0110111, 7'b0010111: begin // LUI, AUIPC
                dec.imm = {{32{instr_bits[31]}}, instr_bits[31:12], 12'b0};
                dec.is_alu = 1'b1;
            end
            7'b1101111: begin // JAL
                dec.imm = {{43{instr_bits[31]}}, instr_bits[31], instr_bits[19:12], instr_bits[20], instr_bits[30:21], 1'b0};
                dec.is_branch = 1'b1; dec.is_jal = 1'b1;
            end
            7'b1100111: begin // JALR
                dec.imm = {{52{instr_bits[31]}}, instr_bits[31:20]};
                dec.is_branch = 1'b1; dec.is_jalr = 1'b1;
            end
            7'b1100011: begin // Branch
                dec.imm = {{51{instr_bits[31]}}, instr_bits[31], instr_bits[7], instr_bits[30:25], instr_bits[11:8], 1'b0};
                dec.is_branch = 1'b1;
            end
            7'b0000011: begin // Load
                dec.imm = {{52{instr_bits[31]}}, instr_bits[31:20]};
                dec.is_load = 1'b1;
            end
            7'b0100011: begin // Store
                dec.imm = {{52{instr_bits[31]}}, instr_bits[31:25], instr_bits[11:7]};
                dec.is_store = 1'b1;
            end
            7'b0010011, 7'b0011011: begin // ALU-I, ALU-I-W
                dec.imm = {{52{instr_bits[31]}}, instr_bits[31:20]};
                dec.is_alu = 1'b1; dec.is_alu_w = (instr_bits[6:0] == 7'b0011011);
            end
            7'b0110011, 7'b0111011: begin // ALU-R, ALU-R-W
                dec.is_alu = 1'b1; dec.is_alu_w = (instr_bits[6:0] == 7'b0111011);
                if (instr_bits[31:25] == 7'b0000001) dec.is_mdu = 1'b1;
            end
            7'b0101111: begin // AMO
                dec.is_amo = 1'b1;
                dec.is_lr = (instr_bits[31:27] == 5'b00010);
                dec.is_sc = (instr_bits[31:27] == 5'b00011);
            end
            7'b0000111: begin // FP Load
                dec.imm = {{52{instr_bits[31]}}, instr_bits[31:20]};
                dec.is_fp_load = 1'b1; dec.is_fp = 1'b1;
            end
            7'b0100111: begin // FP Store
                dec.imm = {{52{instr_bits[31]}}, instr_bits[31:25], instr_bits[11:7]};
                dec.is_fp_store = 1'b1; dec.is_fp = 1'b1;
            end
            7'b1000011, 7'b1000111, 7'b1001011, 7'b1001111: begin // FMA
                dec.is_fp = 1'b1; dec.is_fma = 1'b1;
                dec.is_fp_single = (instr_bits[26:25] == 2'b00);
                dec.is_fp_double = (instr_bits[26:25] == 2'b01);
                dec.rm = rounding_mode_t'(instr_bits[14:12]);
            end
            7'b1010011: begin // FP ops
                dec.is_fp = 1'b1;
                dec.is_fp_single = (instr_bits[26:25] == 2'b00);
                dec.is_fp_double = (instr_bits[26:25] == 2'b01);
                dec.rm = rounding_mode_t'(instr_bits[14:12]);
            end
            7'b1110011: begin // SYSTEM
                dec.csr_addr = instr_bits[31:20];
                if (instr_bits[14:12] != 3'b000) dec.is_csr = 1'b1;
                else begin
                    case (instr_bits[31:20])
                        12'h000: dec.is_ecall  = 1'b1;
                        12'h001: dec.is_ebreak = 1'b1;
                        12'h302: dec.is_mret   = 1'b1;
                        12'h102: dec.is_sret   = 1'b1;
                        12'h105: dec.is_wfi    = 1'b1;
                        default: ;
                    endcase
                end
            end
            7'b0001111: begin // FENCE
                if (instr_bits[14:12] == 3'b001) dec.is_fence_i = 1'b1;
                else dec.is_fence = 1'b1;
            end
            default: ;
        endcase

        if (instr_bits[31:25] == 7'b0001001 && instr_bits[14:12] == 3'b000 && instr_bits[6:0] == 7'b1110011)
            dec.is_sfence_vma = 1'b1;
            
        return dec;
    endfunction

    // =========================================================================
    // Dual Issue Hazard Unit
    // =========================================================================
    decoded_instr_t decoded_instr0, decoded_instr1;
    logic can_dual_issue;

    always_comb begin
        instr0_pc = pc_reg;
        instr1_pc = pc_reg + (i0_is_compressed ? 2 : 4);

        decoded_instr0 = decode_instr(final_instr0, i0_is_compressed, i0_illegal, instr0_pc);
        decoded_instr1 = decode_instr(final_instr1, i1_is_compressed, i1_illegal, instr1_pc);

        // Desabilita se não estivermos no estado DECODE ou não tivermos instrução
        if (!i0_have_full || state != S_DECODE) begin
            decoded_instr0.valid = 1'b0;
        end
        if (!i1_have_full || state != S_DECODE) begin
            decoded_instr1.valid = 1'b0;
        end

        can_dual_issue = 1'b1;
        
        if (!decoded_instr1.valid) can_dual_issue = 1'b0;

        if (!decoded_instr1.is_alu || decoded_instr1.is_mdu) can_dual_issue = 1'b0;
        
        if (decoded_instr0.is_ecall || decoded_instr0.is_ebreak || decoded_instr0.is_mret) can_dual_issue = 1'b0;

        // Regra de Dependências de Dados (RAW e WAW)
        if (decoded_instr0.rd != 5'd0) begin
            // RAW
            if ((decoded_instr1.rs1 == decoded_instr0.rd) || 
                (decoded_instr1.rs2 == decoded_instr0.rd) || 
                (decoded_instr1.rs3 == decoded_instr0.rd)) begin
                can_dual_issue = 1'b0;
            end
            // WAW
            if (decoded_instr1.rd == decoded_instr0.rd) begin
                can_dual_issue = 1'b0;
            end
        end
    end

    // Cálculo do próximo PC dependendo se despachamos 1 ou 2 instruções
    always_comb begin
        if (can_dual_issue)
            next_pc = instr1_pc + (i1_is_compressed ? 2 : 4);
        else
            next_pc = instr1_pc; // O pc da instr1 vira o próximo
            
        fetch_pc = {pc_reg[VADDR_WIDTH-1:3], 3'b000};
    end

    // =========================================================================
    // FSM
    // =========================================================================
    typedef enum logic [3:0] {
        S_RESET, S_FETCH_REQ, S_TLB_LOOKUP, S_TLB_WAIT, 
        S_ICACHE_REQ, S_ICACHE_WAIT, S_PROCESS, S_DECODE, 
        S_STALL, S_FLUSH, S_EXCEPTION
    } state_t;

    state_t state, next_state;
    logic [PPN_WIDTH-1:0] cached_ppn;
    logic tlb_done;

    always_comb begin
        next_state = state;
        case (state)
            S_RESET:       next_state = S_FETCH_REQ;
            S_FETCH_REQ:   next_state = backend_flush ? S_FLUSH : (mmu_enabled ? S_TLB_LOOKUP : S_ICACHE_REQ);
            S_TLB_LOOKUP:  next_state = backend_flush ? S_FLUSH : (itlb_page_fault ? S_EXCEPTION : (itlb_hit ? S_ICACHE_REQ : (ptw_ready ? S_TLB_WAIT : S_TLB_LOOKUP)));
            S_TLB_WAIT:    next_state = backend_flush ? S_FLUSH : (ptw_resp_valid ? (ptw_page_fault ? S_EXCEPTION : S_TLB_LOOKUP) : S_TLB_WAIT);
            S_ICACHE_REQ:  next_state = backend_flush ? S_FLUSH : (icache_ready ? S_ICACHE_WAIT : S_ICACHE_REQ);
            S_ICACHE_WAIT: next_state = backend_flush ? S_FLUSH : (icache_resp_valid ? (icache_resp_error ? S_EXCEPTION : S_PROCESS) : S_ICACHE_WAIT);
            S_PROCESS:     next_state = backend_flush ? S_FLUSH : (i0_have_full ? S_DECODE : S_FETCH_REQ);
            S_DECODE:      next_state = backend_flush ? S_FLUSH : (backend_stall ? S_STALL : (i0_have_full ? S_DECODE : S_FETCH_REQ));
            S_STALL:       next_state = backend_flush ? S_FLUSH : (!backend_stall ? S_DECODE : S_STALL);
            S_FLUSH:       next_state = S_FETCH_REQ;
            S_EXCEPTION:   if (backend_flush) next_state = S_FLUSH;
            default:       next_state = S_RESET;
        endcase
    end

    // =========================================================================
    // Outputs
    // =========================================================================
    assign frontend_out.instr0       = decoded_instr0;
    assign frontend_out.instr1       = decoded_instr1;
    assign frontend_out.instr0_valid = decoded_instr0.valid;
    assign frontend_out.instr1_valid = (decoded_instr1.valid && can_dual_issue);
    assign frontend_out.dual_issue   = can_dual_issue;

    assign frontend_valid = (state == S_DECODE) && i0_have_full && !backend_stall;
    assign fetch_pc_out   = instr0_pc;

    assign icache_req  = (state == S_ICACHE_REQ);
    assign icache_addr = mmu_enabled ? {{(PADDR_WIDTH-PPN_WIDTH-12){1'b0}}, cached_ppn, fetch_pc[11:0]} : {{(PADDR_WIDTH-VADDR_WIDTH){1'b0}}, fetch_pc};
    assign itlb_req = (state == S_TLB_LOOKUP);
    assign itlb_vpn = fetch_pc[VADDR_WIDTH-1:12];

    assign ptw_req = (state == S_TLB_LOOKUP) && !itlb_hit && !itlb_page_fault;
    assign ptw_vpn = fetch_pc[VADDR_WIDTH-1:12];

    assign fetch_exception        = (state == S_EXCEPTION);
    assign fetch_exception_cause  = (itlb_page_fault || ptw_page_fault) ? 6'(EXC_INSTR_PAGE_FAULT) : 6'(EXC_INSTR_ACCESS_FAULT);
    assign fetch_exception_value  = {{(XLEN-VADDR_WIDTH){pc_reg[VADDR_WIDTH-1]}}, pc_reg};

    // =========================================================================
    // Lógica Sequencial (Atualização Dinâmica do Buffer)
    // =========================================================================
    logic [2:0] consumed_halfwords;
    always_comb begin
        consumed_halfwords = 0;
        if (frontend_valid) begin
            consumed_halfwords = (i0_is_compressed ? 1 : 2) + (can_dual_issue ? (i1_is_compressed ? 1 : 2) : 0);
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state              <= S_RESET;
            pc_reg             <= RESET_VECTOR;
            fetch_buffer       <= '0;
            fetch_buffer_valid <= '0;
            fetch_offset       <= '0;
            cached_ppn         <= '0;
            tlb_done           <= 1'b0;
        end else begin
            state <= next_state;
            case (state)
                S_RESET: begin
                    pc_reg             <= RESET_VECTOR;
                    fetch_buffer       <= '0;
                    fetch_buffer_valid <= '0;
                    fetch_offset       <= '0;
                end
                S_FETCH_REQ: begin
                    fetch_offset <= {1'b0, pc_reg[2:1]};
                end
                S_TLB_LOOKUP: begin
                    if (itlb_hit) begin
                        cached_ppn <= itlb_ppn;
                        tlb_done   <= 1'b1;
                    end
                end
                S_ICACHE_WAIT: begin
                    if (icache_resp_valid && !icache_resp_error) begin
                        if (fetch_offset[2]) fetch_buffer[79:64] <= fetch_buffer[63:48];
                        fetch_buffer[63:0]  <= icache_resp_data;
                        fetch_buffer_valid  <= 4'b1111;
                    end
                end
                S_DECODE: begin
                    if (!backend_stall && i0_have_full) begin
                        if (backend_redirect) begin
                            pc_reg             <= backend_redirect_pc;
                            fetch_buffer_valid <= '0;
                            fetch_offset       <= '0;
                        end else if (bp_prediction.valid && bp_prediction.taken) begin
                            pc_reg             <= bp_prediction.target;
                            fetch_buffer_valid <= '0;
                            fetch_offset       <= '0;
                        end else begin
                            pc_reg <= next_pc;
                            
                            // Atualização dinâmica do offset baseada nas instruções consumidas
                            fetch_offset <= fetch_offset + consumed_halfwords;
                            
                            // Limpa os bits válidos baseados no avanço do offset
                            for (int i=0; i<4; i++) begin
                                if (i < (fetch_offset[1:0] + consumed_halfwords)) begin
                                    fetch_buffer_valid[i] <= 1'b0;
                                end
                            end
                        end
                    end
                end
                S_FLUSH: begin
                    if (backend_redirect) pc_reg <= backend_redirect_pc;
                    fetch_buffer       <= '0;
                    fetch_buffer_valid <= '0;
                    fetch_offset       <= '0;
                    tlb_done           <= 1'b0;
                end
                default: ;
            endcase
        end
    end
endmodule
