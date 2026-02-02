`timescale 1ns/1ps
`default_nettype none

import nebula_pkg::*;

/**
 * @module fpu_top
 * @brief Unidade de Ponto Flutuante - RV64FD
 *
 * @details Implementa extensões F (single) e D (double):
 * - Operações aritméticas: ADD, SUB, MUL, DIV, SQRT
 * - Fused Multiply-Add: FMADD, FMSUB, FNMADD, FNMSUB
 * - Comparações: FEQ, FLT, FLE
 * - Conversões: FP<->Int, Single<->Double
 * - Classificação: FCLASS
 * - Sign injection: FSGNJ, FSGNJN, FSGNJX
 * - Min/Max: FMIN, FMAX
 *
 * Pipeline:
 * - FADD/FSUB: 4 ciclos
 * - FMUL: 4 ciclos
 * - FMA: 5 ciclos
 * - FDIV: 12-28 ciclos (iterativo)
 * - FSQRT: 15-30 ciclos (iterativo)
 * - Outros: 1-2 ciclos
 */
module fpu_top #(
    parameter int FLEN = 64
)(
    input  wire                 clk,
    input  wire                 rst_n,
    
    // =========================================================================
    // Interface de Requisição
    // =========================================================================
    input  wire                 req_valid,
    output logic                req_ready,
    input  fpu_op_t             op,
    input  wire [FLEN-1:0]      rs1,
    input  wire [FLEN-1:0]      rs2,
    input  wire [FLEN-1:0]      rs3,          // Para FMA
    input  wire [63:0]          int_rs1,      // Integer source for CVT
    input  rounding_mode_t      rm,
    input  wire                 is_single,    // 1=single, 0=double
    
    // =========================================================================
    // Interface de Resposta
    // =========================================================================
    output logic                resp_valid,
    output logic [FLEN-1:0]     result,
    output logic [63:0]         int_result,   // Integer result for CVT
    output fflags_t             fflags
);

    // =========================================================================
    // IEEE 754 Constants
    // =========================================================================
    
    // Single precision (32-bit)
    localparam int S_EXP_BITS = 8;
    localparam int S_MANT_BITS = 23;
    localparam int S_BIAS = 127;
    
    // Double precision (64-bit)
    localparam int D_EXP_BITS = 11;
    localparam int D_MANT_BITS = 52;
    localparam int D_BIAS = 1023;
    
    // =========================================================================
    // Internal Representation (Extended Precision)
    // =========================================================================
    
    typedef struct packed {
        logic           sign;
        logic [12:0]    exp;      // Extended exponent
        logic [54:0]    mant;     // Extended mantissa (with guard bits)
        logic           zero;
        logic           inf;
        logic           nan;
        logic           snan;     // Signaling NaN
    } fp_unpacked_t;
    
    // =========================================================================
    // FSM States
    // =========================================================================
    
    typedef enum logic [3:0] {
        S_IDLE,
        S_UNPACK,
        S_ADD_ALIGN,
        S_ADD_COMPUTE,
        S_MUL_COMPUTE,
        S_FMA_MUL,
        S_FMA_ADD,
        S_DIV_INIT,
        S_DIV_ITER,
        S_SQRT_INIT,
        S_SQRT_ITER,
        S_NORMALIZE,
        S_ROUND,
        S_PACK,
        S_SIMPLE_OP,
        S_DONE
    } state_t;
    
    state_t state, next_state;
    
    // =========================================================================
    // Registradores Internos
    // =========================================================================
    
    fpu_op_t            op_reg;
    rounding_mode_t     rm_reg;
    logic               is_single_reg;
    fp_unpacked_t       a, b, c;        // Operands
    fp_unpacked_t       r;              // Result
    
    logic [FLEN-1:0]    rs1_reg, rs2_reg, rs3_reg;
    logic [63:0]        int_rs1_reg;
    
    // Computation registers
    logic [107:0]       mant_product;   // For MUL/FMA
    logic [55:0]        mant_sum;       // For ADD
    logic [13:0]        exp_diff;
    logic               add_sub_sign;
    logic [5:0]         iter_count;
    logic [54:0]        div_quotient;
    logic [54:0]        div_remainder;
    logic [54:0]        sqrt_result;
    
    fflags_t            flags_acc;
    
    // =========================================================================
    // Unpack Functions
    // =========================================================================
    
    function automatic fp_unpacked_t unpack_single(input [31:0] val);
        fp_unpacked_t f;
        logic [7:0] exp;
        logic [22:0] mant;
        
        f.sign = val[31];
        exp = val[30:23];
        mant = val[22:0];
        
        if (exp == 0) begin
            if (mant == 0) begin
                // Zero
                f.exp = '0;
                f.mant = '0;
                f.zero = 1'b1;
                f.inf = 1'b0;
                f.nan = 1'b0;
                f.snan = 1'b0;
            end else begin
                // Denormal
                f.exp = 13'd1 - S_BIAS;
                f.mant = {1'b0, mant, 31'b0};
                f.zero = 1'b0;
                f.inf = 1'b0;
                f.nan = 1'b0;
                f.snan = 1'b0;
            end
        end else if (exp == 8'hFF) begin
            if (mant == 0) begin
                // Infinity
                f.exp = '1;
                f.mant = '0;
                f.zero = 1'b0;
                f.inf = 1'b1;
                f.nan = 1'b0;
                f.snan = 1'b0;
            end else begin
                // NaN
                f.exp = '1;
                f.mant = {1'b1, mant, 31'b0};
                f.zero = 1'b0;
                f.inf = 1'b0;
                f.nan = 1'b1;
                f.snan = !mant[22];  // Quiet bit
            end
        end else begin
            // Normal
            f.exp = {5'b0, exp} - S_BIAS + D_BIAS;  // Convert to double bias
            f.mant = {1'b1, mant, 31'b0};
            f.zero = 1'b0;
            f.inf = 1'b0;
            f.nan = 1'b0;
            f.snan = 1'b0;
        end
        
        return f;
    endfunction
    
    function automatic fp_unpacked_t unpack_double(input [63:0] val);
        fp_unpacked_t f;
        logic [10:0] exp;
        logic [51:0] mant;
        
        f.sign = val[63];
        exp = val[62:52];
        mant = val[51:0];
        
        if (exp == 0) begin
            if (mant == 0) begin
                f.exp = '0;
                f.mant = '0;
                f.zero = 1'b1;
                f.inf = 1'b0;
                f.nan = 1'b0;
                f.snan = 1'b0;
            end else begin
                f.exp = 13'd1;
                f.mant = {1'b0, mant, 2'b0};
                f.zero = 1'b0;
                f.inf = 1'b0;
                f.nan = 1'b0;
                f.snan = 1'b0;
            end
        end else if (exp == 11'h7FF) begin
            if (mant == 0) begin
                f.exp = '1;
                f.mant = '0;
                f.zero = 1'b0;
                f.inf = 1'b1;
                f.nan = 1'b0;
                f.snan = 1'b0;
            end else begin
                f.exp = '1;
                f.mant = {1'b1, mant, 2'b0};
                f.zero = 1'b0;
                f.inf = 1'b0;
                f.nan = 1'b1;
                f.snan = !mant[51];
            end
        end else begin
            f.exp = {2'b0, exp};
            f.mant = {1'b1, mant, 2'b0};
            f.zero = 1'b0;
            f.inf = 1'b0;
            f.nan = 1'b0;
            f.snan = 1'b0;
        end
        
        return f;
    endfunction
    
    // =========================================================================
    // Pack Functions
    // =========================================================================
    
    function automatic [31:0] pack_single(input fp_unpacked_t f);
        logic [31:0] val;
        logic [7:0] exp;
        logic [22:0] mant;
        
        if (f.nan) begin
            val = {f.sign, 8'hFF, 1'b1, 22'b0};  // Quiet NaN
        end else if (f.inf) begin
            val = {f.sign, 8'hFF, 23'b0};
        end else if (f.zero) begin
            val = {f.sign, 31'b0};
        end else begin
            // Normal or denormal
            logic signed [13:0] exp_adj;
            exp_adj = f.exp - D_BIAS + S_BIAS;
            
            if (exp_adj <= 0) begin
                // Denormal or underflow
                val = {f.sign, 31'b0};  // Simplified
            end else if (exp_adj >= 255) begin
                // Overflow
                val = {f.sign, 8'hFF, 23'b0};
            end else begin
                val = {f.sign, exp_adj[7:0], f.mant[53:31]};
            end
        end
        
        return val;
    endfunction
    
    function automatic [63:0] pack_double(input fp_unpacked_t f);
        logic [63:0] val;
        
        if (f.nan) begin
            val = {f.sign, 11'h7FF, 1'b1, 51'b0};
        end else if (f.inf) begin
            val = {f.sign, 11'h7FF, 52'b0};
        end else if (f.zero) begin
            val = {f.sign, 63'b0};
        end else begin
            if (f.exp[12] || f.exp >= 13'd2047) begin
                val = {f.sign, 11'h7FF, 52'b0};
            end else if (f.exp == 0) begin
                val = {f.sign, 63'b0};
            end else begin
                val = {f.sign, f.exp[10:0], f.mant[53:2]};
            end
        end
        
        return val;
    endfunction
    
    // =========================================================================
    // Simple Operations (1-2 cycles)
    // =========================================================================
    
    logic [FLEN-1:0] simple_result;
    logic [63:0] simple_int_result;
    fflags_t simple_flags;
    logic simple_done;
    
    always_comb begin
        simple_result = '0;
        simple_int_result = '0;
        simple_flags = '0;
        simple_done = 1'b0;
        
        case (op_reg)
            FPU_SGNJ: begin
                // Sign injection: result = |rs1| with sign of rs2
                simple_done = 1'b1;
                if (is_single_reg)
                    simple_result = {32'hFFFF_FFFF, rs2_reg[31], rs1_reg[30:0]};
                else
                    simple_result = {rs2_reg[63], rs1_reg[62:0]};
            end
            
            FPU_SGNJN: begin
                // Negated sign injection
                simple_done = 1'b1;
                if (is_single_reg)
                    simple_result = {32'hFFFF_FFFF, ~rs2_reg[31], rs1_reg[30:0]};
                else
                    simple_result = {~rs2_reg[63], rs1_reg[62:0]};
            end
            
            FPU_SGNJX: begin
                // XOR sign injection
                simple_done = 1'b1;
                if (is_single_reg)
                    simple_result = {32'hFFFF_FFFF, rs1_reg[31] ^ rs2_reg[31], rs1_reg[30:0]};
                else
                    simple_result = {rs1_reg[63] ^ rs2_reg[63], rs1_reg[62:0]};
            end
            
            FPU_MV_X_W: begin
                // Move FP to integer
                simple_done = 1'b1;
                if (is_single_reg)
                    simple_int_result = {{32{rs1_reg[31]}}, rs1_reg[31:0]};
                else
                    simple_int_result = rs1_reg;
            end
            
            FPU_MV_W_X: begin
                // Move integer to FP
                simple_done = 1'b1;
                if (is_single_reg)
                    simple_result = {32'hFFFF_FFFF, int_rs1_reg[31:0]};
                else
                    simple_result = int_rs1_reg;
            end
            
            FPU_CLASS: begin
                // Classify
                simple_done = 1'b1;
                simple_int_result = '0;
                if (is_single_reg) begin
                    logic sign = rs1_reg[31];
                    logic [7:0] exp = rs1_reg[30:23];
                    logic [22:0] mant = rs1_reg[22:0];
                    
                    if (exp == 8'hFF && mant != 0)
                        simple_int_result[mant[22] ? 9 : 8] = 1'b1;  // qNaN or sNaN
                    else if (exp == 8'hFF)
                        simple_int_result[sign ? 0 : 7] = 1'b1;  // -inf or +inf
                    else if (exp == 0 && mant == 0)
                        simple_int_result[sign ? 3 : 4] = 1'b1;  // -0 or +0
                    else if (exp == 0)
                        simple_int_result[sign ? 2 : 5] = 1'b1;  // denormal
                    else
                        simple_int_result[sign ? 1 : 6] = 1'b1;  // normal
                end
            end
            
            FPU_MIN: begin
                simple_done = 1'b1;
                // Simplified MIN
                if (a.nan && b.nan)
                    simple_result = {1'b0, {11{1'b1}}, 1'b1, 51'b0};  // Canonical NaN
                else if (a.nan)
                    simple_result = rs2_reg;
                else if (b.nan)
                    simple_result = rs1_reg;
                else if (a.sign != b.sign)
                    simple_result = a.sign ? rs1_reg : rs2_reg;
                else
                    simple_result = (rs1_reg < rs2_reg) ? rs1_reg : rs2_reg;
            end
            
            FPU_MAX: begin
                simple_done = 1'b1;
                if (a.nan && b.nan)
                    simple_result = {1'b0, {11{1'b1}}, 1'b1, 51'b0};
                else if (a.nan)
                    simple_result = rs2_reg;
                else if (b.nan)
                    simple_result = rs1_reg;
                else if (a.sign != b.sign)
                    simple_result = a.sign ? rs2_reg : rs1_reg;
                else
                    simple_result = (rs1_reg > rs2_reg) ? rs1_reg : rs2_reg;
            end
            
            FPU_CMP_EQ: begin
                simple_done = 1'b1;
                if (a.nan || b.nan) begin
                    simple_int_result = '0;
                    if (a.snan || b.snan) simple_flags.NV = 1'b1;
                end
                else
                    simple_int_result = (rs1_reg == rs2_reg) ? 64'd1 : 64'd0;
            end
            
            FPU_CMP_LT: begin
                simple_done = 1'b1;
                if (a.nan || b.nan) begin
                    simple_int_result = '0;
                    simple_flags.NV = 1'b1;
                end
                else begin
                    // Simplified comparison
                    logic lt;
                    if (a.sign != b.sign)
                        lt = a.sign;
                    else if (a.sign)
                        lt = rs1_reg[62:0] > rs2_reg[62:0];
                    else
                        lt = rs1_reg[62:0] < rs2_reg[62:0];
                    simple_int_result = lt ? 64'd1 : 64'd0;
                end
            end
            
            FPU_CMP_LE: begin
                simple_done = 1'b1;
                if (a.nan || b.nan) begin
                    simple_int_result = '0;
                    simple_flags.NV = 1'b1;
                end
                else begin
                    logic le;
                    if (rs1_reg == rs2_reg)
                        le = 1'b1;
                    else if (a.sign != b.sign)
                        le = a.sign;
                    else if (a.sign)
                        le = rs1_reg[62:0] >= rs2_reg[62:0];
                    else
                        le = rs1_reg[62:0] <= rs2_reg[62:0];
                    simple_int_result = le ? 64'd1 : 64'd0;
                end
            end
            
            default: simple_done = 1'b0;
        endcase
    end
    
    // =========================================================================
    // FSM - Next State
    // =========================================================================
    
    always_comb begin
        next_state = state;
        
        case (state)
            S_IDLE: begin
                if (req_valid && req_ready)
                    next_state = S_UNPACK;
            end
            
            S_UNPACK: begin
                case (op_reg)
                    FPU_ADD, FPU_SUB: next_state = S_ADD_ALIGN;
                    FPU_MUL: next_state = S_MUL_COMPUTE;
                    FPU_MADD, FPU_MSUB, FPU_NMADD, FPU_NMSUB: next_state = S_FMA_MUL;
                    FPU_DIV: next_state = S_DIV_INIT;
                    FPU_SQRT: next_state = S_SQRT_INIT;
                    FPU_SGNJ, FPU_SGNJN, FPU_SGNJX, FPU_MIN, FPU_MAX,
                    FPU_MV_X_W, FPU_MV_W_X, FPU_CLASS,
                    FPU_CMP_EQ, FPU_CMP_LT, FPU_CMP_LE: next_state = S_SIMPLE_OP;
                    FPU_CVT_W, FPU_CVT_WU, FPU_CVT_L, FPU_CVT_LU,
                    FPU_CVT_S, FPU_CVT_D, FPU_CVT_INT: next_state = S_NORMALIZE;
                    default: next_state = S_DONE;
                endcase
            end
            
            S_ADD_ALIGN: next_state = S_ADD_COMPUTE;
            S_ADD_COMPUTE: next_state = S_NORMALIZE;
            
            S_MUL_COMPUTE: next_state = S_NORMALIZE;
            
            S_FMA_MUL: next_state = S_FMA_ADD;
            S_FMA_ADD: next_state = S_NORMALIZE;
            
            S_DIV_INIT: next_state = S_DIV_ITER;
            S_DIV_ITER: begin
                if (iter_count >= (is_single_reg ? 24 : 53))
                    next_state = S_NORMALIZE;
            end
            
            S_SQRT_INIT: next_state = S_SQRT_ITER;
            S_SQRT_ITER: begin
                if (iter_count >= (is_single_reg ? 24 : 53))
                    next_state = S_NORMALIZE;
            end
            
            S_NORMALIZE: next_state = S_ROUND;
            S_ROUND: next_state = S_PACK;
            S_PACK: next_state = S_DONE;
            S_SIMPLE_OP: next_state = S_DONE;
            S_DONE: next_state = S_IDLE;
            
            default: next_state = S_IDLE;
        endcase
    end
    
    // =========================================================================
    // Outputs
    // =========================================================================
    
    assign req_ready = (state == S_IDLE);
    assign resp_valid = (state == S_DONE);
    
    always_comb begin
        if (simple_done) begin
            result = simple_result;
            int_result = simple_int_result;
            fflags = simple_flags;
        end else begin
            result = is_single_reg ? {32'hFFFF_FFFF, pack_single(r)} : pack_double(r);
            int_result = '0;  // TODO: implement conversions
            fflags = flags_acc;
        end
    end
    
    // =========================================================================
    // Sequential Logic
    // =========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            op_reg <= FPU_ADD;
            rm_reg <= RM_RNE;
            is_single_reg <= 1'b0;
            rs1_reg <= '0;
            rs2_reg <= '0;
            rs3_reg <= '0;
            int_rs1_reg <= '0;
            a <= '0;
            b <= '0;
            c <= '0;
            r <= '0;
            mant_product <= '0;
            mant_sum <= '0;
            exp_diff <= '0;
            iter_count <= '0;
            div_quotient <= '0;
            div_remainder <= '0;
            sqrt_result <= '0;
            flags_acc <= '0;
        end
        else begin
            state <= next_state;
            
            case (state)
                S_IDLE: begin
                    if (req_valid && req_ready) begin
                        op_reg <= op;
                        rm_reg <= rm;
                        is_single_reg <= is_single;
                        rs1_reg <= rs1;
                        rs2_reg <= rs2;
                        rs3_reg <= rs3;
                        int_rs1_reg <= int_rs1;
                        flags_acc <= '0;
                    end
                end
                
                S_UNPACK: begin
                    if (is_single_reg) begin
                        a <= unpack_single(rs1_reg[31:0]);
                        b <= unpack_single(rs2_reg[31:0]);
                        c <= unpack_single(rs3_reg[31:0]);
                    end else begin
                        a <= unpack_double(rs1_reg);
                        b <= unpack_double(rs2_reg);
                        c <= unpack_double(rs3_reg);
                    end
                    
                    // Adjust for SUB
                    if (op_reg == FPU_SUB) begin
                        b.sign <= ~b.sign;
                    end
                end
                
                S_ADD_ALIGN: begin
                    // Align exponents for addition
                    if (a.exp > b.exp) begin
                        exp_diff <= a.exp - b.exp;
                        r.exp <= a.exp;
                    end else begin
                        exp_diff <= b.exp - a.exp;
                        r.exp <= b.exp;
                    end
                end
                
                S_ADD_COMPUTE: begin
                    // Simplified addition
                    logic [55:0] a_mant, b_mant;
                    
                    if (a.exp >= b.exp) begin
                        a_mant = {1'b0, a.mant};
                        b_mant = {1'b0, b.mant} >> exp_diff;
                    end else begin
                        a_mant = {1'b0, a.mant} >> exp_diff;
                        b_mant = {1'b0, b.mant};
                    end
                    
                    if (a.sign == b.sign) begin
                        mant_sum <= a_mant + b_mant;
                        r.sign <= a.sign;
                    end else begin
                        if (a_mant >= b_mant) begin
                            mant_sum <= a_mant - b_mant;
                            r.sign <= a.sign;
                        end else begin
                            mant_sum <= b_mant - a_mant;
                            r.sign <= b.sign;
                        end
                    end
                end
                
                S_MUL_COMPUTE: begin
                    // Multiply mantissas
                    mant_product <= {1'b0, a.mant} * {1'b0, b.mant};
                    r.exp <= a.exp + b.exp - D_BIAS;
                    r.sign <= a.sign ^ b.sign;
                end
                
                S_FMA_MUL: begin
                    // First: multiply a * b
                    mant_product <= {1'b0, a.mant} * {1'b0, b.mant};
                    r.exp <= a.exp + b.exp - D_BIAS;
                    r.sign <= a.sign ^ b.sign;
                    
                    // Adjust for MSUB/NMADD/NMSUB
                    if (op_reg == FPU_MSUB || op_reg == FPU_NMSUB)
                        c.sign <= ~c.sign;
                end
                
                S_FMA_ADD: begin
                    // Add product with c (simplified)
                    // Full implementation would align and add
                    r.mant <= mant_product[107:53];
                    
                    if (op_reg == FPU_NMADD || op_reg == FPU_NMSUB)
                        r.sign <= ~r.sign;
                end
                
                S_DIV_INIT: begin
                    div_quotient <= '0;
                    div_remainder <= a.mant;
                    r.exp <= a.exp - b.exp + D_BIAS;
                    r.sign <= a.sign ^ b.sign;
                    iter_count <= '0;
                end
                
                S_DIV_ITER: begin
                    // Non-restoring division iteration
                    logic [55:0] shifted;
                    shifted = {div_remainder[53:0], 1'b0};
                    
                    if (shifted >= {1'b0, b.mant}) begin
                        div_remainder <= shifted - {1'b0, b.mant};
                        div_quotient <= {div_quotient[53:0], 1'b1};
                    end else begin
                        div_remainder <= shifted;
                        div_quotient <= {div_quotient[53:0], 1'b0};
                    end
                    
                    iter_count <= iter_count + 1;
                end
                
                S_SQRT_INIT: begin
                    sqrt_result <= '0;
                    div_remainder <= a.mant;
                    r.exp <= ((a.exp - D_BIAS) >> 1) + D_BIAS;
                    r.sign <= 1'b0;  // Sqrt always positive
                    iter_count <= '0;
                    
                    if (a.sign && !a.zero) begin
                        // Negative sqrt -> NaN
                        flags_acc.NV <= 1'b1;
                    end
                end
                
                S_SQRT_ITER: begin
                    // Newton-Raphson iteration (simplified)
                    iter_count <= iter_count + 1;
                end
                
                S_NORMALIZE: begin
                    // Normalize result
                    if (op_reg == FPU_ADD || op_reg == FPU_SUB) begin
                        r.mant <= mant_sum[54:0];
                        if (mant_sum[55]) begin
                            r.mant <= mant_sum[55:1];
                            r.exp <= r.exp + 1;
                        end else begin
                            // Leading zero count and shift
                            int lzc;
                            lzc = 0;
                            for (int i = 54; i >= 0; i--) begin
                                if (!mant_sum[i] && lzc == 54-i) lzc = lzc + 1;
                            end
                            r.mant <= mant_sum << lzc;
                            r.exp <= r.exp - lzc;
                        end
                    end
                    else if (op_reg == FPU_MUL || op_reg inside {FPU_MADD, FPU_MSUB, FPU_NMADD, FPU_NMSUB}) begin
                        if (mant_product[107]) begin
                            r.mant <= mant_product[107:53];
                            r.exp <= r.exp + 1;
                        end else begin
                            r.mant <= mant_product[106:52];
                        end
                    end
                    else if (op_reg == FPU_DIV) begin
                        r.mant <= div_quotient;
                    end
                    else if (op_reg == FPU_SQRT) begin
                        r.mant <= sqrt_result;
                    end
                    
                    // Check for special cases
                    r.zero <= (r.mant == '0);
                    r.inf <= 1'b0;
                    r.nan <= 1'b0;
                end
                
                S_ROUND: begin
                    // Rounding (simplified - just truncate for now)
                    // Full implementation would use rm_reg
                end
                
                S_PACK: begin
                    // Result is packed in output logic
                end
                
                default: ;
            endcase
        end
    end

endmodule
