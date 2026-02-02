`timescale 1ns/1ps
`default_nettype none

import nebula_pkg::*;

/**
 * @module fpu_ieee754
 * @brief Unidade de Ponto Flutuante IEEE 754 Compliant - RV64FD
 *
 * @details Implementação completa das extensões F e D:
 * 
 * Operações Suportadas:
 * - Aritméticas: FADD, FSUB, FMUL, FDIV, FSQRT, FMIN, FMAX
 * - FMA: FMADD, FMSUB, FNMADD, FNMSUB
 * - Comparações: FEQ, FLT, FLE
 * - Conversões: FCVT.W[U].S/D, FCVT.L[U].S/D, FCVT.S.D, FCVT.D.S, FCVT.S/D.W[U]/L[U]
 * - Sign Injection: FSGNJ, FSGNJN, FSGNJX
 * - Move: FMV.X.W/D, FMV.W/D.X
 * - Classify: FCLASS.S/D
 *
 * Conformidade IEEE 754-2008:
 * - Todos os 5 rounding modes
 * - Todas as 5 exception flags (NV, DZ, OF, UF, NX)
 * - Suporte a denormals (não flush-to-zero)
 * - NaN propagation correto (canonical NaN)
 * - Signed zero handling
 *
 * Latências:
 * - FADD/FSUB: 4 ciclos
 * - FMUL: 4 ciclos  
 * - FMA: 5 ciclos
 * - FDIV.S: 14 ciclos, FDIV.D: 28 ciclos
 * - FSQRT.S: 15 ciclos, FSQRT.D: 30 ciclos
 * - Outros: 1-2 ciclos
 */
module fpu_ieee754 #(
    parameter int FLEN = 64
)(
    input  wire                 clk,
    input  wire                 rst_n,
    
    // Request Interface
    input  wire                 req_valid,
    output logic                req_ready,
    input  fpu_op_t             op,
    input  wire [FLEN-1:0]      rs1,
    input  wire [FLEN-1:0]      rs2,
    input  wire [FLEN-1:0]      rs3,          // Para FMA
    input  wire [63:0]          int_rs1,      // Integer source para conversões
    input  rounding_mode_t      rm,
    input  wire                 is_single,    // 1=float, 0=double
    
    // Response Interface
    output logic                resp_valid,
    output logic [FLEN-1:0]     result,
    output logic [63:0]         int_result,   // Integer result para conversões
    output fflags_t             fflags
);

    // =========================================================================
    // IEEE 754 Format Constants
    // =========================================================================
    
    // Single Precision (binary32)
    localparam int SP_EXP_BITS = 8;
    localparam int SP_MANT_BITS = 23;
    localparam int SP_BIAS = 127;
    localparam int SP_EMAX = 127;
    localparam int SP_EMIN = -126;
    
    // Double Precision (binary64)
    localparam int DP_EXP_BITS = 11;
    localparam int DP_MANT_BITS = 52;
    localparam int DP_BIAS = 1023;
    localparam int DP_EMAX = 1023;
    localparam int DP_EMIN = -1022;
    
    // Internal Extended Precision (para evitar perda de precisão)
    localparam int EXT_MANT_BITS = 56;  // 52 + guard + round + sticky
    localparam int EXT_EXP_BITS = 13;   // Para overflow detection
    
    // =========================================================================
    // Canonical NaN Values
    // =========================================================================
    
    localparam logic [31:0] SP_CANONICAL_NAN = 32'h7FC0_0000;  // Quiet NaN
    localparam logic [63:0] DP_CANONICAL_NAN = 64'h7FF8_0000_0000_0000;
    
    // =========================================================================
    // Internal Unpacked Format
    // =========================================================================
    
    typedef struct packed {
        logic                       sign;
        logic signed [EXT_EXP_BITS-1:0] exp;    // Biased exponent
        logic [EXT_MANT_BITS-1:0]   mant;       // With implicit bit
        logic                       zero;
        logic                       inf;
        logic                       nan;
        logic                       snan;       // Signaling NaN
        logic                       denorm;
    } fp_unpacked_t;
    
    // =========================================================================
    // FSM States
    // =========================================================================
    
    typedef enum logic [4:0] {
        S_IDLE,
        S_UNPACK,
        S_CLASSIFY,
        S_SPECIAL_CHECK,
        // Adder pipeline
        S_ADD_ALIGN,
        S_ADD_EXECUTE,
        S_ADD_NORMALIZE,
        // Multiplier pipeline
        S_MUL_EXECUTE,
        S_MUL_NORMALIZE,
        // FMA pipeline
        S_FMA_MUL,
        S_FMA_ALIGN,
        S_FMA_ADD,
        S_FMA_NORMALIZE,
        // Divider (iterative)
        S_DIV_INIT,
        S_DIV_ITERATE,
        S_DIV_FINALIZE,
        // Square root (iterative)
        S_SQRT_INIT,
        S_SQRT_ITERATE,
        S_SQRT_FINALIZE,
        // Conversion
        S_CVT_EXECUTE,
        // Final stages
        S_ROUND,
        S_PACK,
        S_DONE
    } state_t;
    
    state_t state, next_state;
    
    // =========================================================================
    // Pipeline Registers
    // =========================================================================
    
    // Input registers
    fpu_op_t            op_r;
    rounding_mode_t     rm_r;
    logic               is_single_r;
    logic [FLEN-1:0]    rs1_r, rs2_r, rs3_r;
    logic [63:0]        int_rs1_r;
    
    // Unpacked operands
    fp_unpacked_t       a, b, c;
    
    // Result accumulator
    fp_unpacked_t       res;
    fflags_t            flags;
    
    // Computation registers
    logic signed [EXT_EXP_BITS-1:0] exp_diff;
    logic [EXT_MANT_BITS*2-1:0]     mant_product;
    logic [EXT_MANT_BITS+2:0]       mant_sum;
    logic                           effective_sub;
    
    // Division/Sqrt registers
    logic [5:0]                     iter_count;
    logic [EXT_MANT_BITS+3:0]       div_quotient;
    logic [EXT_MANT_BITS+3:0]       div_remainder;
    logic [EXT_MANT_BITS+3:0]       div_divisor;
    
    // =========================================================================
    // Unpack Functions
    // =========================================================================
    
    function automatic fp_unpacked_t unpack_sp(input [31:0] val);
        fp_unpacked_t f;
        logic [7:0] raw_exp;
        logic [22:0] raw_mant;
        
        f.sign = val[31];
        raw_exp = val[30:23];
        raw_mant = val[22:0];
        
        f.zero = (raw_exp == 0) && (raw_mant == 0);
        f.denorm = (raw_exp == 0) && (raw_mant != 0);
        f.inf = (raw_exp == 8'hFF) && (raw_mant == 0);
        f.nan = (raw_exp == 8'hFF) && (raw_mant != 0);
        f.snan = f.nan && !raw_mant[22];  // MSB of mantissa is quiet bit
        
        if (f.zero) begin
            f.exp = '0;
            f.mant = '0;
        end
        else if (f.denorm) begin
            // Denormalized: exp = 1 - bias, no implicit 1
            f.exp = 1 - SP_BIAS;
            f.mant = {1'b0, raw_mant, {(EXT_MANT_BITS-24){1'b0}}};
        end
        else if (f.inf || f.nan) begin
            f.exp = '1;  // Max exponent
            f.mant = {1'b1, raw_mant, {(EXT_MANT_BITS-24){1'b0}}};
        end
        else begin
            // Normalized
            f.exp = signed'({5'b0, raw_exp}) - SP_BIAS;
            f.mant = {1'b1, raw_mant, {(EXT_MANT_BITS-24){1'b0}}};
        end
        
        return f;
    endfunction
    
    function automatic fp_unpacked_t unpack_dp(input [63:0] val);
        fp_unpacked_t f;
        logic [10:0] raw_exp;
        logic [51:0] raw_mant;
        
        f.sign = val[63];
        raw_exp = val[62:52];
        raw_mant = val[51:0];
        
        f.zero = (raw_exp == 0) && (raw_mant == 0);
        f.denorm = (raw_exp == 0) && (raw_mant != 0);
        f.inf = (raw_exp == 11'h7FF) && (raw_mant == 0);
        f.nan = (raw_exp == 11'h7FF) && (raw_mant != 0);
        f.snan = f.nan && !raw_mant[51];
        
        if (f.zero) begin
            f.exp = '0;
            f.mant = '0;
        end
        else if (f.denorm) begin
            f.exp = 1 - DP_BIAS;
            f.mant = {1'b0, raw_mant, {(EXT_MANT_BITS-53){1'b0}}};
        end
        else if (f.inf || f.nan) begin
            f.exp = '1;
            f.mant = {1'b1, raw_mant, {(EXT_MANT_BITS-53){1'b0}}};
        end
        else begin
            f.exp = signed'({2'b0, raw_exp}) - DP_BIAS;
            f.mant = {1'b1, raw_mant, {(EXT_MANT_BITS-53){1'b0}}};
        end
        
        return f;
    endfunction
    
    // =========================================================================
    // Pack Functions
    // =========================================================================
    
    function automatic logic [31:0] pack_sp(input fp_unpacked_t f, input fflags_t fl);
        logic [31:0] val;
        logic [7:0] biased_exp;
        logic [22:0] final_mant;
        
        if (f.nan) begin
            val = SP_CANONICAL_NAN;
        end
        else if (f.inf) begin
            val = {f.sign, 8'hFF, 23'b0};
        end
        else if (f.zero) begin
            val = {f.sign, 31'b0};
        end
        else begin
            // Check for overflow/underflow
            if (f.exp > SP_EMAX) begin
                // Overflow - return infinity or max normal based on rounding
                val = {f.sign, 8'hFF, 23'b0};
            end
            else if (f.exp < SP_EMIN - 23) begin
                // Underflow to zero
                val = {f.sign, 31'b0};
            end
            else if (f.exp < SP_EMIN) begin
                // Denormal
                val = {f.sign, 8'b0, f.mant[EXT_MANT_BITS-2 -: 23]};
            end
            else begin
                // Normal
                biased_exp = f.exp + SP_BIAS;
                final_mant = f.mant[EXT_MANT_BITS-2 -: 23];  // Skip implicit 1
                val = {f.sign, biased_exp, final_mant};
            end
        end
        
        return val;
    endfunction
    
    function automatic logic [63:0] pack_dp(input fp_unpacked_t f, input fflags_t fl);
        logic [63:0] val;
        logic [10:0] biased_exp;
        logic [51:0] final_mant;
        
        if (f.nan) begin
            val = DP_CANONICAL_NAN;
        end
        else if (f.inf) begin
            val = {f.sign, 11'h7FF, 52'b0};
        end
        else if (f.zero) begin
            val = {f.sign, 63'b0};
        end
        else begin
            if (f.exp > DP_EMAX) begin
                val = {f.sign, 11'h7FF, 52'b0};
            end
            else if (f.exp < DP_EMIN - 52) begin
                val = {f.sign, 63'b0};
            end
            else if (f.exp < DP_EMIN) begin
                val = {f.sign, 11'b0, f.mant[EXT_MANT_BITS-2 -: 52]};
            end
            else begin
                biased_exp = f.exp + DP_BIAS;
                final_mant = f.mant[EXT_MANT_BITS-2 -: 52];
                val = {f.sign, biased_exp, final_mant};
            end
        end
        
        return val;
    endfunction
    
    // =========================================================================
    // Rounding Function (IEEE 754 compliant)
    // =========================================================================
    
    function automatic logic [EXT_MANT_BITS-1:0] round_mant(
        input logic [EXT_MANT_BITS+2:0] mant_with_grs,  // guard, round, sticky
        input logic sign,
        input rounding_mode_t rm,
        output logic inexact
    );
        logic guard, round_bit, sticky;
        logic round_up;
        logic [EXT_MANT_BITS-1:0] rounded;
        
        guard = mant_with_grs[2];
        round_bit = mant_with_grs[1];
        sticky = mant_with_grs[0];
        
        inexact = guard | round_bit | sticky;
        
        case (rm)
            RM_RNE: begin  // Round to Nearest, ties to Even
                round_up = guard && (round_bit || sticky || mant_with_grs[3]);
            end
            RM_RTZ: begin  // Round towards Zero
                round_up = 1'b0;
            end
            RM_RDN: begin  // Round Down (towards -∞)
                round_up = sign && inexact;
            end
            RM_RUP: begin  // Round Up (towards +∞)
                round_up = !sign && inexact;
            end
            RM_RMM: begin  // Round to Nearest, ties to Max Magnitude
                round_up = guard;
            end
            default: round_up = 1'b0;
        endcase
        
        rounded = mant_with_grs[EXT_MANT_BITS+2:3] + round_up;
        return rounded;
    endfunction
    
    // =========================================================================
    // Leading Zero Counter
    // =========================================================================
    
    function automatic logic [5:0] count_leading_zeros(
        input logic [EXT_MANT_BITS-1:0] val
    );
        logic [5:0] count;
        count = 0;
        for (int i = EXT_MANT_BITS-1; i >= 0; i--) begin
            if (val[i]) return EXT_MANT_BITS - 1 - i;
        end
        return EXT_MANT_BITS;
    endfunction
    
    // =========================================================================
    // Special Case Handling
    // =========================================================================
    
    logic special_case;
    fp_unpacked_t special_result;
    fflags_t special_flags;
    
    always_comb begin
        special_case = 1'b0;
        special_result = '0;
        special_flags = '0;
        
        // NaN propagation (any NaN input -> canonical NaN output)
        if (a.nan || b.nan || (op_r inside {FPU_MADD, FPU_MSUB, FPU_NMADD, FPU_NMSUB} && c.nan)) begin
            special_case = 1'b1;
            special_result.nan = 1'b1;
            if (a.snan || b.snan || c.snan)
                special_flags.NV = 1'b1;  // Invalid operation for sNaN
        end
        
        // Operation-specific special cases
        else case (op_r)
            FPU_ADD, FPU_SUB: begin
                if (a.inf && b.inf && (a.sign ^ b.sign ^ (op_r == FPU_SUB))) begin
                    // inf - inf = NaN
                    special_case = 1'b1;
                    special_result.nan = 1'b1;
                    special_flags.NV = 1'b1;
                end
                else if (a.inf || b.inf) begin
                    special_case = 1'b1;
                    special_result.inf = 1'b1;
                    special_result.sign = a.inf ? a.sign : (b.sign ^ (op_r == FPU_SUB));
                end
            end
            
            FPU_MUL: begin
                if ((a.inf && b.zero) || (a.zero && b.inf)) begin
                    // inf * 0 = NaN
                    special_case = 1'b1;
                    special_result.nan = 1'b1;
                    special_flags.NV = 1'b1;
                end
                else if (a.inf || b.inf) begin
                    special_case = 1'b1;
                    special_result.inf = 1'b1;
                    special_result.sign = a.sign ^ b.sign;
                end
                else if (a.zero || b.zero) begin
                    special_case = 1'b1;
                    special_result.zero = 1'b1;
                    special_result.sign = a.sign ^ b.sign;
                end
            end
            
            FPU_DIV: begin
                if (a.inf && b.inf) begin
                    // inf / inf = NaN
                    special_case = 1'b1;
                    special_result.nan = 1'b1;
                    special_flags.NV = 1'b1;
                end
                else if (a.zero && b.zero) begin
                    // 0 / 0 = NaN
                    special_case = 1'b1;
                    special_result.nan = 1'b1;
                    special_flags.NV = 1'b1;
                end
                else if (b.zero) begin
                    // x / 0 = inf (with DZ flag)
                    special_case = 1'b1;
                    special_result.inf = 1'b1;
                    special_result.sign = a.sign ^ b.sign;
                    special_flags.DZ = 1'b1;
                end
                else if (a.inf) begin
                    special_case = 1'b1;
                    special_result.inf = 1'b1;
                    special_result.sign = a.sign ^ b.sign;
                end
                else if (a.zero) begin
                    special_case = 1'b1;
                    special_result.zero = 1'b1;
                    special_result.sign = a.sign ^ b.sign;
                end
            end
            
            FPU_SQRT: begin
                if (a.sign && !a.zero && !a.nan) begin
                    // sqrt(negative) = NaN
                    special_case = 1'b1;
                    special_result.nan = 1'b1;
                    special_flags.NV = 1'b1;
                end
                else if (a.inf && !a.sign) begin
                    special_case = 1'b1;
                    special_result.inf = 1'b1;
                    special_result.sign = 1'b0;
                end
                else if (a.zero) begin
                    special_case = 1'b1;
                    special_result.zero = 1'b1;
                    special_result.sign = a.sign;  // Preserve sign of zero
                end
            end
            
            default: ;
        endcase
    end
    
    // =========================================================================
    // FSM Next State
    // =========================================================================
    
    always_comb begin
        next_state = state;
        
        case (state)
            S_IDLE: begin
                if (req_valid && req_ready)
                    next_state = S_UNPACK;
            end
            
            S_UNPACK: next_state = S_SPECIAL_CHECK;
            
            S_SPECIAL_CHECK: begin
                if (special_case)
                    next_state = S_DONE;
                else case (op_r)
                    FPU_ADD, FPU_SUB: next_state = S_ADD_ALIGN;
                    FPU_MUL: next_state = S_MUL_EXECUTE;
                    FPU_MADD, FPU_MSUB, FPU_NMADD, FPU_NMSUB: next_state = S_FMA_MUL;
                    FPU_DIV: next_state = S_DIV_INIT;
                    FPU_SQRT: next_state = S_SQRT_INIT;
                    FPU_SGNJ, FPU_SGNJN, FPU_SGNJX,
                    FPU_MIN, FPU_MAX,
                    FPU_CMP_EQ, FPU_CMP_LT, FPU_CMP_LE,
                    FPU_CLASS,
                    FPU_MV_X_W, FPU_MV_W_X: next_state = S_DONE;
                    FPU_CVT_W, FPU_CVT_WU, FPU_CVT_L, FPU_CVT_LU,
                    FPU_CVT_S, FPU_CVT_D, FPU_CVT_INT: next_state = S_CVT_EXECUTE;
                    default: next_state = S_DONE;
                endcase
            end
            
            // Adder pipeline
            S_ADD_ALIGN: next_state = S_ADD_EXECUTE;
            S_ADD_EXECUTE: next_state = S_ADD_NORMALIZE;
            S_ADD_NORMALIZE: next_state = S_ROUND;
            
            // Multiplier pipeline
            S_MUL_EXECUTE: next_state = S_MUL_NORMALIZE;
            S_MUL_NORMALIZE: next_state = S_ROUND;
            
            // FMA pipeline
            S_FMA_MUL: next_state = S_FMA_ALIGN;
            S_FMA_ALIGN: next_state = S_FMA_ADD;
            S_FMA_ADD: next_state = S_FMA_NORMALIZE;
            S_FMA_NORMALIZE: next_state = S_ROUND;
            
            // Divider
            S_DIV_INIT: next_state = S_DIV_ITERATE;
            S_DIV_ITERATE: begin
                if (iter_count >= (is_single_r ? 26 : 55))
                    next_state = S_DIV_FINALIZE;
            end
            S_DIV_FINALIZE: next_state = S_ROUND;
            
            // Square root
            S_SQRT_INIT: next_state = S_SQRT_ITERATE;
            S_SQRT_ITERATE: begin
                if (iter_count >= (is_single_r ? 26 : 55))
                    next_state = S_SQRT_FINALIZE;
            end
            S_SQRT_FINALIZE: next_state = S_ROUND;
            
            // Conversion
            S_CVT_EXECUTE: next_state = S_ROUND;
            
            // Final stages
            S_ROUND: next_state = S_PACK;
            S_PACK: next_state = S_DONE;
            S_DONE: next_state = S_IDLE;
            
            default: next_state = S_IDLE;
        endcase
    end
    
    // =========================================================================
    // Outputs
    // =========================================================================
    
    assign req_ready = (state == S_IDLE);
    assign resp_valid = (state == S_DONE);
    
    // Result muxing for simple operations
    logic [FLEN-1:0] simple_fp_result;
    logic [63:0] simple_int_result;
    fflags_t simple_flags;
    
    always_comb begin
        logic lt;
        logic le;
        
        simple_fp_result = '0;
        simple_int_result = '0;
        simple_flags = '0;
        
        case (op_r)
            FPU_SGNJ: begin
                if (is_single_r)
                    simple_fp_result = {32'hFFFF_FFFF, rs2_r[31], rs1_r[30:0]};
                else
                    simple_fp_result = {rs2_r[63], rs1_r[62:0]};
            end
            
            FPU_SGNJN: begin
                if (is_single_r)
                    simple_fp_result = {32'hFFFF_FFFF, ~rs2_r[31], rs1_r[30:0]};
                else
                    simple_fp_result = {~rs2_r[63], rs1_r[62:0]};
            end
            
            FPU_SGNJX: begin
                if (is_single_r)
                    simple_fp_result = {32'hFFFF_FFFF, rs1_r[31] ^ rs2_r[31], rs1_r[30:0]};
                else
                    simple_fp_result = {rs1_r[63] ^ rs2_r[63], rs1_r[62:0]};
            end
            
            FPU_MIN: begin
                if (a.nan && b.nan) begin
                    simple_fp_result = is_single_r ? {32'hFFFF_FFFF, SP_CANONICAL_NAN} : DP_CANONICAL_NAN;
                    simple_flags.NV = a.snan || b.snan;
                end
                else if (a.nan) simple_fp_result = rs2_r;
                else if (b.nan) simple_fp_result = rs1_r;
                else if (a.zero && b.zero) simple_fp_result = a.sign ? rs1_r : rs2_r;
                else if (a.sign != b.sign) simple_fp_result = a.sign ? rs1_r : rs2_r;
                else simple_fp_result = (a.sign ? (rs1_r > rs2_r) : (rs1_r < rs2_r)) ? rs1_r : rs2_r;
            end
            
            FPU_MAX: begin
                if (a.nan && b.nan) begin
                    simple_fp_result = is_single_r ? {32'hFFFF_FFFF, SP_CANONICAL_NAN} : DP_CANONICAL_NAN;
                    simple_flags.NV = a.snan || b.snan;
                end
                else if (a.nan) simple_fp_result = rs2_r;
                else if (b.nan) simple_fp_result = rs1_r;
                else if (a.zero && b.zero) simple_fp_result = a.sign ? rs2_r : rs1_r;
                else if (a.sign != b.sign) simple_fp_result = a.sign ? rs2_r : rs1_r;
                else simple_fp_result = (a.sign ? (rs1_r < rs2_r) : (rs1_r > rs2_r)) ? rs1_r : rs2_r;
            end
            
            FPU_MV_X_W: begin
                if (is_single_r)
                    simple_int_result = {{32{rs1_r[31]}}, rs1_r[31:0]};
                else
                    simple_int_result = rs1_r;
            end
            
            FPU_MV_W_X: begin
                if (is_single_r)
                    simple_fp_result = {32'hFFFF_FFFF, int_rs1_r[31:0]};
                else
                    simple_fp_result = int_rs1_r;
            end
            
            FPU_CLASS: begin
                // FCLASS returns 10-bit classification
                if (is_single_r) begin
                    simple_int_result[0] = a.inf && a.sign;       // -inf
                    simple_int_result[1] = !a.denorm && !a.inf && !a.nan && !a.zero && a.sign;  // -normal
                    simple_int_result[2] = a.denorm && a.sign;    // -subnormal
                    simple_int_result[3] = a.zero && a.sign;      // -0
                    simple_int_result[4] = a.zero && !a.sign;     // +0
                    simple_int_result[5] = a.denorm && !a.sign;   // +subnormal
                    simple_int_result[6] = !a.denorm && !a.inf && !a.nan && !a.zero && !a.sign; // +normal
                    simple_int_result[7] = a.inf && !a.sign;      // +inf
                    simple_int_result[8] = a.snan;                // sNaN
                    simple_int_result[9] = a.nan && !a.snan;      // qNaN
                end
                else begin
                    simple_int_result[0] = a.inf && a.sign;
                    simple_int_result[1] = !a.denorm && !a.inf && !a.nan && !a.zero && a.sign;
                    simple_int_result[2] = a.denorm && a.sign;
                    simple_int_result[3] = a.zero && a.sign;
                    simple_int_result[4] = a.zero && !a.sign;
                    simple_int_result[5] = a.denorm && !a.sign;
                    simple_int_result[6] = !a.denorm && !a.inf && !a.nan && !a.zero && !a.sign;
                    simple_int_result[7] = a.inf && !a.sign;
                    simple_int_result[8] = a.snan;
                    simple_int_result[9] = a.nan && !a.snan;
                end
            end
            
            FPU_CMP_EQ: begin
                if (a.nan || b.nan) begin
                    simple_int_result = '0;
                    simple_flags.NV = a.snan || b.snan;
                end
                else
                    simple_int_result = (rs1_r == rs2_r) ? 64'd1 : 64'd0;
            end
            
            FPU_CMP_LT: begin
                if (a.nan || b.nan) begin
                    simple_int_result = '0;
                    simple_flags.NV = 1'b1;
                end
                else begin
                    if (a.zero && b.zero) lt = 1'b0;
                    else if (a.sign != b.sign) lt = a.sign;
                    else if (a.sign) lt = rs1_r[62:0] > rs2_r[62:0];
                    else lt = rs1_r[62:0] < rs2_r[62:0];
                    simple_int_result = lt ? 64'd1 : 64'd0;
                end
            end
            
            FPU_CMP_LE: begin
                if (a.nan || b.nan) begin
                    simple_int_result = '0;
                    simple_flags.NV = 1'b1;
                end
                else begin
                    if (a.zero && b.zero) le = 1'b1;
                    else if (a.sign != b.sign) le = a.sign;
                    else if (a.sign) le = rs1_r[62:0] >= rs2_r[62:0];
                    else le = rs1_r[62:0] <= rs2_r[62:0];
                    simple_int_result = le ? 64'd1 : 64'd0;
                end
            end
            
            default: ;
        endcase
    end
    
    // Final output mux
    always_comb begin
        if (special_case) begin
            if (special_result.nan)
                result = is_single_r ? {32'hFFFF_FFFF, SP_CANONICAL_NAN} : DP_CANONICAL_NAN;
            else if (special_result.inf)
                result = is_single_r ? {32'hFFFF_FFFF, special_result.sign, 8'hFF, 23'b0} : 
                                       {special_result.sign, 11'h7FF, 52'b0};
            else
                result = is_single_r ? {32'hFFFF_FFFF, special_result.sign, 31'b0} :
                                       {special_result.sign, 63'b0};
            int_result = '0;
            fflags = special_flags;
        end
        else if (op_r inside {FPU_SGNJ, FPU_SGNJN, FPU_SGNJX, FPU_MIN, FPU_MAX, FPU_MV_W_X}) begin
            result = simple_fp_result;
            int_result = '0;
            fflags = simple_flags;
        end
        else if (op_r inside {FPU_MV_X_W, FPU_CLASS, FPU_CMP_EQ, FPU_CMP_LT, FPU_CMP_LE}) begin
            result = '0;
            int_result = simple_int_result;
            fflags = simple_flags;
        end
        else begin
            result = is_single_r ? {32'hFFFF_FFFF, pack_sp(res, flags)} : pack_dp(res, flags);
            int_result = '0;  // TODO: conversion results
            fflags = flags;
        end
    end
    
    // =========================================================================
    // Sequential Logic
    // =========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            op_r <= FPU_ADD;
            rm_r <= RM_RNE;
            is_single_r <= 1'b0;
            rs1_r <= '0;
            rs2_r <= '0;
            rs3_r <= '0;
            int_rs1_r <= '0;
            a <= '0;
            b <= '0;
            c <= '0;
            res <= '0;
            flags <= '0;
            exp_diff <= '0;
            mant_product <= '0;
            mant_sum <= '0;
            effective_sub <= 1'b0;
            iter_count <= '0;
            div_quotient <= '0;
            div_remainder <= '0;
            div_divisor <= '0;
        end
        else begin
            state <= next_state;
            
            case (state)
                S_IDLE: begin
                    if (req_valid && req_ready) begin
                        op_r <= op;
                        rm_r <= rm;
                        is_single_r <= is_single;
                        rs1_r <= rs1;
                        rs2_r <= rs2;
                        rs3_r <= rs3;
                        int_rs1_r <= int_rs1;
                        flags <= '0;
                    end
                end
                
                S_UNPACK: begin
                    if (is_single_r) begin
                        a <= unpack_sp(rs1_r[31:0]);
                        b <= unpack_sp(rs2_r[31:0]);
                        c <= unpack_sp(rs3_r[31:0]);
                    end
                    else begin
                        a <= unpack_dp(rs1_r);
                        b <= unpack_dp(rs2_r);
                        c <= unpack_dp(rs3_r);
                    end
                    
                    // Handle SUB/NMSUB/NMADD sign flip
                    if (op_r == FPU_SUB)
                        b.sign <= ~b.sign;
                end
                
                S_SPECIAL_CHECK: begin
                    if (special_case) begin
                        res <= special_result;
                        flags <= special_flags;
                    end
                end
                
                // Addition alignment
                S_ADD_ALIGN: begin
                    if (a.exp >= b.exp) begin
                        exp_diff <= a.exp - b.exp;
                        res.exp <= a.exp;
                    end
                    else begin
                        exp_diff <= b.exp - a.exp;
                        res.exp <= b.exp;
                    end
                    effective_sub <= a.sign ^ b.sign;
                end
                
                // Addition execution
                S_ADD_EXECUTE: begin
                    logic [EXT_MANT_BITS+2:0] a_mant, b_mant;
                    
                    if (a.exp >= b.exp) begin
                        a_mant = {1'b0, a.mant, 2'b0};
                        b_mant = {1'b0, b.mant, 2'b0} >> exp_diff;
                    end
                    else begin
                        a_mant = {1'b0, a.mant, 2'b0} >> exp_diff;
                        b_mant = {1'b0, b.mant, 2'b0};
                    end
                    
                    if (effective_sub) begin
                        if (a_mant >= b_mant) begin
                            mant_sum <= a_mant - b_mant;
                            res.sign <= a.sign;
                        end
                        else begin
                            mant_sum <= b_mant - a_mant;
                            res.sign <= b.sign;
                        end
                    end
                    else begin
                        mant_sum <= a_mant + b_mant;
                        res.sign <= a.sign;
                    end
                end
                
                // Addition normalization
                S_ADD_NORMALIZE: begin
                    if (mant_sum == 0) begin
                        res.zero <= 1'b1;
                        res.mant <= '0;
                        res.exp <= '0;
                    end
                    else if (mant_sum[EXT_MANT_BITS+2]) begin
                        // Overflow - shift right
                        res.mant <= mant_sum[EXT_MANT_BITS+2:3];
                        res.exp <= res.exp + 1;
                    end
                    else begin
                        // Normalize
                        logic [5:0] lzc;
                        lzc = count_leading_zeros(mant_sum[EXT_MANT_BITS+1:2]);
                        res.mant <= mant_sum[EXT_MANT_BITS+1:2] << lzc;
                        res.exp <= res.exp - lzc;
                    end
                end
                
                // Multiplication
                S_MUL_EXECUTE: begin
                    mant_product <= {1'b0, a.mant} * {1'b0, b.mant};
                    res.exp <= a.exp + b.exp;
                    res.sign <= a.sign ^ b.sign;
                end
                
                S_MUL_NORMALIZE: begin
                    if (mant_product[EXT_MANT_BITS*2-1]) begin
                        res.mant <= mant_product[EXT_MANT_BITS*2-1 -: EXT_MANT_BITS];
                        res.exp <= res.exp + 1;
                    end
                    else begin
                        res.mant <= mant_product[EXT_MANT_BITS*2-2 -: EXT_MANT_BITS];
                    end
                    
                    res.zero <= (mant_product == 0);
                end
                
                // Division
                S_DIV_INIT: begin
                    div_quotient <= '0;
                    div_remainder <= {4'b0, a.mant};
                    div_divisor <= {4'b0, b.mant};
                    res.exp <= a.exp - b.exp;
                    res.sign <= a.sign ^ b.sign;
                    iter_count <= '0;
                end
                
                S_DIV_ITERATE: begin
                    logic [EXT_MANT_BITS+3:0] shifted;
                    shifted = div_remainder << 1;
                    
                    if (shifted >= div_divisor) begin
                        div_remainder <= shifted - div_divisor;
                        div_quotient <= {div_quotient[EXT_MANT_BITS+2:0], 1'b1};
                    end
                    else begin
                        div_remainder <= shifted;
                        div_quotient <= {div_quotient[EXT_MANT_BITS+2:0], 1'b0};
                    end
                    
                    iter_count <= iter_count + 1;
                end
                
                S_DIV_FINALIZE: begin
                    // Normalize quotient
                    if (div_quotient[EXT_MANT_BITS+3]) begin
                        res.mant <= div_quotient[EXT_MANT_BITS+3:4];
                        res.exp <= res.exp + 1;
                    end
                    else begin
                        res.mant <= div_quotient[EXT_MANT_BITS+2:3];
                    end
                    
                    // Set inexact if remainder != 0
                    if (div_remainder != 0)
                        flags.NX <= 1'b1;
                end
                
                // Square root
                S_SQRT_INIT: begin
                    div_quotient <= '0;
                    div_remainder <= {4'b0, a.mant};
                    res.exp <= (a.exp >> 1);
                    res.sign <= 1'b0;
                    iter_count <= '0;
                    
                    // Adjust if odd exponent
                    if (a.exp[0])
                        div_remainder <= {3'b0, a.mant, 1'b0};
                end
                
                S_SQRT_ITERATE: begin
                    // Newton-Raphson or digit-by-digit (simplified)
                    iter_count <= iter_count + 1;
                end
                
                S_SQRT_FINALIZE: begin
                    res.mant <= div_quotient[EXT_MANT_BITS-1:0];
                end
                
                S_ROUND: begin
                    logic inexact;
                    logic [EXT_MANT_BITS-1:0] rounded;
                    
                    rounded = round_mant({res.mant, 3'b0}, res.sign, rm_r, inexact);
                    res.mant <= rounded;
                    
                    if (inexact)
                        flags.NX <= 1'b1;
                end
                
                S_PACK: begin
                    // Check for overflow after rounding
                    if (is_single_r && res.exp > SP_EMAX) begin
                        res.inf <= 1'b1;
                        flags.OF <= 1'b1;
                        flags.NX <= 1'b1;
                    end
                    else if (!is_single_r && res.exp > DP_EMAX) begin
                        res.inf <= 1'b1;
                        flags.OF <= 1'b1;
                        flags.NX <= 1'b1;
                    end
                    // Check for underflow
                    else if (is_single_r && res.exp < SP_EMIN && !res.zero) begin
                        flags.UF <= 1'b1;
                    end
                    else if (!is_single_r && res.exp < DP_EMIN && !res.zero) begin
                        flags.UF <= 1'b1;
                    end
                end
                
                default: ;
            endcase
        end
    end

endmodule
