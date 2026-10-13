`timescale 1ns/1ps
`default_nettype none

/**
 * @module nebula_div
 * @brief Divisor iterativo RV64M (DIV/DIVU/REM/REMU e variantes W).
 *
 * 2 bits de quociente por ciclo, pulando os zeros à esquerda do dividendo.
 * Divisão por zero, overflow e dividendo zero resolvidos em 1 ciclo.
 * Protocolo: 'start' (1 ciclo, em IDLE) -> 'done' pulsa com 'result' válido.
 */
module nebula_div (
    input  wire          clk,
    input  wire          rst_n,
    input  wire          start,
    input  wire          kill,
    input  wire  [2:0]   funct3,     // 100 DIV, 101 DIVU, 110 REM, 111 REMU
    input  wire          word,
    input  wire  [63:0]  a,
    input  wire  [63:0]  b,
    output logic         done,
    output logic [63:0]  result
);
    logic is_signed, is_rem;
    assign is_signed = !funct3[0];
    assign is_rem    = funct3[1];

    logic [63:0] a_ext, b_ext;
    logic        a_neg, b_neg;
    always_comb begin
        if (word) begin
            a_ext = is_signed ? {{32{a[31]}}, a[31:0]} : {32'b0, a[31:0]};
            b_ext = is_signed ? {{32{b[31]}}, b[31:0]} : {32'b0, b[31:0]};
        end else begin
            a_ext = a;
            b_ext = b;
        end
        a_neg = is_signed && a_ext[63];
        b_neg = is_signed && b_ext[63];
    end

    typedef enum logic [1:0] { D_IDLE, D_RUN, D_DONE } dstate_t;
    dstate_t state;

    logic [63:0] divisor, quot, dividend;
    logic [64:0] rem;
    logic [6:0]  count;
    logic        neg_q, neg_r, rem_q, word_q, special;
    logic [63:0] special_res;

    function automatic logic [6:0] lzc64(input logic [63:0] v);
        lzc64 = 7'd64;
        for (int i = 0; i < 64; i++) if (v[i]) lzc64 = 7'(63 - i);
    endfunction

    // passo radix-2 de divisão com restauração
    function automatic logic [128:0] step(input logic [64:0] r, input logic [63:0] q,
                                          input logic dbit, input logic [63:0] dv);
        logic [64:0] r2;
        logic        qb;
        r2 = {r[63:0], dbit};
        qb = (r2 >= {1'b0, dv});
        if (qb) r2 = r2 - {1'b0, dv};
        return {r2, q[62:0], qb};
    endfunction

    assign done = (state == D_DONE);

    always_comb begin
        logic [63:0] qf, rf, o;
        qf = neg_q ? -quot : quot;
        rf = neg_r ? -rem[63:0] : rem[63:0];
        o  = special ? special_res : (rem_q ? rf : qf);
        if (word_q) o = {{32{o[31]}}, o[31:0]};
        result = o;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= D_IDLE;
            divisor <= '0; quot <= '0; rem <= '0; dividend <= '0; count <= '0;
            neg_q <= 0; neg_r <= 0; rem_q <= 0; word_q <= 0; special <= 0; special_res <= '0;
        end else if (kill) begin
            state <= D_IDLE;
        end else begin
            case (state)
                D_IDLE: begin
                    if (start) begin
                        logic [63:0] ua, ub;
                        logic [6:0]  lz;
                        ua = a_neg ? -a_ext : a_ext;
                        ub = b_neg ? -b_ext : b_ext;
                        lz = lzc64(ua);
                        rem_q  <= is_rem;
                        word_q <= word;
                        special <= 1'b0;
                        if (b_ext == 64'd0) begin
                            special     <= 1'b1;
                            special_res <= is_rem ? a_ext : 64'hFFFF_FFFF_FFFF_FFFF;
                            state       <= D_DONE;
                        end else if (is_signed && b_ext == 64'hFFFF_FFFF_FFFF_FFFF &&
                                     a_ext == (word ? 64'hFFFF_FFFF_8000_0000 : 64'h8000_0000_0000_0000)) begin
                            special     <= 1'b1;
                            special_res <= is_rem ? 64'd0 : a_ext;
                            state       <= D_DONE;
                        end else if (lz == 7'd64) begin
                            special     <= 1'b1;
                            special_res <= 64'd0;
                            state       <= D_DONE;
                        end else begin
                            divisor  <= ub;
                            dividend <= ua << lz;
                            quot     <= '0;
                            rem      <= '0;
                            count    <= 7'd64 - lz;
                            neg_q    <= a_neg ^ b_neg;
                            neg_r    <= a_neg;
                            state    <= D_RUN;
                        end
                    end
                end
                D_RUN: begin
                    logic [128:0] s1, s2;
                    s1 = step(rem, quot, dividend[63], divisor);
                    if (count >= 7'd2) begin
                        s2 = step(s1[128:64], s1[63:0], dividend[62], divisor);
                        rem      <= s2[128:64];
                        quot     <= s2[63:0];
                        dividend <= {dividend[61:0], 2'b00};
                        count    <= count - 7'd2;
                        if (count == 7'd2) state <= D_DONE;
                    end else begin
                        rem      <= s1[128:64];
                        quot     <= s1[63:0];
                        dividend <= {dividend[62:0], 1'b0};
                        count    <= count - 7'd1;
                        state    <= D_DONE;
                    end
                end
                D_DONE: state <= D_IDLE;
                default: state <= D_IDLE;
            endcase
        end
    end

endmodule
