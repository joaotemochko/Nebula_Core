`timescale 1ns/1ps
`default_nettype none

import nebula_pkg::*;

/**
 * @module nebula_fpu
 * @brief FPU IEEE 754-2008 para RV64FD (precisão simples e dupla).
 *
 * - FADD/FSUB/FMUL/FMADD/FMSUB/FNMADD/FNMSUB: caminho FMA exato (produto de
 *   106 bits + adendo alinhado em janela de 222 bits com sticky) e
 *   arredondamento único -> resultados corretamente arredondados.
 * - FDIV/FSQRT: recorrência de dígitos radix-2 (1 bit/ciclo) com sticky.
 * - Conversões FP<->FP, FP<->inteiro (W/WU/L/LU) com saturação RISC-V.
 * - FMIN/FMAX/FEQ/FLT/FLE/FCLASS/FSGNJ(N/X), FMV.
 * - NaN canônico, NaN-boxing de valores single, flags NV/DZ/OF/UF/NX,
 *   underflow com detecção de tininess APÓS o arredondamento (RISC-V).
 *
 * Latências: operações simples 1 ciclo, add/mul/fma 3 ciclos,
 * div/sqrt ~ (bits+4) ciclos. Protocolo: 'start' -> 'done' (1 ciclo).
 */
module nebula_fpu (
    input  wire          clk,
    input  wire          rst_n,
    input  wire          start,
    input  wire          kill,
    input  fpu_op_t      op,
    input  wire          dbl,
    input  wire          src_dbl,
    input  wire  [1:0]   int_sz,
    input  wire  [2:0]   rm,
    input  wire  [63:0]  a,
    input  wire  [63:0]  b,
    input  wire  [63:0]  c,
    output logic         done,
    output logic [63:0]  result,
    output logic [4:0]   fflags      // {NV, DZ, OF, UF, NX}
);
    localparam logic [63:0] QNAN_D = 64'h7FF8_0000_0000_0000;
    localparam logic [63:0] QNAN_S = 64'hFFFF_FFFF_7FC0_0000;

    // =====================================================================
    // Tipos e funções auxiliares
    // =====================================================================
    typedef struct packed {
        logic        sign;
        logic signed [15:0] exp;     // expoente não-polarizado do bit líder
        logic [52:0] mant;           // normalizada: bit 52 = 1 (se não zero)
        logic        zero, inf, nan, snan, sub;
    } unp_t;

    function automatic logic [31:0] unbox(input logic [63:0] x);
        return (x[63:32] == 32'hFFFF_FFFF) ? x[31:0] : 32'h7FC0_0000;
    endfunction

    function automatic unp_t unpack(input logic [63:0] x, input logic d);
        unp_t u;
        logic [10:0] e;
        logic [51:0] f;
        logic [31:0] s;
        u = '0;
        if (d) begin
            u.sign = x[63]; e = x[62:52]; f = x[51:0];
            u.zero = (e == 0) && (f == 0);
            u.sub  = (e == 0) && (f != 0);
            u.inf  = (e == 11'h7FF) && (f == 0);
            u.nan  = (e == 11'h7FF) && (f != 0);
            u.snan = u.nan && !f[51];
            if (e != 0) begin
                u.exp  = 16'(e) - 16'sd1023;
                u.mant = {1'b1, f};
            end else begin
                logic [52:0] m;
                int lz;
                m = {1'b0, f};
                lz = 0;
                for (int i = 52; i >= 0; i--) if (m[i]) begin lz = 52 - i; break; end
                u.mant = m << lz;
                u.exp  = -16'sd1022 - 16'(lz);
            end
        end else begin
            s = unbox(x);
            u.sign = s[31];
            u.zero = (s[30:23] == 0) && (s[22:0] == 0);
            u.sub  = (s[30:23] == 0) && (s[22:0] != 0);
            u.inf  = (s[30:23] == 8'hFF) && (s[22:0] == 0);
            u.nan  = (s[30:23] == 8'hFF) && (s[22:0] != 0);
            u.snan = u.nan && !s[22];
            if (s[30:23] != 0) begin
                u.exp  = 16'(s[30:23]) - 16'sd127;
                u.mant = {1'b1, s[22:0], 29'b0};
            end else begin
                logic [52:0] m;
                int lz;
                m = {1'b0, s[22:0], 29'b0};
                lz = 0;
                for (int i = 52; i >= 0; i--) if (m[i]) begin lz = 52 - i; break; end
                u.mant = m << lz;
                u.exp  = -16'sd126 - 16'(lz);
            end
        end
        return u;
    endfunction

    function automatic logic [63:0] pack_inf(input logic s, input logic d);
        return d ? {s, 11'h7FF, 52'b0} : {32'hFFFF_FFFF, s, 8'hFF, 23'b0};
    endfunction
    function automatic logic [63:0] pack_zero(input logic s, input logic d);
        return d ? {s, 63'b0} : {32'hFFFF_FFFF, s, 31'b0};
    endfunction
    function automatic logic [63:0] qnan(input logic d);
        return d ? QNAN_D : QNAN_S;
    endfunction

    // ---------------------------------------------------------------------
    // Arredondamento + empacotamento
    //   sig: bit 63 = bit líder (1), e = expoente do bit líder, st = sticky
    //   retorna {resultado[63:0], flags[4:0]}
    // ---------------------------------------------------------------------
    function automatic logic rinc(input logic [2:0] r, input logic s, input logic lsb,
                                  input logic rb, input logic st);
        case (r)
            3'b000:  return rb && (st || lsb);   // RNE
            3'b001:  return 1'b0;                // RTZ
            3'b010:  return s && (rb || st);     // RDN
            3'b011:  return !s && (rb || st);    // RUP
            default: return rb;                  // RMM
        endcase
    endfunction

    function automatic logic [68:0] round_pack(input logic s, input int e_in, input logic [63:0] sig,
                                               input logic sticky, input logic d, input logic [2:0] r);
        int p, emin, emax, bias, e, sh;
        logic [63:0] sg;
        logic        st, rb, inc, nx, of, uf, tiny;
        logic [53:0] keep, mant;
        logic [63:0] res;
        p    = d ? 53 : 24;
        emin = d ? -1022 : -126;
        emax = d ? 1023 : 127;
        bias = d ? 1023 : 127;
        e    = e_in;
        sg   = sig;
        st   = sticky;

        // tininess após arredondamento (expoente ilimitado)
        tiny = 1'b0;
        if (e < emin - 1) tiny = 1'b1;
        else if (e == emin - 1) begin
            logic [53:0] kn;
            logic rbn, stn, incn;
            kn   = 54'(sg >> (64 - p));
            rbn  = sg[63 - p];
            stn  = st || ((sg & ((64'd1 << (63 - p)) - 64'd1)) != 0);
            incn = rinc(r, s, kn[0], rbn, stn);
            tiny = !((kn == ((54'd1 << p) - 54'd1)) && incn);
        end

        // desnormalização
        if (e < emin) begin
            sh = emin - e;
            if (sh >= 64) begin
                st = st || (sg != 0);
                sg = '0;
            end else begin
                st = st || ((sg & ((64'd1 << sh) - 64'd1)) != 0);
                sg = sg >> sh;
            end
            e = emin;
        end

        keep = 54'(sg >> (64 - p));
        rb   = sg[63 - p];
        st   = st || ((sg & ((64'd1 << (63 - p)) - 64'd1)) != 0);
        inc  = rinc(r, s, keep[0], rb, st);
        mant = keep + 54'(inc);
        if (mant[p]) begin
            mant = mant >> 1;
            e    = e + 1;
        end
        nx = rb || st;
        of = 1'b0;
        uf = tiny && nx;

        if (e > emax) begin
            of = 1'b1; nx = 1'b1;
            if (r == 3'b001 || (r == 3'b010 && !s) || (r == 3'b011 && s)) begin
                // maior finito
                res = d ? {s, 11'h7FE, {52{1'b1}}} : {32'hFFFF_FFFF, s, 8'hFE, {23{1'b1}}};
            end else begin
                res = pack_inf(s, d);
            end
        end else begin
            logic [10:0] be;
            be = mant[p-1] ? 11'(e + bias) : 11'd0;
            if (d) res = {s, be, mant[51:0]};
            else   res = {32'hFFFF_FFFF, s, be[7:0], mant[22:0]};
        end
        return {res, 1'b0, 1'b0, of, uf, nx};
    endfunction

    // ---------------------------------------------------------------------
    // FMA exato: (-1)^sp * ma*mb*2^(ea+eb) + (-1)^sc * mc*2^ec
    // ---------------------------------------------------------------------
    function automatic logic [68:0] fma_core(input unp_t x, input unp_t y, input unp_t z,
                                             input logic neg_prod, input logic neg_add,
                                             input logic has_add, input logic d, input logic [2:0] r);
        logic sp, sc;
        logic [63:0] res;
        logic [4:0]  fl;
        sp = x.sign ^ y.sign ^ neg_prod;
        sc = z.sign ^ neg_add;
        fl = '0;

        // casos especiais
        if (x.nan || y.nan || (has_add && z.nan) || ((x.inf && y.zero) || (x.zero && y.inf))) begin
            if (x.snan || y.snan || (has_add && z.snan) || (x.inf && y.zero) || (x.zero && y.inf))
                fl[4] = 1'b1;
            return {qnan(d), fl};
        end
        if (x.inf || y.inf) begin
            if (has_add && z.inf && (sc != sp)) return {qnan(d), 5'b10000};
            return {pack_inf(sp, d), 5'b0};
        end
        if (has_add && z.inf) return {pack_inf(sc, d), 5'b0};
        if (x.zero || y.zero) begin
            if (!has_add) return {pack_zero(sp, d), 5'b0};
            if (z.zero) begin
                if (sp == sc) return {pack_zero(sp, d), 5'b0};
                return {pack_zero(r == 3'b010, d), 5'b0};
            end
            // resultado = adendo (já representável no formato)
            return round_pack(sc, int'(z.exp), {z.mant, 11'b0}, 1'b0, d, r);
        end

        begin
            logic [105:0] P;
            int lx, hx, ly, hy, H, base, sx, sy;
            logic [221:0] X2, Y2, S;
            logic         stx, sty, sgn;
            int m;
            logic [63:0] sig;
            logic        st;

            P  = 106'(x.mant) * 106'(y.mant);
            lx = int'(x.exp) + int'(y.exp) - 104;
            hx = lx + 105;
            if (!has_add || z.zero) begin
                // apenas o produto
                m = P[105] ? 105 : 104;
                sig = P[105] ? P[105:42] : P[104:41];
                st  = P[105] ? (P[41:0] != 0) : (P[40:0] != 0);
                return round_pack(sp, lx + m, sig, st, d, r);
            end
            ly = int'(z.exp) - 52;
            hy = int'(z.exp);
            H  = (hx > hy) ? hx : hy;
            base = H - 219;
            sx = lx - base;
            sy = ly - base;
            // produto na janela (220 bits) + bit de guarda/sticky abaixo
            stx = 1'b0; sty = 1'b0;
            if (sx >= 0) X2 = {222'(P) << (sx + 1)};
            else if (-sx >= 106) begin X2 = '0; stx = 1'b1; end
            else begin
                X2  = 222'(P >> (-sx)) << 1;
                stx = ((P & ((106'd1 << (-sx)) - 106'd1)) != 0);
            end
            if (sy >= 0) Y2 = {222'(z.mant) << (sy + 1)};
            else if (-sy >= 53) begin Y2 = '0; sty = 1'b1; end
            else begin
                Y2  = 222'(z.mant >> (-sy)) << 1;
                sty = ((z.mant & ((53'd1 << (-sy)) - 53'd1)) != 0);
            end
            X2[0] = stx;
            Y2[0] = sty;

            if (sp == sc) begin
                S = X2 + Y2; sgn = sp;
            end else if (X2 >= Y2) begin
                S = X2 - Y2; sgn = sp;
            end else begin
                S = Y2 - X2; sgn = sc;
            end
            if (S == 0) return {pack_zero(r == 3'b010, d), 5'b0};

            m = 0;
            for (int i = 221; i >= 0; i--) if (S[i]) begin m = i; break; end
            // bit i da janela S tem expoente base + i - 1
            if (m >= 63) begin
                sig = 64'(S >> (m - 63));
                st  = ((S & ((222'd1 << (m - 63)) - 222'd1)) != 0);
            end else begin
                sig = 64'(S) << (63 - m);
                st  = 1'b0;
            end
            return round_pack(sgn, base + m - 1, sig, st, d, r);
        end
    endfunction

    // ---------------------------------------------------------------------
    // Comparação (a < b, a == b) para valores não-NaN
    // ---------------------------------------------------------------------
    function automatic logic [1:0] fcmp(input logic [63:0] x, input logic [63:0] y, input logic d);
        logic sx, sy, lt, eq;
        logic [62:0] mx, my;
        if (d) begin sx = x[63]; sy = y[63]; mx = x[62:0]; my = y[62:0]; end
        else begin
            logic [31:0] ux, uy;
            ux = unbox(x); uy = unbox(y);
            sx = ux[31]; sy = uy[31]; mx = {32'b0, ux[30:0]}; my = {32'b0, uy[30:0]};
        end
        if (mx == 0 && my == 0) begin lt = 1'b0; eq = 1'b1; end
        else begin
            eq = (sx == sy) && (mx == my);
            if (sx != sy) lt = sx;
            else if (!sx) lt = mx < my;
            else          lt = mx > my;
        end
        return {lt, eq};
    endfunction

    // =====================================================================
    // Operações de 1 ciclo / multiplicação-adição (combinacionais)
    // =====================================================================

    logic is_long;   // div/sqrt

    function automatic logic [68:0] compute_simple(input fpu_op_t op, input logic dbl, input logic src_dbl,
                                                   input logic [1:0] int_sz, input logic [2:0] rm,
                                                   input logic [63:0] a, input logic [63:0] b, input logic [63:0] c);
        logic [68:0] t;
        unp_t one, ua, ub, uc;
        logic [63:0] simple_res;
        logic [4:0]  simple_fl;
        ua = unpack(a, (op == FOP_CVT_FF) ? src_dbl : dbl);
        ub = unpack(b, dbl);
        uc = unpack(c, dbl);
        simple_res = '0;
        simple_fl  = '0;
        one        = '0;
        one.mant   = 53'h10_0000_0000_0000;
        case (op)
            FOP_ADD: begin t = fma_core(ua, one, ub, 1'b0, 1'b0, 1'b1, dbl, rm); simple_res = t[68:5]; simple_fl = t[4:0]; end
            FOP_SUB: begin t = fma_core(ua, one, ub, 1'b0, 1'b1, 1'b1, dbl, rm); simple_res = t[68:5]; simple_fl = t[4:0]; end
            FOP_MUL: begin t = fma_core(ua, ub, uc, 1'b0, 1'b0, 1'b0, dbl, rm); simple_res = t[68:5]; simple_fl = t[4:0]; end
            FOP_MADD:  begin t = fma_core(ua, ub, uc, 1'b0, 1'b0, 1'b1, dbl, rm); simple_res = t[68:5]; simple_fl = t[4:0]; end
            FOP_MSUB:  begin t = fma_core(ua, ub, uc, 1'b0, 1'b1, 1'b1, dbl, rm); simple_res = t[68:5]; simple_fl = t[4:0]; end
            FOP_NMSUB: begin t = fma_core(ua, ub, uc, 1'b1, 1'b0, 1'b1, dbl, rm); simple_res = t[68:5]; simple_fl = t[4:0]; end
            FOP_NMADD: begin t = fma_core(ua, ub, uc, 1'b1, 1'b1, 1'b1, dbl, rm); simple_res = t[68:5]; simple_fl = t[4:0]; end

            FOP_SGNJ, FOP_SGNJN, FOP_SGNJX: begin
                logic sa, sb, sr;
                if (dbl) begin sa = a[63]; sb = b[63]; end
                else begin sa = unbox(a)[31]; sb = unbox(b)[31]; end
                sr = (op == FOP_SGNJ) ? sb : (op == FOP_SGNJN) ? !sb : (sa ^ sb);
                if (dbl) simple_res = {sr, a[62:0]};
                else     simple_res = {32'hFFFF_FFFF, sr, unbox(a)[30:0]};
            end

            FOP_MIN, FOP_MAX: begin
                logic [1:0] cr;
                logic [63:0] av, bv;
                av = dbl ? a : {32'hFFFF_FFFF, unbox(a)};
                bv = dbl ? b : {32'hFFFF_FFFF, unbox(b)};
                if (ua.snan || ub.snan) simple_fl[4] = 1'b1;
                if (ua.nan && ub.nan)  simple_res = qnan(dbl);
                else if (ua.nan)       simple_res = bv;
                else if (ub.nan)       simple_res = av;
                else begin
                    cr = fcmp(a, b, dbl);
                    if (cr[0] && cr[1] == 1'b0 && (ua.sign != ub.sign)) begin
                        // -0 vs +0
                        simple_res = (op == FOP_MIN) ? (ua.sign ? av : bv) : (ua.sign ? bv : av);
                    end else if (op == FOP_MIN) simple_res = cr[1] ? av : bv;
                    else                        simple_res = cr[1] ? bv : av;
                end
            end

            FOP_EQ, FOP_LT, FOP_LE: begin
                logic [1:0] cr;
                cr = fcmp(a, b, dbl);
                if (ua.nan || ub.nan) begin
                    simple_res = '0;
                    if (op == FOP_EQ) simple_fl[4] = ua.snan || ub.snan;
                    else              simple_fl[4] = 1'b1;
                end else begin
                    case (op)
                        FOP_EQ:  simple_res = {63'b0, cr[0]};
                        FOP_LT:  simple_res = {63'b0, cr[1]};
                        default: simple_res = {63'b0, cr[1] | cr[0]};
                    endcase
                end
            end

            FOP_CLASS: begin
                logic [9:0] cl;
                logic norm;
                norm = !ua.zero && !ua.sub && !ua.inf && !ua.nan;
                cl = '0;
                cl[0] = ua.sign && ua.inf;
                cl[1] = ua.sign && norm;
                cl[2] = ua.sign && ua.sub;
                cl[3] = ua.sign && ua.zero;
                cl[4] = !ua.sign && ua.zero;
                cl[5] = !ua.sign && ua.sub;
                cl[6] = !ua.sign && norm;
                cl[7] = !ua.sign && ua.inf;
                cl[8] = ua.snan;
                cl[9] = ua.nan && !ua.snan;
                simple_res = {54'b0, cl};
            end

            FOP_MV_XF: simple_res = dbl ? a : {{32{a[31]}}, a[31:0]};
            FOP_MV_FX: simple_res = dbl ? a : {32'hFFFF_FFFF, a[31:0]};

            FOP_CVT_FF: begin
                // a está no formato src_dbl; destino é dbl
                if (ua.nan) begin
                    simple_res = qnan(dbl); simple_fl[4] = ua.snan;
                end else if (ua.inf)  simple_res = pack_inf(ua.sign, dbl);
                else if (ua.zero)     simple_res = pack_zero(ua.sign, dbl);
                else begin
                    t = round_pack(ua.sign, int'(ua.exp), {ua.mant, 11'b0}, 1'b0, dbl, rm);
                    simple_res = t[68:5]; simple_fl = t[4:0];
                end
            end

            FOP_CVT_IF: begin
                // FP -> inteiro
                logic        sgn_t, uns, w;
                logic [64:0] mag, lim;
                logic        rb, st, inc, inval;
                int          e;
                uns = int_sz[0];
                w   = !int_sz[1];
                e   = int'(ua.exp);
                mag = '0; rb = 0; st = 0;
                inval = 1'b0;
                if (ua.nan || ua.inf) inval = 1'b1;
                else if (!ua.zero) begin
                    if (e >= 64) inval = 1'b1;
                    else if (e >= 52) mag = 65'(ua.mant) << (e - 52);
                    else if (e >= -1) begin
                        logic [52:0] fr;
                        mag = 65'(ua.mant >> (52 - e));
                        fr  = ua.mant & ((53'd1 << (52 - e)) - 53'd1);
                        rb  = fr[51 - e];
                        st  = (fr & ((53'd1 << (51 - e)) - 53'd1)) != 0;
                    end else begin
                        mag = '0; rb = 1'b0; st = 1'b1;
                    end
                end
                inc = rinc(rm, ua.sign, mag[0], rb, st);
                mag = mag + 65'(inc);
                // limites
                if (!inval) begin
                    if (uns) begin
                        if (ua.sign) inval = (mag != 0);
                        else inval = w ? (mag > 65'h0_FFFF_FFFF) : (mag > 65'h0_FFFF_FFFF_FFFF_FFFF);
                    end else begin
                        lim = w ? (ua.sign ? 65'h8000_0000 : 65'h7FFF_FFFF)
                                : (ua.sign ? 65'h0_8000_0000_0000_0000 : 65'h0_7FFF_FFFF_FFFF_FFFF);
                        inval = mag > lim;
                    end
                end
                if (inval) begin
                    simple_fl[4] = 1'b1;
                    sgn_t = ua.sign && !ua.nan;
                    if (uns) simple_res = sgn_t ? 64'd0 : (w ? 64'hFFFF_FFFF_FFFF_FFFF : 64'hFFFF_FFFF_FFFF_FFFF);
                    else     simple_res = sgn_t ? (w ? 64'hFFFF_FFFF_8000_0000 : 64'h8000_0000_0000_0000)
                                                : (w ? 64'h0000_0000_7FFF_FFFF : 64'h7FFF_FFFF_FFFF_FFFF);
                end else begin
                    logic [63:0] v;
                    v = ua.sign ? -mag[63:0] : mag[63:0];
                    simple_res = w ? {{32{v[31]}}, v[31:0]} : v;
                    simple_fl[0] = rb || st;
                end
            end

            FOP_CVT_FI: begin
                // inteiro -> FP
                logic [63:0] v, mg;
                logic        sg;
                int          m;
                case (int_sz)
                    2'd0: v = {{32{a[31]}}, a[31:0]};
                    2'd1: v = {32'b0, a[31:0]};
                    default: v = a;
                endcase
                sg = !int_sz[0] && v[63];
                mg = sg ? -v : v;
                if (mg == 0) simple_res = pack_zero(1'b0, dbl);
                else begin
                    m = 0;
                    for (int i = 63; i >= 0; i--) if (mg[i]) begin m = i; break; end
                    t = round_pack(sg, m, mg << (63 - m), 1'b0, dbl, rm);
                    simple_res = t[68:5]; simple_fl = t[4:0];
                end
            end
            default: ;
        endcase
    return {simple_res, simple_fl};
    endfunction

    assign is_long = (op == FOP_DIV) || (op == FOP_SQRT);

    // =====================================================================
    // DIV / SQRT iterativos
    // =====================================================================
    typedef enum logic [2:0] { P_IDLE, P_WAIT, P_DIV, P_SQRT, P_FIN, P_DONE } pstate_t;
    pstate_t state;
    logic [2:0]   wait_cnt;
    logic [63:0]  res_q;
    logic [4:0]   fl_q;
    // estado iterativo
    logic [6:0]   iter;
    logic [57:0]  q;          // quociente / raiz parcial
    logic [117:0] rem;        // resto
    logic [52:0]  divisor;
    logic [113:0] sq_rad;     // radicando restante (sqrt)
    logic         l_sign, l_dbl;
    logic [2:0]   l_rm;
    int           l_exp;

    assign done   = (state == P_DONE);
    assign result = res_q;
    assign fflags = fl_q;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= P_IDLE; wait_cnt <= '0; res_q <= '0; fl_q <= '0;
            iter <= '0; q <= '0; rem <= '0; divisor <= '0; sq_rad <= '0;
            l_sign <= 0; l_dbl <= 0; l_rm <= 0; l_exp <= 0;
        end else if (kill) begin
            state <= P_IDLE;
        end else begin
            case (state)
                P_IDLE: if (start) begin
                    unp_t ua, ub;
                    ua = unpack(a, dbl);
                    ub = unpack(b, dbl);
                    l_dbl <= dbl; l_rm <= rm;
                    if (!is_long) begin
                        logic [68:0] sr;
                        sr = compute_simple(op, dbl, src_dbl, int_sz, rm, a, b, c);
                        res_q <= sr[68:5]; fl_q <= sr[4:0];
                        if (op == FOP_ADD || op == FOP_SUB || op == FOP_MUL ||
                            op == FOP_MADD || op == FOP_MSUB || op == FOP_NMADD || op == FOP_NMSUB) begin
                            wait_cnt <= 3'd1; state <= P_WAIT;
                        end else state <= P_DONE;
                    end else if (op == FOP_DIV) begin
                        l_sign <= ua.sign ^ ub.sign;
                        fl_q   <= '0;
                        if (ua.nan || ub.nan || (ua.zero && ub.zero) || (ua.inf && ub.inf)) begin
                            res_q <= qnan(dbl);
                            fl_q  <= {(ua.snan || ub.snan || (ua.zero && ub.zero) || (ua.inf && ub.inf)), 4'b0};
                            state <= P_DONE;
                        end else if (ua.inf || ub.zero) begin
                            res_q <= pack_inf(ua.sign ^ ub.sign, dbl);
                            fl_q  <= {1'b0, ub.zero && !ua.inf, 3'b0};
                            state <= P_DONE;
                        end else if (ua.zero || ub.inf) begin
                            res_q <= pack_zero(ua.sign ^ ub.sign, dbl);
                            state <= P_DONE;
                        end else begin
                            // q = ma / mb com 56 bits (inclui guarda/round)
                            rem     <= 118'(ua.mant);
                            divisor <= ub.mant;
                            q       <= '0;
                            iter    <= 7'd56;
                            l_exp   <= int'(ua.exp) - int'(ub.exp);
                            state   <= P_DIV;
                        end
                    end else begin // SQRT
                        l_sign <= ua.sign;
                        fl_q   <= '0;
                        if (ua.nan) begin
                            res_q <= qnan(dbl); fl_q <= {ua.snan, 4'b0}; state <= P_DONE;
                        end else if (ua.zero) begin
                            res_q <= pack_zero(ua.sign, dbl); state <= P_DONE;
                        end else if (ua.sign) begin
                            res_q <= qnan(dbl); fl_q <= 5'b10000; state <= P_DONE;
                        end else if (ua.inf) begin
                            res_q <= pack_inf(1'b0, dbl); state <= P_DONE;
                        end else begin
                            // valor = mant * 2^(exp-52); torna o expoente par
                            logic [113:0] rad;
                            int E;
                            E = int'(ua.exp) - 52;
                            if (E & 1) begin rad = 114'(ua.mant) << 61; E = E - 1; end
                            else       begin rad = 114'(ua.mant) << 60; end
                            // sqrt(rad) * 2^((E-60)/2)
                            sq_rad <= rad;
                            rem    <= '0;
                            q      <= '0;
                            iter   <= 7'd57;
                            l_exp  <= (E - 60) / 2;
                            state  <= P_SQRT;
                        end
                    end
                end

                P_WAIT: begin
                    if (wait_cnt == 0) state <= P_DONE;
                    else wait_cnt <= wait_cnt - 1'b1;
                end

                P_DIV: begin
                    logic [117:0] r2;
                    r2 = rem;
                    if (r2 >= 118'(divisor)) begin
                        r2 = r2 - 118'(divisor);
                        q  <= {q[56:0], 1'b1};
                    end else begin
                        q  <= {q[56:0], 1'b0};
                    end
                    rem  <= r2 << 1;
                    iter <= iter - 1'b1;
                    if (iter == 7'd1) state <= P_FIN;
                end

                P_SQRT: begin
                    // método dígito-a-dígito: processa 2 bits do radicando por iteração
                    logic [117:0] r2, trial;
                    r2    = (rem << 2) | 118'(sq_rad[113:112]);
                    trial = {58'b0, q, 2'b01};
                    if (r2 >= trial) begin
                        r2 = r2 - trial;
                        q  <= {q[56:0], 1'b1};
                    end else begin
                        q  <= {q[56:0], 1'b0};
                    end
                    rem    <= r2;
                    sq_rad <= sq_rad << 2;
                    iter   <= iter - 1'b1;
                    if (iter == 7'd1) state <= P_FIN;
                end

                P_FIN: begin
                    logic [68:0] t;
                    logic [63:0] sig;
                    int m;
                    m = 0;
                    for (int i = 57; i >= 0; i--) if (q[i]) begin m = i; break; end
                    sig = 64'(q) << (63 - m);
                    if (op == FOP_DIV)
                        // q tem 56 bits: q = floor(ma/mb * 2^55); bit m tem expoente l_exp + m - 55
                        t = round_pack(l_sign, l_exp + m - 55, sig, rem != 0, l_dbl, l_rm);
                    else
                        t = round_pack(1'b0, l_exp + m, sig, (rem != 0) || (sq_rad != 0), l_dbl, l_rm);
                    res_q <= t[68:5];
                    fl_q  <= t[4:0];
                    state <= P_DONE;
                end

                P_DONE: state <= P_IDLE;
                default: state <= P_IDLE;
            endcase
        end
    end

endmodule
