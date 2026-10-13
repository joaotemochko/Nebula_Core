`timescale 1ns/1ps
`default_nettype none

/**
 * @module nebula_axi_bridge
 * @brief Ponte da porta de memória do cluster Nebula para AXI4 (64 bits).
 *
 *  - Leitura cacheável: burst INCR de 8 beats (linha de 64 B).
 *  - Acesso não-cacheável (MMIO): 1 beat "estreito" com endereço exato e
 *    AxSIZE = tamanho do acesso (deduzidos dos byte-strobes), evitando
 *    efeitos colaterais em bytes vizinhos de registradores de dispositivo.
 *  - Escrita: burst INCR cobrindo apenas os beats com strobes ativos
 *    (um store de 1 doubleword vira 1 beat).
 *  - Erros (RRESP/BRESP = SLVERR/DECERR) são propagados em mem_error.
 *  - Uma transação por vez (o cluster já serializa os acessos).
 */
module nebula_axi_bridge #(
    parameter int ID_W = 4
)(
    input  wire              clk,
    input  wire              rst_n,

    // porta de memória do cluster
    input  wire              mem_req,
    input  wire              mem_we,
    input  wire  [55:0]      mem_addr,
    input  wire  [511:0]     mem_wdata,
    input  wire  [63:0]      mem_wstrb,
    input  wire              mem_uncached,
    output logic             mem_ack,
    output logic [511:0]     mem_rdata,
    output logic             mem_error,

    // AXI4 master
    output logic [ID_W-1:0]  m_axi_awid,
    output logic [63:0]      m_axi_awaddr,
    output logic [7:0]       m_axi_awlen,
    output logic [2:0]       m_axi_awsize,
    output logic [1:0]       m_axi_awburst,
    output logic             m_axi_awlock,
    output logic [3:0]       m_axi_awcache,
    output logic [2:0]       m_axi_awprot,
    output logic [3:0]       m_axi_awqos,
    output logic             m_axi_awvalid,
    input  wire              m_axi_awready,
    output logic [63:0]      m_axi_wdata,
    output logic [7:0]       m_axi_wstrb,
    output logic             m_axi_wlast,
    output logic             m_axi_wvalid,
    input  wire              m_axi_wready,
    input  wire  [ID_W-1:0]  m_axi_bid,
    input  wire  [1:0]       m_axi_bresp,
    input  wire              m_axi_bvalid,
    output logic             m_axi_bready,
    output logic [ID_W-1:0]  m_axi_arid,
    output logic [63:0]      m_axi_araddr,
    output logic [7:0]       m_axi_arlen,
    output logic [2:0]       m_axi_arsize,
    output logic [1:0]       m_axi_arburst,
    output logic             m_axi_arlock,
    output logic [3:0]       m_axi_arcache,
    output logic [2:0]       m_axi_arprot,
    output logic [3:0]       m_axi_arqos,
    output logic             m_axi_arvalid,
    input  wire              m_axi_arready,
    input  wire  [ID_W-1:0]  m_axi_rid,
    input  wire  [63:0]      m_axi_rdata,
    input  wire  [1:0]       m_axi_rresp,
    input  wire              m_axi_rlast,
    input  wire              m_axi_rvalid,
    output logic             m_axi_rready
);
    typedef enum logic [2:0] { B_IDLE, B_AR, B_R, B_AW, B_W, B_B, B_DONE } bstate_t;
    bstate_t state;

    logic [55:0]  a_line;
    logic [511:0] wbuf, rbuf;
    logic [63:0]  sbuf;
    logic         unc, err;
    logic [2:0]   beat, first, last, lane;
    logic [2:0]   nb_off;       // primeiro byte do acesso estreito
    logic [2:0]   nb_size;      // log2(bytes)
    logic [2:0]   f_off, f_size;
    always_comb begin
        logic [7:0] m;
        int n;
        m = mem_wstrb[mem_addr[5:3]*8 +: 8];
        f_off = 3'd0;
        for (int b = 7; b >= 0; b--) if (m[b]) f_off = 3'(b);
        n = 0;
        for (int b = 0; b < 8; b++) n += int'(m[b]);
        f_size = (n <= 1) ? 3'd0 : (n == 2) ? 3'd1 : (n <= 4) ? 3'd2 : 3'd3;
    end

    // primeiro/último beat com strobes ativos
    logic [2:0] f_first, f_last;
    always_comb begin
        f_first = 3'd0; f_last = 3'd0;
        for (int b = 7; b >= 0; b--) if (mem_wstrb[b*8 +: 8] != 8'd0) f_first = 3'(b);
        for (int b = 0; b < 8; b++)  if (mem_wstrb[b*8 +: 8] != 8'd0) f_last  = 3'(b);
    end

    assign m_axi_awid = '0;     assign m_axi_arid = '0;
    assign m_axi_awsize = unc ? nb_size : 3'd3;
    assign m_axi_arsize = unc ? nb_size : 3'd3;
    assign m_axi_awburst = 2'b01; assign m_axi_arburst = 2'b01;
    assign m_axi_awlock = 1'b0; assign m_axi_arlock = 1'b0;
    assign m_axi_awcache = unc ? 4'b0000 : 4'b0011;
    assign m_axi_arcache = unc ? 4'b0000 : 4'b0011;
    assign m_axi_awprot = 3'b000; assign m_axi_arprot = 3'b000;
    assign m_axi_awqos = '0;    assign m_axi_arqos = '0;

    assign m_axi_awvalid = (state == B_AW);
    assign m_axi_awaddr  = unc ? {8'b0, a_line[55:6], first, nb_off} : {8'b0, a_line[55:6], first, 3'b000};
    assign m_axi_awlen   = 8'(last - first);
    assign m_axi_wvalid  = (state == B_W);
    assign m_axi_wdata   = wbuf[beat*64 +: 64];
    assign m_axi_wstrb   = sbuf[beat*8 +: 8];
    assign m_axi_wlast   = (beat == last);
    assign m_axi_bready  = (state == B_B);

    assign m_axi_arvalid = (state == B_AR);
    assign m_axi_araddr  = unc ? {8'b0, a_line[55:6], lane, nb_off} : {8'b0, a_line[55:6], 6'b0};
    assign m_axi_arlen   = unc ? 8'd0 : 8'd7;
    assign m_axi_rready  = (state == B_R);

    assign mem_ack   = (state == B_DONE);
    assign mem_rdata = rbuf;
    assign mem_error = err;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= B_IDLE;
            a_line <= '0; wbuf <= '0; rbuf <= '0; sbuf <= '0;
            unc <= 1'b0; err <= 1'b0; beat <= '0; first <= '0; last <= '0; lane <= '0;
            nb_off <= '0; nb_size <= '0;
        end else begin
            case (state)
                B_IDLE: if (mem_req) begin
                    a_line <= mem_addr;
                    lane   <= mem_addr[5:3];
                    unc    <= mem_uncached;
                    err    <= 1'b0;
                    nb_off <= f_off;
                    nb_size<= f_size;
                    if (mem_we) begin
                        wbuf  <= mem_wdata;
                        sbuf  <= mem_wstrb;
                        first <= f_first;
                        last  <= f_last;
                        beat  <= f_first;
                        state <= B_AW;
                    end else begin
                        beat  <= mem_uncached ? mem_addr[5:3] : 3'd0;
                        state <= B_AR;
                    end
                end
                B_AR: if (m_axi_arready) state <= B_R;
                B_R: if (m_axi_rvalid) begin
                    if (unc) rbuf <= {8{m_axi_rdata}};
                    else     rbuf[beat*64 +: 64] <= m_axi_rdata;
                    if (m_axi_rresp[1]) err <= 1'b1;
                    beat <= beat + 3'd1;
                    if (m_axi_rlast) state <= B_DONE;
                end
                B_AW: if (m_axi_awready) state <= B_W;
                B_W: if (m_axi_wready) begin
                    if (beat == last) state <= B_B;
                    else beat <= beat + 3'd1;
                end
                B_B: if (m_axi_bvalid) begin
                    if (m_axi_bresp[1]) err <= 1'b1;
                    state <= B_DONE;
                end
                B_DONE: state <= B_IDLE;
                default: state <= B_IDLE;
            endcase
        end
    end

endmodule
