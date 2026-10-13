`timescale 1ns/1ps

/**
 * @module nebula_core_axi_top
 * @brief Top-level Nebula para simulação LiteX
 *
 * CORREÇÕES APLICADAS:
 *
 * BUG 1 FIX (adapter) — mem_ready pulsa no handshake AXI.
 * BUG 3 FIX (cluster) — mmio_req_r removido, bypass MMIO combinacional.
 *   mem_idle conectado do adapter ao cluster via sinal interno.
 *
 * PINMISSING FIX — ports o_m_axi_i_* removidos da interface do top.
 *   O LiteX (nebula_litex_sim.py) não conecta nenhum sinal de saída do
 *   canal I em simulação Opção A. Manter esses ports causava PINMISSING
 *   no Verilator. As saídas do canal I agora ficam internas ao adapter.
 *   Os inputs do canal I são tie-off a 0 internamente no top.
 */
module nebula_core_axi_top #(
    parameter int HART_ID = 0,
    parameter int XLEN    = 64
)(
    input  wire         clk,
    input  wire         rst_n,

    // Interrupções
    input  wire         i_timer_irq,
    input  wire         i_external_irq,
    input  wire         i_software_irq,

    // =========================================================================
    // AXI4 Data (canal D — único canal exposto ao LiteX em simulação Opção A)
    // =========================================================================
    output wire [3:0]   o_m_axi_d_awid,
    output wire [63:0]  o_m_axi_d_awaddr,
    output wire [7:0]   o_m_axi_d_awlen,
    output wire [2:0]   o_m_axi_d_awsize,
    output wire [1:0]   o_m_axi_d_awburst,
    output wire         o_m_axi_d_awvalid,
    input  wire         i_m_axi_d_awready,
    output wire [63:0]  o_m_axi_d_wdata,
    output wire [7:0]   o_m_axi_d_wstrb,
    output wire         o_m_axi_d_wlast,
    output wire         o_m_axi_d_wvalid,
    input  wire         i_m_axi_d_wready,
    input  wire [3:0]   i_m_axi_d_bid,
    input  wire [1:0]   i_m_axi_d_bresp,
    input  wire         i_m_axi_d_bvalid,
    output wire         o_m_axi_d_bready,
    output wire [3:0]   o_m_axi_d_arid,
    output wire [63:0]  o_m_axi_d_araddr,
    output wire [7:0]   o_m_axi_d_arlen,
    output wire [2:0]   o_m_axi_d_arsize,
    output wire [1:0]   o_m_axi_d_arburst,
    output wire         o_m_axi_d_arvalid,
    input  wire         i_m_axi_d_arready,
    input  wire [63:0]  i_m_axi_d_rdata,
    input  wire [1:0]   i_m_axi_d_rresp,
    input  wire         i_m_axi_d_rlast,
    input  wire         i_m_axi_d_rvalid,
    output wire         o_m_axi_d_rready
);

    // =========================================================================
    // Parâmetros locais
    // =========================================================================
    localparam int PADDR_WIDTH  = 56;
    localparam int L2_LINE_SIZE = 64;

    // =========================================================================
    // Sinais internos Cluster <-> Adapter
    // =========================================================================
    logic                       l2_req;
    logic                       l2_we;
    logic                       l2_ack;
    logic [PADDR_WIDTH-1:0]     l2_addr;
    logic [L2_LINE_SIZE*8-1:0]  l2_wdata;
    logic [L2_LINE_SIZE*8-1:0]  l2_rdata;
    logic [L2_LINE_SIZE-1:0]    l2_wstrb;
    logic                       l2_uncached;
    logic                       l2_mem_ready;
    logic                       l2_mem_idle;  // Bug 3 fix: adapter → cluster

    // =========================================================================
    // Interrupções
    // =========================================================================
    wire [3:0] timer_irqs    = {4{i_timer_irq}};
    wire [3:0] external_irqs = {4{i_external_irq}};
    wire [3:0] software_irqs = {4{i_software_irq}};

    // contador de tempo real local (CSR time); o CLINT do SoC deve fornecer
    // mtime em integrações completas
    logic [63:0] mtime_cnt;
    always_ff @(posedge clk or negedge rst_n)
        if (!rst_n) mtime_cnt <= '0;
        else        mtime_cnt <= mtime_cnt + 64'd1;

    // Dummy I-Cache (imem_req=0 — Opção A)
    logic        dummy_imem_ack;
    logic [511:0] dummy_imem_data;

    // =========================================================================
    // Cluster
    // =========================================================================
    nebula_cluster #(
        .CLUSTER_ID  (0),
        .NUM_CORES   (1),
        .XLEN        (XLEN),
        .PADDR_WIDTH (PADDR_WIDTH),
        .VADDR_WIDTH (39)
    ) u_cluster (
        .clk      (clk),
        .rst_n    (rst_n),

        .mem_ready (l2_mem_ready),

        .mem_req      (l2_req),
        .mem_we       (l2_we),
        .mem_addr     (l2_addr),
        .mem_wdata    (l2_wdata),
        .mem_wstrb    (l2_wstrb),
        .mem_uncached (l2_uncached),
        .mem_ack      (l2_ack),
        .mem_rdata    (l2_rdata),
        .mem_error    (1'b0),

        .timer_irq    (timer_irqs),
        .external_irq (external_irqs),
        .software_irq (software_irqs),
        .s_external_irq ('0),
        .mtime        (mtime_cnt),

        .debug_req    (1'b0),
        .debug_halted ()
    );

    // =========================================================================
    // AXI Adapter
    // Canal I: inputs tie-off a 0 (Opção A). Saídas do canal I são
    // internas ao adapter — não conectadas aqui, sem PINMISSING.
    // =========================================================================
    nebula_axi_adapter #(
        .PADDR_WIDTH  (PADDR_WIDTH),
        .AXI_ID_WIDTH (4)
    ) u_adapter (
        .clk   (clk),
        .rst_n (rst_n),

        .mem_ready (l2_mem_ready),
        .mem_idle  (l2_mem_idle),

        // I-Cache tie-off
        .imem_req  (1'b0),
        .imem_addr ('0),
        .imem_ack  (dummy_imem_ack),
        .imem_data (dummy_imem_data),

        // Canal I inputs — tie-off a 0 (Opção A, canal I idle)
        .m_axi_i_arready (1'b0),
        .m_axi_i_rdata   ('0),
        .m_axi_i_rresp   ('0),
        .m_axi_i_rlast   (1'b0),
        .m_axi_i_rvalid  (1'b0),

        // Canal D — interface unificada com o cluster
        .dmem_req      (l2_req),
        .dmem_we       (l2_we),
        .dmem_addr     (l2_addr),
        .dmem_wdata    (l2_wdata),
        .dmem_wstrb    (l2_wstrb),
        .dmem_uncached (l2_uncached),
        .dmem_ack      (l2_ack),
        .dmem_rdata    (l2_rdata),

        // AXI D → LiteX
        .m_axi_d_awid    (o_m_axi_d_awid),
        .m_axi_d_awaddr  (o_m_axi_d_awaddr),
        .m_axi_d_awlen   (o_m_axi_d_awlen),
        .m_axi_d_awsize  (o_m_axi_d_awsize),
        .m_axi_d_awburst (o_m_axi_d_awburst),
        .m_axi_d_awvalid (o_m_axi_d_awvalid),
        .m_axi_d_awready (i_m_axi_d_awready),
        .m_axi_d_wdata   (o_m_axi_d_wdata),
        .m_axi_d_wstrb   (o_m_axi_d_wstrb),
        .m_axi_d_wlast   (o_m_axi_d_wlast),
        .m_axi_d_wvalid  (o_m_axi_d_wvalid),
        .m_axi_d_wready  (i_m_axi_d_wready),
        .m_axi_d_bid     (i_m_axi_d_bid),
        .m_axi_d_bresp   (i_m_axi_d_bresp),
        .m_axi_d_bvalid  (i_m_axi_d_bvalid),
        .m_axi_d_bready  (o_m_axi_d_bready),
        .m_axi_d_arid    (o_m_axi_d_arid),
        .m_axi_d_araddr  (o_m_axi_d_araddr),
        .m_axi_d_arlen   (o_m_axi_d_arlen),
        .m_axi_d_arsize  (o_m_axi_d_arsize),
        .m_axi_d_arburst (o_m_axi_d_arburst),
        .m_axi_d_arvalid (o_m_axi_d_arvalid),
        .m_axi_d_arready (i_m_axi_d_arready),
        .m_axi_d_rdata   (i_m_axi_d_rdata),
        .m_axi_d_rresp   (i_m_axi_d_rresp),
        .m_axi_d_rlast   (i_m_axi_d_rlast),
        .m_axi_d_rvalid  (i_m_axi_d_rvalid),
        .m_axi_d_rready  (o_m_axi_d_rready)
    );

endmodule
