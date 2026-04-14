`timescale 1ns/1ps

/**
 * @module nebula_core_axi_top
 * @brief Top-level Nebula para simulação LiteX
 *
 * NOTA SOBRE mem_wstrb E MMIO:
 * O nebula_cluster.sv expõe mem_wstrb (64 bits) e mem_uncached
 * para suportar o bypass de requisições MMIO (ex: printf na UART).
 * O D-Cache L1 envia o strobing exato para stores parciais e a flag
 * uncached avisa o AXI para não fazer burst na UART.
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
    // AXI4 Instruction (I-Cache — agora funcional)
    // =========================================================================
    output wire [3:0]   o_m_axi_i_arid,
    output wire [63:0]  o_m_axi_i_araddr,
    output wire [7:0]   o_m_axi_i_arlen,
    output wire [2:0]   o_m_axi_i_arsize,
    output wire [1:0]   o_m_axi_i_arburst,
    output wire         o_m_axi_i_arvalid,
    input  wire         i_m_axi_i_arready,
    input  wire [63:0]  i_m_axi_i_rdata,
    input  wire [1:0]   i_m_axi_i_rresp,
    input  wire         i_m_axi_i_rlast,
    input  wire         i_m_axi_i_rvalid,
    output wire         o_m_axi_i_rready,

    // =========================================================================
    // AXI4 Data (D-Cache / L2 write-back / MMIO)
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
    // Sinais internos — Cluster → Adapter (512 bits)
    // =========================================================================

    // Canal de dados (D-Cache / L2 write-back / MMIO)
    logic           l2_req, l2_we, l2_ack;
    logic [55:0]    l2_addr;
    logic [511:0]   l2_wdata, l2_rdata;
    
    // wstrb e uncached — roteados do cluster para suportar MMIO bypass
    logic [63:0]    l2_wstrb;
    logic           l2_uncached;

    // Canal de instruções (I-Cache)
    logic           l2_imem_req, l2_imem_ack;
    logic [55:0]    l2_imem_addr;
    logic [511:0]   l2_imem_data;

    // Distribuição de interrupções para os 4 cores
    wire [3:0] timer_irqs    = {4{i_timer_irq}};
    wire [3:0] external_irqs = {4{i_external_irq}};
    wire [3:0] software_irqs = {4{i_software_irq}};

    // =========================================================================
    // Cluster de 4 núcleos
    // =========================================================================
    nebula_cluster #(
        .CLUSTER_ID(0),
        .NUM_CORES(1),
        .XLEN(XLEN),
        .PADDR_WIDTH(56),
        .VADDR_WIDTH(39)
    ) u_cluster (
        .clk      (clk),
        .rst_n    (rst_n),

        // Interface com a memória principal (via AXI adapter)
        .mem_req      (l2_req),
        .mem_we       (l2_we),
        .mem_addr     (l2_addr),
        .mem_wdata    (l2_wdata),
        .mem_wstrb    (l2_wstrb),       // LIGAÇÃO DA MÁSCARA
        .mem_uncached (l2_uncached),    // LIGAÇÃO DO MMIO
        .mem_ack      (l2_ack),
        .mem_rdata    (l2_rdata),
        .mem_error    (1'b0),

        .timer_irq    (timer_irqs),
        .external_irq (external_irqs),
        .software_irq (software_irqs),

        .debug_req    (1'b0),
        .debug_halted ()
    );

    // =========================================================================
    // FIX 1: Gerar requisição de I-Cache a partir da L2
    //
    // O nebula_cluster unifica I e D numa única interface mem_* para a L2.
    // O campo is_ifetch da l2_req_t distingue os tipos. Como a interface
    // mem_* do cluster já é unificada, precisamos de uma segunda porta para
    // o adapter AXI poder ter canais AR separados para I e D.
    //
    // ESTRATÉGIA: O cluster expõe UMA interface mem_* unificada. O adapter
    // recebe essa interface no canal D. Para o canal I (instrução), usamos
    // o PTW/imem interno do nebula_core_full via imem_* do cluster.
    //
    // Como o nebula_cluster NÃO expõe imem_* diretamente (ele agrega tudo
    // em mem_*), a solução correta é:
    //   Opção A (implementada aqui): usar a interface mem_* para TUDO e
    //   colocar ambos I e D no canal D do AXI (simplificação válida para
    //   simulação — o LiteX não distingue as transações por canal ID).
    //
    //   Opção B (produção): expor imem_* e dmem_* separados no cluster e
    //   roteá-los para os canais AXI I e D respectivamente.
    //
    // Para a simulação LiteX, Opção A é suficiente e mais simples.
    // O canal AXI I fica sem uso (arvalid=0) e o canal D carrega tudo.
    // Isso funciona pois o LiteX usa um único barramento Wishbone/AXI
    // que não distingue fetch de data em simulação.
    // =========================================================================

    // =========================================================================
    // Adapter AXI — tudo no canal D (Opção A para simulação)
    // =========================================================================
    nebula_axi_adapter #(
        .PADDR_WIDTH  (56),
        .AXI_ID_WIDTH (4)
    ) u_adapter (
        .clk   (clk),
        .rst_n (rst_n),

        // I-Cache: tie-off (Opção A — tudo vai pelo canal D)
        // Para Opção B: conectar l2_imem_* aqui
        .imem_req (1'b0),
        .imem_addr('0),
        .imem_ack (l2_imem_ack),
        .imem_data(l2_imem_data),

        // D-Cache / L2 (canal principal — carrega I+D em simulação)
        .dmem_req     (l2_req),
        .dmem_we      (l2_we),
        .dmem_addr    (l2_addr),
        .dmem_wdata   (l2_wdata),
        .dmem_wstrb   (l2_wstrb),       // LIGAÇÃO DA MÁSCARA
        .dmem_uncached(l2_uncached),    // LIGAÇÃO DO MMIO
        .dmem_ack     (l2_ack),
        .dmem_rdata   (l2_rdata),

        // AXI D → LiteX
        .m_axi_d_awid   (o_m_axi_d_awid),
        .m_axi_d_awaddr (o_m_axi_d_awaddr),
        .m_axi_d_awlen  (o_m_axi_d_awlen),
        .m_axi_d_awsize (o_m_axi_d_awsize),
        .m_axi_d_awburst(o_m_axi_d_awburst),
        .m_axi_d_awvalid(o_m_axi_d_awvalid),
        .m_axi_d_awready(i_m_axi_d_awready),
        .m_axi_d_wdata  (o_m_axi_d_wdata),
        .m_axi_d_wstrb  (o_m_axi_d_wstrb),
        .m_axi_d_wlast  (o_m_axi_d_wlast),
        .m_axi_d_wvalid (o_m_axi_d_wvalid),
        .m_axi_d_wready (i_m_axi_d_wready),
        .m_axi_d_bid    (i_m_axi_d_bid),
        .m_axi_d_bresp  (i_m_axi_d_bresp),
        .m_axi_d_bvalid (i_m_axi_d_bvalid),
        .m_axi_d_bready (o_m_axi_d_bready),
        .m_axi_d_arid   (o_m_axi_d_arid),
        .m_axi_d_araddr (o_m_axi_d_araddr),
        .m_axi_d_arlen  (o_m_axi_d_arlen),
        .m_axi_d_arsize (o_m_axi_d_arsize),
        .m_axi_d_arburst(o_m_axi_d_arburst),
        .m_axi_d_arvalid(o_m_axi_d_arvalid),
        .m_axi_d_arready(i_m_axi_d_arready),
        .m_axi_d_rdata  (i_m_axi_d_rdata),
        .m_axi_d_rresp  (i_m_axi_d_rresp),
        .m_axi_d_rlast  (i_m_axi_d_rlast),
        .m_axi_d_rvalid (i_m_axi_d_rvalid),
        .m_axi_d_rready (o_m_axi_d_rready),

        // AXI I — não utilizado em simulação (Opção A)
        .m_axi_i_arid   (o_m_axi_i_arid),
        .m_axi_i_araddr (o_m_axi_i_araddr),
        .m_axi_i_arlen  (o_m_axi_i_arlen),
        .m_axi_i_arsize (o_m_axi_i_arsize),
        .m_axi_i_arburst(o_m_axi_i_arburst),
        .m_axi_i_arvalid(o_m_axi_i_arvalid),
        .m_axi_i_arready(i_m_axi_i_arready),
        .m_axi_i_rdata  (i_m_axi_i_rdata),
        .m_axi_i_rresp  (i_m_axi_i_rresp),
        .m_axi_i_rlast  (i_m_axi_i_rlast),
        .m_axi_i_rvalid (i_m_axi_i_rvalid),
        .m_axi_i_rready (o_m_axi_i_rready)
    );

endmodule
