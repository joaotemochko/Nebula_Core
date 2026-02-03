`timescale 1ns/1ps

module nebula_core_axi_top #(
    parameter int HART_ID = 0,
    parameter int XLEN = 64
)(
    input  wire         clk,
    input  wire         rst_n, // Active Low (Direto do LiteX)

    // Interrupções
    input  wire         i_timer_irq,
    input  wire         i_external_irq,
    input  wire         i_software_irq,

    // =========================================================================
    // AXI4 Instruction (Não usada - Tie-off)
    // =========================================================================
    output wire [3:0]   o_m_axi_i_arid,
    output wire [63:0]  o_m_axi_i_araddr,
    output wire [7:0]   o_m_axi_i_arlen,
    output wire [2:0]   o_m_axi_i_arsize,
    output wire [1:0]   o_m_axi_i_arburst,
    output wire         o_m_axi_i_arvalid,
    input  wire         i_m_axi_i_arready,
    input  wire [63:0]  i_m_axi_i_rdata, // CORRIGIDO PARA 64 BITS
    input  wire [1:0]   i_m_axi_i_rresp,
    input  wire         i_m_axi_i_rlast,
    input  wire         i_m_axi_i_rvalid,
    output wire         o_m_axi_i_rready,

    // =========================================================================
    // AXI4 Data (Unificada para o Cluster)
    // =========================================================================
    output wire [3:0]   o_m_axi_d_awid,
    output wire [63:0]  o_m_axi_d_awaddr,
    output wire [7:0]   o_m_axi_d_awlen,
    output wire [2:0]   o_m_axi_d_awsize,
    output wire [1:0]   o_m_axi_d_awburst,
    output wire         o_m_axi_d_awvalid,
    input  wire         i_m_axi_d_awready,
    output wire [63:0]  o_m_axi_d_wdata, // CORRIGIDO PARA 64 BITS
    output wire [7:0]   o_m_axi_d_wstrb, // CORRIGIDO PARA 8 BITS (Mask)
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
    input  wire [63:0]  i_m_axi_d_rdata, // CORRIGIDO PARA 64 BITS
    input  wire [1:0]   i_m_axi_d_rresp,
    input  wire         i_m_axi_d_rlast,
    input  wire         i_m_axi_d_rvalid,
    output wire         o_m_axi_d_rready
);

    wire cluster_rst_n = rst_n; 
    
    // Sinais do Cluster (Internos - 512 bits)
    logic           l2_req, l2_we, l2_ack;
    logic [55:0]    l2_addr;
    logic [511:0]   l2_wdata, l2_rdata;

    // Distribuição de Interrupções
    wire [3:0] timer_irqs    = {4{i_timer_irq}};
    wire [3:0] external_irqs = {4{i_external_irq}};
    wire [3:0] software_irqs = {4{i_software_irq}};

    // Instância do Cluster
    nebula_cluster #(
        .CLUSTER_ID(0), .NUM_CORES(4), .XLEN(XLEN),
        .PADDR_WIDTH(56), .VADDR_WIDTH(39)
    ) u_cluster (
        .clk(clk), .rst_n(cluster_rst_n),
        .mem_req(l2_req), .mem_we(l2_we), .mem_addr(l2_addr),
        .mem_wdata(l2_wdata), .mem_ack(l2_ack), .mem_rdata(l2_rdata),
        .mem_error(1'b0),
        .timer_irq(timer_irqs), .external_irq(external_irqs), .software_irq(software_irqs),
        .debug_req(1'b0), .debug_halted()
    );

    // Instância do Adapter (Agora com portas AXI de 64 bits)
    nebula_axi_adapter #(
        .PADDR_WIDTH(56), .AXI_ID_WIDTH(4)
    ) u_adapter (
        .clk(clk), .rst_n(cluster_rst_n),
        // Lado Cluster (512 bits)
        .dmem_req(l2_req), .dmem_we(l2_we), .dmem_addr(l2_addr), 
        .dmem_wdata(l2_wdata), .dmem_ack(l2_ack), .dmem_rdata(l2_rdata),
        
        // I-Cache desligado
        .imem_req(1'b0), .imem_addr('0), .imem_ack(), .imem_data(),

        // Lado AXI (64 bits)
        .m_axi_d_awid(o_m_axi_d_awid), .m_axi_d_awaddr(o_m_axi_d_awaddr),
        .m_axi_d_awlen(o_m_axi_d_awlen), .m_axi_d_awsize(o_m_axi_d_awsize),
        .m_axi_d_awburst(o_m_axi_d_awburst), .m_axi_d_awvalid(o_m_axi_d_awvalid),
        .m_axi_d_awready(i_m_axi_d_awready), .m_axi_d_wdata(o_m_axi_d_wdata),
        .m_axi_d_wstrb(o_m_axi_d_wstrb), .m_axi_d_wlast(o_m_axi_d_wlast),
        .m_axi_d_wvalid(o_m_axi_d_wvalid), .m_axi_d_wready(i_m_axi_d_wready),
        .m_axi_d_bid(i_m_axi_d_bid), .m_axi_d_bresp(i_m_axi_d_bresp),
        .m_axi_d_bvalid(i_m_axi_d_bvalid), .m_axi_d_bready(o_m_axi_d_bready),
        .m_axi_d_arid(o_m_axi_d_arid), .m_axi_d_araddr(o_m_axi_d_araddr),
        .m_axi_d_arlen(o_m_axi_d_arlen), .m_axi_d_arsize(o_m_axi_d_arsize),
        .m_axi_d_arburst(o_m_axi_d_arburst), .m_axi_d_arvalid(o_m_axi_d_arvalid),
        .m_axi_d_arready(i_m_axi_d_arready), .m_axi_d_rdata(i_m_axi_d_rdata),
        .m_axi_d_rresp(i_m_axi_d_rresp), .m_axi_d_rlast(i_m_axi_d_rlast),
        .m_axi_d_rvalid(i_m_axi_d_rvalid), .m_axi_d_rready(o_m_axi_d_rready),

        // I-Master (Tie-off)
        .m_axi_i_arid(o_m_axi_i_arid), .m_axi_i_araddr(o_m_axi_i_araddr),
        .m_axi_i_arlen(o_m_axi_i_arlen), .m_axi_i_arsize(o_m_axi_i_arsize),
        .m_axi_i_arburst(o_m_axi_i_arburst), .m_axi_i_arvalid(o_m_axi_i_arvalid),
        .m_axi_i_arready(i_m_axi_i_arready), .m_axi_i_rdata(i_m_axi_i_rdata),
        .m_axi_i_rresp(i_m_axi_i_rresp), .m_axi_i_rlast(i_m_axi_i_rlast),
        .m_axi_i_rvalid(i_m_axi_i_rvalid), .m_axi_i_rready(o_m_axi_i_rready)
    );
endmodule
