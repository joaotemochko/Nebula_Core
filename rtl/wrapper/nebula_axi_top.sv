`timescale 1ns/1ps
`default_nettype none

import nebula_pkg::*;

/**
 * @module nebula_axi_top
 * @brief Cluster Nebula com interface AXI4 (64 bits) para integração em SoC
 *        (ex.: LiteX).
 *
 * - Um único master AXI4 (instruções, dados, PTW e MMIO).
 * - Interrupções por hart: timer (MTIP), software (MSIP), externa M (MEIP) e
 *   externa S (SEIP). O CLINT/PLIC ficam no SoC.
 * - mtime: contador local incrementado a cada TIMER_DIV ciclos, usado pelo
 *   CSR 'time'. Num SoC com CLINT, use o mesmo divisor do CLINT.
 * - Mapa físico (PMA) parametrizável; o padrão corresponde ao SoC LiteX de
 *   SoC_Simulation/ (ROM/SRAM 0x1000_0000, RAM 0x4000_0000, CSR 0xF000_0000).
 */
module nebula_axi_top #(
    parameter int          NUM_CORES    = 1,
    parameter logic [63:0] RESET_VECTOR = 64'h0000_0000_1000_0000,
    parameter int          TIMER_DIV    = 1,
    parameter logic [PMA_N-1:0][55:0] PMA_BASE = {
        56'h0, 56'h0, 56'h0, 56'h0, 56'h0,
        56'h0000_00F0_000000,    // CSRs LiteX (IO)
        56'h0000_0040_000000,    // RAM principal
        56'h0000_0010_000000     // ROM + SRAM
    },
    parameter logic [PMA_N-1:0][55:0] PMA_SIZE = {
        56'h0, 56'h0, 56'h0, 56'h0, 56'h0,
        56'h0000_0010_000000,
        56'h0000_0040_000000,
        56'h0000_0000_100000
    },
    parameter logic [PMA_N-1:0][2:0]  PMA_ATTR = {
        3'b000, 3'b000, 3'b000, 3'b000, 3'b000, 3'b001, 3'b111, 3'b111
    }
)(
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire [63:0]           i_reset_vector,   // normalmente = RESET_VECTOR

    input  wire [NUM_CORES-1:0]  i_timer_irq,
    input  wire [NUM_CORES-1:0]  i_software_irq,
    input  wire [NUM_CORES-1:0]  i_external_irq,
    input  wire [NUM_CORES-1:0]  i_s_external_irq,

    output wire [3:0]   o_m_axi_awid,
    output wire [63:0]  o_m_axi_awaddr,
    output wire [7:0]   o_m_axi_awlen,
    output wire [2:0]   o_m_axi_awsize,
    output wire [1:0]   o_m_axi_awburst,
    output wire         o_m_axi_awlock,
    output wire [3:0]   o_m_axi_awcache,
    output wire [2:0]   o_m_axi_awprot,
    output wire [3:0]   o_m_axi_awqos,
    output wire         o_m_axi_awvalid,
    input  wire         i_m_axi_awready,
    output wire [63:0]  o_m_axi_wdata,
    output wire [7:0]   o_m_axi_wstrb,
    output wire         o_m_axi_wlast,
    output wire         o_m_axi_wvalid,
    input  wire         i_m_axi_wready,
    input  wire [3:0]   i_m_axi_bid,
    input  wire [1:0]   i_m_axi_bresp,
    input  wire         i_m_axi_bvalid,
    output wire         o_m_axi_bready,
    output wire [3:0]   o_m_axi_arid,
    output wire [63:0]  o_m_axi_araddr,
    output wire [7:0]   o_m_axi_arlen,
    output wire [2:0]   o_m_axi_arsize,
    output wire [1:0]   o_m_axi_arburst,
    output wire         o_m_axi_arlock,
    output wire [3:0]   o_m_axi_arcache,
    output wire [2:0]   o_m_axi_arprot,
    output wire [3:0]   o_m_axi_arqos,
    output wire         o_m_axi_arvalid,
    input  wire         i_m_axi_arready,
    input  wire [3:0]   i_m_axi_rid,
    input  wire [63:0]  i_m_axi_rdata,
    input  wire [1:0]   i_m_axi_rresp,
    input  wire         i_m_axi_rlast,
    input  wire         i_m_axi_rvalid,
    output wire         o_m_axi_rready
);
    logic         mem_req, mem_we, mem_uncached, mem_ack, mem_error;
    logic [55:0]  mem_addr;
    logic [511:0] mem_wdata, mem_rdata;
    logic [63:0]  mem_wstrb;

    // contador de tempo real local
    logic [63:0] mtime;
    logic [31:0] tdiv;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mtime <= '0; tdiv <= '0;
        end else if (tdiv == 32'(TIMER_DIV - 1)) begin
            tdiv <= '0; mtime <= mtime + 64'd1;
        end else begin
            tdiv <= tdiv + 32'd1;
        end
    end

    nebula_cluster #(
        .NUM_CORES(NUM_CORES), .PMA_BASE(PMA_BASE), .PMA_SIZE(PMA_SIZE), .PMA_ATTR(PMA_ATTR)
    ) u_cluster (
        .clk, .rst_n, .reset_vector(i_reset_vector),
        .mem_req, .mem_we, .mem_addr, .mem_wdata, .mem_wstrb, .mem_uncached,
        .mem_ack, .mem_rdata, .mem_error,
        .timer_irq(i_timer_irq), .software_irq(i_software_irq),
        .external_irq(i_external_irq), .s_external_irq(i_s_external_irq), .mtime,
        .trace_valid(), .trace_pc(), .trace_instr(), .trace_rd_en(), .trace_rd_fp(),
        .trace_rd(), .trace_rd_val(), .trace_mmio(), .trace_trap(), .trace_trap_intr(),
        .trace_trap_cause(), .trace_trap_epc(), .trace_trap_tval(), .trace_priv()
    );

    nebula_axi_bridge #(.ID_W(4)) u_bridge (
        .clk, .rst_n,
        .mem_req, .mem_we, .mem_addr, .mem_wdata, .mem_wstrb, .mem_uncached,
        .mem_ack, .mem_rdata, .mem_error,
        .m_axi_awid(o_m_axi_awid), .m_axi_awaddr(o_m_axi_awaddr), .m_axi_awlen(o_m_axi_awlen),
        .m_axi_awsize(o_m_axi_awsize), .m_axi_awburst(o_m_axi_awburst), .m_axi_awlock(o_m_axi_awlock),
        .m_axi_awcache(o_m_axi_awcache), .m_axi_awprot(o_m_axi_awprot), .m_axi_awqos(o_m_axi_awqos),
        .m_axi_awvalid(o_m_axi_awvalid), .m_axi_awready(i_m_axi_awready),
        .m_axi_wdata(o_m_axi_wdata), .m_axi_wstrb(o_m_axi_wstrb), .m_axi_wlast(o_m_axi_wlast),
        .m_axi_wvalid(o_m_axi_wvalid), .m_axi_wready(i_m_axi_wready),
        .m_axi_bid(i_m_axi_bid), .m_axi_bresp(i_m_axi_bresp), .m_axi_bvalid(i_m_axi_bvalid),
        .m_axi_bready(o_m_axi_bready),
        .m_axi_arid(o_m_axi_arid), .m_axi_araddr(o_m_axi_araddr), .m_axi_arlen(o_m_axi_arlen),
        .m_axi_arsize(o_m_axi_arsize), .m_axi_arburst(o_m_axi_arburst), .m_axi_arlock(o_m_axi_arlock),
        .m_axi_arcache(o_m_axi_arcache), .m_axi_arprot(o_m_axi_arprot), .m_axi_arqos(o_m_axi_arqos),
        .m_axi_arvalid(o_m_axi_arvalid), .m_axi_arready(i_m_axi_arready),
        .m_axi_rid(i_m_axi_rid), .m_axi_rdata(i_m_axi_rdata), .m_axi_rresp(i_m_axi_rresp),
        .m_axi_rlast(i_m_axi_rlast), .m_axi_rvalid(i_m_axi_rvalid), .m_axi_rready(o_m_axi_rready)
    );

endmodule
