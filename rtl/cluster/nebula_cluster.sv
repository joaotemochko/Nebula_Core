`timescale 1ns/1ps
`default_nettype none

import nebula_pkg::*;

/**
 * @module nebula_cluster
 * @brief Cluster SMP de NUM_CORES núcleos Nebula com memória coerente.
 *
 * Cada núcleo expõe portas I/D/PTW à nebula_interconnect, que serializa os
 * acessos para uma única porta de memória (linha de 64 B com byte-strobes),
 * mantém a coerência das L1D write-through por invalidação e executa os
 * atômicos (AMO/LR/SC) com reservas por hart.
 */
module nebula_cluster #(
    parameter int NUM_CORES = 1,
    parameter logic [PMA_N-1:0][55:0] PMA_BASE = PMA_DEF_BASE,
    parameter logic [PMA_N-1:0][55:0] PMA_SIZE = PMA_DEF_SIZE,
    parameter logic [PMA_N-1:0][2:0]  PMA_ATTR = PMA_DEF_ATTR
)(
    input  wire                         clk,
    input  wire                         rst_n,
    input  wire  [63:0]                 reset_vector,

    // memória (linha de 64 B; MMIO com uncached=1)
    output logic                        mem_req,
    output logic                        mem_we,
    output logic [55:0]                 mem_addr,
    output logic [511:0]                mem_wdata,
    output logic [63:0]                 mem_wstrb,
    output logic                        mem_uncached,
    input  wire                         mem_ack,
    input  wire  [511:0]                mem_rdata,
    input  wire                         mem_error,

    // interrupções por hart
    input  wire  [NUM_CORES-1:0]        timer_irq,
    input  wire  [NUM_CORES-1:0]        software_irq,
    input  wire  [NUM_CORES-1:0]        external_irq,
    input  wire  [NUM_CORES-1:0]        s_external_irq,
    input  wire  [63:0]                 mtime,

    // trace de commit por hart
    output logic [NUM_CORES-1:0][1:0]        trace_valid,
    output logic [NUM_CORES-1:0][1:0][63:0]  trace_pc,
    output logic [NUM_CORES-1:0][1:0][31:0]  trace_instr,
    output logic [NUM_CORES-1:0][1:0]        trace_rd_en,
    output logic [NUM_CORES-1:0][1:0]        trace_rd_fp,
    output logic [NUM_CORES-1:0][1:0][4:0]   trace_rd,
    output logic [NUM_CORES-1:0][1:0][63:0]  trace_rd_val,
    output logic [NUM_CORES-1:0][1:0]        trace_mmio,
    output logic [NUM_CORES-1:0]             trace_trap,
    output logic [NUM_CORES-1:0]             trace_trap_intr,
    output logic [NUM_CORES-1:0][4:0]        trace_trap_cause,
    output logic [NUM_CORES-1:0][63:0]       trace_trap_epc,
    output logic [NUM_CORES-1:0][63:0]       trace_trap_tval,
    output logic [NUM_CORES-1:0][1:0]        trace_priv
);
    localparam int NC = NUM_CORES;

    logic [NC-1:0]          i_req, i_ack, d_req, d_we, d_unc, d_amo, d_ack, p_req, p_ack, inv_valid;
    logic [NC-1:0][55:0]    i_addr, d_addr, p_addr;
    logic [NC-1:0][511:0]   d_wdata;
    logic [NC-1:0][63:0]    d_wstrb;
    logic [NC-1:0][4:0]     d_amo_op;
    logic [511:0]           i_data, d_rdata;
    logic [63:0]            p_data;
    logic                   i_error, d_error, p_error;
    logic [49:0]            inv_line;

    for (genvar c = 0; c < NC; c++) begin : g_core
        snoop_req_t snoop;
        assign snoop = '{valid: inv_valid[c], op: COH_INVALIDATE, addr: {inv_line, 6'b0}, requester: '0};

        nebula_core_full #(
            .HART_ID(c), .PMA_BASE(PMA_BASE), .PMA_SIZE(PMA_SIZE), .PMA_ATTR(PMA_ATTR)
        ) u_core (
            .clk, .rst_n, .reset_vector,
            .imem_req(i_req[c]), .imem_addr(i_addr[c]), .imem_ack(i_ack[c]),
            .imem_data(i_data), .imem_error(i_error),
            .dmem_req(d_req[c]), .dmem_we(d_we[c]), .dmem_addr(d_addr[c]),
            .dmem_wdata(d_wdata[c]), .dmem_wstrb(d_wstrb[c]), .dmem_uncached(d_unc[c]),
            .dmem_ack(d_ack[c]), .dmem_rdata(d_rdata), .dmem_error(d_error),
            .dmem_is_amo(d_amo[c]), .dmem_amo_op(d_amo_op[c]), .dmem_upgrade(),
            .ptw_mem_req(p_req[c]), .ptw_mem_addr(p_addr[c]), .ptw_mem_we(), .ptw_mem_wdata(),
            .ptw_mem_ack(p_ack[c]), .ptw_mem_data(p_data), .ptw_mem_error(p_error),
            .snoop_req_in(snoop), .snoop_resp_out(),
            .timer_irq(timer_irq[c]), .external_irq(external_irq[c]),
            .software_irq(software_irq[c]), .s_external_irq(s_external_irq[c]),
            .mtime, .debug_req(1'b0), .debug_halted(),
            .trace_valid(trace_valid[c]), .trace_pc(trace_pc[c]), .trace_instr(trace_instr[c]),
            .trace_rd_en(trace_rd_en[c]), .trace_rd_fp(trace_rd_fp[c]), .trace_rd(trace_rd[c]),
            .trace_rd_val(trace_rd_val[c]), .trace_mmio(trace_mmio[c]),
            .trace_trap(trace_trap[c]), .trace_trap_intr(trace_trap_intr[c]),
            .trace_trap_cause(trace_trap_cause[c]), .trace_trap_epc(trace_trap_epc[c]),
            .trace_trap_tval(trace_trap_tval[c]), .trace_priv(trace_priv[c])
        );
    end

    nebula_interconnect #(.NC(NC)) u_ic (
        .clk, .rst_n,
        .i_req, .i_addr, .i_ack, .i_data, .i_error,
        .d_req, .d_we, .d_addr, .d_wdata, .d_wstrb, .d_uncached(d_unc), .d_is_amo(d_amo),
        .d_amo_op, .d_ack, .d_rdata, .d_error,
        .p_req, .p_addr, .p_ack, .p_data, .p_error,
        .inv_valid, .inv_line,
        .mem_req, .mem_we, .mem_addr, .mem_wdata, .mem_wstrb, .mem_uncached,
        .mem_ack, .mem_rdata, .mem_error
    );

endmodule
