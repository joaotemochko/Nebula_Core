`timescale 1ns/1ps
`default_nettype none

import nebula_pkg::*;

/**
 * @module nebula_cluster
 * @brief Cluster de 4 núcleos Nebula com L2 compartilhado
 *
 * CORREÇÕES APLICADAS:
 * 1. Novos sinais PTW de write (ptw_mem_we, ptw_mem_wdata) adicionados
 *    à interface do nebula_core e roteados pelo árbitro L2.
 * 2. PTW write requests têm prioridade sobre leituras normais para
 *    garantir que os bits A/D sejam escritos antes que outra tradução
 *    do mesmo endereço aconteça.
 */
module nebula_cluster #(
    parameter int CLUSTER_ID    = 0,
    parameter int NUM_CORES     = 4,
    parameter int XLEN          = 64,
    parameter int PADDR_WIDTH   = 56,
    parameter int VADDR_WIDTH   = 39,
    parameter int L1I_SIZE_KB   = 32,
    parameter int L1D_SIZE_KB   = 32,
    parameter int L1_LINE_SIZE  = 64,
    parameter int L1_WAYS       = 4,
    parameter int L2_SIZE_KB    = 512,
    parameter int L2_LINE_SIZE  = 64,
    parameter int L2_WAYS       = 8,
    parameter int L2_BANKS      = 4
)(
    input  wire                       clk,
    input  wire                       rst_n,

    output logic                      mem_req,
    output logic                      mem_we,
    output logic [PADDR_WIDTH-1:0]    mem_addr,
    output logic [L2_LINE_SIZE*8-1:0] mem_wdata,
    input  wire                       mem_ack,
    input  wire [L2_LINE_SIZE*8-1:0]  mem_rdata,
    input  wire                       mem_error,

    input  wire [NUM_CORES-1:0]       timer_irq,
    input  wire [NUM_CORES-1:0]       external_irq,
    input  wire [NUM_CORES-1:0]       software_irq,

    input  wire                       debug_req,
    output logic [NUM_CORES-1:0]      debug_halted
);

    // =========================================================================
    // L2 / Snoop interfaces
    // =========================================================================
    l2_req_t    core_l2_req  [NUM_CORES];
    l2_resp_t   core_l2_resp [NUM_CORES];
    snoop_req_t  snoop_req   [NUM_CORES];
    snoop_resp_t snoop_resp  [NUM_CORES];

    // =========================================================================
    // Per-core L1 <-> L2 signals
    // =========================================================================
    logic                      l1i_l2_req    [NUM_CORES];
    logic [PADDR_WIDTH-1:0]    l1i_l2_addr   [NUM_CORES];
    logic                      l1i_l2_ack    [NUM_CORES];
    logic [L1_LINE_SIZE*8-1:0] l1i_l2_data   [NUM_CORES];

    logic                      l1d_l2_req    [NUM_CORES];
    logic                      l1d_l2_we     [NUM_CORES];
    logic [PADDR_WIDTH-1:0]    l1d_l2_addr   [NUM_CORES];
    logic [L1_LINE_SIZE*8-1:0] l1d_l2_wdata  [NUM_CORES];
    logic                      l1d_l2_ack    [NUM_CORES];
    logic [L1_LINE_SIZE*8-1:0] l1d_l2_rdata  [NUM_CORES];
    logic                      l1d_l2_is_amo [NUM_CORES];
    logic [4:0]                l1d_l2_amo_op [NUM_CORES];
    logic                      l1d_l2_upgrade[NUM_CORES];

    // FIX 1: sinais de write do PTW (A/D writeback)
    logic                      ptw_mem_req   [NUM_CORES];
    logic [PADDR_WIDTH-1:0]    ptw_mem_addr  [NUM_CORES];
    logic                      ptw_mem_we    [NUM_CORES];
    logic [XLEN-1:0]           ptw_mem_wdata [NUM_CORES];
    logic                      ptw_mem_ack   [NUM_CORES];
    logic [XLEN-1:0]           ptw_mem_data  [NUM_CORES];

    // =========================================================================
    // Core Instances
    // =========================================================================
    genvar c;
    generate
        for (c = 0; c < NUM_CORES; c++) begin : core_gen

            localparam int HART_ID = CLUSTER_ID * NUM_CORES + c;

            nebula_core_full #(
                .HART_ID(HART_ID), .XLEN(XLEN),
                .PADDR_WIDTH(PADDR_WIDTH), .VADDR_WIDTH(VADDR_WIDTH),
                .L1I_SIZE_KB(L1I_SIZE_KB), .L1D_SIZE_KB(L1D_SIZE_KB),
                .L1_LINE_SIZE(L1_LINE_SIZE), .L1_WAYS(L1_WAYS)
            ) u_core (
                .clk, .rst_n,

                .imem_req(l1i_l2_req[c]),
                .imem_addr(l1i_l2_addr[c]),
                .imem_ack(l1i_l2_ack[c]),
                .imem_data(l1i_l2_data[c]),
                .imem_error(1'b0),

                .dmem_req(l1d_l2_req[c]),
                .dmem_we(l1d_l2_we[c]),
                .dmem_addr(l1d_l2_addr[c]),
                .dmem_wdata(l1d_l2_wdata[c]),
                .dmem_ack(l1d_l2_ack[c]),
                .dmem_rdata(l1d_l2_rdata[c]),
                .dmem_error(1'b0),
                .dmem_is_amo(l1d_l2_is_amo[c]),
                .dmem_amo_op(l1d_l2_amo_op[c]),
                .dmem_upgrade(l1d_l2_upgrade[c]),

                // FIX 2: conectar sinais PTW write
                .ptw_mem_req(ptw_mem_req[c]),
                .ptw_mem_addr(ptw_mem_addr[c]),
                .ptw_mem_we(ptw_mem_we[c]),
                .ptw_mem_wdata(ptw_mem_wdata[c]),
                .ptw_mem_ack(ptw_mem_ack[c]),
                .ptw_mem_data(ptw_mem_data[c]),
                .ptw_mem_error(1'b0),

                .snoop_req_in(snoop_req[c]),
                .snoop_resp_out(snoop_resp[c]),

                .timer_irq(timer_irq[c]),
                .external_irq(external_irq[c]),
                .software_irq(software_irq[c]),

                .debug_req,
                .debug_halted(debug_halted[c])
            );

            // =================================================================
            // Combinar requests I, D e PTW -> L2
            // FIX 3: PTW write tem prioridade para garantir A/D writeback
            //        antes de nova tradução do mesmo endereço
            // =================================================================
            always_comb begin
                core_l2_req[c] = '0;

                if (ptw_mem_req[c] && ptw_mem_we[c]) begin
                    // PTW write (A/D writeback) — máxima prioridade
                    core_l2_req[c].valid   = 1'b1;
                    core_l2_req[c].core_id = c[$clog2(NUM_CORES)-1:0];
                    core_l2_req[c].is_ifetch = 1'b0;
                    core_l2_req[c].is_write  = 1'b1;
                    core_l2_req[c].is_amo    = 1'b0;
                    core_l2_req[c].amo_op    = '0;
                    core_l2_req[c].addr      = ptw_mem_addr[c];
                    // Empacotar 64-bit write em linha de cache (posição 0)
                    core_l2_req[c].wdata     = {'0, ptw_mem_wdata[c]};
                    core_l2_req[c].upgrade   = 1'b0;
                end
                else if (l1d_l2_req[c]) begin
                    core_l2_req[c].valid    = 1'b1;
                    core_l2_req[c].core_id  = c[$clog2(NUM_CORES)-1:0];
                    core_l2_req[c].is_ifetch = 1'b0;
                    core_l2_req[c].is_write  = l1d_l2_we[c];
                    core_l2_req[c].is_amo    = l1d_l2_is_amo[c];
                    core_l2_req[c].amo_op    = l1d_l2_amo_op[c];
                    core_l2_req[c].addr      = l1d_l2_addr[c];
                    core_l2_req[c].wdata     = l1d_l2_wdata[c];
                    core_l2_req[c].upgrade   = l1d_l2_upgrade[c];
                end
                else if (l1i_l2_req[c]) begin
                    core_l2_req[c].valid    = 1'b1;
                    core_l2_req[c].core_id  = c[$clog2(NUM_CORES)-1:0];
                    core_l2_req[c].is_ifetch = 1'b1;
                    core_l2_req[c].is_write  = 1'b0;
                    core_l2_req[c].is_amo    = 1'b0;
                    core_l2_req[c].amo_op    = '0;
                    core_l2_req[c].addr      = l1i_l2_addr[c];
                    core_l2_req[c].wdata     = '0;
                    core_l2_req[c].upgrade   = 1'b0;
                end
                else if (ptw_mem_req[c] && !ptw_mem_we[c]) begin
                    // PTW read — menor prioridade que D-Cache
                    core_l2_req[c].valid    = 1'b1;
                    core_l2_req[c].core_id  = c[$clog2(NUM_CORES)-1:0];
                    core_l2_req[c].is_ifetch = 1'b0;
                    core_l2_req[c].is_write  = 1'b0;
                    core_l2_req[c].addr      = ptw_mem_addr[c];
                end
            end

            // =================================================================
            // Rotear respostas L2 de volta para L1 / PTW
            // =================================================================
            always_comb begin
                l1i_l2_ack[c]  = core_l2_resp[c].valid && core_l2_resp[c].is_ifetch;
                l1i_l2_data[c] = core_l2_resp[c].rdata;

                // D-Cache ack: resposta não-ifetch e PTW não está fazendo write
                l1d_l2_ack[c]  = core_l2_resp[c].valid &&
                                  !core_l2_resp[c].is_ifetch &&
                                  !(ptw_mem_req[c] && ptw_mem_we[c]);
                l1d_l2_rdata[c] = core_l2_resp[c].rdata;

                // PTW ack: qualquer resposta enquanto PTW tem pedido pendente
                ptw_mem_ack[c] = core_l2_resp[c].valid && ptw_mem_req[c];

                // Extrair palavra de 64 bits da linha de cache (endereço[5:3])
                case (ptw_mem_addr[c][5:3])
                    3'd0: ptw_mem_data[c] = core_l2_resp[c].rdata[63:0];
                    3'd1: ptw_mem_data[c] = core_l2_resp[c].rdata[127:64];
                    3'd2: ptw_mem_data[c] = core_l2_resp[c].rdata[191:128];
                    3'd3: ptw_mem_data[c] = core_l2_resp[c].rdata[255:192];
                    3'd4: ptw_mem_data[c] = core_l2_resp[c].rdata[319:256];
                    3'd5: ptw_mem_data[c] = core_l2_resp[c].rdata[383:320];
                    3'd6: ptw_mem_data[c] = core_l2_resp[c].rdata[447:384];
                    3'd7: ptw_mem_data[c] = core_l2_resp[c].rdata[511:448];
                    default: ptw_mem_data[c] = '0;
                endcase
            end

        end // core_gen
    endgenerate

    // =========================================================================
    // L2 Cache
    // =========================================================================
    l2_cache #(
        .NUM_CORES(NUM_CORES), .PADDR_WIDTH(PADDR_WIDTH),
        .LINE_SIZE(L2_LINE_SIZE), .NUM_WAYS(L2_WAYS),
        .SIZE_KB(L2_SIZE_KB), .NUM_BANKS(L2_BANKS)
    ) u_l2_cache (
        .clk, .rst_n,
        .l1_req(core_l2_req),
        .l1_resp(core_l2_resp),
        .snoop_req, .snoop_resp,
        .mem_req, .mem_we, .mem_addr, .mem_wdata,
        .mem_ack, .mem_rdata, .mem_error
    );

endmodule
