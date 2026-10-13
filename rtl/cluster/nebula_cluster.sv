`timescale 1ns/1ps
`default_nettype none

import nebula_pkg::*;

/**
 * @module nebula_cluster
 * @brief Cluster de 4 núcleos Nebula com L2 compartilhado
 *
 * CORREÇÕES APLICADAS:
 *
 * BUG 3 FIX — mmio_req_r removido; bypass MMIO puramente combinacional.
 *
 *   Antes: mmio_req_r era um FF com enable mem_ready. Com o fix do Bug 1,
 *   mem_ready passou a pulsar apenas no handshake AXI (arready/awready),
 *   nunca quando o adapter estava idle aguardando nova requisição.
 *   Consequência: mem_ready=0 quando MMIO chegava → FF nunca capturava
 *   mmio_req_r=1 → mux apontava para l2_mem_req=0 → D-Cache presa em
 *   S_MMIO_WAIT sem receber mem_ack.
 *
 *   Depois: wire mmio_req combinacional direto. A D-Cache mantém
 *   l1d_l2_req[0]=1 e l1d_uncached[0]=1 estáveis durante S_MMIO_REQ e
 *   S_MMIO_WAIT — o wire reflete isso sem nenhum registrador intermediário.
 *
 *   l1d_l2_ack e l1d_l2_rdata também trocados de mmio_req_r para mmio_req.
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
    parameter int L2_BANKS      = 4,
    parameter logic [63:0] RESET_VECTOR = 64'h0000_0000_1000_0000
)(
    input  wire                       clk,
    input  wire                       rst_n,

    output logic                      mem_req,
    output logic                      mem_we,
    output logic [PADDR_WIDTH-1:0]    mem_addr,
    output logic [L2_LINE_SIZE*8-1:0] mem_wdata,
    output logic [L2_LINE_SIZE-1:0]   mem_wstrb,
    output logic                      mem_uncached,
    input  wire                       mem_ack,
    input  wire [L2_LINE_SIZE*8-1:0]  mem_rdata,
    input  wire                       mem_error,
    input  wire                       mem_ready,

    input  wire [NUM_CORES-1:0]       timer_irq,
    input  wire [NUM_CORES-1:0]       external_irq,
    input  wire [NUM_CORES-1:0]       software_irq,
    input  wire [NUM_CORES-1:0]       s_external_irq,
    input  wire [63:0]                mtime,

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
    logic [L1_LINE_SIZE-1:0]   l1d_l2_wstrb  [NUM_CORES];
    logic                      l1d_uncached  [NUM_CORES];
    logic                      l1d_l2_req    [NUM_CORES];
    logic                      l1d_l2_we     [NUM_CORES];
    logic [PADDR_WIDTH-1:0]    l1d_l2_addr   [NUM_CORES];
    logic [L1_LINE_SIZE*8-1:0] l1d_l2_wdata  [NUM_CORES];
    logic                      l1d_l2_ack    [NUM_CORES];
    logic [L1_LINE_SIZE*8-1:0] l1d_l2_rdata  [NUM_CORES];
    logic                      l1d_l2_is_amo [NUM_CORES];
    logic [4:0]                l1d_l2_amo_op [NUM_CORES];
    logic                      l1d_l2_upgrade[NUM_CORES];

    // Sinais PTW
    logic                      ptw_mem_req   [NUM_CORES];
    logic [PADDR_WIDTH-1:0]    ptw_mem_addr  [NUM_CORES];
    logic                      ptw_mem_we    [NUM_CORES];
    logic [XLEN-1:0]           ptw_mem_wdata [NUM_CORES];
    logic                      ptw_mem_ack   [NUM_CORES];
    logic [XLEN-1:0]           ptw_mem_data  [NUM_CORES];

    // =========================================================================
    // BUG 3 FIX: bypass MMIO puramente combinacional.
    // Sem registrador — sem dependência de mem_ready.
    // =========================================================================
    wire mmio_req = l1d_l2_req[0] && l1d_uncached[0];

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
                .reset_vector(RESET_VECTOR),

                .imem_req(l1i_l2_req[c]),
                .imem_addr(l1i_l2_addr[c]),
                .imem_ack(l1i_l2_ack[c]),
                .imem_data(l1i_l2_data[c]),
                .imem_error(1'b0),

                .dmem_req(l1d_l2_req[c]),
                .dmem_we(l1d_l2_we[c]),
                .dmem_addr(l1d_l2_addr[c]),
                .dmem_wdata(l1d_l2_wdata[c]),
                .dmem_wstrb(l1d_l2_wstrb[c]),
                .dmem_uncached(l1d_uncached[c]),
                .dmem_ack(l1d_l2_ack[c]),
                .dmem_rdata(l1d_l2_rdata[c]),
                .dmem_error(1'b0),
                .dmem_is_amo(l1d_l2_is_amo[c]),
                .dmem_amo_op(l1d_l2_amo_op[c]),
                .dmem_upgrade(l1d_l2_upgrade[c]),

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
                .s_external_irq(s_external_irq[c]),
                .mtime(mtime),

                .debug_req,
                .debug_halted(debug_halted[c])
            );

            // =================================================================
            // Combinar requests I, D e PTW -> L2
            // PTW write tem prioridade para garantir A/D writeback correto.
            // Acessos uncached (MMIO) não entram na L2 — bypass direto.
            // =================================================================
            always_comb begin
                core_l2_req[c] = '0;

                if (ptw_mem_req[c] && ptw_mem_we[c]) begin
                    core_l2_req[c].valid     = 1'b1;
                    core_l2_req[c].core_id   = c[$clog2(NUM_CORES)-1:0];
                    core_l2_req[c].is_ifetch = 1'b0;
                    core_l2_req[c].is_write  = 1'b1;
                    core_l2_req[c].is_amo    = 1'b0;
                    core_l2_req[c].amo_op    = '0;
                    core_l2_req[c].addr      = ptw_mem_addr[c];
                    core_l2_req[c].wdata     = {'0, ptw_mem_wdata[c]};
                    core_l2_req[c].upgrade   = 1'b0;
                end
                else if (l1d_l2_req[c] && !l1d_uncached[c]) begin
                    // BUG 3 FIX: uncached não vai para a L2
                    core_l2_req[c].valid     = 1'b1;
                    core_l2_req[c].core_id   = c[$clog2(NUM_CORES)-1:0];
                    core_l2_req[c].is_ifetch = 1'b0;
                    core_l2_req[c].is_write  = l1d_l2_we[c];
                    core_l2_req[c].is_amo    = l1d_l2_is_amo[c];
                    core_l2_req[c].amo_op    = l1d_l2_amo_op[c];
                    core_l2_req[c].addr      = l1d_l2_addr[c];
                    core_l2_req[c].wdata     = l1d_l2_wdata[c];
                    core_l2_req[c].upgrade   = l1d_l2_upgrade[c];
                end
                else if (l1i_l2_req[c]) begin
                    core_l2_req[c].valid     = 1'b1;
                    core_l2_req[c].core_id   = c[$clog2(NUM_CORES)-1:0];
                    core_l2_req[c].is_ifetch = 1'b1;
                    core_l2_req[c].is_write  = 1'b0;
                    core_l2_req[c].is_amo    = 1'b0;
                    core_l2_req[c].amo_op    = '0;
                    core_l2_req[c].addr      = l1i_l2_addr[c];
                    core_l2_req[c].wdata     = '0;
                    core_l2_req[c].upgrade   = 1'b0;
                end
                else if (ptw_mem_req[c] && !ptw_mem_we[c]) begin
                    core_l2_req[c].valid     = 1'b1;
                    core_l2_req[c].core_id   = c[$clog2(NUM_CORES)-1:0];
                    core_l2_req[c].is_ifetch = 1'b0;
                    core_l2_req[c].is_write  = 1'b0;
                    core_l2_req[c].addr      = ptw_mem_addr[c];
                end
            end

            // =================================================================
            // Rotear respostas L2 -> L1 / PTW
            // BUG 3 FIX: mmio_req_r → mmio_req (wire combinacional)
            // =================================================================
            always_comb begin
                l1i_l2_ack[c]  = core_l2_resp[c].valid && core_l2_resp[c].is_ifetch;
                l1i_l2_data[c] = core_l2_resp[c].rdata;

                // MMIO: ack vem direto do adapter via mem_ack
                // Cache: ack vem da L2 via core_l2_resp
                l1d_l2_ack[c] = (c == 0 && mmio_req) ? mem_ack :
                    (core_l2_resp[c].valid &&
                     !core_l2_resp[c].is_ifetch &&
                     !(ptw_mem_req[c] && ptw_mem_we[c]));

                l1d_l2_rdata[c] = (c == 0 && mmio_req) ? mem_rdata
                                                        : core_l2_resp[c].rdata;

                ptw_mem_ack[c] = core_l2_resp[c].valid && ptw_mem_req[c];

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
    // L2 Bypass para MMIO — mux combinacional, sem registrador
    //
    // BUG 3 FIX: removido always_ff com mmio_req_r.
    // mmio_req=1 → adapter recebe req diretamente da D-Cache (uncached)
    // mmio_req=0 → adapter recebe req da L2 (cached)
    // =========================================================================
    logic l2_mem_req, l2_mem_we, l2_mem_ack;
    logic [PADDR_WIDTH-1:0]    l2_mem_addr;
    logic [L2_LINE_SIZE*8-1:0] l2_mem_wdata;

    assign mem_req      = mmio_req ? l1d_l2_req[0]   : l2_mem_req;
    assign mem_we       = mmio_req ? l1d_l2_we[0]    : l2_mem_we;
    assign mem_addr     = mmio_req ? l1d_l2_addr[0]  : l2_mem_addr;
    assign mem_wdata    = mmio_req ? l1d_l2_wdata[0] : l2_mem_wdata;
    assign mem_wstrb    = mmio_req ? l1d_l2_wstrb[0] : {L2_LINE_SIZE{1'b1}};
    assign mem_uncached = mmio_req;
    assign l2_mem_ack   = mmio_req ? 1'b0 : mem_ack;

    // =========================================================================
    // L2 Cache
    // =========================================================================
    l2_cache #(
        .NUM_CORES(NUM_CORES), .PADDR_WIDTH(PADDR_WIDTH),
        .LINE_SIZE(L2_LINE_SIZE), .NUM_WAYS(L2_WAYS),
        .SIZE_KB(L2_SIZE_KB), .NUM_BANKS(L2_BANKS)
    ) u_l2_cache (
        .clk, .rst_n,
        .l1_req(core_l2_req), .l1_resp(core_l2_resp),
        .snoop_req, .snoop_resp,
        .mem_req(l2_mem_req), .mem_we(l2_mem_we),
        .mem_addr(l2_mem_addr), .mem_wdata(l2_mem_wdata),
        .mem_ack(l2_mem_ack), .mem_rdata, .mem_error,
        .mem_ready(mem_ready)
    );

endmodule
