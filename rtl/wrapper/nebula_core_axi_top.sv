`timescale 1ns/1ps

module nebula_core_axi_top #(
    parameter int HART_ID = 0,
    parameter int XLEN = 64
)(
    input  wire         clk,
    input  wire         rst_n,

    // Interrupções (Conectadas ao LiteX)
    input  wire         i_timer_irq,
    input  wire         i_external_irq,
    input  wire         i_software_irq,

    // =========================================================================
    // Interface AXI4 Instruction (Master)
    // =========================================================================
    output wire [3:0]   o_m_axi_i_arid,
    output wire [63:0]  o_m_axi_i_araddr,
    output wire [7:0]   o_m_axi_i_arlen,
    output wire [2:0]   o_m_axi_i_arsize,
    output wire [1:0]   o_m_axi_i_arburst,
    output wire         o_m_axi_i_arvalid,
    input  wire         i_m_axi_i_arready,
    input  wire [511:0] i_m_axi_i_rdata,
    input  wire [1:0]   i_m_axi_i_rresp,
    input  wire         i_m_axi_i_rlast,
    input  wire         i_m_axi_i_rvalid,
    output wire         o_m_axi_i_rready,

    // =========================================================================
    // Interface AXI4 Data (Master)
    // =========================================================================
    output wire [3:0]   o_m_axi_d_awid,
    output wire [63:0]  o_m_axi_d_awaddr,
    output wire [7:0]   o_m_axi_d_awlen,
    output wire [2:0]   o_m_axi_d_awsize,
    output wire [1:0]   o_m_axi_d_awburst,
    output wire         o_m_axi_d_awvalid,
    input  wire         i_m_axi_d_awready,
    output wire [511:0] o_m_axi_d_wdata,
    output wire [63:0]  o_m_axi_d_wstrb,
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
    input  wire [511:0] i_m_axi_d_rdata,
    input  wire [1:0]   i_m_axi_d_rresp,
    input  wire         i_m_axi_d_rlast,
    input  wire         i_m_axi_d_rvalid,
    output wire         o_m_axi_d_rready
);

    // Sinais Internos (Core <-> Adapter)
    logic           imem_req;
    logic [55:0]    imem_addr;
    logic           imem_ack;
    logic [511:0]   imem_data;
    logic           imem_error; // Adapter não gera erro por enquanto

    logic           dmem_req;
    logic           dmem_we;
    logic [55:0]    dmem_addr;
    logic [511:0]   dmem_wdata;
    logic           dmem_ack;
    logic [511:0]   dmem_rdata;

    // Tie-off error signal (AXI adapter handle errors via response codes if implemented)
    assign imem_error = 1'b0; 

    // 1. Instância do Core
    nebula_core #(
        .HART_ID(HART_ID),
        .XLEN(XLEN),
        .PADDR_WIDTH(56), // 56 bits internos
        .VADDR_WIDTH(39)
    ) u_core (
        .clk            (clk),
        .rst_n          (!rst_n),
        
        // I-Cache Interface
        .imem_req       (imem_req),
        .imem_addr      (imem_addr),
        .imem_ack       (imem_ack),
        .imem_data      (imem_data),
        .imem_error     (imem_error),
        
        // D-Cache Interface
        .dmem_req       (dmem_req),
        .dmem_we        (dmem_we),
        .dmem_addr      (dmem_addr),
        .dmem_wdata     (dmem_wdata), // O Core escreve a linha inteira (512) ou parcial?
                                      // O L1 Cache atual escreve linhas.
        .dmem_ack       (dmem_ack),
        .dmem_rdata     (dmem_rdata),
        .dmem_error     (1'b0), // TODO: mapear erro AXI
        .dmem_is_amo    (), // Ignorado pelo adaptador simples
        .dmem_amo_op    (),
        .dmem_upgrade   (),
        
        // PTW (Page Table Walker) compartilha porta com D-Cache na L2
        // Neste wrapper simples, assumimos que PTW arbitra D-Cache internamente
        // ou precisariamos de um arbiter aqui.
        // NOTA: O seu nebula_core.sv tem porta dedicada ptw_mem_req.
        // Vamos precisar de um ARBITER simples aqui se quisermos usar PTW.
        // Por agora, vamos deixar PTW desconectado ou fazer um OR simples (PERIGOSO)
        // Para simulação inicial "bare metal", MMU desligada, OK.
        .ptw_mem_req    (), 
        .ptw_mem_addr   (),
        .ptw_mem_ack    (1'b0),
        .ptw_mem_data   (64'd0),
        .ptw_mem_error  (1'b0),
        
        // Snooping (Stub)
        .snoop_req_in   ('0),
        .snoop_resp_out (),
        
        // Interrupts
        .timer_irq      (i_timer_irq),
        .external_irq   (i_external_irq),
        .software_irq   (i_software_irq),
        
        // Debug
        .debug_req      (1'b0),
        .debug_halted   ()
    );

    // 2. Instância do Adapter AXI
    nebula_axi_adapter #(
        .PADDR_WIDTH(56),
        .AXI_ID_WIDTH(4)
    ) u_adapter (
        .clk            (clk),
        .rst_n          (!rst_n),

        // Lado Core
        .imem_req       (imem_req),
        .imem_addr      (imem_addr),
        .imem_ack       (imem_ack),
        .imem_data      (imem_data),
        
        .dmem_req       (dmem_req),
        .dmem_we        (dmem_we),
        .dmem_addr      (dmem_addr),
        .dmem_wdata     (dmem_wdata), // Adapter deve suportar 512
        .dmem_ack       (dmem_ack),
        .dmem_rdata     (dmem_rdata),
        
        // Lado AXI (Conecta às saídas do módulo)
        .m_axi_i_arid   (o_m_axi_i_arid),
        .m_axi_i_araddr (o_m_axi_i_araddr[55:0]), // Truncate to 56 if adapter supports it
        // ... (mapeamento 1:1 dos fios AXI) ...
        .m_axi_i_arlen  (o_m_axi_i_arlen),
        .m_axi_i_arsize (o_m_axi_i_arsize),
        .m_axi_i_arburst(o_m_axi_i_arburst),
        .m_axi_i_arvalid(o_m_axi_i_arvalid),
        .m_axi_i_arready(i_m_axi_i_arready),
        .m_axi_i_rdata  (i_m_axi_i_rdata),
        .m_axi_i_rresp  (i_m_axi_i_rresp),
        .m_axi_i_rlast  (i_m_axi_i_rlast),
        .m_axi_i_rvalid (i_m_axi_i_rvalid),
        .m_axi_i_rready (o_m_axi_i_rready),
        
        .m_axi_d_awid   (o_m_axi_d_awid),
        .m_axi_d_awaddr (o_m_axi_d_awaddr[55:0]),
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
        .m_axi_d_araddr (o_m_axi_d_araddr[55:0]),
        .m_axi_d_arlen  (o_m_axi_d_arlen),
        .m_axi_d_arsize (o_m_axi_d_arsize),
        .m_axi_d_arburst(o_m_axi_d_arburst),
        .m_axi_d_arvalid(o_m_axi_d_arvalid),
        .m_axi_d_arready(i_m_axi_d_arready),
        .m_axi_d_rdata  (i_m_axi_d_rdata),
        .m_axi_d_rresp  (i_m_axi_d_rresp),
        .m_axi_d_rlast  (i_m_axi_d_rlast),
        .m_axi_d_rvalid (i_m_axi_d_rvalid),
        .m_axi_d_rready (o_m_axi_d_rready)
    );
    
endmodule
