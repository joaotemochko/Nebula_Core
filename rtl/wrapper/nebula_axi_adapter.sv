`timescale 1ns/1ps

module nebula_axi_adapter #(
    parameter int PADDR_WIDTH = 56,
    parameter int AXI_ID_WIDTH = 4
)(
    input  wire                     clk,
    input  wire                     rst_n,

    // =========================================================================
    // Interface Nebula (Nativa - Cache Lines de 512 bits)
    // =========================================================================
    
    // I-Cache
    input  wire                     imem_req,
    input  wire [PADDR_WIDTH-1:0]   imem_addr,
    output logic                    imem_ack,
    output logic [511:0]            imem_data,
    
    // D-Cache
    input  wire                     dmem_req,
    input  wire                     dmem_we,
    input  wire [PADDR_WIDTH-1:0]   dmem_addr,
    input  wire [511:0]             dmem_wdata,
    output logic                    dmem_ack,
    output logic [511:0]            dmem_rdata,

    // =========================================================================
    // Interface AXI4 (Master) - 512-bit Data Width
    // =========================================================================
    
    // AXI4 Instruction Master (Read Only)
    output logic [AXI_ID_WIDTH-1:0] m_axi_i_arid,
    output logic [PADDR_WIDTH-1:0]  m_axi_i_araddr,
    output logic [7:0]              m_axi_i_arlen,   // 0 = 1 beat
    output logic [2:0]              m_axi_i_arsize,  // 6 = 64 bytes (512 bits)
    output logic [1:0]              m_axi_i_arburst, // 1 = INCR
    output logic                    m_axi_i_arvalid,
    input  wire                     m_axi_i_arready,
    input  wire [511:0]             m_axi_i_rdata,
    input  wire [1:0]               m_axi_i_rresp,
    input  wire                     m_axi_i_rlast,
    input  wire                     m_axi_i_rvalid,
    output logic                    m_axi_i_rready,
    
    // AXI4 Data Master (Read/Write)
    output logic [AXI_ID_WIDTH-1:0] m_axi_d_awid,
    output logic [PADDR_WIDTH-1:0]  m_axi_d_awaddr,
    output logic [7:0]              m_axi_d_awlen,
    output logic [2:0]              m_axi_d_awsize,
    output logic [1:0]              m_axi_d_awburst,
    output logic                    m_axi_d_awvalid,
    input  wire                     m_axi_d_awready,
    output logic [511:0]            m_axi_d_wdata,
    output logic [63:0]             m_axi_d_wstrb,   // 64 bytes mask
    output logic                    m_axi_d_wlast,
    output logic                    m_axi_d_wvalid,
    input  wire                     m_axi_d_wready,
    input  wire [AXI_ID_WIDTH-1:0]  m_axi_d_bid,
    input  wire [1:0]               m_axi_d_bresp,
    input  wire                     m_axi_d_bvalid,
    output logic                    m_axi_d_bready,
    output logic [AXI_ID_WIDTH-1:0] m_axi_d_arid,
    output logic [PADDR_WIDTH-1:0]  m_axi_d_araddr,
    output logic [7:0]              m_axi_d_arlen,
    output logic [2:0]              m_axi_d_arsize,
    output logic [1:0]              m_axi_d_arburst,
    output logic                    m_axi_d_arvalid,
    input  wire                     m_axi_d_arready,
    input  wire [511:0]             m_axi_d_rdata,
    input  wire [1:0]               m_axi_d_rresp,
    input  wire                     m_axi_d_rlast,
    input  wire                     m_axi_d_rvalid,
    output logic                    m_axi_d_rready
);

    // =========================================================================
    // Instruction Path (Read Only)
    // =========================================================================
    
    // Constantes AXI fixas
    assign m_axi_i_arid    = '0;
    assign m_axi_i_arlen   = 8'd0;       // 1 beat (512 bits)
    assign m_axi_i_arsize  = 3'b110;     // 64 bytes = 512 bits
    assign m_axi_i_arburst = 2'b01;      // INCR
    assign m_axi_i_rready  = 1'b1;       // Sempre pronto para receber

    typedef enum logic [1:0] {I_IDLE, I_ADDR, I_WAIT_DATA} i_state_t;
    i_state_t i_state;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            i_state <= I_IDLE;
            m_axi_i_arvalid <= 1'b0;
            imem_ack <= 1'b0;
        end else begin
            case (i_state)
                I_IDLE: begin
                    imem_ack <= 1'b0;
                    if (imem_req) begin
                        m_axi_i_araddr <= imem_addr;
                        m_axi_i_arvalid <= 1'b1;
                        i_state <= I_ADDR;
                    end
                end
                
                I_ADDR: begin
                    if (m_axi_i_arready) begin
                        m_axi_i_arvalid <= 1'b0;
                        i_state <= I_WAIT_DATA;
                    end
                end
                
                I_WAIT_DATA: begin
                    if (m_axi_i_rvalid) begin
                        imem_data <= m_axi_i_rdata;
                        imem_ack <= 1'b1;
                        i_state <= I_IDLE; // Request deve baixar após Ack
                    end
                end
            endcase
        end
    end

    // =========================================================================
    // Data Path (Read / Write)
    // =========================================================================

    // Constantes AXI
    assign m_axi_d_awid    = '0;
    assign m_axi_d_arid    = '0;
    assign m_axi_d_awlen   = 8'd0;
    assign m_axi_d_arlen   = 8'd0;
    assign m_axi_d_awsize  = 3'b110;     // 64 bytes
    assign m_axi_d_arsize  = 3'b110;
    assign m_axi_d_awburst = 2'b01;
    assign m_axi_d_arburst = 2'b01;
    
    assign m_axi_d_wstrb   = {64{1'b1}}; // Write full cache line
    assign m_axi_d_wlast   = 1'b1;       // Single beat
    assign m_axi_d_bready  = 1'b1;
    assign m_axi_d_rready  = 1'b1;

    typedef enum logic [2:0] {D_IDLE, D_ADDR_R, D_WAIT_R, D_ADDR_W, D_DATA_W, D_WAIT_B} d_state_t;
    d_state_t d_state;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            d_state <= D_IDLE;
            m_axi_d_arvalid <= 0;
            m_axi_d_awvalid <= 0;
            m_axi_d_wvalid  <= 0;
            dmem_ack <= 0;
        end else begin
            case (d_state)
                D_IDLE: begin
                    dmem_ack <= 0;
                    if (dmem_req) begin
                        if (dmem_we) begin
                            // WRITE
                            m_axi_d_awaddr <= dmem_addr;
                            m_axi_d_wdata  <= dmem_wdata;
                            m_axi_d_awvalid <= 1'b1;
                            d_state <= D_ADDR_W;
                        end else begin
                            // READ
                            m_axi_d_araddr <= dmem_addr;
                            m_axi_d_arvalid <= 1'b1;
                            d_state <= D_ADDR_R;
                        end
                    end
                end

                // --- LEITURA ---
                D_ADDR_R: begin
                    if (m_axi_d_arready) begin
                        m_axi_d_arvalid <= 0;
                        d_state <= D_WAIT_R;
                    end
                end
                D_WAIT_R: begin
                    if (m_axi_d_rvalid) begin
                        dmem_rdata <= m_axi_d_rdata;
                        dmem_ack <= 1;
                        d_state <= D_IDLE;
                    end
                end

                // --- ESCRITA ---
                D_ADDR_W: begin
                    if (m_axi_d_awready) begin
                        m_axi_d_awvalid <= 0;
                        m_axi_d_wvalid <= 1; // Send data now
                        d_state <= D_DATA_W;
                    end
                end
                D_DATA_W: begin
                    if (m_axi_d_wready) begin
                        m_axi_d_wvalid <= 0;
                        d_state <= D_WAIT_B;
                    end
                end
                D_WAIT_B: begin
                    if (m_axi_d_bvalid) begin
                        dmem_ack <= 1;
                        d_state <= D_IDLE;
                    end
                end
            endcase
        end
    end

endmodule
