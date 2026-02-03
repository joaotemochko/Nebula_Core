`timescale 1ns/1ps

module nebula_axi_adapter #(
    parameter int PADDR_WIDTH = 56,
    parameter int AXI_ID_WIDTH = 4
)(
    input  wire                     clk,
    input  wire                     rst_n,

    // Interface Nebula (Cache - 512 bits)
    input  wire                     imem_req,
    input  wire [PADDR_WIDTH-1:0]   imem_addr,
    output logic                    imem_ack,
    output logic [511:0]            imem_data,
    
    input  wire                     dmem_req,
    input  wire                     dmem_we,
    input  wire [PADDR_WIDTH-1:0]   dmem_addr,
    input  wire [511:0]             dmem_wdata,
    output logic                    dmem_ack,
    output logic [511:0]            dmem_rdata,

    // Interface AXI4 (Master - 64 bits para LiteX)
    output logic [AXI_ID_WIDTH-1:0] m_axi_i_arid,
    output logic [PADDR_WIDTH-1:0]  m_axi_i_araddr,
    output logic [7:0]              m_axi_i_arlen,
    output logic [2:0]              m_axi_i_arsize,
    output logic [1:0]              m_axi_i_arburst,
    output logic                    m_axi_i_arvalid,
    input  wire                     m_axi_i_arready,
    input  wire [63:0]              m_axi_i_rdata,
    input  wire [1:0]               m_axi_i_rresp,
    input  wire                     m_axi_i_rlast,
    input  wire                     m_axi_i_rvalid,
    output logic                    m_axi_i_rready,
    
    output logic [AXI_ID_WIDTH-1:0] m_axi_d_awid,
    output logic [PADDR_WIDTH-1:0]  m_axi_d_awaddr,
    output logic [7:0]              m_axi_d_awlen,
    output logic [2:0]              m_axi_d_awsize,
    output logic [1:0]              m_axi_d_awburst,
    output logic                    m_axi_d_awvalid,
    input  wire                     m_axi_d_awready,
    output logic [63:0]             m_axi_d_wdata,
    output logic [7:0]              m_axi_d_wstrb,
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
    input  wire [63:0]              m_axi_d_rdata,
    input  wire [1:0]               m_axi_d_rresp,
    input  wire                     m_axi_d_rlast,
    input  wire                     m_axi_d_rvalid,
    output logic                    m_axi_d_rready
);

    typedef enum logic [3:0] {IDLE, R_ADDR, R_WAIT_DATA, R_NEXT, W_ADDR, W_DATA, W_RESP, W_NEXT} state_t;
    state_t state;
    logic [3:0] cnt; 
    logic [511:0] data_buf;

    // Configurações AXI Fixas
    assign m_axi_d_arid = '0; assign m_axi_d_arlen = 8'd0; assign m_axi_d_arsize = 3'b011; assign m_axi_d_arburst = 2'b01;
    assign m_axi_d_awid = '0; assign m_axi_d_awlen = 8'd0; assign m_axi_d_awsize = 3'b011; assign m_axi_d_awburst = 2'b01;
    assign m_axi_d_wstrb = 8'hFF; assign m_axi_d_wlast = 1'b1;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            cnt <= 0;
            dmem_ack <= 0;
            m_axi_d_arvalid <= 0; m_axi_d_rready <= 0;
            m_axi_d_awvalid <= 0; m_axi_d_wvalid <= 0; m_axi_d_bready <= 0;
        end else begin
            case (state)
                IDLE: begin
                    dmem_ack <= 0;
                    cnt <= 0;
                    if (dmem_req) begin
                        if (!dmem_we) begin 
                            m_axi_d_araddr <= dmem_addr; 
                            m_axi_d_arvalid <= 1;
                            state <= R_ADDR;
                        end else begin
                            m_axi_d_awaddr <= dmem_addr;
                            m_axi_d_awvalid <= 1;
                            m_axi_d_wdata <= dmem_wdata[63:0];
                            state <= W_ADDR;
                        end
                    end
                end

                R_ADDR: begin
                    if (m_axi_d_arready) begin
                        m_axi_d_arvalid <= 0;
                        m_axi_d_rready <= 1;
                        state <= R_WAIT_DATA;
                    end
                end

                R_WAIT_DATA: begin
                    if (m_axi_d_rvalid) begin
                        m_axi_d_rready <= 0;
                        data_buf[cnt*64 +: 64] <= m_axi_d_rdata;
                        state <= R_NEXT;
                    end
                end

                R_NEXT: begin
                    if (cnt == 7) begin
                        dmem_rdata <= {m_axi_d_rdata, data_buf[447:0]};
                        dmem_ack <= 1;
                        state <= IDLE;
                    end else begin
                        cnt <= cnt + 1;
                        m_axi_d_araddr <= m_axi_d_araddr + 8;
                        m_axi_d_arvalid <= 1;
                        state <= R_ADDR;
                    end
                end

                W_ADDR: begin
                    if (m_axi_d_awready) begin
                        m_axi_d_awvalid <= 0;
                        m_axi_d_wvalid <= 1;
                        state <= W_DATA;
                    end
                end
                
                W_DATA: begin
                    if (m_axi_d_wready) begin
                        m_axi_d_wvalid <= 0;
                        m_axi_d_bready <= 1;
                        state <= W_RESP;
                    end
                end

                W_RESP: begin
                    if (m_axi_d_bvalid) begin
                        m_axi_d_bready <= 0;
                        state <= W_NEXT;
                    end
                end

                W_NEXT: begin
                    if (cnt == 7) begin
                        dmem_ack <= 1;
                        state <= IDLE;
                    end else begin
                        cnt <= cnt + 1;
                        m_axi_d_awaddr <= m_axi_d_awaddr + 8;
                        m_axi_d_wdata <= dmem_wdata[(cnt+1)*64 +: 64];
                        m_axi_d_awvalid <= 1;
                        state <= W_ADDR;
                    end
                end
            endcase
        end
    end

    // Tie-offs
    assign m_axi_i_arid = '0; assign m_axi_i_araddr = '0; assign m_axi_i_arlen = 0;
    assign m_axi_i_arsize = 0; assign m_axi_i_arburst = 0; assign m_axi_i_arvalid = 0;
    assign m_axi_i_rready = 0; assign imem_ack = 0; assign imem_data = '0;

endmodule
