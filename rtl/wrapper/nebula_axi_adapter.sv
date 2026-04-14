`timescale 1ns/1ps

/**
 * @module nebula_axi_adapter
 * @brief Adaptador Nebula (512-bit) <-> AXI4 (64-bit)
 *
 * PROTOCOLO AXI4 utilizado:
 * - INCR burst (arburst/awburst = 2'b01)
 * - 8 beats de 64 bits = 512 bits por transação (arlen/awlen = 8'd7)
 * - arsize/awsize = 3'b011 (8 bytes por beat)
 * - Endereço sempre alinhado a 64 bytes (linha de cache)
 */
module nebula_axi_adapter #(
    parameter int PADDR_WIDTH  = 56,
    parameter int AXI_ID_WIDTH = 4
)(
    input  wire                     clk,
    input  wire                     rst_n,

    // I-Cache (512-bit)
    input  wire                     imem_req,
    input  wire [PADDR_WIDTH-1:0]   imem_addr,
    output logic                    imem_ack,
    output logic [511:0]            imem_data,

    // D-Cache (512-bit)
    input  wire                     dmem_req,
    input  wire                     dmem_we,
    input  wire [PADDR_WIDTH-1:0]   dmem_addr,
    input  wire [511:0]             dmem_wdata,
    
    // FIX 1: wstrb por beat ampliado para a linha completa (64 bits)
    input  wire [63:0]              dmem_wstrb,
    input  wire                     dmem_uncached, // Flag MMIO

    output logic                    dmem_ack,
    output logic [511:0]            dmem_rdata,

    // AXI4 Instruction
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

    // AXI4 Data
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

    // =========================================================================
    // Constantes AXI fixas (burst de 8 beats de 64 bits = 512 bits)
    // =========================================================================
    localparam logic [7:0]  AXI_LEN   = 8'd7;       // FIX 3: 8 beats (0-indexed)
    localparam logic [2:0]  AXI_SIZE  = 3'b011;     // 8 bytes por beat
    localparam logic [1:0]  AXI_BURST = 2'b01;      // INCR

    // =========================================================================
    // FSM
    // =========================================================================
    typedef enum logic [2:0] {
        IDLE,
        R_ADDR,
        R_DATA,   // recebe todos os beats do burst de leitura
        W_ADDR,
        W_DATA,   // envia todos os beats do burst de escrita
        W_RESP
    } state_t;

    // =========================================================================
    // D-Cache FSM
    // =========================================================================
    state_t         d_state;
    logic [2:0]     d_beat;          // contador de beats 0..7
    logic [511:0]   d_rbuf;          // buffer de leitura
    logic [511:0]   d_wbuf_reg;      // linha a escrever (registrada)
    logic [63:0]    d_wstrb_reg;     // wstrb da requisição (64 bits)
    logic [PADDR_WIDTH-1:0] d_base;  // endereço base alinhado

    // Saídas fixas do canal D
    assign m_axi_d_arid    = '0;
    assign m_axi_d_awid    = '0;
    assign m_axi_d_arlen   = dmem_uncached ? 8'h00 : AXI_LEN;
    assign m_axi_d_arsize  = AXI_SIZE;
    assign m_axi_d_arburst = AXI_BURST;
    
    // Bypass MMIO: Se uncached, apenas 1 batida. Senão, burst completo.
    assign m_axi_d_awlen   = dmem_uncached ? 8'h00 : 8'h07;
    assign m_axi_d_awsize  = AXI_SIZE;
    assign m_axi_d_awburst = AXI_BURST;

    // O beat alvo é determinado pelo offset[5:3] do endereço original.
    logic [2:0] d_target_beat;
    logic [2:0] d_target_beat_reg;
    assign d_target_beat = d_base[5:3];  // offset dentro da linha de 64 bytes

    // Dado e strobe do beat atual (com suporte a MMIO que busca a palavra exata)
    wire [63:0] d_wbeat_data = dmem_uncached ? d_wbuf_reg[d_target_beat_reg * 64 +: 64] : d_wbuf_reg[d_beat * 64 +: 64];
    wire [7:0]  d_wbeat_strb = dmem_uncached ? d_wstrb_reg[d_target_beat_reg * 8 +: 8]  : d_wstrb_reg[d_beat * 8 +: 8];

    always_comb begin
        if (dmem_uncached)
            m_axi_d_wstrb = d_wbeat_strb;
        else if (d_wstrb_reg == 64'hFFFFFFFFFFFFFFFF || d_beat != d_target_beat_reg)
            m_axi_d_wstrb = 8'hFF;
        else
            m_axi_d_wstrb = d_wbeat_strb;
    end

    assign m_axi_d_wdata = d_wbeat_data;
    
    // Se for MMIO, a primeira batida já é a última!
    assign m_axi_d_wlast = dmem_uncached ? 1'b1 : (d_beat == 3'd7);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            d_state         <= IDLE;
            d_beat          <= '0;
            d_base          <= '0;
            d_rbuf          <= '0;
            d_wbuf_reg      <= '0;
            d_wstrb_reg     <= '0;
            dmem_ack        <= 1'b0;
            dmem_rdata      <= '0;
            m_axi_d_arvalid <= 1'b0;
            m_axi_d_araddr  <= '0;
            m_axi_d_rready  <= 1'b0;
            m_axi_d_awvalid <= 1'b0;
            m_axi_d_awaddr  <= '0;
            m_axi_d_wvalid  <= 1'b0;
            m_axi_d_bready  <= 1'b0;
        end else begin
            dmem_ack <= 1'b0;
            case (d_state)

                IDLE: begin
                    d_beat <= '0;
                    if (dmem_req) begin
                        d_base            <= {dmem_addr[PADDR_WIDTH-1:6], 6'b0};
                        d_wbuf_reg        <= dmem_wdata;
                        d_wstrb_reg       <= dmem_wstrb;
                        d_target_beat_reg <= dmem_addr[5:3];

                        if (!dmem_we) begin
                            // Para MMIO passamos o endereço exato
                            m_axi_d_araddr  <= dmem_uncached ? dmem_addr : {dmem_addr[PADDR_WIDTH-1:6], 6'b0};
                            m_axi_d_arvalid <= 1'b1;
                            d_state         <= R_ADDR;
                        end else begin
                            // Para MMIO passamos o endereço exato
                            m_axi_d_awaddr  <= dmem_uncached ? dmem_addr : {dmem_addr[PADDR_WIDTH-1:6], 6'b0};
                            m_axi_d_awvalid <= 1'b1;
                            d_state         <= W_ADDR;
                        end
                    end
                end

                // --- Leitura ---
                R_ADDR: begin
                    if (m_axi_d_arready) begin
                        m_axi_d_arvalid <= 1'b0;
                        m_axi_d_rready  <= 1'b1;
                        d_state         <= R_DATA;
                    end
                end

                // Recebe burst de 8 beats em R_DATA sincronizado
                R_DATA: begin
                    if (m_axi_d_rvalid) begin
                        // Captura o beat atual para o buffer
                        d_rbuf[d_beat * 64 +: 64] <= m_axi_d_rdata; 
                        
                        if (m_axi_d_rlast) begin
                            m_axi_d_rready <= 1'b0;
                            dmem_rdata     <= dmem_uncached ?
                                                {8{m_axi_d_rdata}} : 
                                                {m_axi_d_rdata,
                                                d_rbuf[6*64 +: 64],
                                                d_rbuf[5*64 +: 64],
                                                d_rbuf[4*64 +: 64],
                                                d_rbuf[3*64 +: 64],
                                                d_rbuf[2*64 +: 64],
                                                d_rbuf[1*64 +: 64],
                                                d_rbuf[0*64 +: 64]};
                            dmem_ack <= 1'b1;
                            d_state  <= IDLE;
                        end else begin
                            d_beat <= d_beat + 1;
                        end
                    end
                end

                // --- Escrita ---
                W_ADDR: begin
                    if (m_axi_d_awready) begin
                        m_axi_d_awvalid <= 1'b0;
                        m_axi_d_wvalid  <= 1'b1;
                        d_state         <= W_DATA;
                    end
                end

                // FIX 3: envia burst de 8 beats em W_DATA (wlast no beat 7)
                // FIX 2: beat avança SOMENTE quando wready é aceito
                W_DATA: begin
                    if (m_axi_d_wready) begin
                        if (m_axi_d_wlast) begin
                            m_axi_d_wvalid <= 1'b0;
                            m_axi_d_bready <= 1'b1;
                            d_state        <= W_RESP;
                        end else begin
                            d_beat <= d_beat + 1; // FIX 2: incremento aqui
                        end
                    end
                end

                W_RESP: begin
                    if (m_axi_d_bvalid) begin
                        m_axi_d_bready <= 1'b0;
                        dmem_ack       <= 1'b1;
                        d_beat         <= '0;
                        d_state        <= IDLE;
                    end
                end

                default: d_state <= IDLE;
            endcase
        end
    end

    // =========================================================================
    // I-Cache FSM (mesma estrutura — burst de 8 beats de leitura)
    // D-Cache tem prioridade: I-Cache só inicia quando D está IDLE.
    // =========================================================================
    state_t         i_state;
    logic [2:0]     i_beat;
    logic [511:0]   i_rbuf;
    logic [PADDR_WIDTH-1:0] i_base;

    assign m_axi_i_arid    = '0;
    assign m_axi_i_arlen   = AXI_LEN;
    assign m_axi_i_arsize  = AXI_SIZE;
    assign m_axi_i_arburst = AXI_BURST;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            i_state         <= IDLE;
            i_beat          <= '0;
            i_base          <= '0;
            i_rbuf          <= '0;
            imem_ack        <= 1'b0;
            imem_data       <= '0;
            m_axi_i_arvalid <= 1'b0;
            m_axi_i_araddr  <= '0;
            m_axi_i_rready  <= 1'b0;
        end else begin
            imem_ack <= 1'b0;
            case (i_state)

                IDLE: begin
                    i_beat <= '0;
                    // D-Cache tem prioridade
                    if (imem_req && d_state == IDLE) begin
                        i_base          <= {imem_addr[PADDR_WIDTH-1:6], 6'b0};
                        m_axi_i_araddr  <= {imem_addr[PADDR_WIDTH-1:6], 6'b0};
                        m_axi_i_arvalid <= 1'b1;
                        i_state         <= R_ADDR;
                    end
                end

                R_ADDR: begin
                    if (m_axi_i_arready) begin
                        m_axi_i_arvalid <= 1'b0;
                        m_axi_i_rready  <= 1'b1;
                        i_state         <= R_DATA;
                    end
                end

                R_DATA: begin
                    if (m_axi_i_rvalid) begin
                        i_rbuf[i_beat * 64 +: 64] <= m_axi_i_rdata;
                        if (m_axi_i_rlast) begin
                            m_axi_i_rready <= 1'b0;
                            imem_data      <= {m_axi_i_rdata,
                                              i_rbuf[6*64 +: 64],
                                              i_rbuf[5*64 +: 64],
                                              i_rbuf[4*64 +: 64],
                                              i_rbuf[3*64 +: 64],
                                              i_rbuf[2*64 +: 64],
                                              i_rbuf[1*64 +: 64],
                                              i_rbuf[0*64 +: 64]};
                            imem_ack <= 1'b1;
                            i_state  <= IDLE;
                        end else begin
                            i_beat <= i_beat + 1;
                        end
                    end
                end

                default: i_state <= IDLE;
            endcase
        end
    end

    // Debug para LiteX
    logic dbg_arvalid_ant;
    logic dbg_awvalid_ant;
    always_ff @(posedge clk) begin
        dbg_arvalid_ant <= m_axi_d_arvalid;
        dbg_awvalid_ant <= m_axi_d_awvalid;
        
        if (m_axi_d_arvalid && !dbg_arvalid_ant) begin
            $display("[NEBULA READ] Endereco: %h", m_axi_d_araddr);
        end
        
        if (m_axi_d_awvalid && !dbg_awvalid_ant) begin
            $display("\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            $display("[NEBULA WRITE] O CPU TENTOU ESCREVER EM: %h", m_axi_d_awaddr);
            $display("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
        end
    end

endmodule
