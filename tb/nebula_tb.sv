`timescale 1ns/1ps
`default_nettype none

import nebula_pkg::*;

module nebula_tb;

    // =========================================================================
    // Parâmetros
    // =========================================================================
    parameter int    XLEN        = 64;
    parameter int    PADDR_WIDTH = 56;
    parameter int    VADDR_WIDTH = 39;
    parameter int    MEM_SIZE_MB = 64;
    parameter int    CLK_PERIOD  = 10;
    parameter int    TIMEOUT_CYC = 2_000_000;

    parameter logic [63:0] MEM_BASE    = 64'h8000_0000;
    parameter logic [63:0] TOHOST_ADDR = 64'h8000_1000;

    localparam int MEM_WORDS  = (MEM_SIZE_MB * 1024 * 1024) / 8;
    localparam int TOHOST_OFF = int'((TOHOST_ADDR - MEM_BASE) >> 3);

    // =========================================================================
    // Clock / Reset
    // =========================================================================
    logic clk   = 1'b0;
    logic rst_n = 1'b0;

    always #(CLK_PERIOD/2) clk = ~clk;

    initial begin
        rst_n = 1'b0;
        repeat(20) @(posedge clk);
        @(negedge clk);
        rst_n = 1'b1;
    end

    // =========================================================================
    // Memória
    // =========================================================================
    logic [63:0] mem [0:MEM_WORDS-1];

    initial begin
        for (int i = 0; i < MEM_WORDS; i++) mem[i] = 64'h0;
        begin
            string hexfile;
            if ($value$plusargs("HEXFILE=%s", hexfile)) begin
                $display("[TB] Carregando: %s", hexfile);
                $readmemh(hexfile, mem);
            end else
                $fatal(1, "[TB] ERRO: +HEXFILE=<path> não fornecido");
        end
    end

    // Lê cache line de 64 bytes alinhada
    function automatic logic [511:0] read_line(input logic [PADDR_WIDTH-1:0] addr);
        logic [511:0] line;
        logic [63:0]  base, off;
        base = {56'b0, addr[PADDR_WIDTH-1:6], 6'b0};
        for (int i = 0; i < 8; i++) begin
            off = (base + i*8 - MEM_BASE) >> 3;
            if (base + i*8 >= MEM_BASE && off < MEM_WORDS)
                line[i*64 +: 64] = mem[off];
            else
                line[i*64 +: 64] = 64'hDEAD_BEEF_DEAD_BEEF;
        end
        return line;
    endfunction

    task automatic write_line(
        input logic [PADDR_WIDTH-1:0] addr,
        input logic [511:0]           data
    );
        logic [63:0] base, off;
        base = {56'b0, addr[PADDR_WIDTH-1:6], 6'b0};
        for (int i = 0; i < 8; i++) begin
            off = (base + i*8 - MEM_BASE) >> 3;
            if (base + i*8 >= MEM_BASE && off < MEM_WORDS)
                mem[off] = data[i*64 +: 64];
        end
    endtask

    // =========================================================================
    // Sinais do core
    // =========================================================================
    logic                   imem_req, imem_ack;
    logic [PADDR_WIDTH-1:0] imem_addr;
    logic [511:0]           imem_data;

    logic                   dmem_req, dmem_we, dmem_ack;
    logic                   dmem_is_amo, dmem_upgrade;
    logic [4:0]             dmem_amo_op;
    logic [PADDR_WIDTH-1:0] dmem_addr;
    logic [511:0]           dmem_wdata, dmem_rdata;

    logic                   ptw_mem_req, ptw_mem_we, ptw_mem_ack, ptw_mem_error;
    logic [PADDR_WIDTH-1:0] ptw_mem_addr;
    logic [63:0]            ptw_mem_wdata, ptw_mem_data;

    // =========================================================================
    // Modelos de memória — resposta COMBINACIONAL (0 ciclos de latência)
    //
    // CRÍTICO: o icache espera mem_ack=1 enquanto está em S_REFILL_WAIT.
    // Com ack registrado (latência 1), o icache já saiu de S_REFILL_REQ
    // quando o ack chega, e fica preso em S_REFILL_WAIT para sempre.
    // O dcache tem o mesmo problema em S_WRITEBACK_WAIT e S_REFILL_WAIT.
    // Solução: ack combinacional no mesmo ciclo do req.
    // =========================================================================

    // I-Cache — combinacional
    always_comb begin

        imem_ack   = imem_req;
        imem_data  = imem_req ? read_line(imem_addr) : '0;
    end

    // D-Cache — combinacional
    // Writes vão para memória via always_ff para evitar race
    always_comb begin

        dmem_ack   = dmem_req;
        dmem_rdata = (dmem_req && !dmem_we) ? read_line(dmem_addr) : '0;
    end

    always_ff @(posedge clk) begin
        if (dmem_req && dmem_we)
            write_line(dmem_addr, dmem_wdata);
    end

    // PTW — combinacional para leitura, ff para escrita
    always_comb begin
        ptw_mem_ack  = ptw_mem_req;
        ptw_mem_data = '0;
        if (ptw_mem_req && !ptw_mem_we) begin
            logic [63:0] off;
            off = ({8'b0, ptw_mem_addr} - MEM_BASE) >> 3;
            if ({8'b0, ptw_mem_addr} >= MEM_BASE && off < MEM_WORDS)
                ptw_mem_data = mem[off];
        end
    end

    always_ff @(posedge clk) begin
        if (ptw_mem_req && ptw_mem_we) begin
            logic [63:0] off;
            off = ({8'b0, ptw_mem_addr} - MEM_BASE) >> 3;
            if ({8'b0, ptw_mem_addr} >= MEM_BASE && off < MEM_WORDS)
                mem[off] <= ptw_mem_wdata;
        end
    end


    // =========================================================================
    // DUT
    // =========================================================================
    nebula_core #(
        .HART_ID(0), .XLEN(XLEN),
        .PADDR_WIDTH(PADDR_WIDTH), .VADDR_WIDTH(VADDR_WIDTH)
    ) dut (
        .clk, .rst_n,
        .imem_req,  .imem_addr,  .imem_ack,  .imem_data,  .imem_error(1'b0),
        .dmem_req,  .dmem_we,    .dmem_addr, .dmem_wdata,
        .dmem_ack,  .dmem_rdata, .dmem_error(1'b0),
        .dmem_is_amo, .dmem_amo_op, .dmem_upgrade,
        .ptw_mem_req,  .ptw_mem_addr,
        .ptw_mem_we,   .ptw_mem_wdata,
        .ptw_mem_ack,  .ptw_mem_data, .ptw_mem_error(1'b0),
        .snoop_req_in('0), .snoop_resp_out(),
        .timer_irq(1'b0),  .external_irq(1'b0), .software_irq(1'b0),
        .debug_req(1'b0),  .debug_halted()
    );

    // =========================================================================
    // Usa initial + @(posedge clk) loop — Verilator suporta com --timing
    // =========================================================================
    initial begin : monitor
        logic [63:0] tv;
        integer      cyc;
        integer      max_cyc;
        string       cycles_str;
        cyc     = 0;
        max_cyc = TIMEOUT_CYC;
        if ($value$plusargs("CYCLES=%s", cycles_str))
            max_cyc = cycles_str.atoi();

        // Aguardar fim do reset
        @(posedge rst_n);
        @(posedge clk);

        forever begin
            @(posedge clk);
            cyc = cyc + 1;

            // Ler tohost direto da memória
            tv = mem[TOHOST_OFF];

            if (tv == 64'h1) begin
                $display("[PASS] ciclos=%0d", cyc);
                $finish;
            end else if (tv != 64'h0) begin
                $display("[FAIL] ciclos=%0d tohost=0x%016x (caso %0d)",
                         cyc, tv, tv >> 1);
                $finish;
            end

            if (cyc >= max_cyc) begin
                $display("[TIMEOUT] %0d ciclos tohost=0x%016x", cyc, tv);
                $finish;
            end
        end
    end

    // =========================================================================
    // Debug: mostrar PC e estado a cada ciclo (ativado com +DEBUG)
    // =========================================================================
    initial begin : debug_monitor
        if ($test$plusargs("DEBUG")) begin
            forever begin
                @(posedge clk);
                if (rst_n) begin
                    $display("CYC=%0d imem_req=%b imem_addr=0x%014x imem_ack=%b | dmem_req=%b dmem_we=%b dmem_addr=0x%014x dmem_ack=%b",
                        $time/10,
                        imem_req, imem_addr, imem_ack,
                        dmem_req, dmem_we, dmem_addr, dmem_ack);
                end
            end
        end
    end

    // =========================================================================
    // Waveform — FST é mais rápido que VCD no Verilator
    // =========================================================================
    initial begin
        if ($test$plusargs("WAVES")) begin
            $dumpfile("dump.fst");
            $dumpvars(0, nebula_tb);
            $display("[TB] Waveform: dump.fst");
        end
    end

endmodule
