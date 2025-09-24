`timescale 1ns/1ps

module addi_tb;

  reg clk;
  reg reset_n;

  // Bloco de instruções: 4x 32 bits
  reg [127:0] block_data_in;
  reg         block_valid;
  reg block_id_in;

  // Instância do DUT
  soc_top uut (
    .clk          (clk),
    .rst      (reset_n),
    .block_data_in(block_data_in),
    .block_id_in(block_id_in),
    .block_valid_in  (block_valid)
  );

  // Clock: 100ns período
  initial begin
    clk = 0;
    forever #5 clk = ~clk;
  end

  // Reset ativo baixo
  initial begin
    reset_n = 0;
    #10;
    reset_n = 1;
  end

  // Sequência de envio de blocos de instruções
  initial begin
    block_data_in = 128'd0;
    block_valid   = 0;

    // Espera sair do reset
    @(posedge reset_n);
    @(posedge clk);

    // Bloco 1: 4 instruções de 32 bits
    block_data_in = {32'h00500513, 32'h00520293, 32'h00600593, 32'h00628313};
    block_valid   = 1;
    block_id_in = 7'h01;

    // Espera DUT processar
    repeat(50000) @(posedge clk);
    block_valid = 0;
    // Bloco 2
    block_data_in = {32'h00418213, 32'h00520293, 32'h00628313, 32'h00730393};
    block_valid   = 1;
    @(posedge clk);
    block_valid   = 0;

    repeat(10000) @(posedge clk);
    $finish;
  end

endmodule
