`timescale 1ns/1ps

module soc_tb;

  reg clk;
  reg reset_n;  // reset ativo em nível baixo (se for ativo alto, inverta)

  // Instanciação do DUT (Device Under Test)

  soc_top uut (
    .clk    (clk),
    .rst(reset_n)
  );

  // Geração do clock: período = 100 ns
  initial begin
    clk = 0;
  end

    always #5 clk= ~clk;
  // Sequência de reset
  initial begin
    reset_n = 0;      // aplica reset
    #10;              // mantém por 10ns
    reset_n = 1;      // libera reset
  end
  
  // Tempo de simulação
  initial begin
    #1000;            // roda por 1000ns
    $finish;
  end

endmodule
