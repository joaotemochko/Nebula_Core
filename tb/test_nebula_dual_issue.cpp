#include <iostream>
#include "Vnebula_core.h"   // O cabeçalho gerado pelo Verilator
#include "verilated.h"
#include "verilated_vcd_c.h" // Biblioteca nativa de ondas do Verilator

vluint64_t main_time = 0;
double sc_time_stamp() { return main_time; }

int main(int argc, char** argv) {
    // Inicializa o Verilator e ativa o rastreamento (Trace)
    Verilated::commandArgs(argc, argv);
    Verilated::traceEverOn(true);

    // Instancia o nosso processador e o gerador de ondas
    Vnebula_core* top = new Vnebula_core;
    VerilatedVcdC* tfp = new VerilatedVcdC;

    // Configura o VCD para gravar até 99 níveis de profundidade
    top->trace(tfp, 99);
    tfp->open("dump.vcd");

    // O nosso programa de teste (16 instruções = 64 bytes = 512 bits)
    // Teste Exaustivo da ALU em Dual Issue
    uint32_t INSTRUCTIONS[16] = {
        0x00f00093, 0x00700113, // ADDI x1=15, ADDI x2=7
        0x002081b3, 0x40208233, // ADD x3, SUB x4
        0x0020c2b3, 0x0020e333, // XOR x5, OR x6
        0x0020f3b3, 0x00209433, // AND x7, SLL x8
        0x0020d4b3, 0x4020d533, // SRL x9, SRA x10
        0x0020a5b3, 0x0020b633, // SLT x11, SLTU x12
        0x00000013, 0x00000013, 0x00000013, 0x00000013 // NOPs
    };

    // Estado inicial dos pinos
    top->clk = 0;
    top->rst_n = 0;
    top->imem_ack = 0;
    
    // Zera o barramento de memória (512 bits = 16 words de 32 bits)
    for(int i=0; i<16; i++) top->imem_data[i] = 0;

    int imem_wait = 0;

    // Roda a simulação por 150 meios-ciclos (75 ciclos de clock completos)
    for (int i = 0; i < 150; i++) {
        top->clk = !top->clk; // Inverte o clock

        // Liberta o Reset após 10 unidades de tempo
        if (main_time > 10) top->rst_n = 1;

        top->eval(); // Avalia o circuito combinacional

        // Lógica da Memória (Simula o atraso do barramento na subida do clock)
        if (top->clk == 1 && top->rst_n == 1) {
            if (top->imem_req) {
                if (imem_wait == 0) {
                    imem_wait = 2; // Simula 2 ciclos de latência
                } else {
                    imem_wait--;
                    if (imem_wait == 0) {
                        top->imem_ack = 1;
                        for(int j=0; j<16; j++) {
                            top->imem_data[j] = INSTRUCTIONS[j];
                        }
                    }
                }
            } else {
                top->imem_ack = 0;
            }
        }

        // Grava o estado atual no ficheiro de ondas
        tfp->dump(main_time);
        main_time++;
    }

    // Fecha os ficheiros e limpa a memória
    top->final();
    tfp->close();
    delete top;
    delete tfp;
    
    std::cout << "Simulacao concluida! Ficheiro dump.vcd gerado com sucesso." << std::endl;
    return 0;
}