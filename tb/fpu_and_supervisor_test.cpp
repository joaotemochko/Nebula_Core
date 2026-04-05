#include <iostream>
#include "Vnebula_core_full.h"
#include "verilated.h"
#include "verilated_vcd_c.h"

vluint64_t main_time = 0;
double sc_time_stamp() { return main_time; }

// Simulador de RAM Simples (8KB = 128 linhas de 64 bytes)
uint32_t data_ram[128][16] = {0}; 

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    Verilated::traceEverOn(true);

    Vnebula_core_full* top = new Vnebula_core_full;
    VerilatedVcdC* tfp = new VerilatedVcdC;

    top->trace(tfp, 99);
    tfp->open("dump.vcd");

    /*
     * PROGRAMA DE TESTE: TRAPS, CSRs e FPU (Fase 1 - Linux Ready)
     * ----------------------------------------------------
     * 0: LUI x1, 0x10000      | 1: ADDI x1, x1, 0x28  -> Define x1 = 0x10000028 (Endereço do Trap Handler)
     * 2: CSRRW x0, mtvec, x1  | 3: ADDI x2, x0, 42    -> Configura mtvec e prepara valor 42
     * 4: FCVT.D.W f1, x2      | 5: FADD.D f2, f1, f1  -> TESTE FPU: f1 = 42.0, f2 = 84.0
     * 6: ECALL                | 7: ADDI x6, x0, 100   -> ECALL causa Trap! MRET volta para Index 7.
     * 8: JAL x0, 0            | 9: NOP                -> BARREIRA: Fica em loop infinito aqui. Fim do teste.
     * --- INÍCIO DO TRAP HANDLER (Index 10 / PC = 0x10000028) ---
     * 10: CSRRS x3, mepc, x0  | 11: CSRRS x4, mcause, x0 -> Lê mepc (0x10000018) e mcause (11)
     * 12: ADDI x3, x3, 4      | 13: CSRRW x0, mepc, x3   -> Soma 4 ao mepc (pula a instr ECALL) e guarda
     * 14: FCVT.W.D x5, f2     | 15: MRET                 -> Recupera f2 para x5 (84). Retorna para Index 7!
     */
    uint32_t INSTRUCTIONS[16] = {
        0x100000b7, 0x02808093, // 0/1: LUI x1 | ADDI x1
        0x30509073, 0x02a00113, // 2/3: CSRRW mtvec | ADDI x2
        0xd20100d3, 0x02108153, // 4/5: FCVT.D.W | FADD.D
        0x00000073, 0x06400313, // 6/7: ECALL | ADDI x6 (O MRET volta para o Index 7)
        0x0000006f, 0x00000013, // 8/9: JAL x0, 0 (Barreira) | NOP
        0x341021f3, 0x34202273, // 10/11: CSRRS mepc | CSRRS mcause
        0x00418193, 0x34119073, // 12/13: ADDI x3 | CSRRW mepc
        0xc20102d3, 0x30200073  // 14/15: FCVT.W.D | MRET
    };

    top->clk = 0;
    top->rst_n = 0;
    top->imem_ack = 0;
    top->dmem_ack = 0;

    int imem_wait = 0;
    int dmem_wait = 0;
    
    // Latches para desacoplar a memória do FSM do Core
    bool ireq_latched = false;
    bool dreq_latched = false;
    uint64_t latched_daddr = 0;
    bool latched_dwe = false;

    // Roda por 200 meios-ciclos (100 ciclos de clock)
    for (int i = 0; i < 150000; i++) {
        top->clk = !top->clk;
        if (main_time > 10) top->rst_n = 1;

        top->eval();

        if (top->clk == 1 && top->rst_n == 1) {
            // ==========================================
            // LÓGICA DO I-CACHE (Instruções)
            // ==========================================
            if (top->imem_req) ireq_latched = true;

            if (ireq_latched && !top->imem_ack) {
                if (imem_wait == 0) imem_wait = 2; // 2 ciclos de latência
                else {
                    imem_wait--;
                    if (imem_wait == 0) {
                        top->imem_ack = 1;
                        ireq_latched = false;
                        for(int j=0; j<16; j++) top->imem_data[j] = INSTRUCTIONS[j];
                    }
                }
            } else if (top->imem_ack) {
                top->imem_ack = 0;
            }

            // ==========================================
            // LÓGICA DO D-CACHE (Dados - Load/Store)
            // ==========================================
            if (top->dmem_req) {
                dreq_latched = true;
                latched_daddr = top->dmem_addr;
                latched_dwe = top->dmem_we;
            }

            if (dreq_latched && !top->dmem_ack) {
                if (dmem_wait == 0) dmem_wait = 3; // 3 ciclos de latência para a RAM
                else {
                    dmem_wait--;
                    if (dmem_wait == 0) {
                        top->dmem_ack = 1;
                        dreq_latched = false;
                        
                        // Calcula qual linha de cache L1 foi solicitada (64 bytes = 16 words)
                        uint32_t line_idx = (latched_daddr / 64) % 128;
                        
                        if (latched_dwe) {
                            // WRITEBACK: L1 faz eviction e grava na RAM
                            for(int j=0; j<16; j++) {
                                data_ram[line_idx][j] = top->dmem_wdata[j];
                            }
                        } else {
                            // REFILL: L1 deu miss e pede os dados à RAM
                            for(int j=0; j<16; j++) {
                                top->dmem_rdata[j] = data_ram[line_idx][j];
                            }
                        }
                    }
                }
            } else if (top->dmem_ack) {
                top->dmem_ack = 0;
            }
        }

        tfp->dump(main_time);
        main_time++;
    }

    top->final();
    tfp->close();
    delete top;
    delete tfp;
    
    std::cout << "Teste de FPU e Supervisor concluido! Ficheiro dump.vcd gerado." << std::endl;
    return 0;
}