#include <iostream>
#include "Vnebula_core_full.h"
#include "verilated.h"
#include "verilated_vcd_c.h"

vluint64_t main_time = 0;
double sc_time_stamp() { return main_time; }

uint32_t data_ram[128][16] = {0}; 

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    Verilated::traceEverOn(true);

    Vnebula_core_full* top = new Vnebula_core_full;
    VerilatedVcdC* tfp = new VerilatedVcdC;

    top->trace(tfp, 99);
    tfp->open("dump.vcd");

    // =========================================================================
    // BIOS: IDENTITY PAGING
    // =========================================================================
    data_ram[64][2] = 0x100000DF; // PTE: PPN = 0x40000. Flags ligadas.
    data_ram[64][3] = 0x00000000;

    /*
     * PROGRAMA DE TESTE DA MMU: SV39 Paging e PTW (Com Identity Map)
     * ----------------------------------------------------
     * 0: LUI x3, 0x20           -> mstatus: MPRV = 1
     * 1: ADDI x4, x0, 1         -> x4 = 1
     * 2: SLLI x4, x4, 11        -> x4 = 0x0800 (Modo Supervisor)
     * 3: OR x3, x3, x4          -> Fundir bits
     * 4: CSRRW x0, mstatus, x3  -> Ativar MPRV
     * 5: ADDI x5, x0, 1         -> Construir SATP (SV39)
     * 6: SLLI x5, x5, 63        -> Base = 0x8000...
     * 7: ADDI x5, x5, 1         -> Root Table = 0x1000
     * 8: CSRRW x0, satp, x5     -> LIGA A MMU!
     * 9: SFENCE.VMA x0, x0      -> Limpar TLBs
     * 10: LUI x6, 0x40000       -> x6 = 0x40000000 (Isto é a RAM de C++!)
     * 11: ADDI x6, x6, 0x18     -> x6 = 0x40000018 (Offset 24 bytes = Linha 0, Word 6)
     * 12: ADDI x7, x0, 99       -> x7 = 99
     * 13: SD x7, 0(x6)          -> MMU Traduz: 0x40000018 -> 0x40000018 (Físico)
     * 14: NOP                   -> Espera de ciclo
     * 15: LD x8, 0(x6)          -> MMU Traduz e LÊ O 99 DA CACHE L1
     * 16: JAL x0, 0             -> Loop Fim
     * 17: NOP
     */
    uint32_t INSTRUCTIONS[18] = {
        0x000201b7, 0x00100213, // 0/1: LUI x3 | ADDI x4, 1
        0x00b21213, 0x0041e1b3, // 2/3: SLLI x4| OR x3
        0x30009073, 0x00100293, // 4/5: CSRRW  | ADDI x5, 1
        0x03f29293, 0x00128293, // 6/7: SLLI x5| ADDI x5, 1
        0x18029073, 0x12000073, // 8/9: CSRRW  | SFENCE.VMA
        0x40000337, 0x01830313, // 10/11:LUI x6| ADDI x6, 0x18
        0x06300393, 0x00733023, // 12/13:ADDI 99| SD x7
        0x00000013, 0x00033403, // 14/15:NOP   | LD x8
        0x0000006f, 0x00000013  // 16/17:JAL   | NOP
    };

    top->clk = 0;
    top->rst_n = 0;
    top->imem_ack = 0;
    top->dmem_ack = 0;
    top->ptw_mem_ack = 0;

    int imem_wait = 0, dmem_wait = 0, ptw_wait = 0;
    bool ireq_latched = false, dreq_latched = false, ptw_latched = false;
    uint64_t latched_daddr = 0, latched_ptw_addr = 0;
    bool latched_dwe = false;

    for (int i = 0; i < 150000; i++) {
        top->clk = !top->clk;
        if (main_time > 10) top->rst_n = 1;

        top->eval();

        if (top->clk == 1 && top->rst_n == 1) {
            // ================== I-CACHE ==================
            if (top->imem_req) ireq_latched = true;
            if (ireq_latched && !top->imem_ack) {
                if (imem_wait == 0) imem_wait = 2;
                else {
                    imem_wait--;
                    if (imem_wait == 0) {
                        top->imem_ack = 1;
                        ireq_latched = false;
                        
                        uint32_t local_addr = top->imem_addr & 0xFFF; 
                        uint32_t base_word_idx = local_addr / 4;
                        
                        for(int j=0; j<16; j++) {
                            uint32_t instr_idx = base_word_idx + j;
                            if (instr_idx < 18) {
                                top->imem_data[j] = INSTRUCTIONS[instr_idx];
                            } else {
                                top->imem_data[j] = 0x00000013; // NOP
                            }
                        }
                    }
                }
            } else if (top->imem_ack) top->imem_ack = 0;

            // ================== D-CACHE ==================
            if (top->dmem_req) {
                dreq_latched = true;
                latched_daddr = top->dmem_addr;
                latched_dwe = top->dmem_we;
            }
            if (dreq_latched && !top->dmem_ack) {
                if (dmem_wait == 0) dmem_wait = 3;
                else {
                    dmem_wait--;
                    if (dmem_wait == 0) {
                        top->dmem_ack = 1;
                        dreq_latched = false;
                        uint32_t line_idx = (latched_daddr / 64) % 128;
                        if (latched_dwe) {
                            for(int j=0; j<16; j++) data_ram[line_idx][j] = top->dmem_wdata[j];
                        } else {
                            for(int j=0; j<16; j++) top->dmem_rdata[j] = data_ram[line_idx][j];
                        }
                    }
                }
            } else if (top->dmem_ack) top->dmem_ack = 0;
            
            // ================== PTW / MMU ==================
            if (top->ptw_mem_req) {
                ptw_latched = true;
                latched_ptw_addr = top->ptw_mem_addr;
            }
            if (ptw_latched && !top->ptw_mem_ack) {
                if (ptw_wait == 0) ptw_wait = 2; 
                else {
                    ptw_wait--;
                    if (ptw_wait == 0) {
                        top->ptw_mem_ack = 1;
                        ptw_latched = false;
                        uint32_t line_idx = (latched_ptw_addr / 64) % 128;
                        uint32_t word_idx = (latched_ptw_addr % 64) / 4;
                        if (top->ptw_mem_we) {
                            data_ram[line_idx][word_idx] = top->ptw_mem_wdata & 0xFFFFFFFF;
                            data_ram[line_idx][word_idx+1] = top->ptw_mem_wdata >> 32;
                        } else {
                            uint64_t lower_word = data_ram[line_idx][word_idx];
                            uint64_t upper_word = data_ram[line_idx][word_idx+1];
                            top->ptw_mem_data = (upper_word << 32) | lower_word;
                        }
                    }
                }
            } else if (top->ptw_mem_ack) top->ptw_mem_ack = 0;
        }

        tfp->dump(main_time);
        main_time++;
    }

    top->final();
    tfp->close();
    delete top;
    delete tfp;
    return 0;
}