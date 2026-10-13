// cosim.h — interface de co-simulação lockstep (implementação: cosim_spike.cpp)
#pragma once
#include <cstdint>
#include <string>
#include <vector>

class Cosim {
public:
    virtual ~Cosim() = default;
    // retorna nullptr se a co-simulação não foi compilada
    static Cosim* create(const uint8_t* ram, uint64_t ram_size, const uint8_t* rom, uint64_t rom_size,
                         uint64_t start_pc);
    // instrução efetivada pelo RTL
    virtual bool step(uint64_t pc, uint32_t insn, bool rd_en, bool rd_fp, unsigned rd,
                      uint64_t rd_val, bool mmio, uint64_t cycle) = 0;
    // trap (exceção ou interrupção) tomado pelo RTL
    virtual bool trap(bool intr, unsigned cause, uint64_t epc, uint64_t tval, uint64_t cycle) = 0;
};
