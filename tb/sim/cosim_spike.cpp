// =============================================================================
// cosim_spike.cpp — co-simulação lockstep do Nebula Core com o Spike (libriscv)
//
// A cada instrução efetivada pelo RTL, o Spike executa um passo e comparam-se
// PC e o valor escrito no registrador destino. Traps do RTL (exceções e
// interrupções) devem ser reproduzidos pelo Spike com a mesma causa.
//
// Sincronizações (valores dependentes do SoC/tempo, copiados do RTL):
//   - loads de MMIO;
//   - leituras de CSRs voláteis/dependentes de implementação
//     (cycle/time/mip/sip/ids/PMP/triggers/HPM/envcfg).
// Interrupções externas (MTIP/MSIP/MEIP/SEIP) só são apresentadas ao Spike no
// instante em que o RTL as toma.
// =============================================================================
#include "cosim.h"
#include <riscv/processor.h>
#include <riscv/simif.h>
#include <riscv/cfg.h>
#include <riscv/mmu.h>
#include <riscv/encoding.h>
#include <riscv/csrs.h>
#include <cstdio>
#include <cstring>
#include <sstream>
#include <map>
#include <set>

class NebulaSimIf : public simif_t {
public:
    std::vector<uint8_t> ram, rom;
    uint64_t ram_base = 0x80000000ULL, rom_base = 0x1000;
    cfg_t cfg;
    std::map<size_t, processor_t*> harts;

    char* addr_to_mem(reg_t a) override {
        if (a >= ram_base && a < ram_base + ram.size()) return (char*)&ram[a - ram_base];
        if (a >= rom_base && a < rom_base + rom.size()) return (char*)&rom[a - rom_base];
        return nullptr;
    }
    static bool is_dev(reg_t a) {
        return (a >= 0x100000 && a < 0x101000) || (a >= 0x2000000 && a < 0x2010000) ||
               (a >= 0xC000000 && a < 0xC400000) || (a >= 0x10000000 && a < 0x10000100);
    }
    bool mmio_load(reg_t a, size_t len, uint8_t* b) override { memset(b, 0, len); return is_dev(a); }
    bool mmio_store(reg_t a, size_t, const uint8_t*) override { return is_dev(a); }
    void proc_reset(unsigned) override {}
    const cfg_t& get_cfg() const override { return cfg; }
    const std::map<size_t, processor_t*>& get_harts() const override { return harts; }
    const char* get_symbol(uint64_t) override { return nullptr; }
};

static const reg_t EXT_MIP = MIP_MTIP | MIP_MSIP | MIP_MEIP | MIP_SEIP;

class SpikeCosim : public Cosim {
public:
    NebulaSimIf sim;
    processor_t* proc = nullptr;
    std::ostringstream sout;
    FILE* devnull = nullptr;
    uint64_t steps = 0;
    std::map<unsigned, int> csr_sync_notes;

    SpikeCosim(const uint8_t* ram, uint64_t ram_size, const uint8_t* rom, uint64_t rom_size, uint64_t start_pc) {
        sim.ram.assign(ram, ram + ram_size);
        sim.rom.assign(rom, rom + rom_size);
        sim.cfg.isa = "rv64imafdc_zicsr_zifencei_zicntr_zihpm";
        sim.cfg.priv = "MSU";
        sim.cfg.pmpregions = 0;
        sim.cfg.trigger_count = 0;
        sim.cfg.hartids = {0};
        devnull = fopen("/dev/null", "w");
        proc = new processor_t(sim.cfg.isa, sim.cfg.priv, &sim.cfg, &sim, 0, false, devnull, sout);
        sim.harts[0] = proc;
        proc->set_max_vaddr_bits(39);
        proc->set_impl(IMPL_MMU_ASID, false);
        proc->reset();
        proc->enable_log_commits();
        proc->get_state()->pc = start_pc;
    }
    ~SpikeCosim() override { delete proc; if (devnull) fclose(devnull); }

    static bool volatile_csr(unsigned c) {
        return c == CSR_MCYCLE || c == CSR_CYCLE || c == CSR_TIME || c == CSR_MIP || c == CSR_SIP ||
               (c >= 0xF11 && c <= 0xF15) || (c >= 0x3A0 && c <= 0x3EF) || (c >= 0x7A0 && c <= 0x7AF) ||
               (c >= 0xB03 && c <= 0xB1F) || (c >= 0xC03 && c <= 0xC1F) || (c >= 0x323 && c <= 0x33F) ||
               c == CSR_MCOUNTINHIBIT || c == CSR_MENVCFG || c == CSR_SENVCFG || c == CSR_MISA ||
               c == CSR_MCONFIGPTR || c == CSR_MSECCFG;
    }

    void dump(const char* why, uint64_t pc, uint32_t insn, uint64_t cycle) {
        state_t* s = proc->get_state();
        fprintf(stderr, "\n[cosim] %s\n  ciclo=%lu passo=%lu  RTL pc=0x%016lx insn=0x%08x  | Spike pc=0x%016lx prv=%lu\n",
                why, (unsigned long)cycle, (unsigned long)steps, (unsigned long)pc, insn,
                (unsigned long)s->pc, (unsigned long)s->prv);
        fprintf(stderr, "  Spike: mstatus=%016lx mcause=%016lx mepc=%016lx mtval=%016lx scause=%016lx sepc=%016lx satp=%016lx\n",
                (unsigned long)s->mstatus->read(), (unsigned long)s->mcause->read(), (unsigned long)s->mepc->read(),
                (unsigned long)s->mtval->read(), (unsigned long)s->scause->read(), (unsigned long)s->sepc->read(),
                (unsigned long)proc->get_csr(CSR_SATP));
    }

    // executa um passo do Spike; retorna true se o Spike tomou trap
    bool spike_step() {
        state_t* s = proc->get_state();
        reg_t ir0 = s->minstret->read();
        s->log_reg_write.clear();
        proc->step(1);
        if (proc->is_waiting_for_interrupt()) proc->clear_waiting_for_interrupt();
        steps++;
        return s->minstret->read() == ir0;
    }

    bool step(uint64_t pc, uint32_t insn, bool rd_en, bool rd_fp, unsigned rd, uint64_t rd_val,
              bool mmio, uint64_t cycle) override {
        state_t* s = proc->get_state();
        if (s->pc != pc) { dump("PC divergente (commit)", pc, insn, cycle); return false; }
        s->mip->backdoor_write_with_mask(EXT_MIP, 0);
        bool is_csr = ((insn & 0x7F) == 0x73) && (((insn >> 12) & 7) != 0);
        unsigned csr = insn >> 20;
        bool trapped = spike_step();
        bool minstret_write = is_csr && (csr == CSR_MINSTRET || csr == CSR_INSTRET);
        if (trapped && !minstret_write) {
            dump("Spike tomou trap, RTL efetivou a instrução", pc, insn, cycle);
            return false;
        }
        // registradores escritos pelo Spike
        for (auto& kv : s->log_reg_write) {
            unsigned r = kv.first >> 4, type = kv.first & 0xF;
            if (type == 0 && r != 0) {
                if (!rd_en || rd_fp || rd != r) {
                    dump("Spike escreveu xreg que o RTL não escreveu", pc, insn, cycle);
                    fprintf(stderr, "  x%u = 0x%016lx (RTL rd_en=%d fp=%d rd=%u)\n", r,
                            (unsigned long)s->XPR[r], rd_en, rd_fp, rd);
                    return false;
                }
            } else if (type == 1) {
                if (!rd_en || !rd_fp || rd != r) {
                    dump("Spike escreveu freg que o RTL não escreveu", pc, insn, cycle);
                    return false;
                }
            }
        }
        if (rd_en) {
            if (rd_fp) {
                uint64_t sv = s->FPR[rd].v[0];
                if (sv != rd_val) {
                    dump("valor de f-reg divergente", pc, insn, cycle);
                    fprintf(stderr, "  f%u: RTL=0x%016lx Spike=0x%016lx\n", rd, (unsigned long)rd_val, (unsigned long)sv);
                    return false;
                }
            } else if (rd != 0) {
                uint64_t sv = s->XPR[rd];
                if (mmio || (is_csr && volatile_csr(csr))) {
                    if (is_csr && sv != rd_val && csr != CSR_MCYCLE && csr != CSR_CYCLE && csr != CSR_TIME &&
                        csr_sync_notes[csr]++ < 3)
                        fprintf(stderr, "[cosim] nota: CSR 0x%03x lido RTL=0x%lx Spike=0x%lx (sincronizado)\n",
                                csr, (unsigned long)rd_val, (unsigned long)sv);
                    s->XPR.write(rd, rd_val);
                } else if (sv != rd_val) {
                    dump("valor de x-reg divergente", pc, insn, cycle);
                    fprintf(stderr, "  x%u: RTL=0x%016lx Spike=0x%016lx\n", rd, (unsigned long)rd_val, (unsigned long)sv);
                    return false;
                }
            }
        }
        return true;
    }

    bool trap(bool intr, unsigned cause, uint64_t epc, uint64_t tval, uint64_t cycle) override {
        state_t* s = proc->get_state();
        if (s->pc != epc) { dump("PC divergente (trap)", epc, 0, cycle); return false; }
        s->mip->backdoor_write_with_mask(EXT_MIP, 0);
        if (intr) {
            reg_t bit = 1ULL << cause;
            if (bit & EXT_MIP) s->mip->backdoor_write_with_mask(bit, bit);
        }
        bool trapped = spike_step();
        s->mip->backdoor_write_with_mask(EXT_MIP, 0);
        if (!trapped) {
            dump(intr ? "RTL tomou interrupção, Spike não" : "RTL tomou exceção, Spike executou a instrução",
                 epc, 0, cycle);
            fprintf(stderr, "  RTL: intr=%d cause=%u tval=0x%016lx\n", intr, cause, (unsigned long)tval);
            return false;
        }
        reg_t exp = ((reg_t)intr << 63) | cause;
        bool to_m = s->prv == PRV_M;
        reg_t c = to_m ? s->mcause->read() : s->scause->read();
        reg_t tv = to_m ? s->mtval->read() : s->stval->read();
        if (c != exp || (!intr && tv != tval)) {
            dump("causa/tval do trap divergente", epc, 0, cycle);
            fprintf(stderr, "  RTL: cause=0x%lx tval=0x%016lx | Spike: cause=0x%lx tval=0x%016lx\n",
                    (unsigned long)exp, (unsigned long)tval, (unsigned long)c, (unsigned long)tv);
            return false;
        }
        return true;
    }
};

Cosim* Cosim::create(const uint8_t* ram, uint64_t ram_size, const uint8_t* rom, uint64_t rom_size,
                     uint64_t start_pc) {
    return new SpikeCosim(ram, ram_size, rom, rom_size, start_pc);
}
