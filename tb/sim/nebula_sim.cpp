// =============================================================================
// nebula_sim.cpp — Testbench Verilator do Nebula Core
//
// Modelo de SoC:
//   0x0000_1000  Boot ROM (4 KB)           — boot Linux (a0=hartid, a1=DTB)
//   0x0010_0000  Finisher (sifive,test0)   — 0x5555 = pass, 0x3333|(c<<16) = fail
//   0x0200_0000  CLINT (msip, mtimecmp, mtime)
//   0x0C00_0000  PLIC (2 contextos: M e S)
//   0x1000_0000  UART 16550A (reg-shift 0)
//   0x8000_0000  RAM (--ram-mb, padrão 256 MB)
//
// Portas do núcleo modeladas com latência fixa (imem/dmem/ptw).
// Opções principais:
//   nebula_sim [opções] programa.elf
//     --max-cycles=N        limite de ciclos
//     --load=arquivo@addr   carrega binário cru em addr (pode repetir)
//     --dtb=arquivo@addr    idem, e passa addr em a1 pela boot ROM
//     --boot-rom            inicia na boot ROM (0x1000) saltando p/ 0x80000000
//     --trace=arquivo       log de commits
//     --mtime-div=N         ciclos por tick de mtime (padrão 100)
//     --rd-lat=N --wr-lat=N latência de leitura/escrita da memória
//     --cosim               co-simulação lockstep com Spike (se compilado)
//     --quiet
// =============================================================================
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <string>
#include <vector>
#include <map>
#include <deque>
#include <fstream>
#include <memory>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include "Vnebula_cluster.h"
#include "verilated.h"
#if VM_TRACE_FST
#include "verilated_fst_c.h"
#endif

#include "cosim.h"

// extração de bits de sinais do Verilator (escalares ou VlWide)
template <typename T> static inline uint64_t bits(const T& v, int lsb, int w) {
    uint64_t x = (uint64_t)(v >> lsb);
    return w >= 64 ? x : (x & ((1ULL << w) - 1));
}
template <std::size_t N> static inline uint64_t bits(const VlWide<N>& v, int lsb, int w) {
    uint64_t r = 0;
    for (int i = 0; i < w; i++) {
        int b = lsb + i;
        if ((v[b / 32] >> (b % 32)) & 1) r |= 1ULL << i;
    }
    return r;
}

static uint64_t g_cycle = 0;
double sc_time_stamp() { return (double)g_cycle; }

#include "soc_model.h"

// -----------------------------------------------------------------------------
// Portas de memória com latência
// -----------------------------------------------------------------------------
struct Port { bool busy = false; int count = 0; uint64_t addr = 0; bool we = false; bool unc = false;
              uint64_t wstrb = 0; uint32_t wdata[16]; };

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);

    uint64_t max_cycles = 2000000000ULL;
    uint64_t ram_mb = 256;
    std::vector<std::string> loads;
    std::string dtb_spec, trace_file, elf;
    bool boot_rom = false, quiet = false, cosim = false;
    int mtime_div = 100, rd_lat = 10, wr_lat = 2;
    uint64_t heartbeat = 0;
    uint64_t stall_limit = 50000000;
    uint64_t trace_from = 0;
    bool mmio_only = false;
    std::string dump_ram;
    std::string fst_file;
    uint64_t fst_from = 0, fst_cycles = 1000;
    std::vector<std::string> expects, sends;
    uint64_t last_pc = 0;
    uint64_t hart_pc[NCORES] = {0};
    uint64_t hart_last[NCORES] = {0};
    int      hart_priv[NCORES] = {0};
    uint64_t stats_from = 0;
    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        auto val = [&](const char* k) -> const char* { size_t n = strlen(k); return a.compare(0, n, k) == 0 ? a.c_str() + n : nullptr; };
        if (const char* v = val("--max-cycles=")) max_cycles = strtoull(v, nullptr, 0);
        else if (const char* v = val("--ram-mb=")) ram_mb = strtoull(v, nullptr, 0);
        else if (const char* v = val("--load=")) loads.push_back(v);
        else if (const char* v = val("--dtb=")) dtb_spec = v;
        else if (const char* v = val("--trace=")) trace_file = v;
        else if (const char* v = val("--mtime-div=")) mtime_div = atoi(v);
        else if (const char* v = val("--rd-lat=")) rd_lat = atoi(v);
        else if (const char* v = val("--heartbeat=")) heartbeat = strtoull(v, nullptr, 0);
        else if (const char* v = val("--stall-limit=")) stall_limit = strtoull(v, nullptr, 0);
        else if (const char* v = val("--trace-from=")) trace_from = strtoull(v, nullptr, 0);
        else if (a == "--trace-mmio-only") mmio_only = true;
        else if (const char* v = val("--dump-ram=")) dump_ram = v;
        else if (const char* v = val("--fst=")) fst_file = v;
        else if (const char* v = val("--fst-from=")) fst_from = strtoull(v, nullptr, 0);
        else if (const char* v = val("--fst-cycles=")) fst_cycles = strtoull(v, nullptr, 0);
        else if (const char* v = val("--expect=")) expects.push_back(v);
        else if (const char* v = val("--send=")) {
            std::string t;
            for (const char* q = v; *q; q++) {
                if (q[0] == '\\' && q[1] == 'n') { t.push_back('\n'); q++; } else t.push_back(*q);
            }
            sends.push_back(t);
        }
        else if (const char* v = val("--wr-lat=")) wr_lat = atoi(v);
        else if (a == "--boot-rom") boot_rom = true;
        else if (a == "--quiet") quiet = true;
        else if (a == "--cosim") cosim = true;
        else if (a[0] == '+') continue;
        else if (a[0] != '-') elf = a;
        else { fprintf(stderr, "opção desconhecida: %s\n", a.c_str()); return 2; }
    }

    Soc soc(ram_mb);
    soc.quiet = quiet;
    for (size_t i = 0; i < expects.size() && i < sends.size(); i++) soc.script.push_back({expects[i], sends[i]});
    ElfInfo info;
    if (!elf.empty() && !load_elf(elf.c_str(), soc, info)) return 2;
    for (auto& l : loads) if (!load_bin(l, soc)) return 2;
    uint64_t dtb_addr = 0;
    if (!dtb_spec.empty() && !load_bin(dtb_spec, soc, &dtb_addr)) return 2;
    if (info.syms.count("tohost")) soc.tohost_addr = info.syms["tohost"];

    // boot ROM: a0 = mhartid, a1 = DTB, salta para 0x80000000
    {
        uint32_t code[16];
        int n = 0;
        uint32_t dtb_hi = (uint32_t)((dtb_addr - 0x80000000ULL) >> 12);
        code[n++] = 0xf1402573;                         // csrr a0, mhartid
        code[n++] = 0x00100293;                         // li   t0, 1
        code[n++] = 0x01f29293;                         // slli t0, t0, 31
        code[n++] = (dtb_hi << 12) | (11 << 7) | 0x37;  // lui  a1, hi(dtb-0x80000000)
        code[n++] = 0x005585b3;                         // add  a1, a1, t0
        code[n++] = 0x00028067;                         // jr   t0
        memcpy(soc.rom, code, n * 4);
    }

    uint64_t start_pc = boot_rom ? Soc::ROM_BASE : (info.entry ? info.entry : 0x80000000ULL);
    Vnebula_cluster* top = new Vnebula_cluster;
    std::unique_ptr<Cosim> cs;
    if (cosim) {
        if (NCORES != 1) { fprintf(stderr, "cosim só com 1 hart\n"); return 2; }
        cs.reset(Cosim::create(soc.ram.data(), soc.ram_size, soc.rom, Soc::ROM_SIZE, start_pc));
        if (!cs) { fprintf(stderr, "cosim indisponível (compile com make COSIM=1)\n"); return 2; }
    }

    FILE* tf = trace_file.empty() ? nullptr : fopen(trace_file.c_str(), "w");
#if VM_TRACE_FST
    VerilatedFstC* fst = nullptr;
    if (!fst_file.empty()) Verilated::traceEverOn(true);
#endif

    Port mp;
    uint64_t instret = 0;
    int ret = 0;

    top->clk = 0;
    top->rst_n = 0;
    top->mem_ack = 0; top->mem_error = 0;
    top->timer_irq = 0; top->external_irq = 0; top->software_irq = 0; top->s_external_irq = 0;
    top->mtime = 0;
    top->reset_vector = start_pc;
    for (int i = 0; i < 5; i++) { top->clk = 0; top->eval(); top->clk = 1; top->eval(); }
    top->clk = 0; top->eval();
    top->rst_n = 1;

    uint64_t last_commit_cycle = 0;
    bool tohost_done = false;

    for (g_cycle = 0; g_cycle < max_cycles; g_cycle++) {
        // ---------------- relógio de tempo real / interrupções ----------------
        if (mtime_div > 0 && g_cycle % mtime_div == 0 && g_cycle) soc.mtime++;
        {
            uint64_t ti = 0, si = 0, ei = 0, sei = 0;
            for (int h = 0; h < NCORES; h++) {
                ti  |= (uint64_t)(soc.mtime >= soc.mtimecmp[h]) << h;
                si  |= (uint64_t)(soc.msip[h] & 1) << h;
                ei  |= (uint64_t)soc.plic_irq(2 * h) << h;
                sei |= (uint64_t)soc.plic_irq(2 * h + 1) << h;
            }
            top->mtime = soc.mtime;
            top->timer_irq = ti; top->software_irq = si;
            top->external_irq = ei; top->s_external_irq = sei;
        }

        // ---------------- resposta da memória deste ciclo ----------------
        top->mem_ack = 0; top->mem_error = 0;
        if (mp.busy && mp.count == 0) {
            bool ok = true;
            if (mp.we) {
                for (int dw = 0; dw < 8; dw++) {
                    uint8_t be = (mp.wstrb >> (8 * dw)) & 0xFF;
                    if (!be) continue;
                    uint64_t aw = (mp.addr & ~63ULL) + 8 * dw;
                    uint64_t data = (uint64_t)mp.wdata[2 * dw] | ((uint64_t)mp.wdata[2 * dw + 1] << 32);
                    if (soc.tohost_addr && aw == (soc.tohost_addr & ~7ULL) && (be & 0x0F)) {
                        uint64_t th = data;
                        if (th & 1) { tohost_done = true; ret = (int)(th >> 1); }
                    }
                    if (mp.unc && tf && g_cycle >= trace_from)
                        fprintf(tf, "%lu MMIO W 0x%lx be=%02x data=0x%lx\n", (unsigned long)g_cycle, (unsigned long)aw, be, (unsigned long)data);
                    if (mp.unc) ok &= soc.uncached_write(aw, data, be);
                    else {
                        if (uint8_t* p = soc.ptr(aw)) {
                            for (int i = 0; i < 8; i++) if (be >> i & 1) p[i] = (uint8_t)(data >> (8 * i));
                        } else ok = false;
                    }
                }
                for (int w = 0; w < 16; w++) top->mem_rdata[w] = 0;
            } else if (mp.unc) {
                uint64_t aw = mp.addr & ~7ULL;
                uint8_t be = (mp.wstrb >> (8 * ((mp.addr >> 3) & 7))) & 0xFF;
                uint64_t v = 0;
                ok = soc.uncached_read(aw, be, v);
                if (tf && g_cycle >= trace_from)
                    fprintf(tf, "%lu MMIO R 0x%lx be=%02x data=0x%lx\n", (unsigned long)g_cycle, (unsigned long)aw, be, (unsigned long)v);
                for (int w = 0; w < 16; w++) top->mem_rdata[w] = 0;
                int lane = (mp.addr >> 3) & 7;
                top->mem_rdata[2 * lane] = (uint32_t)v;
                top->mem_rdata[2 * lane + 1] = (uint32_t)(v >> 32);
            } else {
                for (int w = 0; w < 16; w++) {
                    uint64_t a = (mp.addr & ~63ULL) + 4 * w;
                    uint32_t v = 0;
                    if (uint8_t* p = soc.ptr(a)) memcpy(&v, p, 4); else ok = false;
                    top->mem_rdata[w] = v;
                }
            }
            top->mem_ack = 1; top->mem_error = !ok;
        }

        top->clk = 0;
        top->eval();

        // ---------------- nova requisição ----------------
        if (!mp.busy && top->mem_req) {
            mp.busy = true; mp.addr = top->mem_addr; mp.we = top->mem_we; mp.unc = top->mem_uncached;
            mp.wstrb = top->mem_wstrb;
            for (int w = 0; w < 16; w++) mp.wdata[w] = top->mem_wdata[w];
            mp.count = mp.we ? wr_lat : rd_lat;
        }

        // ---------------- commits ----------------
        for (int h = 0; h < NCORES; h++) {
            for (int s = 0; s < 2; s++) {
                int k = 2 * h + s;
                if (!bits(top->trace_valid, k, 1)) continue;
                instret++;
                last_commit_cycle = g_cycle;
                uint64_t pc = bits(top->trace_pc, 64 * k, 64);
                last_pc = pc;
                hart_pc[h] = pc;
                hart_last[h] = g_cycle;
                uint32_t insn = (uint32_t)bits(top->trace_instr, 32 * k, 32);
                bool rd_en = bits(top->trace_rd_en, k, 1);
                bool rd_fp = bits(top->trace_rd_fp, k, 1);
                unsigned rd = (unsigned)bits(top->trace_rd, 5 * k, 5);
                uint64_t rv = bits(top->trace_rd_val, 64 * k, 64);
                bool mmio = bits(top->trace_mmio, k, 1);
                int pv = (int)bits(top->trace_priv, 2 * h, 2);
                hart_priv[h] = pv;
                if (tf && g_cycle >= trace_from && !mmio_only) {
                    if (NCORES > 1) fprintf(tf, "h%d ", h);
                    fprintf(tf, "%lu %d 0x%016lx (0x%08x)", (unsigned long)g_cycle, pv, (unsigned long)pc, insn);
                    if (rd_en) fprintf(tf, " %c%-2u 0x%016lx", rd_fp ? 'f' : 'x', rd, (unsigned long)rv);
                    fprintf(tf, "\n");
                }
                if (cs && !cs->step(pc, insn, rd_en, rd_fp, rd, rv, mmio, g_cycle)) {
                    fprintf(stderr, "[cosim] divergência no ciclo %lu, instret %lu\n", (unsigned long)g_cycle, (unsigned long)instret);
                    ret = 3; goto done;
                }
            }
            if (bits(top->trace_trap, h, 1)) {
                bool intr = bits(top->trace_trap_intr, h, 1);
                unsigned cause = (unsigned)bits(top->trace_trap_cause, 5 * h, 5);
                uint64_t epc = bits(top->trace_trap_epc, 64 * h, 64);
                uint64_t tval = bits(top->trace_trap_tval, 64 * h, 64);
                if (tf && g_cycle >= trace_from) {
                    if (NCORES > 1) fprintf(tf, "h%d ", h);
                    fprintf(tf, "%lu trap %s cause=%u epc=0x%016lx tval=0x%016lx\n", (unsigned long)g_cycle,
                            intr ? "intr" : "exc", cause, (unsigned long)epc, (unsigned long)tval);
                }
                uint64_t pins = (bits(top->timer_irq, h, 1) << 7) | (bits(top->software_irq, h, 1) << 3) |
                                (bits(top->external_irq, h, 1) << 11) | (bits(top->s_external_irq, h, 1) << 9);
                if (cs && !cs->trap(intr, cause, epc, tval, pins, g_cycle)) {
                    fprintf(stderr, "[cosim] divergência em trap no ciclo %lu\n", (unsigned long)g_cycle);
                    ret = 3; goto done;
                }
            }
        }

        top->clk = 1;
        top->eval();
#if VM_TRACE_FST
        if (!fst_file.empty()) {
            if (!fst && g_cycle >= fst_from) {
                fst = new VerilatedFstC; top->trace(fst, 99); fst->open(fst_file.c_str());
            }
            if (fst) {
                fst->dump((uint64_t)g_cycle * 2);
                if (g_cycle >= fst_from + fst_cycles) { fst->close(); delete fst; fst = nullptr; fst_file.clear(); }
            }
        }
#endif

        // ---------------- avança porta ----------------
        if (mp.busy) { if (mp.count == 0) mp.busy = false; else mp.count--; }

        if (heartbeat && g_cycle % heartbeat == 0) {
            fprintf(stderr, "[hb] ciclo=%lu instret=%lu", (unsigned long)g_cycle, (unsigned long)instret);
            for (int h = 0; h < NCORES; h++)
                fprintf(stderr, " h%d:%d:0x%016lx(+%lu)", h, hart_priv[h], (unsigned long)hart_pc[h],
                        (unsigned long)(g_cycle - hart_last[h]));
            fprintf(stderr, "\n");
        }
        if (tohost_done || soc.finished) break;
        if (g_cycle - last_commit_cycle > stall_limit) {
            fprintf(stderr, "\n[nebula_sim] TIMEOUT: nenhum commit há %lu ciclos (ciclo %lu)\n",
                    (unsigned long)stall_limit, (unsigned long)g_cycle);
            ret = 4; goto done;
        }
    }
    if (!tohost_done && !soc.finished && g_cycle >= max_cycles) {
        fprintf(stderr, "\n[nebula_sim] limite de ciclos atingido\n");
        ret = 5;
    }
    if (soc.finished) ret = soc.exit_code;

done:
    fprintf(stderr, "[nebula_sim] ciclos=%lu instret=%lu IPC=%.3f resultado=%s (%d)\n",
            (unsigned long)g_cycle, (unsigned long)instret,
            g_cycle ? (double)instret / (double)g_cycle : 0.0, ret == 0 ? "PASS" : "FAIL", ret);
    if (tf) fclose(tf);
    if (!dump_ram.empty()) {
        FILE* df = fopen(dump_ram.c_str(), "wb");
        if (df) { fwrite(soc.ram.data(), 1, soc.ram.size(), df); fclose(df); }
    }
    top->final();
    delete top;
    (void)stats_from;
    return ret;
}
