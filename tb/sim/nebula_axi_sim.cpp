// =============================================================================
// nebula_axi_sim.cpp — testbench do nebula_axi_top (cluster + ponte AXI4)
//
// Escravo AXI4 em C++ com atrasos aleatórios em todos os canais (AR/R/AW/W/B),
// ligado ao mesmo modelo de SoC do nebula_sim (RAM, CLINT, PLIC, UART,
// finisher). Valida a ponte AXI rodando riscv-tests, Embench e Linux.
//
//   nebula_axi_sim [--seed=N] [--stall=P] [opções do nebula_sim] programa.elf
// =============================================================================
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <string>
#include <vector>
#include <deque>
#include <random>

#include "Vnebula_axi_top.h"
#include "verilated.h"
#include "soc_model.h"

static uint64_t g_cycle = 0;
double sc_time_stamp() { return (double)g_cycle; }

struct Burst { uint64_t addr; int len; int size; int beat; };

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    uint64_t max_cycles = 2000000000ULL, ram_mb = 256;
    std::vector<std::string> loads, expects, sends;
    std::string dtb_spec, elf;
    bool boot_rom = false, quiet = false;
    int mtime_div = 100, stall_pct = 30;
    unsigned seed = 1;
    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        auto val = [&](const char* k) -> const char* { size_t n = strlen(k); return a.compare(0, n, k) == 0 ? a.c_str() + n : nullptr; };
        if (const char* v = val("--max-cycles=")) max_cycles = strtoull(v, nullptr, 0);
        else if (const char* v = val("--ram-mb=")) ram_mb = strtoull(v, nullptr, 0);
        else if (const char* v = val("--load=")) loads.push_back(v);
        else if (const char* v = val("--dtb=")) dtb_spec = v;
        else if (const char* v = val("--mtime-div=")) mtime_div = atoi(v);
        else if (const char* v = val("--seed=")) seed = (unsigned)strtoul(v, nullptr, 0);
        else if (const char* v = val("--stall=")) stall_pct = atoi(v);
        else if (const char* v = val("--expect=")) expects.push_back(v);
        else if (const char* v = val("--send=")) {
            std::string t;
            for (const char* q = v; *q; q++) { if (q[0] == '\\' && q[1] == 'n') { t.push_back('\n'); q++; } else t.push_back(*q); }
            sends.push_back(t);
        }
        else if (a == "--boot-rom") boot_rom = true;
        else if (a == "--quiet") quiet = true;
        else if (a[0] == '+' || a[0] == '-') continue;
        else elf = a;
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
    {
        uint32_t code[8]; int n = 0;
        uint32_t dtb_hi = (uint32_t)((dtb_addr - 0x80000000ULL) >> 12);
        code[n++] = 0xf1402573; code[n++] = 0x00100293; code[n++] = 0x01f29293;
        code[n++] = (dtb_hi << 12) | (11 << 7) | 0x37; code[n++] = 0x005585b3; code[n++] = 0x00028067;
        memcpy(soc.rom, code, n * 4);
    }
    uint64_t start_pc = boot_rom ? Soc::ROM_BASE : (info.entry ? info.entry : 0x80000000ULL);

    std::mt19937 rng(seed);
    auto stall = [&]() { return (int)(rng() % 100) < stall_pct; };

    Vnebula_axi_top* top = new Vnebula_axi_top;
    top->clk = 0; top->rst_n = 0; top->i_reset_vector = start_pc;
    top->i_m_axi_awready = 0; top->i_m_axi_wready = 0; top->i_m_axi_bvalid = 0;
    top->i_m_axi_arready = 0; top->i_m_axi_rvalid = 0;
    for (int i = 0; i < 5; i++) { top->clk = 0; top->eval(); top->clk = 1; top->eval(); }
    top->clk = 0; top->eval();
    top->rst_n = 1;

    std::deque<Burst> rq;          // leituras aceitas
    std::deque<Burst> wq;          // escritas aceitas (AW)
    int bpend = 0;                 // respostas B pendentes
    bool berr_pend[64] = {false};
    std::deque<bool> berr;
    bool tohost_done = false;
    int ret = 0;
    uint64_t protocol_errors = 0;

    // estado dos sinais de saída do escravo (válidos no ciclo corrente)
    bool r_valid = false, b_valid = false;
    uint64_t r_data = 0; bool r_last = false; int r_resp = 0; int b_resp = 0;

    for (g_cycle = 0; g_cycle < max_cycles; g_cycle++) {
        if (mtime_div > 0 && g_cycle % mtime_div == 0 && g_cycle) soc.mtime++;
        uint64_t ti = 0, si = 0, ei = 0, sei = 0;
        for (int h = 0; h < NCORES; h++) {
            ti  |= (uint64_t)(soc.mtime >= soc.mtimecmp[h]) << h;
            si  |= (uint64_t)(soc.msip[h] & 1) << h;
            ei  |= (uint64_t)soc.plic_irq(2 * h) << h;
            sei |= (uint64_t)soc.plic_irq(2 * h + 1) << h;
        }
        top->i_timer_irq = ti; top->i_software_irq = si;
        top->i_external_irq = ei; top->i_s_external_irq = sei;

        // ---- prepara R ----
        if (!r_valid && !rq.empty() && !stall()) {
            Burst& b = rq.front();
            uint64_t a = b.addr + (uint64_t)b.beat * (1ULL << b.size);
            uint64_t aw = a & ~7ULL;
            bool ok = true;
            uint64_t v = 0;
            if (b.size < 3 || !soc.in_ram(aw, 8)) {
                // estreito ou MMIO: lê só os bytes endereçados
                uint8_t be = (uint8_t)(((1u << (1u << b.size)) - 1) << (a & 7));
                ok = soc.uncached_read(aw, be, v);
            } else {
                memcpy(&v, soc.ptr(aw), 8);
            }
            r_valid = true; r_data = v; r_last = (b.beat == b.len); r_resp = ok ? 0 : 2;
        }
        // ---- prepara B ----
        if (!b_valid && bpend > 0 && !stall()) { b_valid = true; b_resp = berr.front() ? 2 : 0; }

        top->i_m_axi_rvalid = r_valid; top->i_m_axi_rdata = r_data; top->i_m_axi_rlast = r_last;
        top->i_m_axi_rresp = r_resp; top->i_m_axi_rid = 0;
        top->i_m_axi_bvalid = b_valid; top->i_m_axi_bresp = b_resp; top->i_m_axi_bid = 0;
        bool arready = !stall(), awready = !stall(), wready = !stall();
        top->i_m_axi_arready = arready; top->i_m_axi_awready = awready; top->i_m_axi_wready = wready;

        top->clk = 0; top->eval();

        // ---- handshakes deste ciclo ----
        if (top->o_m_axi_arvalid && arready) {
            if (top->o_m_axi_arburst != 1 && top->o_m_axi_arlen) protocol_errors++;
            rq.push_back({top->o_m_axi_araddr, top->o_m_axi_arlen, top->o_m_axi_arsize, 0});
        }
        if (r_valid && top->o_m_axi_rready) {
            Burst& b = rq.front();
            r_valid = false;
            if (b.beat == b.len) rq.pop_front(); else b.beat++;
        }
        if (top->o_m_axi_awvalid && awready)
            wq.push_back({top->o_m_axi_awaddr, top->o_m_axi_awlen, top->o_m_axi_awsize, 0});
        if (top->o_m_axi_wvalid && wready) {
            if (wq.empty()) { protocol_errors++; }
            else {
                Burst& b = wq.front();
                uint64_t a = (b.addr & ~((1ULL << b.size) - 1)) + (uint64_t)b.beat * (1ULL << b.size);
                uint64_t aw = a & ~7ULL;
                uint8_t be = top->o_m_axi_wstrb;
                uint64_t data = top->o_m_axi_wdata;
                bool ok = true;
                if (soc.tohost_addr && aw == (soc.tohost_addr & ~7ULL) && (be & 0x0F) && (data & 1)) {
                    tohost_done = true; ret = (int)(data >> 1);
                }
                if (be) ok = soc.uncached_write(aw, data, be);
                bool last = top->o_m_axi_wlast;
                if (last != (b.beat == b.len)) protocol_errors++;
                if (b.beat == 0) berr.push_back(!ok); else if (!ok) berr.back() = true;
                if (last) { wq.pop_front(); bpend++; } else b.beat++;
            }
        }
        if (b_valid && top->o_m_axi_bready) { b_valid = false; bpend--; berr.pop_front(); }

        top->clk = 1; top->eval();
        if (tohost_done || soc.finished) break;
    }
    (void)berr_pend;
    if (soc.finished) ret = soc.exit_code;
    else if (!tohost_done) { fprintf(stderr, "\n[nebula_axi_sim] limite de ciclos\n"); ret = 5; }
    if (protocol_errors) { fprintf(stderr, "[nebula_axi_sim] %lu erros de protocolo AXI\n", (unsigned long)protocol_errors); ret = 6; }
    fprintf(stderr, "[nebula_axi_sim] ciclos=%lu resultado=%s (%d)\n", (unsigned long)g_cycle, ret == 0 ? "PASS" : "FAIL", ret);
    top->final();
    delete top;
    return ret;
}
