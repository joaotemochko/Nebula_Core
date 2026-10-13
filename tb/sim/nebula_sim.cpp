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

#include "Vnebula_core_full.h"
#include "verilated.h"
#ifdef VM_TRACE_FST
#include "verilated_fst_c.h"
#endif

#include "cosim.h"

static uint64_t g_cycle = 0;
double sc_time_stamp() { return (double)g_cycle; }

// -----------------------------------------------------------------------------
// Memória e dispositivos
// -----------------------------------------------------------------------------
struct Soc {
    static constexpr uint64_t ROM_BASE   = 0x1000;
    static constexpr uint64_t ROM_SIZE   = 0x1000;
    static constexpr uint64_t FIN_BASE   = 0x100000;
    static constexpr uint64_t CLINT_BASE = 0x2000000;
    static constexpr uint64_t CLINT_SIZE = 0x10000;
    static constexpr uint64_t PLIC_BASE  = 0xC000000;
    static constexpr uint64_t PLIC_SIZE  = 0x400000;
    static constexpr uint64_t UART_BASE  = 0x10000000;
    static constexpr uint64_t UART_SIZE  = 0x100;
    static constexpr uint64_t RAM_BASE   = 0x80000000ULL;

    uint64_t ram_size;
    std::vector<uint8_t> ram;
    uint8_t rom[ROM_SIZE];

    // CLINT
    uint64_t mtime = 0, mtimecmp = ~0ULL;
    uint32_t msip = 0;
    // UART
    uint8_t uart_ier = 0, uart_lcr = 0, uart_mcr = 0, uart_scr = 0, uart_dll = 0, uart_dlm = 0, uart_fcr = 0;
    bool uart_thre_ip = false;
    std::deque<uint8_t> uart_rx;
    std::string uart_line;
    // PLIC (fontes 1..31, contextos 0=M 1=S)
    uint32_t plic_prio[32] = {0};
    uint32_t plic_enable[2] = {0, 0};
    uint32_t plic_thresh[2] = {0, 0};
    uint32_t plic_claimed = 0;   // fontes em atendimento
    // finisher / tohost
    bool finished = false;
    int  exit_code = 0;
    uint64_t tohost_addr = 0;
    bool quiet = false;

    Soc(uint64_t mb) : ram_size(mb << 20), ram(ram_size, 0) { memset(rom, 0, sizeof(rom)); }

    bool in_ram(uint64_t a, uint64_t n = 1) const { return a >= RAM_BASE && a + n <= RAM_BASE + ram_size; }
    bool in_rom(uint64_t a, uint64_t n = 1) const { return a >= ROM_BASE && a + n <= ROM_BASE + ROM_SIZE; }

    uint8_t* ptr(uint64_t a) {
        if (in_ram(a)) return &ram[a - RAM_BASE];
        if (in_rom(a)) return &rom[a - ROM_BASE];
        return nullptr;
    }

    // linha de interrupção da UART (fonte 10 no PLIC)
    bool uart_irq() const {
        return ((uart_ier & 0x2) && uart_thre_ip) || ((uart_ier & 0x1) && !uart_rx.empty());
    }
    uint32_t plic_pending() const {
        uint32_t p = 0;
        if (uart_irq()) p |= (1u << 10);
        return p & ~plic_claimed;
    }
    bool plic_irq(int ctx) const {
        uint32_t p = plic_pending() & plic_enable[ctx];
        for (int i = 1; i < 32; i++)
            if ((p >> i & 1) && plic_prio[i] > plic_thresh[ctx]) return true;
        return false;
    }
    uint32_t plic_claim(int ctx) {
        uint32_t p = plic_pending() & plic_enable[ctx];
        uint32_t best = 0, bp = 0;
        for (int i = 1; i < 32; i++)
            if ((p >> i & 1) && plic_prio[i] > plic_thresh[ctx] && plic_prio[i] > bp) { best = i; bp = plic_prio[i]; }
        if (best) plic_claimed |= (1u << best);
        return best;
    }

    // roteiro expect/send pela UART: espera o texto, injeta a resposta no RX
    std::vector<std::pair<std::string, std::string>> script;
    size_t script_pos = 0;
    std::string out_tail;

    void uart_tx(uint8_t c) {
        if (!quiet) { fputc(c, stdout); fflush(stdout); }
        uart_thre_ip = true;
        out_tail.push_back((char)c);
        if (out_tail.size() > 256) out_tail.erase(0, out_tail.size() - 256);
        if (script_pos < script.size() &&
            out_tail.find(script[script_pos].first) != std::string::npos) {
            for (char ch : script[script_pos].second) uart_rx.push_back((uint8_t)ch);
            script_pos++;
            out_tail.clear();
        }
    }

    // leitura de dispositivo (byte)
    uint8_t dev_read8(uint64_t a, bool& ok) {
        ok = true;
        if (a >= UART_BASE && a < UART_BASE + UART_SIZE) {
            uint64_t r = a - UART_BASE;
            bool dlab = uart_lcr & 0x80;
            switch (r) {
                case 0: if (dlab) return uart_dll;
                        if (!uart_rx.empty()) { uint8_t c = uart_rx.front(); uart_rx.pop_front(); return c; }
                        return 0;
                case 1: return dlab ? uart_dlm : uart_ier;
                case 2: {
                    uint8_t iir = 0xC0;
                    if ((uart_ier & 1) && !uart_rx.empty()) iir |= 0x04;
                    else if ((uart_ier & 2) && uart_thre_ip) { iir |= 0x02; uart_thre_ip = false; }
                    else iir |= 0x01;
                    return iir;
                }
                case 3: return uart_lcr;
                case 4: return uart_mcr;
                case 5: return 0x60 | (uart_rx.empty() ? 0 : 1);
                case 6: return 0xB0;
                case 7: return uart_scr;
                default: return 0;
            }
        }
        if (a >= FIN_BASE && a < FIN_BASE + 0x1000) return 0;
        ok = false;
        return 0;
    }
    void dev_write8(uint64_t a, uint8_t v, bool& ok) {
        ok = true;
        if (a >= UART_BASE && a < UART_BASE + UART_SIZE) {
            uint64_t r = a - UART_BASE;
            bool dlab = uart_lcr & 0x80;
            switch (r) {
                case 0: if (dlab) uart_dll = v; else uart_tx(v); break;
                case 1: if (dlab) uart_dlm = v; else { uart_ier = v & 0x0F; if (v & 2) uart_thre_ip = true; } break;
                case 2: uart_fcr = v; break;
                case 3: uart_lcr = v; break;
                case 4: uart_mcr = v; break;
                case 7: uart_scr = v; break;
                default: break;
            }
            return;
        }
        ok = false;
    }

    // leitura/escrita de 32/64 bits alinhados em CLINT/PLIC/finisher
    bool reg_read(uint64_t a, unsigned size, uint64_t& v) {
        v = 0;
        if (a >= CLINT_BASE && a < CLINT_BASE + CLINT_SIZE) {
            uint64_t o = a - CLINT_BASE;
            uint64_t base = 0, val = 0;
            if (o < 0x4000)                 { base = o & ~3ULL; val = msip; size = 4; }
            else if (o >= 0x4000 && o < 0x4008) { base = 0x4000; val = mtimecmp; }
            else if (o >= 0xBFF8 && o < 0xC000) { base = 0xBFF8; val = mtime; }
            else return true;
            v = val >> (8 * (o - base));
            return true;
        }
        if (a >= PLIC_BASE && a < PLIC_BASE + PLIC_SIZE) {
            uint64_t o = (a - PLIC_BASE) & ~3ULL;
            uint32_t r = 0;
            if (o < 0x80)                      r = plic_prio[o / 4];
            else if (o == 0x1000)              r = plic_pending();
            else if (o == 0x2000)              r = plic_enable[0];
            else if (o == 0x2080)              r = plic_enable[1];
            else if (o == 0x200000)            r = plic_thresh[0];
            else if (o == 0x200004)            r = plic_claim(0);
            else if (o == 0x201000)            r = plic_thresh[1];
            else if (o == 0x201004)            r = plic_claim(1);
            v = r;
            return true;
        }
        if (a >= FIN_BASE && a < FIN_BASE + 0x1000) return true;
        return false;
    }
    bool reg_write(uint64_t a, uint64_t v, uint8_t strb) {
        if (a >= CLINT_BASE && a < CLINT_BASE + CLINT_SIZE) {
            uint64_t o = a - CLINT_BASE;
            auto merge = [&](uint64_t& dst, uint64_t base) {
                for (int i = 0; i < 8; i++)
                    if (strb >> i & 1) {
                        uint64_t bo = (a & ~7ULL) + i - (CLINT_BASE + base);
                        if (bo < 8) { dst &= ~(0xFFULL << (8 * bo)); dst |= ((v >> (8 * i)) & 0xFF) << (8 * bo); }
                    }
            };
            if (o < 0x4000) { uint64_t t = msip; merge(t, o & ~3ULL); msip = t & 1; }
            else if (o >= 0x4000 && o < 0x4008) merge(mtimecmp, 0x4000);
            else if (o >= 0xBFF8 && o < 0xC000) merge(mtime, 0xBFF8);
            return true;
        }
        if (a >= PLIC_BASE && a < PLIC_BASE + PLIC_SIZE) {
            // escrita de 32 bits: pega a palavra selecionada pelos strobes
            uint64_t aw = a & ~7ULL;
            for (int half = 0; half < 2; half++) {
                if (!((strb >> (4 * half)) & 0xF)) continue;
                uint64_t o = aw + 4 * half - PLIC_BASE;
                uint32_t w = (uint32_t)(v >> (32 * half));
                if (o < 0x80)              plic_prio[o / 4] = w & 7;
                else if (o == 0x2000)      plic_enable[0] = w;
                else if (o == 0x2080)      plic_enable[1] = w;
                else if (o == 0x200000)    plic_thresh[0] = w & 7;
                else if (o == 0x200004)    { if (w < 32) plic_claimed &= ~(1u << w); }
                else if (o == 0x201000)    plic_thresh[1] = w & 7;
                else if (o == 0x201004)    { if (w < 32) plic_claimed &= ~(1u << w); }
            }
            return true;
        }
        if (a >= FIN_BASE && a < FIN_BASE + 0x1000) {
            uint64_t aw = a & ~7ULL;
            if (aw == FIN_BASE) {
                uint32_t w = (uint32_t)v;
                if ((w & 0xFFFF) == 0x5555 || (w & 0xFFFF) == 0x7777) { finished = true; exit_code = 0; }
                else if ((w & 0xFFFF) == 0x3333) { finished = true; exit_code = (int)(w >> 16) ? (int)(w >> 16) : 1; }
            }
            return true;
        }
        return false;
    }

    // acesso não-cacheável de 8 bytes alinhados com byte-enables
    bool uncached_read(uint64_t aw, uint8_t be, uint64_t& out) {
        out = 0;
        if (uint8_t* p = ptr(aw)) { memcpy(&out, p, 8); return true; }
        if ((aw >= UART_BASE && aw < UART_BASE + UART_SIZE)) {
            bool ok = true;
            for (int i = 0; i < 8; i++) if (be >> i & 1) { out |= (uint64_t)dev_read8(aw + i, ok) << (8 * i); }
            return ok;
        }
        // registrador 32/64: lê no primeiro byte habilitado
        int first = 0; while (first < 8 && !((be >> first) & 1)) first++;
        if (first == 8) first = 0;
        uint64_t v;
        if (!reg_read(aw + first, 8, v)) return false;
        out = v << (8 * first);
        return true;
    }
    bool uncached_write(uint64_t aw, uint64_t data, uint8_t be) {
        if (uint8_t* p = ptr(aw)) {
            if (in_rom(aw)) return true;
            for (int i = 0; i < 8; i++) if (be >> i & 1) p[i] = (uint8_t)(data >> (8 * i));
            return true;
        }
        if (aw >= UART_BASE && aw < UART_BASE + UART_SIZE) {
            bool ok = true;
            for (int i = 0; i < 8; i++) if (be >> i & 1) dev_write8(aw + i, (uint8_t)(data >> (8 * i)), ok);
            return ok;
        }
        return reg_write(aw, data, be);
    }
};

// -----------------------------------------------------------------------------
// ELF loader (ELF64 little-endian)
// -----------------------------------------------------------------------------
struct ElfInfo { uint64_t entry = 0; std::map<std::string, uint64_t> syms; };

static bool load_elf(const char* path, Soc& soc, ElfInfo& info) {
    FILE* f = fopen(path, "rb");
    if (!f) { perror(path); return false; }
    std::vector<uint8_t> d;
    fseek(f, 0, SEEK_END); d.resize(ftell(f)); fseek(f, 0, SEEK_SET);
    if (fread(d.data(), 1, d.size(), f) != d.size()) { fclose(f); return false; }
    fclose(f);
    if (d.size() < 64 || memcmp(d.data(), "\177ELF", 4) || d[4] != 2) { fprintf(stderr, "não é ELF64: %s\n", path); return false; }
    auto r16 = [&](size_t o) { uint16_t v; memcpy(&v, &d[o], 2); return v; };
    auto r32 = [&](size_t o) { uint32_t v; memcpy(&v, &d[o], 4); return v; };
    auto r64 = [&](size_t o) { uint64_t v; memcpy(&v, &d[o], 8); return v; };
    info.entry = r64(24);
    uint64_t phoff = r64(32), shoff = r64(40);
    uint16_t phentsize = r16(54), phnum = r16(56), shentsize = r16(58), shnum = r16(60);
    for (int i = 0; i < phnum; i++) {
        size_t ph = phoff + (size_t)i * phentsize;
        if (r32(ph) != 1) continue; // PT_LOAD
        uint64_t off = r64(ph + 8), paddr = r64(ph + 24), filesz = r64(ph + 32), memsz = r64(ph + 40);
        for (uint64_t k = 0; k < memsz; k++) {
            uint8_t* p = soc.ptr(paddr + k);
            if (!p) { fprintf(stderr, "segmento fora da memória: 0x%lx\n", (unsigned long)(paddr + k)); return false; }
            *p = (k < filesz) ? d[off + k] : 0;
        }
    }
    // símbolos
    for (int i = 0; i < shnum; i++) {
        size_t sh = shoff + (size_t)i * shentsize;
        if (r32(sh + 4) != 2) continue; // SHT_SYMTAB
        uint64_t off = r64(sh + 24), size = r64(sh + 32), entsize = r64(sh + 56);
        uint32_t link = r32(sh + 40);
        size_t strsh = shoff + (size_t)link * shentsize;
        uint64_t stroff = r64(strsh + 24);
        for (uint64_t k = 0; k < size / entsize; k++) {
            size_t s = off + k * entsize;
            uint32_t name = r32(s);
            uint64_t val = r64(s + 8);
            info.syms[std::string((const char*)&d[stroff + name])] = val;
        }
    }
    return true;
}

static bool load_bin(const std::string& spec, Soc& soc, uint64_t* where = nullptr) {
    size_t at = spec.find('@');
    if (at == std::string::npos) { fprintf(stderr, "use arquivo@endereço: %s\n", spec.c_str()); return false; }
    std::string file = spec.substr(0, at);
    uint64_t addr = strtoull(spec.c_str() + at + 1, nullptr, 0);
    FILE* f = fopen(file.c_str(), "rb");
    if (!f) { perror(file.c_str()); return false; }
    std::vector<uint8_t> d;
    fseek(f, 0, SEEK_END); d.resize(ftell(f)); fseek(f, 0, SEEK_SET);
    size_t n = fread(d.data(), 1, d.size(), f);
    fclose(f);
    for (size_t k = 0; k < n; k++) {
        uint8_t* p = soc.ptr(addr + k);
        if (!p) { fprintf(stderr, "binário fora da memória\n"); return false; }
        *p = d[k];
    }
    if (where) *where = addr;
    return true;
}

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
    int mtime_div = 100, rd_lat = 10, wr_lat = 2, ptw_lat = 10;
    uint64_t heartbeat = 0;
    uint64_t stall_limit = 50000000;
    uint64_t trace_from = 0;
    std::vector<std::string> expects, sends;
    uint64_t last_pc = 0;
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
    Vnebula_core_full* top = new Vnebula_core_full;
    std::unique_ptr<Cosim> cs;
    if (cosim) {
        cs.reset(Cosim::create(soc.ram.data(), soc.ram_size, soc.rom, Soc::ROM_SIZE, start_pc));
        if (!cs) { fprintf(stderr, "cosim indisponível (compile com make COSIM=1)\n"); return 2; }
    }

    FILE* tf = trace_file.empty() ? nullptr : fopen(trace_file.c_str(), "w");

    Port ip, dp, pp;
    uint64_t instret = 0;
    int ret = 0;

    top->clk = 0;
    top->rst_n = 0;
    top->imem_ack = 0; top->dmem_ack = 0; top->ptw_mem_ack = 0;
    top->imem_error = 0; top->dmem_error = 0; top->ptw_mem_error = 0;
    top->timer_irq = 0; top->external_irq = 0; top->software_irq = 0; top->s_external_irq = 0;
    top->debug_req = 0;
    top->mtime = 0;
    top->reset_vector = start_pc;
    for (int i = 0; i < 5; i++) { top->clk = 0; top->eval(); top->clk = 1; top->eval(); }
    top->clk = 0; top->eval();
    top->rst_n = 1;

    uint64_t last_commit_cycle = 0;
    bool tohost_done = false;

    for (g_cycle = 0; g_cycle < max_cycles; g_cycle++) {
        // ---------------- relógio de tempo real ----------------
        if (mtime_div > 0 && g_cycle % mtime_div == 0 && g_cycle) soc.mtime++;
        top->mtime        = soc.mtime;
        top->timer_irq    = soc.mtime >= soc.mtimecmp;
        top->software_irq = soc.msip & 1;
        top->external_irq   = soc.plic_irq(0);
        top->s_external_irq = soc.plic_irq(1);

        // ---------------- respostas (ack) deste ciclo ----------------
        top->imem_ack = 0; top->imem_error = 0;
        top->dmem_ack = 0; top->dmem_error = 0;
        top->ptw_mem_ack = 0; top->ptw_mem_error = 0;

        if (ip.busy && ip.count == 0) {
            bool ok = true;
            for (int w = 0; w < 16; w++) {
                uint64_t a = ip.addr + 4 * w;
                uint32_t v = 0;
                if (uint8_t* p = soc.ptr(a)) memcpy(&v, p, 4); else ok = false;
                top->imem_data[w] = v;
            }
            top->imem_ack = 1; top->imem_error = !ok;
        }
        if (dp.busy && dp.count == 0) {
            bool ok = true;
            if (dp.we) {
                for (int dw = 0; dw < 8; dw++) {
                    uint8_t be = (dp.wstrb >> (8 * dw)) & 0xFF;
                    if (!be) continue;
                    uint64_t aw = (dp.addr & ~63ULL) + 8 * dw;
                    uint64_t data = (uint64_t)dp.wdata[2 * dw] | ((uint64_t)dp.wdata[2 * dw + 1] << 32);
                    if (soc.tohost_addr && aw == (soc.tohost_addr & ~7ULL) && (be & 0x0F)) {
                        uint64_t th = data;
                        if (th & 1) { tohost_done = true; ret = (int)(th >> 1); }
                    }
                    if (dp.unc && tf && g_cycle >= trace_from)
                        fprintf(tf, "%lu MMIO W 0x%lx be=%02x data=0x%lx\n", (unsigned long)g_cycle, (unsigned long)aw, be, (unsigned long)data);
                    if (dp.unc) ok &= soc.uncached_write(aw, data, be);
                    else {
                        if (uint8_t* p = soc.ptr(aw)) {
                            for (int i = 0; i < 8; i++) if (be >> i & 1) p[i] = (uint8_t)(data >> (8 * i));
                        } else ok = false;
                    }
                }
                for (int w = 0; w < 16; w++) top->dmem_rdata[w] = 0;
            } else if (dp.unc) {
                uint64_t aw = dp.addr & ~7ULL;
                uint8_t be = (dp.wstrb >> (8 * ((dp.addr >> 3) & 7))) & 0xFF;
                uint64_t v = 0;
                ok = soc.uncached_read(aw, be, v);
                if (tf && g_cycle >= trace_from)
                    fprintf(tf, "%lu MMIO R 0x%lx be=%02x data=0x%lx\n", (unsigned long)g_cycle, (unsigned long)aw, be, (unsigned long)v);
                for (int w = 0; w < 16; w++) top->dmem_rdata[w] = 0;
                int lane = (dp.addr >> 3) & 7;
                top->dmem_rdata[2 * lane] = (uint32_t)v;
                top->dmem_rdata[2 * lane + 1] = (uint32_t)(v >> 32);
            } else {
                for (int w = 0; w < 16; w++) {
                    uint64_t a = (dp.addr & ~63ULL) + 4 * w;
                    uint32_t v = 0;
                    if (uint8_t* p = soc.ptr(a)) memcpy(&v, p, 4); else ok = false;
                    top->dmem_rdata[w] = v;
                }
            }
            top->dmem_ack = 1; top->dmem_error = !ok;
        }
        if (pp.busy && pp.count == 0) {
            uint64_t v = 0;
            bool ok = false;
            if (uint8_t* p = soc.ptr(pp.addr & ~7ULL)) { memcpy(&v, p, 8); ok = soc.in_ram(pp.addr); }
            top->ptw_mem_data = v;
            top->ptw_mem_ack = 1; top->ptw_mem_error = !ok;
        }

        top->clk = 0;
        top->eval();

        // ---------------- novas requisições ----------------
        if (!ip.busy && top->imem_req) { ip.busy = true; ip.count = rd_lat; ip.addr = top->imem_addr; }
        if (!dp.busy && top->dmem_req) {
            dp.busy = true; dp.addr = top->dmem_addr; dp.we = top->dmem_we; dp.unc = top->dmem_uncached;
            dp.wstrb = top->dmem_wstrb;
            for (int w = 0; w < 16; w++) dp.wdata[w] = top->dmem_wdata[w];
            dp.count = dp.we ? wr_lat : rd_lat;
        }
        if (!pp.busy && top->ptw_mem_req) { pp.busy = true; pp.count = ptw_lat; pp.addr = top->ptw_mem_addr; }

        // ---------------- commits ----------------
        for (int s = 0; s < 2; s++) {
            if (!((top->trace_valid >> s) & 1)) continue;
            instret++;
            last_commit_cycle = g_cycle;
            uint64_t pc = (uint64_t)top->trace_pc[2 * s] | ((uint64_t)top->trace_pc[2 * s + 1] << 32);
            last_pc = pc;
            uint32_t insn = (uint32_t)(top->trace_instr >> (32 * s));
            bool rd_en = (top->trace_rd_en >> s) & 1;
            bool rd_fp = (top->trace_rd_fp >> s) & 1;
            unsigned rd = (top->trace_rd >> (5 * s)) & 31;
            uint64_t rv = (uint64_t)top->trace_rd_val[2 * s] | ((uint64_t)top->trace_rd_val[2 * s + 1] << 32);
            bool mmio = (top->trace_mmio >> s) & 1;
            if (tf && g_cycle >= trace_from) {
                fprintf(tf, "%lu %d 0x%016lx (0x%08x)", (unsigned long)g_cycle, (int)top->trace_priv, (unsigned long)pc, insn);
                if (rd_en) fprintf(tf, " %c%-2u 0x%016lx", rd_fp ? 'f' : 'x', rd, (unsigned long)rv);
                fprintf(tf, "\n");
            }
            if (cs && !cs->step(pc, insn, rd_en, rd_fp, rd, rv, mmio, g_cycle)) {
                fprintf(stderr, "[cosim] divergência no ciclo %lu, instret %lu\n", (unsigned long)g_cycle, (unsigned long)instret);
                ret = 3; goto done;
            }
        }
        if (top->trace_trap) {
            if (tf && g_cycle >= trace_from) fprintf(tf, "%lu trap %s cause=%u epc=0x%016lx tval=0x%016lx\n", (unsigned long)g_cycle,
                            top->trace_trap_intr ? "intr" : "exc", (unsigned)top->trace_trap_cause,
                            (unsigned long)top->trace_trap_epc, (unsigned long)top->trace_trap_tval);
            uint64_t pins = ((uint64_t)top->timer_irq << 7) | ((uint64_t)top->software_irq << 3) |
                            ((uint64_t)top->external_irq << 11) | ((uint64_t)top->s_external_irq << 9);
            if (cs && !cs->trap(top->trace_trap_intr, top->trace_trap_cause, top->trace_trap_epc,
                                top->trace_trap_tval, pins, g_cycle)) {
                fprintf(stderr, "[cosim] divergência em trap no ciclo %lu\n", (unsigned long)g_cycle);
                ret = 3; goto done;
            }
        }

        top->clk = 1;
        top->eval();

        // ---------------- avança portas ----------------
        if (ip.busy) { if (ip.count == 0) ip.busy = false; else ip.count--; }
        if (dp.busy) { if (dp.count == 0) dp.busy = false; else dp.count--; }
        if (pp.busy) { if (pp.count == 0) pp.busy = false; else pp.count--; }

        if (heartbeat && g_cycle % heartbeat == 0)
            fprintf(stderr, "[hb] ciclo=%lu instret=%lu priv=%d pc=0x%016lx\n", (unsigned long)g_cycle,
                    (unsigned long)instret, (int)top->trace_priv, (unsigned long)last_pc);
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
    top->final();
    delete top;
    (void)stats_from;
    return ret;
}
