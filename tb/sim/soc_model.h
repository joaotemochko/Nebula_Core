// soc_model.h — modelo do SoC simulado (memória, CLINT, PLIC, UART, finisher)
// e carregadores de ELF/binários, compartilhado pelos testbenches.
#pragma once
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <string>
#include <vector>
#include <map>
#include <deque>

#ifndef NCORES
#define NCORES 1
#endif

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
    uint64_t mtime = 0, mtimecmp[NCORES];
    uint32_t msip[NCORES];
    // UART
    uint8_t uart_ier = 0, uart_lcr = 0, uart_mcr = 0, uart_scr = 0, uart_dll = 0, uart_dlm = 0, uart_fcr = 0;
    bool uart_thre_ip = false;
    std::deque<uint8_t> uart_rx;
    std::string uart_line;
    // PLIC (fontes 1..31, contextos 0=M 1=S)
    uint32_t plic_prio[32] = {0};
    uint32_t plic_enable[2 * NCORES] = {0};
    uint32_t plic_thresh[2 * NCORES] = {0};
    uint32_t plic_claimed = 0;   // fontes em atendimento
    // finisher / tohost
    bool finished = false;
    int  exit_code = 0;
    uint64_t tohost_addr = 0;
    bool quiet = false;

    Soc(uint64_t mb) : ram_size(mb << 20), ram(ram_size, 0) {
        memset(rom, 0, sizeof(rom));
        for (int h = 0; h < NCORES; h++) { mtimecmp[h] = ~0ULL; msip[h] = 0; }
    }

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
            if (o < 4 * NCORES)                    { base = o & ~3ULL; val = msip[o / 4]; size = 4; }
            else if (o >= 0x4000 && o < 0x4000 + 8 * NCORES) { base = o & ~7ULL; val = mtimecmp[(o - 0x4000) / 8]; }
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
            else if (o >= 0x2000 && o < 0x2000 + 0x80 * 2 * NCORES && (o & 0x7F) == 0)
                r = plic_enable[(o - 0x2000) / 0x80];
            else if (o >= 0x200000 && o < 0x200000 + 0x1000 * 2 * NCORES) {
                int ctx = (o - 0x200000) / 0x1000;
                if ((o & 0xFFF) == 0) r = plic_thresh[ctx];
                else if ((o & 0xFFF) == 4) r = plic_claim(ctx);
            }
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
            if (o < 4 * NCORES) {
                // msip: registradores de 32 bits; trata as duas metades do doubleword
                for (int half = 0; half < 2; half++) {
                    uint64_t ho = (o & ~7ULL) + 4 * half;
                    if (ho < 4 * NCORES && ((strb >> (4 * half)) & 0xF)) msip[ho / 4] = (v >> (32 * half)) & 1;
                }
            }
            else if (o >= 0x4000 && o < 0x4000 + 8 * NCORES) merge(mtimecmp[((o & ~7ULL) - 0x4000) / 8], o & ~7ULL);
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
                else if (o >= 0x2000 && o < 0x2000 + 0x80 * 2 * NCORES && (o & 0x7F) == 0)
                    plic_enable[(o - 0x2000) / 0x80] = w;
                else if (o >= 0x200000 && o < 0x200000 + 0x1000 * 2 * NCORES) {
                    int ctx = (o - 0x200000) / 0x1000;
                    if ((o & 0xFFF) == 0) plic_thresh[ctx] = w & 7;
                    else if ((o & 0xFFF) == 4) { if (w < 32) plic_claimed &= ~(1u << w); }
                }
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

