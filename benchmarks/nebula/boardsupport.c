/* boardsupport.c — Nebula Core: contadores mcycle/minstret e saída via UART 16550.
 * Imprime "NEBULA_RESULT cycles=<n> instret=<n> correct=<0|1>" e encerra a
 * simulação pelo finisher (sifive,test0 @ 0x100000). */
#include <support.h>
#include <stdint.h>

#define UART_THR   ((volatile uint8_t *)0x10000000)
#define UART_LSR   ((volatile uint8_t *)0x10000005)
#define FINISHER   ((volatile uint32_t *)0x100000)

static uint64_t t_cyc0, t_cyc1, t_ins0, t_ins1;
static int bench_correct = -1;

static inline uint64_t rd_cycle(void)   { uint64_t v; __asm__ volatile ("csrr %0, mcycle"   : "=r"(v)); return v; }
static inline uint64_t rd_instret(void) { uint64_t v; __asm__ volatile ("csrr %0, minstret" : "=r"(v)); return v; }

static void putc_u(char c) { while (!(*UART_LSR & 0x20)) ; *UART_THR = (uint8_t)c; }
static void puts_u(const char *s) { while (*s) putc_u(*s++); }
static void putdec(uint64_t v) {
    char b[24]; int i = 0;
    do { b[i++] = (char)('0' + v % 10); v /= 10; } while (v);
    while (i) putc_u(b[--i]);
}
static void puthex(uint64_t v) {
    puts_u("0x");
    for (int i = 60; i >= 0; i -= 4) putc_u("0123456789abcdef"[(v >> i) & 15]);
}

void initialise_board(void) {}

void __attribute__((noinline)) start_trigger(void) {
    t_ins0 = rd_instret();
    t_cyc0 = rd_cycle();
}

void __attribute__((noinline)) stop_trigger(void) {
    t_cyc1 = rd_cycle();
    t_ins1 = rd_instret();
}

void nebula_exit(int code) {
    puts_u("NEBULA_RESULT cycles=");
    putdec(t_cyc1 - t_cyc0);
    puts_u(" instret=");
    putdec(t_ins1 - t_ins0);
    puts_u(" correct=");
    putdec(code == 0);
    puts_u("\n");
    *FINISHER = code == 0 ? 0x5555 : (0x3333 | ((uint32_t)code << 16));
    for (;;) ;
}

void nebula_trap(uint64_t cause, uint64_t epc, uint64_t tval) {
    puts_u("TRAP mcause="); puthex(cause);
    puts_u(" mepc=");       puthex(epc);
    puts_u(" mtval=");      puthex(tval);
    puts_u("\n");
    *FINISHER = 0x3333 | (0xEE << 16);
    for (;;) ;
}
