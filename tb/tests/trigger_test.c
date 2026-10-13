/* trigger_test.c — triggers de debug (Sdtrig) em M e U. Verificado por cosim. */
#include <stdint.h>

#define CSRW(c, v) __asm__ volatile ("csrw " #c ", %0" :: "r"((uint64_t)(v)) : "memory")
#define CSRR(c) ({ uint64_t _v; __asm__ volatile ("csrr %0, " #c : "=r"(_v) :: "memory"); _v; })

/* mcontrol6 */
#define T6      (6ULL << 60)
#define SEL     (1ULL << 21)
#define CHAIN   (1ULL << 11)
#define MATCH(m) ((uint64_t)(m) << 7)
#define TM      (1ULL << 6)
#define TS      (1ULL << 4)
#define TU      (1ULL << 3)
#define TEX     (1ULL << 2)
#define TST     (1ULL << 1)
#define TLD     (1ULL << 0)
/* mcontrol */
#define T2      (2ULL << 60)
#define TIMING2 (1ULL << 18)

volatile uint64_t n_bp, last_tval, last_epc, user_done;
extern char user_back[];
volatile int skip_mode;   /* 0: pula a instrução; 1: não pula (após) */
volatile uint64_t data[8] __attribute__((aligned(64)));

uint64_t m_trap(uint64_t cause, uint64_t tval, uint64_t epc) {
    if (cause == 8) { user_done = 1; CSRW(mstatus, CSRR(mstatus) | 0x1800); return (uint64_t)user_back; }
    if (cause != 3) { for (;;) ; }
    n_bp++; last_tval = tval; last_epc = epc;
    if (skip_mode) return epc;
    uint16_t h = *(volatile uint16_t *)epc;
    return epc + (((h & 3) == 3) ? 4 : 2);
}

static void clear_all(void) {
    for (int i = 0; i < 4; i++) { CSRW(tselect, i); CSRW(tdata1, 0); CSRW(tdata2, 0); }
}
static void set_trig(int i, uint64_t t1, uint64_t t2) { CSRW(tselect, i); CSRW(tdata2, t2); CSRW(tdata1, t1); }

__attribute__((noinline, aligned(4))) int target_fn(int x) { return x * 3 + 1; }

extern void enter_user(void (*f)(void));
/* rotina de usuário sem uso de pilha */
extern void user_fn(void);
__asm__(".align 2\n.globl user_fn\nuser_fn:\n la t0, target_fn\n li a0, 5\n jalr t0\n ecall\n");

static uint64_t ld64(volatile uint64_t *p) { uint64_t v; __asm__ volatile ("ld %0, 0(%1)" : "=r"(v) : "r"(p) : "memory"); return v; }
static void sd64(volatile uint64_t *p, uint64_t v) { __asm__ volatile ("sd %0, 0(%1)" :: "r"(v), "r"(p) : "memory"); }

int main(void) {
    CSRW(mstatus, CSRR(mstatus) | 0x8);   /* MIE=1 (mie=0: sem interrupções) */
    clear_all();
    if (CSRR(tinfo) == 0) return 1;

    /* 1: execução EQUAL na entrada de target_fn */
    set_trig(0, T6 | TM | TEX, (uint64_t)target_fn);
    n_bp = 0; skip_mode = 1;
    /* breakpoint antes da 1ª instrução; o handler retorna em epc -> ainda casaria.
     * desabilita no handler? usamos skip: pula a 1ª instrução da função */
    skip_mode = 0;
    volatile int r = target_fn(4); (void)r;
    if (n_bp != 1 || last_tval != (uint64_t)target_fn || last_epc != (uint64_t)target_fn) return 2;
    CSRW(tselect, 0);
    if (((CSRR(tdata1) >> 22) & 1) != 1) return 3;            /* hit0 */
    clear_all();

    /* 2: load por endereço (antes): rd não é escrito */
    data[0] = 111; data[1] = 222;
    set_trig(1, T6 | TM | TLD, (uint64_t)&data[1]);
    n_bp = 0; skip_mode = 0;
    uint64_t v = 7;
    __asm__ volatile ("ld %0, 0(%1)" : "+r"(v) : "r"(&data[1]) : "memory");
    if (n_bp != 1 || v != 7 || last_tval != (uint64_t)&data[1]) return 4;
    if (ld64(&data[0]) != 111 || n_bp != 1) return 5;
    clear_all();

    /* 3: store NAPOT em 16 bytes (data[2..3]) */
    set_trig(2, T6 | TM | TST | MATCH(1), (uint64_t)&data[2] | 7);
    n_bp = 0;
    sd64(&data[3], 0x33);              /* dentro: bp, não grava */
    sd64(&data[4], 0x44);              /* fora */
    if (n_bp != 1 || data[3] != 0 || data[4] != 0x44) return 6;
    clear_all();

    /* 4: store por valor (select=1) */
    set_trig(0, T6 | TM | TST | SEL, 0x55);
    n_bp = 0;
    sd64(&data[5], 0x56);
    sd64(&data[5], 0x55);
    if (n_bp != 1 || data[5] != 0x56) return 7;
    clear_all();

    /* 5: load por valor (após): load completa, trap na instrução seguinte */
    data[6] = 0xABC;
    set_trig(3, T6 | TM | TLD | SEL, 0xABC);
    n_bp = 0; skip_mode = 1;
    v = 0;
    __asm__ volatile ("ld %0, 0(%1)\n nop" : "=r"(v) : "r"(&data[6]) : "memory");
    skip_mode = 0;
    if (n_bp != 1 || v != 0xABC || last_tval != (uint64_t)&data[6]) return 8;
    CSRW(tselect, 3);
    if (((CSRR(tdata1) >> 22) & 1) != 1 || ((CSRR(tdata1) >> 25) & 1) != 1) return 9;  /* hit = 3 */
    clear_all();

    /* 6: cadeia GE/LT: dispara só em [data[1], data[3]) */
    set_trig(0, T6 | TM | TLD | MATCH(2) | CHAIN, (uint64_t)&data[1]);
    set_trig(1, T6 | TM | TLD | MATCH(3), (uint64_t)&data[3]);
    n_bp = 0;
    ld64(&data[0]); ld64(&data[1]); ld64(&data[2]); ld64(&data[3]); ld64(&data[4]);
    if (n_bp != 2) return 10;
    clear_all();

    /* 7: mcontrol com timing "após" em store */
    set_trig(0, T2 | TM | TST | TIMING2, (uint64_t)&data[7]);
    n_bp = 0; skip_mode = 1;
    sd64(&data[7], 0x77);
    __asm__ volatile ("nop");
    skip_mode = 0;
    if (n_bp != 1 || data[7] != 0x77) return 11;
    clear_all();

    /* 8: MIE=0 em M: não dispara */
    CSRW(mstatus, CSRR(mstatus) & ~0x8ULL);
    set_trig(0, T6 | TM | TLD, (uint64_t)&data[0]);
    n_bp = 0;
    ld64(&data[0]);
    if (n_bp != 0) return 12;
    CSRW(mstatus, CSRR(mstatus) | 0x8);
    clear_all();

    /* 9: U-mode: execução com u=1 (m=0 não dispara em M) */
    set_trig(0, T6 | TU | TEX, (uint64_t)target_fn);
    n_bp = 0; skip_mode = 0; user_done = 0;
    target_fn(1);                       /* M: não dispara */
    if (n_bp != 0) return 13;
    __asm__ volatile ("mv a0, %0\n call enter_user\n .globl user_back\nuser_back:" :: "r"(user_fn)
                      : "a0","a1","a2","a3","a4","a5","a6","a7","t0","t1","t2","t3","t4","t5","t6","ra","memory");
    if (!user_done || n_bp != 1) return 14;
    clear_all();
    return 0;
}
