/* smp_test.c — estresse de coerência/atômicos/ordenação em N harts. */
#include <stdint.h>
#ifndef NHARTS
#define NHARTS 2
#endif
#define ITER 300

#define FINISHER (*(volatile uint32_t *)0x100000)

volatile uint64_t amo_counter, lrsc_counter, lock_counter, lock_word;
volatile uint64_t arrived[NHARTS], done_cnt;
volatile uint64_t mp_data[64], mp_flag[64];
volatile uint64_t shared_line[8] __attribute__((aligned(64)));
volatile uint64_t errors;

static inline void fence(void) { __asm__ volatile ("fence rw, rw" ::: "memory"); }

static void lock(volatile uint64_t *l) {
    uint64_t old;
    do {
        __asm__ volatile ("amoswap.d.aq %0, %1, (%2)" : "=r"(old) : "r"(1ULL), "r"(l) : "memory");
    } while (old);
}
static void unlock(volatile uint64_t *l) {
    __asm__ volatile ("fence rw, w" ::: "memory");
    *l = 0;
}

static void barrier(uint64_t phase) {
    uint64_t me;
    __asm__ volatile ("amoadd.d %0, %1, (%2)" : "=r"(me) : "r"(1ULL), "r"(&done_cnt) : "memory");
    while (done_cnt < phase * NHARTS) ;
    fence();
}

void smp_main(uint64_t hart) {
    if (hart >= NHARTS) for (;;) __asm__ volatile ("wfi");

    for (int i = 0; i < ITER; i++) {
        /* AMO */
        __asm__ volatile ("amoadd.d zero, %0, (%1)" :: "r"(1ULL), "r"(&amo_counter) : "memory");
        /* LR/SC */
        uint64_t v, fail;
        do {
            __asm__ volatile ("lr.d %0, (%2)\n addi %0, %0, 1\n sc.d %1, %0, (%2)"
                              : "=&r"(v), "=&r"(fail) : "r"(&lrsc_counter) : "memory");
        } while (fail);
        /* spinlock + incremento não atômico */
        lock(&lock_word);
        uint64_t t = lock_counter;
        lock_counter = t + 1;
        unlock(&lock_word);
        /* escritas concorrentes em bytes diferentes da mesma linha */
        ((volatile uint8_t *)shared_line)[hart * 4 + (i & 3)] = (uint8_t)(i + 1);
    }
    barrier(1);

    /* passagem de mensagem: hart k escreve para o hart (k+1)%N */
    for (int r = 0; r < 32; r++) {
        uint64_t to = (hart + 1) % NHARTS;
        int slot = (int)(to * 8 + (r % 8));
        mp_data[slot] = 0x1000 * hart + r + 1;
        __asm__ volatile ("fence w, w" ::: "memory");
        mp_flag[slot] = r + 1;
        int my = (int)(hart * 8 + (r % 8));
        while (mp_flag[my] != (uint64_t)(r + 1)) ;
        __asm__ volatile ("fence r, r" ::: "memory");
        uint64_t from = (hart + NHARTS - 1) % NHARTS;
        if (mp_data[my] != 0x1000 * from + r + 1) errors++;
        barrier(2 + r);
    }

    if (hart == 0) {
        uint64_t exp = (uint64_t)NHARTS * ITER;
        int bad = 0;
        if (amo_counter != exp) bad = 1;
        if (lrsc_counter != exp) bad |= 2;
        if (lock_counter != exp) bad |= 4;
        for (int h = 0; h < NHARTS; h++)
            for (int k = 0; k < 4; k++)
                if (((volatile uint8_t *)shared_line)[h * 4 + k] != (uint8_t)(ITER - 4 + k + 1)) bad |= 8;
        if (errors) bad |= 16;
        FINISHER = bad ? (0x3333 | ((uint32_t)bad << 16)) : 0x5555;
    }
    for (;;) __asm__ volatile ("wfi");
}
