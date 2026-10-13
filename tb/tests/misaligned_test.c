/* misaligned_test.c — acessos desalinhados em S-mode com Sv39.
 * Retorna 0 em sucesso, ou um código de erro. */
#include <stdint.h>

#define PTE_V 1
#define PTE_R 2
#define PTE_W 4
#define PTE_X 8
#define PTE_G 0x20
#define PTE_A 0x40
#define PTE_D 0x80

extern char __heap[];
static char *heap = __heap;
static void *alloc_page(void) { char *p = heap; heap += 4096; for (int i = 0; i < 4096; i++) p[i] = 0; return p; }

volatile uint64_t trap_cause, trap_tval, trap_count;
void s_trap(uint64_t cause, uint64_t tval, uint64_t epc) {
    (void)epc; trap_cause = cause; trap_tval = tval; trap_count++;
}

#define VA 0x40000000ULL   /* 4 páginas: 0,1 RW; 2 somente leitura; 3 não mapeada */

static uint8_t ref[4 * 4096];

static uint64_t ld_n(volatile uint8_t *p, int n) {
    uint64_t r;
    switch (n) {
    case 2: __asm__ volatile ("lhu %0, 0(%1)" : "=r"(r) : "r"(p) : "memory"); break;
    case 4: __asm__ volatile ("lwu %0, 0(%1)" : "=r"(r) : "r"(p) : "memory"); break;
    default: __asm__ volatile ("ld %0, 0(%1)" : "=r"(r) : "r"(p) : "memory"); break;
    }
    return r;
}
static void st_n(volatile uint8_t *p, int n, uint64_t v) {
    switch (n) {
    case 2: __asm__ volatile ("sh %0, 0(%1)" :: "r"(v), "r"(p) : "memory"); break;
    case 4: __asm__ volatile ("sw %0, 0(%1)" :: "r"(v), "r"(p) : "memory"); break;
    default: __asm__ volatile ("sd %0, 0(%1)" :: "r"(v), "r"(p) : "memory"); break;
    }
}

int main(void) {
    uint64_t *root = alloc_page(), *l1 = alloc_page(), *l0 = alloc_page();
    uint8_t *pg[3];
    for (int i = 0; i < 3; i++) pg[i] = alloc_page();
    root[2] = ((0x80000000ULL >> 12) << 10) | PTE_V | PTE_R | PTE_W | PTE_X | PTE_A | PTE_D | PTE_G;
    root[(VA >> 30) & 511] = (((uint64_t)l1 >> 12) << 10) | PTE_V;
    l1[(VA >> 21) & 511]   = (((uint64_t)l0 >> 12) << 10) | PTE_V;
    for (int i = 0; i < 3; i++)
        l0[((VA >> 12) & 511) + i] = (((uint64_t)pg[i] >> 12) << 10) | PTE_V | PTE_R | PTE_A |
                                     (i < 2 ? (PTE_W | PTE_D) : 0);
    uint64_t satp = (8ULL << 60) | ((uint64_t)root >> 12);
    __asm__ volatile ("csrw satp, %0; sfence.vma" :: "r"(satp) : "memory");

    volatile uint8_t *m = (volatile uint8_t *)VA;
    for (int i = 0; i < 3 * 4096; i++) {
        uint8_t v = (uint8_t)(i * 7 + 3);
        if (i < 2 * 4096) m[i] = v; else pg[2][i - 2 * 4096] = v;   /* pág. 2 é RO: escreve pelo físico */
        ref[i] = v;
    }

    /* 1) todos os tamanhos/offsets, cruzando doubleword, linha e página 0->1 */
    uint32_t seed = 1;
    const int bases[] = {0, 56, 60, 4088, 4090, 4093, 128, 1000};
    for (int b = 0; b < 8; b++)
        for (int n = 2; n <= 8; n *= 2)
            for (int off = 0; off < 8; off++) {
                int a = bases[b] + off;
                uint64_t exp = 0;
                for (int k = n - 1; k >= 0; k--) exp = (exp << 8) | ref[a + k];
                if (ld_n(m + a, n) != exp) return 100 + b;
                seed = seed * 1103515245 + 12345;
                uint64_t v = ((uint64_t)seed << 32) | (seed ^ 0x9E3779B9u);
                st_n(m + a, n, v);
                for (int k = 0; k < n; k++) ref[a + k] = (uint8_t)(v >> (8 * k));
                for (int k = -2; k < n + 2; k++)
                    if (a + k >= 0 && m[a + k] != ref[a + k]) return 200 + b;
            }
    if (trap_count != 0) return 2;

    /* 2) store cruzando para página somente-leitura: falta na 2ª parte */
    volatile uint8_t *p = m + 2 * 4096 - 3;           /* 3 bytes na pág. 1, 5 na pág. 2 */
    __asm__ volatile ("sd %0, 0(%1)" :: "r"(0x1122334455667788ULL), "r"(p) : "memory");
    if (trap_count != 1 || trap_cause != 15 || trap_tval != VA + 2 * 4096) return 3;
    if (p[0] != 0x88 || p[1] != 0x77 || p[2] != 0x66) return 4;  /* 1ª parte gravada */
    if (m[2 * 4096] != ref[2 * 4096]) return 5;                   /* 2ª parte não */

    /* 3) load cruzando para página não mapeada: falta com stval na fronteira */
    volatile uint8_t *q = m + 3 * 4096 - 2;
    uint64_t sink;
    __asm__ volatile ("lw %0, 0(%1)" : "=r"(sink) : "r"(q) : "memory");
    if (trap_count != 2 || trap_cause != 13 || trap_tval != VA + 3 * 4096) return 6;

    /* 4) load desalinhado totalmente numa página RO: ok */
    __asm__ volatile ("lhu %0, 0(%1)" : "=r"(sink) : "r"(m + 2 * 4096 + 1) : "memory");
    if (sink != (uint16_t)(ref[2 * 4096 + 1] | (ref[2 * 4096 + 2] << 8))) return 7;

    /* 5) AMO desalinhado: access fault de store (7), stval = endereço */
    uint64_t r;
    __asm__ volatile ("amoadd.w %0, %1, (%2)" : "=r"(r) : "r"(1), "r"(m + 2) : "memory");
    if (trap_count != 3 || trap_cause != 7 || trap_tval != VA + 2) return 8;
    __asm__ volatile ("lr.d %0, (%1)" : "=r"(r) : "r"(m + 4) : "memory");
    if (trap_count != 4 || trap_cause != 5) return 9;
    return 0;
}
