/* asid_test.c — ASIDs e SFENCE.VMA seletivo (Sv39), executado em S-mode.
 * Retorna 0 em sucesso, ou o número do passo que falhou. */
#include <stdint.h>

#define PTE_V 1
#define PTE_R 2
#define PTE_W 4
#define PTE_X 8
#define PTE_G 0x20
#define PTE_A 0x40
#define PTE_D 0x80

typedef uint64_t pte_t;
extern char __heap[];
static char *heap = __heap;

static void *alloc_page(void) {
    char *p = heap; heap += 4096;
    for (int i = 0; i < 4096; i++) p[i] = 0;
    return p;
}

#define VA_TEST 0x40000000ULL
#define VA_GLOB 0x40001000ULL

static pte_t *root[3], *l1[3], *l0[3];

static void build(int k, uint64_t pa_test, uint64_t pa_glob) {
    root[k] = alloc_page(); l1[k] = alloc_page(); l0[k] = alloc_page();
    /* código/dados: gigapágina identidade 0x80000000 (global) */
    root[k][2] = ((0x80000000ULL >> 12) << 10) | PTE_V | PTE_R | PTE_W | PTE_X | PTE_A | PTE_D | PTE_G;
    /* VA_TEST: root[VPN2] -> l1 -> l0 */
    root[k][(VA_TEST >> 30) & 511] = (((uint64_t)l1[k] >> 12) << 10) | PTE_V;
    l1[k][(VA_TEST >> 21) & 511] = (((uint64_t)l0[k] >> 12) << 10) | PTE_V;
    l0[k][(VA_TEST >> 12) & 511] = ((pa_test >> 12) << 10) | PTE_V | PTE_R | PTE_W | PTE_A | PTE_D;
    /* VA_GLOB: global, mesma página física em todos os espaços */

    l0[k][(VA_GLOB >> 12) & 511] = ((pa_glob >> 12) << 10) | PTE_V | PTE_R | PTE_W | PTE_A | PTE_D | PTE_G;
}

static inline void set_satp(int k, unsigned asid) {
    uint64_t v = (8ULL << 60) | ((uint64_t)asid << 44) | ((uint64_t)root[k] >> 12);
    __asm__ volatile ("csrw satp, %0" :: "r"(v) : "memory");
}
static inline uint64_t rd(uint64_t va) { return *(volatile uint64_t *)va; }
static inline void sfence(uint64_t va, uint64_t asid, int mode) {
    switch (mode) {
    case 0: __asm__ volatile ("sfence.vma zero, zero" ::: "memory"); break;
    case 1: __asm__ volatile ("sfence.vma %0, zero" :: "r"(va) : "memory"); break;
    case 2: __asm__ volatile ("sfence.vma zero, %0" :: "r"(asid) : "memory"); break;
    default: __asm__ volatile ("sfence.vma %0, %1" :: "r"(va), "r"(asid) : "memory"); break;
    }
}

int main(void) {
    uint64_t *pg[8];
    for (int i = 0; i < 8; i++) { pg[i] = alloc_page(); pg[i][0] = 0x1000 + i; }
    build(1, (uint64_t)pg[1], (uint64_t)pg[0]);
    build(2, (uint64_t)pg[2], (uint64_t)pg[0]);

    set_satp(1, 1);
    sfence(0, 0, 0);
    if (rd(VA_TEST) != 0x1001) return 1;
    if (rd(VA_GLOB) != 0x1000) return 2;
    set_satp(2, 2);                          /* sem SFENCE: ASIDs distintos */
    if (rd(VA_TEST) != 0x1002) return 3;
    if (rd(VA_GLOB) != 0x1000) return 4;
    set_satp(1, 1);
    if (rd(VA_TEST) != 0x1001) return 5;

    /* passo 6: muda PTE do espaço 1, SFENCE VA+ASID 1 */
    l0[1][(VA_TEST >> 12) & 511] = ((uint64_t)pg[4] >> 12 << 10) | PTE_V | PTE_R | PTE_W | PTE_A | PTE_D;
    sfence(VA_TEST, 1, 3);
    if (rd(VA_TEST) != 0x1004) return 6;
    set_satp(2, 2);
    if (rd(VA_TEST) != 0x1002) return 7;

    /* passo 8: muda PTE do espaço 2, SFENCE só por ASID 2 */
    l0[2][(VA_TEST >> 12) & 511] = ((uint64_t)pg[5] >> 12 << 10) | PTE_V | PTE_R | PTE_W | PTE_A | PTE_D;
    sfence(0, 2, 2);
    if (rd(VA_TEST) != 0x1005) return 8;
    if (rd(VA_GLOB) != 0x1000) return 9;

    /* passo 10: muda PTE do espaço 1, SFENCE só por VA (todas as ASIDs) */
    set_satp(1, 1);
    l0[1][(VA_TEST >> 12) & 511] = ((uint64_t)pg[6] >> 12 << 10) | PTE_V | PTE_R | PTE_W | PTE_A | PTE_D;
    sfence(VA_TEST, 0, 1);
    if (rd(VA_TEST) != 0x1006) return 10;

    /* passo 11: global muda; SFENCE por VA invalida também entradas globais */
    l0[1][(VA_GLOB >> 12) & 511] = ((uint64_t)pg[7] >> 12 << 10) | PTE_V | PTE_R | PTE_W | PTE_A | PTE_D | PTE_G;
    sfence(VA_GLOB, 0, 1);
    if (rd(VA_GLOB) != 0x1007) return 11;

    /* passo 12: mesmo ASID reutilizado com outra tabela após SFENCE global */
    set_satp(2, 1);
    sfence(0, 0, 0);
    if (rd(VA_TEST) != 0x1005) return 12;

    /* passo 13: estresse da TLB — 64 páginas no espaço 1 */
    set_satp(1, 1);
    for (int i = 0; i < 64; i++) {
        uint64_t va = VA_TEST + 0x10000 + (uint64_t)i * 4096;
        uint64_t *p = alloc_page(); p[0] = 0x2000 + i;
        l0[1][(va >> 12) & 511] = ((uint64_t)p >> 12 << 10) | PTE_V | PTE_R | PTE_W | PTE_A | PTE_D;
    }
    sfence(0, 0, 0);
    for (int r = 0; r < 3; r++)
        for (int i = 0; i < 64; i++)
            if (rd(VA_TEST + 0x10000 + (uint64_t)i * 4096) != 0x2000 + (uint64_t)i) return 13;
    return 0;
}
