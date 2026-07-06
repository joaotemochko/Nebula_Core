#include "boardsupport.h"
#include <stdint.h>

void initialise_board(void) {
    // Inicialização bare-metal (UART, etc.), se necessário.
}

void start_trigger(void) {
    // Zera os contadores no início da medição
    __asm__ volatile("csrw mcycle, zero");
    __asm__ volatile("csrw minstret, zero");
}

void stop_trigger(void) {
    uint64_t cycles, instret;
    // Lê os contadores no final da medição
    __asm__ volatile("csrr %0, mcycle" : "=r"(cycles));
    __asm__ volatile("csrr %0, minstret" : "=r"(instret));
}
