# crt_smp.S — inicialização multi-hart (M-mode): pilha por hart, smp_main(hartid)
    .section .text.start, "ax"
    .globl _start
_start:
    .option push
    .option norelax
    la    gp, __global_pointer$
    .option pop
    csrr  a0, mhartid
    la    sp, __stack_top
    slli  t0, a0, 16            # 64 KB de pilha por hart
    sub   sp, sp, t0
    la    t0, trap
    csrw  mtvec, t0
    call  smp_main
1:  wfi
    j     1b
trap:
    li    t0, 0x100000
    li    t1, 0x00EE3333
    sw    t1, 0(t0)
2:  j 2b
