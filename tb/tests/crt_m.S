# crt_m.S — testes C em M-mode com handler de trap em C: m_trap(mcause, mtval, mepc)
# O handler C retorna o novo mepc em a0.
    .section .text.start, "ax"
    .globl _start
_start:
    .option push
    .option norelax
    la    gp, __global_pointer$
    .option pop
    la    sp, __stack_top
    la    t0, trap_entry
    csrw  mtvec, t0
    li    t0, -1
    srli  t0, t0, 10
    csrw  pmpaddr0, t0
    li    t0, 0x1f
    csrw  pmpcfg0, t0
    call  main
    li    t0, 0x100000
    bnez  a0, 1f
    li    t1, 0x5555
    sw    t1, 0(t0)
0:  j 0b
1:  slli  a0, a0, 16
    li    t1, 0x3333
    or    t1, t1, a0
    sw    t1, 0(t0)
2:  j 2b

    .align 2
trap_entry:
    addi  sp, sp, -128
    sd    ra, 0(sp)
    sd    t0, 8(sp)
    sd    t1, 16(sp)
    sd    t2, 24(sp)
    sd    a0, 32(sp)
    sd    a1, 40(sp)
    sd    a2, 48(sp)
    sd    a3, 56(sp)
    sd    a4, 64(sp)
    sd    a5, 72(sp)
    sd    a6, 80(sp)
    sd    a7, 88(sp)
    sd    t3, 96(sp)
    sd    t4, 104(sp)
    sd    t5, 112(sp)
    sd    t6, 120(sp)
    csrr  a0, mcause
    csrr  a1, mtval
    csrr  a2, mepc
    call  m_trap
    csrw  mepc, a0
    ld    ra, 0(sp)
    ld    t0, 8(sp)
    ld    t1, 16(sp)
    ld    t2, 24(sp)
    ld    a0, 32(sp)
    ld    a1, 40(sp)
    ld    a2, 48(sp)
    ld    a3, 56(sp)
    ld    a4, 64(sp)
    ld    a5, 72(sp)
    ld    a6, 80(sp)
    ld    a7, 88(sp)
    ld    t3, 96(sp)
    ld    t4, 104(sp)
    ld    t5, 112(sp)
    ld    t6, 120(sp)
    addi  sp, sp, 128
    mret

    .globl enter_user
enter_user:                       # a0 = função U (deve terminar com ecall)
    csrw  mepc, a0
    li    t0, 0x1800
    csrc  mstatus, t0             # MPP = U
    mret
