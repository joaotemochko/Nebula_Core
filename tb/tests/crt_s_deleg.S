# crt_s_deleg.S — como crt_s.S, mas delega faltas de load/store/page para S
# e instala um handler S que chama s_trap(scause, stval, sepc) em C.
    .section .text.start, "ax"
    .globl _start
_start:
    .option push
    .option norelax
    la    gp, __global_pointer$
    .option pop
    la    sp, __stack_top
    la    t0, m_trap
    csrw  mtvec, t0
    li    t0, -1
    srli  t0, t0, 10
    csrw  pmpaddr0, t0
    li    t0, 0x1f
    csrw  pmpcfg0, t0
    li    t0, (1<<4)|(1<<5)|(1<<6)|(1<<7)|(1<<12)|(1<<13)|(1<<15)
    csrw  medeleg, t0
    la    t0, s_trap_entry
    csrw  stvec, t0
    li    t0, 0x1800
    csrc  mstatus, t0
    li    t0, 0x0800
    csrs  mstatus, t0
    la    t0, s_entry
    csrw  mepc, t0
    mret
s_entry:
    call  main
    mv    a1, a0
    li    a0, 0
    ecall
1:  j 1b

    .align 2
s_trap_entry:
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
    csrr  a0, scause
    csrr  a1, stval
    csrr  a2, sepc
    call  s_trap
    csrr  t0, sepc            # pula a instrução (2 ou 4 bytes)
    lhu   t1, 0(t0)
    andi  t1, t1, 3
    addi  t0, t0, 2
    li    t2, 3
    bne   t1, t2, 2f
    addi  t0, t0, 2
2:  csrw  sepc, t0
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
    sret

    .align 2
m_trap:
    csrr  t0, mcause
    li    t1, 9
    bne   t0, t1, m_fail
    li    t0, 0x100000
    bnez  a1, m_fail2
    li    t1, 0x5555
    sw    t1, 0(t0)
3:  j 3b
m_fail2:
    slli  a1, a1, 16
    li    t1, 0x3333
    or    t1, t1, a1
    sw    t1, 0(t0)
4:  j 4b
m_fail:
    li    t0, 0x100000
    li    t1, 0x00EE3333
    sw    t1, 0(t0)
5:  j 5b
