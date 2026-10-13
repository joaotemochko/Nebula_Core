# crt_s.S — inicialização em M-mode e entrada em S-mode para testes em C
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
    li    t0, -1                      # PMP: tudo acessível para S/U
    srli  t0, t0, 10
    csrw  pmpaddr0, t0
    li    t0, 0x1f
    csrw  pmpcfg0, t0
    li    t0, 0x1800
    csrc  mstatus, t0
    li    t0, 0x0800                  # MPP = S
    csrs  mstatus, t0
    la    t0, s_entry
    csrw  mepc, t0
    mret
s_entry:
    call  main
    # main retorna código em a0 -> ecall para M encerrar
    mv    a1, a0
    li    a0, 0
    ecall
1:  j 1b

    .align 2
m_trap:
    csrr  t0, mcause
    li    t1, 9                       # ecall de S: fim
    bne   t0, t1, m_fail
    li    t0, 0x100000
    bnez  a1, m_fail2
    li    t1, 0x5555
    sw    t1, 0(t0)
2:  j 2b
m_fail2:
    slli  a1, a1, 16
    li    t1, 0x3333
    or    t1, t1, a1
    sw    t1, 0(t0)
3:  j 3b
m_fail:
    li    t0, 0x100000
    li    t1, 0x00EE3333
    sw    t1, 0(t0)
4:  j 4b
