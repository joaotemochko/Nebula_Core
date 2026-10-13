# access_fault.S — faltas de acesso precisas (PMA): load/store/AMO/fetch em
# endereços inexistentes. O handler conta traps e pula a instrução.
    .section .text.start, "ax"
    .globl _start
_start:
    la    t0, handler
    csrw  mtvec, t0
    li    s0, 0                 # contador de traps
    li    s1, 0x40000000        # região inexistente
    ld    a0, 0(s1)             # load access fault (5)
    sd    a0, 8(s1)             # store access fault (7)
    sw    a0, 0x100(zero)       # endereço 0x100: inexistente
    amoadd.d a0, a0, (s1)       # AMO access fault (7)
    lb    a0, 0x200(zero)
    la    t1, after_fetch
    li    t2, 0x40000000
    jr    t2                    # fetch access fault (1)
after_fetch:
    li    t0, 6
    bne   s0, t0, fail
    li    t0, 0x100000
    li    t1, 0x5555
    sw    t1, 0(t0)
1:  j 1b
fail:
    li    t0, 0x100000
    li    t1, 0x13333
    sw    t1, 0(t0)
2:  j 2b

    .align 2
handler:
    addi  s0, s0, 1
    csrr  t3, mcause
    li    t4, 1
    beq   t3, t4, fetch_fault
    csrr  t3, mepc
    lh    t4, 0(t3)             # tamanho da instrução (RVC?)
    andi  t4, t4, 3
    li    t5, 3
    addi  t3, t3, 2
    bne   t4, t5, 3f
    addi  t3, t3, 2
3:  csrw  mepc, t3
    mret
fetch_fault:
    csrw  mepc, t1
    mret
