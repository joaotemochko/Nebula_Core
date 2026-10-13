# pmp_rand.S — teste aleatório de PMP (verificado por co-simulação com o Spike)
# Cada iteração: sorteia cfg/addr das 16 entradas, entra em U ou S, executa
# loads/stores/AMO em endereços sorteados de 'arena' e um salto para código
# em 'ucode2'. Traps voltam ao handler M, que pula a instrução ou aborta.
    .section .text.start, "ax"
    .globl _start
_start:
    la    t0, mhandler
    csrw  mtvec, t0
    li    s11, 0x2545F4914F6CDD1D     # semente LFSR (xorshift)
    li    s10, 600                    # iterações
iter:
    # ---- sorteia as 16 entradas ----
    la    s2, arena
    li    s3, 0
cfg_loop:
    call  rnd
    li    t1, 0x7FF                   # deslocamento em palavras (até 8 KB)
    and   t0, a0, t1
    slli  t0, t0, 2
    add   t0, s2, t0
    srli  t0, t0, 2                   # pmpaddr = addr >> 2
    call  rnd
    andi  t1, a0, 7                   # bits de NAPOT extra
    li    t2, 1
    sll   t2, t2, t1
    addi  t2, t2, -1
    or    t0, t0, t2                  # alguns uns finais (NAPOT)
    # código ainda não sabe escrever pmpaddrN indexado: tabela de saltos
    mv    a1, t0
    mv    a2, s3
    call  write_pmpaddr
    addi  s3, s3, 1
    li    t0, 16
    blt   s3, t0, cfg_loop
    # cfg: bytes aleatórios sem L (bit7) e sem bits 5-6
    call  rnd
    li    t1, 0x1F1F1F1F1F1F1F1F
    and   a0, a0, t1
    csrw  pmpcfg0, a0
    call  rnd
    li    t1, 0x1F1F1F1F1F1F1F1F
    and   a0, a0, t1
    csrw  pmpcfg2, a0
    # entrada 15 às vezes cobre tudo (NAPOT RWX) para permitir o código U
    call  rnd
    andi  t0, a0, 1
    beqz  t0, 1f
    li    t0, -1
    srli  t0, t0, 10
    csrw  pmpaddr15, t0
    csrr  t1, pmpcfg2
    li    t2, 0x00FFFFFFFFFFFFFF
    and   t1, t1, t2
    li    t2, 0x1F00000000000000
    or    t1, t1, t2
    csrw  pmpcfg2, t1
1:  csrr  t0, pmpcfg0                 # leituras comparadas pela cosim
    csrr  t0, pmpcfg2
    .irp n,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15
    csrr  t0, pmpaddr\n
    .endr
    # ---- endereços de acesso ----
    la    s4, addrs
    li    s3, 0
addr_loop:
    call  rnd
    li    t1, 0xFF8
    and   t0, a0, t1
    add   t0, s2, t0
    slli  t1, s3, 3
    add   t1, s4, t1
    sd    t0, 0(t1)
    addi  s3, s3, 1
    li    t0, 7
    blt   s3, t0, addr_loop
    # alvo de salto (a7) na área de código, nunca escrita
    call  rnd
    li    t1, 0xFF8
    and   t0, a0, t1
    la    t1, carena
    add   t0, t1, t0
    sd    t0, 56(s4)
    # ---- MSTATUS.MPP aleatório (U ou S), MPRV às vezes ----
    li    t0, 0x1800
    csrc  mstatus, t0
    call  rnd
    andi  t0, a0, 1
    beqz  t0, 2f
    li    t0, 0x0800
    csrs  mstatus, t0
2:  la    t0, ucode
    csrw  mepc, t0
    la    t0, addrs
    ld    a0, 0(t0)
    ld    a1, 8(t0)
    ld    a2, 16(t0)
    ld    a3, 24(t0)
    ld    a4, 32(t0)
    ld    a5, 40(t0)
    ld    a6, 48(t0)
    ld    a7, 56(t0)
    mret
back:
    # acessos M com MPRV=1 e MPP=S (checagem com privilégio efetivo S)
    la    s4, addrs
    ld    t1, 0(s4)
    la    t6, mprv_done
    li    t0, 0x1800
    csrc  mstatus, t0
    li    t0, 0x20800
    csrs  mstatus, t0
    ld    t2, 0(t1)
mprv_done:
    li    t0, 0x20000
    csrc  mstatus, t0
    addi  s10, s10, -1
    bnez  s10, iter
    li    t0, 0x100000
    li    t1, 0x5555
    sw    t1, 0(t0)
3:  j 3b

rnd:                                  # xorshift64 em s11 -> a0
    slli  t5, s11, 13
    xor   s11, s11, t5
    srli  t5, s11, 7
    xor   s11, s11, t5
    slli  t5, s11, 17
    xor   s11, s11, t5
    mv    a0, s11
    ret

write_pmpaddr:                        # a1 = valor, a2 = índice
    la    t5, wp_table
    slli  t6, a2, 3
    add   t5, t5, t6
    jr    t5
wp_table:
    .option push
    .option norvc
    .irp n,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15
    csrw  pmpaddr\n, a1
    ret
    .endr
    .option pop

    .align 2
mhandler:
    csrr  t3, mcause
    csrr  t4, mstatus
    srli  t4, t4, 11
    andi  t4, t4, 3
    li    t5, 3
    beq   t4, t5, from_m              # trap vindo de M (MPRV)
    li    t5, 8
    beq   t3, t5, done_user           # ecall de U
    li    t5, 9
    beq   t3, t5, done_user           # ecall de S
    li    t5, 1
    beq   t3, t5, done_user           # fetch negado: aborta iteração
    csrr  t3, mepc
    addi  t3, t3, 4
    csrw  mepc, t3
    mret
from_m:
    csrw  mepc, t6
    li    t0, 0x1800
    csrs  mstatus, t0                 # MPP=M
    mret
done_user:
    la    t3, back
    csrw  mepc, t3
    li    t0, 0x1800
    csrs  mstatus, t0
    mret

    .section .text.user, "ax"
    .align 12
ucode:                                # a0..a7 = endereços sorteados
    .option push
    .option norvc
    ld    t1, 0(a0)
    sd    t1, 0(a1)
    lw    t1, 4(a2)
    sw    t1, 0(a3)
    amoadd.d t1, t1, (a4)
    lr.d  t1, (a5)
    lbu   t1, 3(a6)
    jalr  a7                          # fetch em endereço da arena
    ecall
    .option pop

    .section .data
    .align 12
arena:
    .fill 512, 8, 0
carena:
    .fill 1024, 8, 0x0000000000000073   # 'ecall' para fetch válido
addrs:
    .fill 8, 8, 0
