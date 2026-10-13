/* crt0.S — inicialização bare-metal para Embench no Nebula Core (M-mode) */
    .section .text.start, "ax"
    .globl _start
_start:
    .option push
    .option norelax
    la      gp, __global_pointer$
    .option pop
    la      sp, __stack_top
    li      t0, (1 << 13)          /* mstatus.FS = Initial */
    csrs    mstatus, t0
    csrwi   fcsr, 0
    la      t0, trap_handler
    csrw    mtvec, t0
    /* zera .bss */
    la      t0, __bss_start
    la      t1, __bss_end
1:  bgeu    t0, t1, 2f
    sd      zero, 0(t0)
    addi    t0, t0, 8
    j       1b
2:  call    main
    call    nebula_exit
3:  j       3b

    .align 2
trap_handler:
    csrr    a0, mcause
    csrr    a1, mepc
    csrr    a2, mtval
    call    nebula_trap
4:  j       4b
