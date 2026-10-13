ENTRY(_start)
SECTIONS {
  . = 0x80000000;
  .text : { *(.text.start) *(.text*) }
  .rodata : { *(.rodata*) *(.srodata*) }
  .data : { __global_pointer$ = . + 0x800; *(.data*) *(.sdata*) }
  .bss : { *(.sbss*) *(.bss*) *(COMMON) }
  . = ALIGN(4096);
  __heap = .;
  __stack_top = 0x80000000 + 0x400000;
}
