OUTPUT_ARCH(riscv)
ENTRY(_start)
MEMORY { RAM (rwx) : ORIGIN = 0x80000000, LENGTH = 64M }
SECTIONS
{
  .text : { *(.text.start) *(.text .text.*) } > RAM
  .rodata : { *(.rodata .rodata.*) *(.srodata .srodata.*) } > RAM
  . = ALIGN(8);
  .data : {
    __global_pointer$ = . + 0x800;
    *(.data .data.*) *(.sdata .sdata.*)
  } > RAM
  . = ALIGN(8);
  .bss : { __bss_start = .; *(.sbss .sbss.*) *(.bss .bss.*) *(COMMON) . = ALIGN(8); __bss_end = .; } > RAM
  . = ALIGN(16);
  _end = .;
  end = .;
  __stack_top = ORIGIN(RAM) + LENGTH(RAM);
}
