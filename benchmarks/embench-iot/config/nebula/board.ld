MEMORY
{
  rom (rx) : ORIGIN = 0x10000000, LENGTH = 256K
  ram (rwx) : ORIGIN = 0x40000000, LENGTH = 64M
}

SECTIONS
{
  .text : { *(.text*) *(.rodata*) } > rom
  .data : { *(.data*) } > ram
  .bss : { *(.bss*) *(COMMON) } > ram
}
