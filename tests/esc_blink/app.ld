ENTRY(Reset_Handler)
MEMORY {
  FLASH (rx) : ORIGIN = 0x08004000, LENGTH = 110K
  RAM (rw) : ORIGIN = 0x20000000, LENGTH = 32K - 16
  MAILBOX (rw) : ORIGIN = 0x20007FF0, LENGTH = 16
}
_estack = ORIGIN(RAM) + LENGTH(RAM);
SECTIONS {
  .isr_vector : { KEEP(*(.isr_vector)) } > FLASH
  .text : { *(.text*) *(.rodata*) } > FLASH
  .bss (NOLOAD) : { *(.bss*) *(COMMON) } > RAM
  .boot_mailbox (NOLOAD) : { . += 16; } > MAILBOX
}
