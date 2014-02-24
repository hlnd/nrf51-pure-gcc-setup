/* Linker script to configure memory regions. */
MEMORY
{
  FLASH (rx) : ORIGIN = 0x0, LENGTH = 0x40000 /* 256k */
  RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x4000 /* 16k */
}
INCLUDE "gcc_nrf51_common.ld"
