/* Linker script to configure memory regions. */
MEMORY
{
  FLASH (rx) : ORIGIN = 0x15000, LENGTH = 0x2B000 /* 84 kB is taken by S110, 172 kB for app. */
  RAM (rwx) : ORIGIN = 0x20002000, LENGTH = 0x2000 /* 8 kB is taken by S110,8 kB for app. */
}
INCLUDE "gcc_nrf51_common.ld"
