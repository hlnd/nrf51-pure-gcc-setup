/* Linker script to configure memory regions. */
MEMORY
{
  FLASH (rx) : ORIGIN = 0x18000, LENGTH = 0x27000 /* 96 kB is taken by S120, 160 kB for app. */
  RAM (rwx) : ORIGIN = 0x20002800, LENGTH = 0x1800 /* 10 kB is taken by S120, 6 kB for app. */
}
INCLUDE "gcc_nrf51_common.ld"
