/* Linker script to configure memory regions. */
MEMORY
{
  FLASH (rx) : ORIGIN = 0x1c000, LENGTH = 0x23000 /* 112 kB is taken by S130, 144 kB for app. */
  RAM (rwx) : ORIGIN = 0x20002800, LENGTH = 0x1800 /* 10 kB is taken by S130, 6 kB for app. */
}
INCLUDE "gcc_nrf51_common.ld"
