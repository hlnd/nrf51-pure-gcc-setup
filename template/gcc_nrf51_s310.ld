/* Linker script to configure memory regions. */
MEMORY
{
  FLASH (rx) : ORIGIN = 0x20000, LENGTH = 0x20000 /* 128 kB is taken by S310, 128 kB for app.  */
  RAM (rwx) : ORIGIN = 0x20002800, LENGTH = 0x1800 /* 10 kB is taken by S310, 6 kB for app. */
}
INCLUDE "gcc_nrf51_common.ld"
