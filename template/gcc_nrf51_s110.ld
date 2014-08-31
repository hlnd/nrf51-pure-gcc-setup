/* Linker script to configure memory regions. */
MEMORY
{
  FLASH (rx) : ORIGIN = 0x0 + 88K, LENGTH = 256K - 88K /* 88 kB is taken by S110, rest for app. */
  RAM (rwx) : ORIGIN = 0x20000000 + 8K, LENGTH = 16K - 8K /* 8 kB is taken by S110, 8 kB for app. */
}
INCLUDE "gcc_nrf51_common.ld"
