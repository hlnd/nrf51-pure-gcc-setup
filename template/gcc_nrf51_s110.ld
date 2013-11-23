/* Linker script to configure memory regions. */
MEMORY
{
  FLASH (rx) : ORIGIN = 0x14000, LENGTH = 0x26000 /* 128k, 128k is taken by S110 */
  RAM (rwx) : ORIGIN = 0x20002000, LENGTH = 0x2000 /* 8k, 8k is taken by S110 */
}
INCLUDE "gcc_nrf51_common.ld"
