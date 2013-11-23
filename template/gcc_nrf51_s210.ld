/* Linker script to configure memory regions. */
MEMORY
{
  FLASH (rx) : ORIGIN = 0xA000, LENGTH = 0x36000 /* 128k, 128k is taken by S110 */
  RAM (rwx) : ORIGIN = 0x20000800, LENGTH = 0x3800 /* 8k, 8k is taken by S110 */
}
INCLUDE "gcc_nrf51_common.ld"
