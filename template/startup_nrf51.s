/* File: startup_ARMCM0.S
 * Purpose: startup file for Cortex-M0 devices. Should use with
 *   GCC for ARM Embedded Processors
 * Version: V1.2
 * Date: 15 Nov 2011
 *
 * Copyright (c) 2011, ARM Limited
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the ARM Limited nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ARM LIMITED BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
    .syntax unified
    .arch armv6-m

    .section .stack
    .align 3
#ifdef __STACK_SIZE
    .equ    Stack_Size, __STACK_SIZE
#else
    .equ    Stack_Size, 0xc00
#endif
    .globl    __StackTop
    .globl    __StackLimit
__StackLimit:
    .space    Stack_Size
    .size __StackLimit, . - __StackLimit
__StackTop:
    .size __StackTop, . - __StackTop

    .section .heap
    .align 3
#ifdef __HEAP_SIZE
    .equ    Heap_Size, __HEAP_SIZE
#else
    .equ    Heap_Size, 0x100
#endif
    .globl    __HeapBase
    .globl    __HeapLimit
__HeapBase:
    .space    Heap_Size
    .size __HeapBase, . - __HeapBase
__HeapLimit:
    .size __HeapLimit, . - __HeapLimit

    .section .isr_vector
    .align 2
    .globl __Vectors
__Vectors:
    .long    __StackTop            /* Top of Stack */
    .long    Reset_Handler         /* Reset Handler */
    .long    NMI_Handler           /* NMI Handler */
    .long    HardFault_Handler     /* Hard Fault Handler */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    SVC_Handler           /* SVCall Handler */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    PendSV_Handler        /* PendSV Handler */
    .long    SysTick_Handler       /* SysTick Handler */

    /* External interrupts */
    .long   POWER_CLOCK_IRQHandler /* POWER_CLOCK */
    .long   RADIO_IRQHandler       /* RADIO */
    .long   UART0_IRQHandler       /* UART0 */
    .long   SPI0_TWI0_IRQHandler   /* SPI0_TWI0 */
    .long   SPI1_TWI1_IRQHandler   /* SPI1_TWI1 */
    .long   0                      /* Reserved */
    .long   GPIOTE_IRQHandler      /* GPIOTE */
    .long   ADC_IRQHandler         /* ADC */
    .long   TIMER0_IRQHandler      /* TIMER0 */
    .long   TIMER1_IRQHandler      /* TIMER1 */
    .long   TIMER2_IRQHandler      /* TIMER2 */
    .long   RTC0_IRQHandler        /* RTC0 */
    .long   TEMP_IRQHandler        /* TEMP */
    .long   RNG_IRQHandler         /* RNG */
    .long   ECB_IRQHandler         /* ECB */
    .long   CCM_AAR_IRQHandler     /* CCM_AAR */
    .long   WDT_IRQHandler         /* WDT */
    .long   RTC1_IRQHandler        /* RTC1 */
    .long   QDEC_IRQHandler        /* QDEC */
    .long   0                      /* Reserved */
    .long   SWI0_IRQHandler        /* SWI0 */
    .long   SWI1_IRQHandler        /* SWI1 */
    .long   SWI2_IRQHandler        /* SWI2 */
    .long   SWI3_IRQHandler        /* SWI3 */
    .long   SWI4_IRQHandler        /* SWI4 */
    .long   SWI5_IRQHandler        /* SWI5 */
    .long   0                      /* Reserved */
    .long   0                      /* Reserved */
    .long   0                      /* Reserved */
    .long   0                      /* Reserved */
    .long   0                      /* Reserved */
    .long   0                      /* Reserved */

    .size    __Vectors, . - __Vectors

    .text
    .thumb
    .thumb_func
    .align 2
    .globl    Reset_Handler
    .type    Reset_Handler, %function
Reset_Handler:
    .equ   NRF_POWER_RAMON_ADDRESS,            0x40000524
    .equ   NRF_POWER_RAMON_RAM1ON_ONMODE_Msk,  0x3
    ldr    r0, =NRF_POWER_RAMON_ADDRESS
    ldr    r2, [r0]
    movs   r1, #NRF_POWER_RAMON_RAM1ON_ONMODE_Msk
    orrs   r2, r1
    str    r2, [r0]

/*     Loop to copy data from read only memory to RAM. The ranges
 *      of copy from/to are specified by following symbols evaluated in
 *      linker script.
 *      __etext: End of code section, i.e., begin of data sections to copy from.
 *      __data_start__/__data_end__: RAM address range that data should be
 *      copied to. Both must be aligned to 4 bytes boundary.  */
    ldr    r1, =__etext
    ldr    r2, =__data_start__
    ldr    r3, =__data_end__

    subs    r3, r2
    ble    .flash_to_ram_loop_end

    movs    r4, 0
.flash_to_ram_loop:
    ldr    r0, [r1,r4]
    str    r0, [r2,r4]
    adds    r4, 4
    cmp    r4, r3
    blt    .flash_to_ram_loop
.flash_to_ram_loop_end:
    ldr    r0, =SystemInit
    blx    r0
    ldr    r0, =_start
    bx     r0
    .pool
    .size Reset_Handler, . - Reset_Handler

/*    Macro to define default handlers. Default handler
 *    will be weak symbol and just dead loops. They can be
 *    overwritten by other handlers */
    .macro    def_default_handler    handler_name
    .align 1
    .thumb_func
    .weak    \handler_name
    .type    \handler_name, %function
\handler_name :
    b    .
    .size    \handler_name, . - \handler_name
    .endm

    def_default_handler    NMI_Handler
    def_default_handler    HardFault_Handler
    def_default_handler    SVC_Handler
    def_default_handler    PendSV_Handler
    def_default_handler    SysTick_Handler
    def_default_handler    Default_Handler
    def_default_handler    POWER_CLOCK_IRQHandler
    def_default_handler    RADIO_IRQHandler
    def_default_handler    UART0_IRQHandler
    def_default_handler    SPI0_TWI0_IRQHandler
    def_default_handler    SPI1_TWI1_IRQHandler
    def_default_handler    GPIOTE_IRQHandler
    def_default_handler    ADC_IRQHandler
    def_default_handler    TIMER0_IRQHandler
    def_default_handler    TIMER1_IRQHandler
    def_default_handler    TIMER2_IRQHandler
    def_default_handler    RTC0_IRQHandler
    def_default_handler    TEMP_IRQHandler
    def_default_handler    RNG_IRQHandler
    def_default_handler    ECB_IRQHandler
    def_default_handler    CCM_AAR_IRQHandler
    def_default_handler    WDT_IRQHandler
    def_default_handler    RTC1_IRQHandler
    def_default_handler    QDEC_IRQHandler
    def_default_handler    SWI0_IRQHandler
    def_default_handler    SWI1_IRQHandler
    def_default_handler    SWI2_IRQHandler
    def_default_handler    SWI3_IRQHandler
    def_default_handler    SWI4_IRQHandler
    def_default_handler    SWI5_IRQHandler

    .weak    DEF_IRQHandler
    .set    DEF_IRQHandler, Default_Handler

    .end
