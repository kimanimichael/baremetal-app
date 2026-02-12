#include "bootloader.h"
#include "startup.h"
#include "system.h"
#include "uart.h"

#define SYS_TICK_FREQ 1000U
#define CPU_FREQ 84000000U

unsigned int* vectors[] __attribute__((section(".vectors"))) = {
    (unsigned int*)0x20030000,  //Pointer to the top of our stack memory
    (unsigned int*)Reset_Handler,  // Pointer to our reset handler - also our startup code
    (unsigned int*)NMI_Handler,  //NMI
    (unsigned int*)HardFault_Handler,  //HardFault
    (unsigned int*)MemManage_Handler,  //MemManage
    (unsigned int*)BusFault_Handler,  //BusFault
    (unsigned int*)UsageFault_Handler,  //MemManage
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    (unsigned int*)SysTick_Handler,  //SysTick_Handler
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    (unsigned int*)usart3_isr,  //UART3 ISR
};

void Reset_Handler(void)
{
    SystemInit();
    SystemCoreClockUpdate();

    uint32_t* init_values_ptr = (uint32_t*) &_etext;
    uint32_t* data_ptr = (uint32_t*) &_data;

    if (data_ptr != init_values_ptr) {
        for (; data_ptr < (uint32_t*) &_edata;) {
            *data_ptr++ = *init_values_ptr++;
        }
    }

    for (uint32_t* bss_ptr = (uint32_t*) &_sbss; bss_ptr < (uint32_t*)&_ebss;) {
        *bss_ptr++ = 0;
    }

    SystemCoreClockUpdate();

    SysTick_Config(CPU_FREQ / SYS_TICK_FREQ);
    main();
}

void HardFault_Handler(void)
{
#ifdef DEBUG
    __BKPT(0);
#endif
    while (1) {
        /* code */

    }

}

void NMI_Handler(void)
{
    while (1) {
        /* code */

    }

}

void MemManage_Handler(void)
{
#ifdef DEBUG
    __BKPT(0);
#endif
    while (1) {
        /* code */

    }

}

void BusFault_Handler(void)
{
#ifdef DEBUG
    __BKPT(0);
#endif
    while (1) {
        /* code */
    }

}

void UsageFault_Handler(void)
{
#ifdef DEBUG
    __BKPT(0);
#endif
    while (1) {
        /* code */
    }

}
