#include "system.h"

#define SYS_TICK_FREQ 1000U
#define CPU_FREQ 84000000U

volatile uint32_t l_tickrCtr;

void SysTick_Handler(void)
{
    ++l_tickrCtr;
}

uint32_t system_ticker(void)
{
    __disable_irq();
    const uint32_t tickrCtr = l_tickrCtr;
    __enable_irq();

    return tickrCtr;
}

void system_delay(const uint32_t ticks)
{
    const uint32_t start = system_ticker();
    while ((system_ticker() - start) < ticks) {
        /* code */

    }


}

void system_setup(void)
{
    SystemCoreClockUpdate();

    SysTick_Config(CPU_FREQ / SYS_TICK_FREQ);
}

void system_teardown()
{
    SysTick->CTRL = 0;   // Disable SysTick (counter + interrupt)
    SysTick->LOAD = 0;   // Clear reload register
    SysTick->VAL  = 0;   // Clear current value register

    NVIC_ClearPendingIRQ(SysTick_IRQn);
}
