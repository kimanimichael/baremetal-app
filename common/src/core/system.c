#include "system.h"

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
