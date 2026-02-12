#ifndef BAREMETAL_APP_SYSTEM_H
#define BAREMETAL_APP_SYSTEM_H

#include "stm32f4xx.h"

void SysTick_Handler(void);

uint32_t system_ticker(void);

void system_delay(uint32_t ticks);

#endif //BAREMETAL_APP_SYSTEM_H
