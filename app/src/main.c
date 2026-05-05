#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "bsp.h"
#include "uart.h"

int __io_putchar(int data)
{
    uart_write((uint8_t*)&data, 1);
    return data;
}

__attribute__((used)) const uint8_t padding[8] = {0};

int main()
{
    BSP_init();
    uart_setup();

    uint32_t start = BSP_Tickr();
    while (true) {
        if ((BSP_Tickr() - start) > 2000) {
            BSP_blueLedToggle();
            start = BSP_Tickr();
        }
        BSP_Delay(10);
    }
}

