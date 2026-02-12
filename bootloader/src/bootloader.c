/* bootloader.c */
#include "uart.h"
#include "stm32f4xx.h"
#include "comms.h"
#include "simple_timer.h"
#include "bootloader.h"

#define BOOTLOADER_SIZE 0x8000U
#define MAIN_APP_START_ADDRESS (0x08000000U + BOOTLOADER_SIZE)

//Base GPIOD register 0x4002 0C00 + offset 0x00 to find GPIOD_MODER
#define GPIOD_MODER (*((unsigned int *)(0x40020C00)))

//Base GPIOD register 0x4002 0C00 + offset 0x24 to find GPIOD_AFRH
#define GPIOD_AFRH (*((unsigned int *)(0x40020C24)))

// Breaks as it causes excedding of allocated size
// const uint8_t data[0x8000] = {0};

static void uart_gpio_setup()
{
    //Bitwise OR the third bit of RCC_AHB1ENR with 1 to enable GPIOD_EN CLOCK
    RCC->AHB1ENR |= (0b01 << 3);

    //Bitwise OR the 17th bit of GPIOD_MODER with 1 - CONFIG PD8 as alternate function
    GPIOD_MODER |= (0b01 << 17);
    //Bitwise AND the 16th bit of GPIOD_MODER with 0 - CONFIG PD8 as alternate function
    GPIOD_MODER &= ~(0b01 << 16);

    //Bitwise OR the 19th bit of GPIOD_MODER with 1 - CONFIG PD9 as alternate function
    GPIOD_MODER |= (0b01 << 19);
    //Bitwise AND the 18th bit of GPIOD_MODER with 0 - CONFIG PD9 as alternate function
    GPIOD_MODER &= ~(0b01 << 18);

    // AF7, USART3TX = PD8
    GPIOD_AFRH |= (0b01 << 0) | (0b01 << 1) | (0b01 << 2);
    GPIOD_AFRH &= ~(0b01 << 3);
    // AF7, USART3RX = PD9
    GPIOD_AFRH |= (0b01 << 4) | (0b01 << 5) | (0b01 << 6);
    GPIOD_AFRH &= ~(0b01 << 7);
}

static void jump_to_application()
{
    typedef void (*void_fn)(void);

    uint32_t* reset_vector_entry = (uint32_t*)(MAIN_APP_START_ADDRESS + 4U);
    uint32_t* reset_vector = (uint32_t*)(*reset_vector_entry);
    void_fn jump_fn = reset_vector;

    jump_fn();
}

int main(void)
{
    uart_gpio_setup();
    uart_setup();
    comms_setup();

    simple_timer_t timer;
    simple_timer_setup(&timer, 1000, false);

    while (true) {
        if (simple_timer_has_elapsed(&timer))
        {
            static volatile int counter = 0;
            counter++;
        }
    }

    // jump_to_application();
}
