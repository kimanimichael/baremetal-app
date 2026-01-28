/* bootloader.c */
#include <inttypes.h>
#include "uart.h"
#include "stm32f4xx.h"
#include "comms.h"

#define BOOTLOADER_SIZE 0x8000U
#define MAIN_APP_START_ADDRESS 0x08000000U + BOOTLOADER_SIZE

//Base GPIOD register 0x4002 0C00 + offset 0x00 to find GPIOD_MODER
#define GPIOD_MODER (*((unsigned int *)(0x40020C00)))

//Base GPIOD register 0x4002 0C00 + offset 0x24 to find GPIOD_AFRH
#define GPIOD_AFRH (*((unsigned int *)(0x40020C24)))

#define SYSTICK_FREQ 1000U
#define CPU_FREQ 84000000U

extern int _stext;
extern int _data;
extern int _etext;
extern int _edata;
extern int _sbss;
extern int _ebss;

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);

// Breaks as it causes excedding of allocated size
// const uint8_t data[0x8000] = {0};

int main(void);
void SysTick_Handler(void);

volatile uint32_t l_tickrCtr;

void SysTick_Handler(void)
{
    ++l_tickrCtr;
}

static uint32_t system_ticker(void)
{
    uint32_t tickrCtr;

    __disable_irq();
    tickrCtr = l_tickrCtr;
    __enable_irq();

    return tickrCtr;
}

void system_delay(uint32_t ticks)
{
    uint32_t start = system_ticker();
    while ((system_ticker() - start) < ticks) {
        /* code */

    }


}

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

unsigned int* vectors[] __attribute__((section(".vectors"))) = {
    (unsigned int*)0x20030000,  //Pointer to the top of our stack memory
    (unsigned int*)main,  // Pointer to our reset handler - also our startup code
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
    (unsigned int*)usart3_isr,  //USART3 ISR
};

int main(void)
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

    SysTick_Config(CPU_FREQ / SYSTICK_FREQ);
    uart_gpio_setup();
    uart_setup();
    comms_setup();

    comms_packet_t dummy_packet = {
        .length = 9,
        .data = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
        .crc = 0
    };

    comms_packet_t packet = {
        .length = 9,
        .data = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
        .crc = 0
    };
    dummy_packet.crc = comms_compute_crc(&dummy_packet);
    packet.crc = comms_compute_crc(&packet);

    /* Not Reached */
    while (true) {
        comms_update();
        comms_write_packet(&packet);
        system_delay(500U);
    }

    jump_to_application();
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
