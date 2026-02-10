#include "bl-flash.h"

#include "stm32f429xx.h"

#define MAIN_APP_SECTOR_START (2)
#define MAIN_APP_SECTOR_END (9)

void bl_flash_init(void)
{

}

void bl_flash_erase_main_application(void)
{
    __disable_irq();
    // unlock flash
    FLASH->KEYR = 0x45670123;
    FLASH->KEYR = 0xCDEF89AB;

    for (uint8_t sector =  MAIN_APP_SECTOR_START; sector <= MAIN_APP_SECTOR_END; sector++) {
        while (FLASH->SR & FLASH_SR_BSY) {}

        FLASH->CR |= FLASH_CR_PSIZE_1;

        FLASH->CR |= FLASH_CR_SER;

        FLASH->CR &= ~(FLASH_CR_SNB_Msk);
        FLASH->CR |= sector << FLASH_CR_SNB_Pos;

        FLASH->CR |= FLASH_CR_STRT;

        while (FLASH->SR & FLASH_SR_BSY) {}

        FLASH->CR &= ~(FLASH_CR_SER);

    }

    // lock flash
    FLASH->CR |= FLASH_CR_LOCK;

    __enable_irq();
}

void bl_flash_write(const uint32_t address, const uint8_t* data, const uint32_t length)
{
    __disable_irq();
    // unlock flash
    FLASH->KEYR = 0x45670123;
    FLASH->KEYR = 0xCDEF89AB;

    for (uint32_t i = 0; i < length; i++) {
        // set max parallelism size to x8
        FLASH->CR &= ~(FLASH_CR_PSIZE_Msk);
        // set programing bit
        FLASH->CR |= FLASH_CR_PG;
        // write data to address
        *(volatile uint8_t*)(address + i) = data[i];
        // clear programming bit
        FLASH->CR &= ~(FLASH_CR_PG);
    }

    // lock flash
    FLASH->CR |= FLASH_CR_LOCK;

    __enable_irq();
}
