#ifndef BL_FLASH_H
#define BL_FLASH_H

#include <stdint.h>

void bl_flash_init(void);

void bl_flash_erase_main_application(void);

void bl_flash_write(uint32_t address, const uint8_t* data, uint32_t length);

#endif
