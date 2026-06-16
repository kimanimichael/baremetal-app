#include "firmware_info.h"

__attribute__((section(".firmware_info")))
firmware_info_t firmware_info = {
    .sentinel   = FW_INFO_SENTINEL,
    .device_id  = DEVICE_ID,
    .version    = 0x00000001,
    .length     = 0xffffffff, // This will be filled in by the bootloader
    .reserved0  = 0xffffffff,
    .reserved1  = 0xffffffff,
    .reserved2  = 0xffffffff,
    .reserved3  = 0xffffffff,
    .reserved4  = 0xffffffff,
    .crc32      = 0xffffffff// This will be filled in by the bootloader
};
