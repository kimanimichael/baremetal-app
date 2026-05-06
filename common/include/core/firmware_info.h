#ifndef CORE_FIRMWARE_INFO_H
#define CORE_FIRMWARE_INFO_H

#include "stdint.h"

#define BOOTLOADER_SIZE                     (0x8000U)
#define MAIN_APP_START_ADDRESS              (0x08000000U + BOOTLOADER_SIZE)
#define MAX_FW_LENGTH                       (0x200000 - BOOTLOADER_SIZE)
#define DEVICE_ID                           (0x42)

#define FW_INFO_SENTINEL                    (0xDEADC0DE)
#define FW_INFO_ADDRESS                     (MAIN_APP_START_ADDRESS +sizeof(vectors))
#define FW_INFO_VALIDATE_FROM               (FW_INFO_ADDRESS + sizeof(firmware_info_t))
#define FW_INFO_VALIDATE_LENGTH(fw_length)  (fw_length - (sizeof(vectors) + sizeof(firmware_info_t)))

typedef struct firmware_info_t {
    uint32_t sentinel;
    uint32_t device_id;
    uint32_t version;
    uint32_t length;
    uint32_t reserved0;
    uint32_t reserved1;
    uint32_t reserved2;
    uint32_t reserved3;
    uint32_t reserved4;
    uint32_t crc32;
} firmware_info_t;

#endif
