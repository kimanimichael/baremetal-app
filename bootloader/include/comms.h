#ifndef COMMS_H
#define COMMS_H

#include <stdbool.h>
#include <stdint.h>

#define PACKET_LENGTH_BYTES 1
#define PACKET_DATA_LENGTH (16)
#define PACKET_CRC_BYTES (1)
#define PACKET_LENGTH (PACKET_LENGTH_BYTES + PACKET_DATA_LENGTH + PACKET_CRC_BYTES)

#define PACKET_ACK_DATA0 (0x15)
#define PACKET_RETX_DATA0 (0x19)

#define BL_PACKET_SYNC_OBSERVED_DATA0 (0X20)
#define BL_PACKET_FW_UPDATE_REQ_DATA0 (0X31)
#define BL_PACKET_FW_UPDATE_RES_DATA0 (0x37)
#define BL_PACKET_DEVICE_ID_REQ_DATA0 (0x3C)
#define BL_PACKET_DEVICE_ID_RES_DATA0 (0x3F)
#define BL_PACKET_FW_LENGTH_REQ_DATA0 (0x42)
#define BL_PACKET_FW_LENGTH_RES_DATA0 (0x45)
#define BL_PACKET_READY_FOR_DATA_DATA0 (0x48)
#define BL_PACKET_UPDATE_SUCCESS_DATA0 (0x54)
#define BL_PACKET_NACK_DATA0 (0x59)

typedef enum comms_state {
    Comms_State_Length = 0,
    Comms_State_Data,
    Comms_State_CRC
} comms_state_t;

typedef struct comms_packet_t {
    uint8_t length;
    uint8_t data[PACKET_DATA_LENGTH];
    uint8_t crc;
} comms_packet_t;

bool comms_packets_available(void);

void comms_setup(void);

void comms_update(void);

void comms_write_packet(comms_packet_t const* packet);

void comms_read_packet(comms_packet_t* packet);

bool comms_is_single_byte_packet(comms_packet_t const* packet, uint8_t byte);

uint8_t comms_compute_crc(comms_packet_t* packet);

void comms_create_single_byte_packet(comms_packet_t* packet, uint8_t byte);

#endif
