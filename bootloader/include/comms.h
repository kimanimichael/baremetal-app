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

void comms_write_packet(comms_packet_t const *packet);

void comms_read_packet(comms_packet_t *packet);

bool comms_is_single_byte_packet(comms_packet_t const *packet, uint8_t byte);

uint8_t comms_compute_crc(comms_packet_t *packet);

void comms_packet_cpy(comms_packet_t *dest, const comms_packet_t *src);

void comms_create_single_byte_packet(comms_packet_t* packet, uint8_t byte);

#endif
