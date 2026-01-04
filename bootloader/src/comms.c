#include "comms.h"
#include "crc8.h"
#include "uart.h"

#include <assert.h>

#define PACKET_BUFFER_LENGTH (8)

static comms_packet_t ack_packet = {.length = 0, .data = {0}, .crc = 0};
static comms_packet_t last_transmitted_packet = {.length = 0, .data = {0}, .crc = 0};
static comms_packet_t rtx_packet = {.length = 0, .data = {0}, .crc = 0};
static comms_packet_t temporary_packet = {.length = 0, .data = {0}, .crc = 0};

static comms_packet_t packet_buffer[PACKET_BUFFER_LENGTH] = {0};
static uint8_t packet_buffer_read_index = 0;
static uint8_t packet_buffer_write_index = 0;
static uint8_t packet_buffer_mask = PACKET_BUFFER_LENGTH - 1;

static comms_state_t state = Comms_State_Length;
static uint8_t data_byte_count = 0;



bool comms_packets_available() {
    return packet_buffer_read_index != packet_buffer_write_index;
}

void comms_create_single_byte_packet(comms_packet_t* packet, uint8_t byte) {
    packet->length = 1;
    packet->data[0] = byte;
    for (int i = 1; i < PACKET_DATA_LENGTH; i++) {
        packet->data[i] = 0xff;
    }
    packet->crc = comms_compute_crc(packet);
}

void comms_setup(void) {
    comms_create_single_byte_packet(&ack_packet, PACKET_ACK_DATA0);
    comms_create_single_byte_packet(&rtx_packet, PACKET_RETX_DATA0);
}

void comms_update(void) {
    while (comms_packets_available()) {
        switch (state) {
            case Comms_State_Length:
                {
                    temporary_packet.length = uart_read_byte();
                    state = Comms_State_Data;
                }
                break;
            case Comms_State_Data:
                {
                    temporary_packet.data[data_byte_count++] = uart_read_byte();
                    if (data_byte_count >= PACKET_DATA_LENGTH) {
                        data_byte_count = 0;
                        state = Comms_State_CRC;
                    }
                }
                break;
            case Comms_State_CRC:
                {
                    temporary_packet.crc = uart_read_byte();

                    const uint8_t computed_crc = comms_compute_crc(&temporary_packet);
                    if (computed_crc != temporary_packet.crc) {
                        comms_write_packet(&rtx_packet);
                        state = Comms_State_Length;
                        break;
                    }

                    if (comms_is_single_byte_packet(&temporary_packet, PACKET_RETX_DATA0)) {
                        comms_write_packet(&last_transmitted_packet);
                        state = Comms_State_Length;
                        break;
                    }

                    if (comms_is_single_byte_packet(&temporary_packet, PACKET_ACK_DATA0)) {
                        state = Comms_State_Length;
                        break;
                    }

                    const uint8_t next_write_index = (packet_buffer_write_index + 1) & packet_buffer_mask;
                    if (next_write_index == packet_buffer_read_index) {
                        __asm__("BKPT 0");
                    }

                    comms_packet_cpy(&packet_buffer[packet_buffer_write_index], &temporary_packet);
                    packet_buffer_write_index = next_write_index;
                    comms_write_packet(&ack_packet);
                    state = Comms_State_Length;
                }
                break;
            default:
                break;
        }
    }
}

void comms_write_packet(comms_packet_t const *packet) {
    uart_write_byte(packet->length);
    for (int i = 0; i < PACKET_DATA_LENGTH; i++) {
        uart_write_byte(packet->data[i]);
    }
    uart_write_byte(packet->crc);
}

void comms_read_packet(comms_packet_t *packet) {
    if (!comms_packets_available()) {
        return;
    }

    comms_packet_cpy(packet, &packet_buffer[packet_buffer_read_index]);
    packet_buffer_read_index = (packet_buffer_read_index + 1) & packet_buffer_mask;
}

bool comms_is_single_byte_packet(comms_packet_t const *packet, const uint8_t byte) {
    for (uint8_t i = 1; i < PACKET_DATA_LENGTH; i++) {
        if (packet->data[i] != 0xff) {
            return false;
        }
    }
    return packet->length == 1 && packet->data[0] == byte;
}

uint8_t  comms_compute_crc(comms_packet_t *packet) {
    return calculate_crc8((uint8_t *)packet, PACKET_LENGTH - PACKET_CRC_BYTES);
}

void comms_packet_cpy(comms_packet_t *dest, const comms_packet_t *src) {
    dest->length = src->length;
    for (int i = 0; i < PACKET_DATA_LENGTH; i++) {
        dest->data[i] = src->data[i];
    }
    dest->crc = src->crc;
}
