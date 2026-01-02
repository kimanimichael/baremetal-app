#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdint.h>
#include <stdbool.h>

typedef struct ring_buffer_t {
    uint8_t *buffer;
    uint32_t mask;
    uint32_t read_index;
    uint32_t write_index;
    uint32_t size;
} ring_buffer_t;

/**
 * @brief initialize ring buffer
 * @param ring_buffer associated ring buffer
 * @param buffer actual buffer where data is stored
 * @param size size of ring buffer (must be power of 2)
 */
void ring_buffer_init(ring_buffer_t *ring_buffer, uint8_t *buffer, uint32_t size);
/**
 * @brief check if the ring buffer is empty
 * @param ring_buffer associated ring buffer
 * @return true if the buffer is empty
 */
bool ring_buffer_is_empty(const ring_buffer_t * ring_buffer);
/**
 * @brief read from the ring buffer
 * @param ring_buffer associated ring buffer
 * @param byte byte to be read into
 * @return
 */
bool ring_buffer_read(ring_buffer_t *ring_buffer, uint8_t *byte);
/**
 * @brief write into the ring buffer
 * @param ring_buffer associated ring buffer
 * @param byte byte to write into @param ring_buffer
 * @return
 */
bool ring_buffer_write(ring_buffer_t *ring_buffer, uint8_t byte);

#endif
