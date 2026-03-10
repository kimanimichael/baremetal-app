/* bootloader.c */
#include "uart.h"
#include "stm32f4xx.h"
#include "comms.h"
#include "simple_timer.h"
#include "bootloader.h"
#include "system.h"
#include "bl-flash.h"

#define BOOTLOADER_SIZE 0x8000U
#define MAIN_APP_START_ADDRESS (0x08000000U + BOOTLOADER_SIZE)
#define MAX_FW_LENGTH (0x200000 - BOOTLOADER_SIZE)

//Base GPIOD register 0x4002 0C00 + offset 0x00 to find GPIOD_MODER
#define GPIOD_MODER (*((unsigned int *)(0x40020C00)))

//Base GPIOD register 0x4002 0C00 + offset 0x24 to find GPIOD_AFRH
#define GPIOD_AFRH (*((unsigned int *)(0x40020C24)))

// Breaks as it causes excedding of allocated size
// const uint8_t data[0x8000] = {0};

#define DEVICE_ID (0x42)

#define SYNC_SEQ_0 (0xC4)
#define SYNC_SEQ_1 (0x55)
#define SYNC_SEQ_2 (0x7e)
#define SYNC_SEQ_3 (0x10)

#define BOOTLOADER_TIMEOUT_MS (5000)

typedef enum bl_state_t {
    BL_State_Sync = 0,
    BL_State_WaitForUpdateReq,
    BL_State_DeviceIDReq,
    BL_State_DeviceIDRes,
    BL_State_FWLengthReq,
    BL_State_FWLengthRes,
    BL_State_EraseApplication,
    BL_State_Receive_Firmware,
    BL_State_Done
} bl_state_t;

bl_state_t state = BL_State_Sync;
uint32_t fw_length = 0;
uint32_t bytes_written = 0;
static uint8_t sync_seq[4] = {0};

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

static void uart_gpio_teardown()
{
    GPIOD_MODER &= ~(0b01 << 17);
    GPIOD_MODER &= ~(0b01 << 16);

    GPIOD_MODER &= ~(0b01 << 19);
    GPIOD_MODER &= ~(0b01 << 18);

    GPIOD_AFRH &= ~((0b01 << 0) | (0b01 << 1) | (0b01 << 2));
    GPIOD_AFRH &= ~(0b01 << 3);

    GPIOD_AFRH &= ~((0b01 << 4) | (0b01 << 5) | (0b01 << 6));
    GPIOD_AFRH &= ~(0b01 << 7);

    RCC->AHB1ENR &= ~(0b01 << 3);
}

static void jump_to_application()
{
    typedef void (*void_fn)(void);

    uint32_t* reset_vector_entry = (uint32_t*)(MAIN_APP_START_ADDRESS + 4U);
    uint32_t* reset_vector = (uint32_t*)(*reset_vector_entry);
    void_fn jump_fn = reset_vector;

    jump_fn();
}

static comms_packet_t packet;
static simple_timer_t timer;

static void boot_loading_fail()
{
    comms_create_single_byte_packet(&packet, BL_PACKET_NACK_DATA0);
    comms_write_packet(&packet);
    simple_timer_reset(&timer);
    state = BL_State_Done;
}

static void check_for_timeout(void)
{
    if (simple_timer_has_elapsed(&timer)) {
        boot_loading_fail();
    }
}

static bool is_device_id_packet(comms_packet_t const* data_packet)
{
    if (data_packet->length == 2 && data_packet->data[0] == BL_PACKET_DEVICE_ID_RES_DATA0) {
        for (uint8_t i = 2; i < PACKET_DATA_LENGTH; i++) {
            if (data_packet->data[i] != 0xff) {
                return false;
            }
        }
        return true;
    }
    return false;
}

static bool is_fw_length_packet(comms_packet_t const* data_packet)
{
    if (data_packet->length == 5 && data_packet->data[0] == BL_PACKET_FW_LENGTH_RES_DATA0) {
        for (uint8_t i = 5; i < PACKET_DATA_LENGTH; i++) {
            if (data_packet->data[i] != 0xff) {
                return false;
            }
        }
        return true;
    }
    return false;
}

int main(void)
{
    uart_gpio_setup();
    uart_setup();
    comms_setup();


    simple_timer_setup(&timer, BOOTLOADER_TIMEOUT_MS, false);

    while (state != BL_State_Done) {
        if (state == BL_State_Sync) {
            if (uart_data_available()) {
                sync_seq[0] = sync_seq[1];
                sync_seq[1] = sync_seq[2];
                sync_seq[2] = sync_seq[3];
                sync_seq[3] = uart_read_byte();

                bool is_match = sync_seq[0] == SYNC_SEQ_0;
                is_match &= sync_seq[1] == SYNC_SEQ_1;
                is_match &= sync_seq[2] == SYNC_SEQ_2;
                is_match &= sync_seq[3] == SYNC_SEQ_3;

                if (is_match) {
                    comms_create_single_byte_packet(&packet, BL_PACKET_SYNC_OBSERVED_DATA0);
                    comms_write_packet(&packet);
                    simple_timer_reset(&timer);
                    state = BL_State_WaitForUpdateReq;
                } else {
                    check_for_timeout();
                }
            } else {
                check_for_timeout();
            }
            continue;
        }

        comms_update();

        switch (state) {
            case BL_State_WaitForUpdateReq: {
                if (comms_packets_available()) {
                    comms_read_packet(&packet);

                    if (comms_is_single_byte_packet(&packet, BL_PACKET_FW_UPDATE_REQ_DATA0)) {
                        simple_timer_reset(&timer);
                        comms_create_single_byte_packet(&packet, BL_PACKET_FW_UPDATE_RES_DATA0);
                        comms_write_packet(&packet);
                        state = BL_State_DeviceIDReq;
                    } else {
                        boot_loading_fail();
                    }

                } else {
                    check_for_timeout();
                }
            }
            break;
            case BL_State_DeviceIDReq: {
                simple_timer_reset(&timer);
                comms_create_single_byte_packet(&packet, BL_PACKET_DEVICE_ID_REQ_DATA0);
                comms_write_packet(&packet);
                state = BL_State_DeviceIDRes;
            }
            break;
            case BL_State_DeviceIDRes: {
                if (comms_packets_available()) {
                    comms_read_packet(&packet);
                    if (is_device_id_packet(&packet) && packet.data[1] == DEVICE_ID) {
                        simple_timer_reset(&timer);
                        state = BL_State_FWLengthReq;
                    } else {
                        boot_loading_fail();
                    }
                } else {
                    check_for_timeout();
                }
            }
            break;
            case BL_State_FWLengthReq: {
                simple_timer_reset(&timer);
                comms_create_single_byte_packet(&packet, BL_PACKET_FW_LENGTH_REQ_DATA0);
                comms_write_packet(&packet);
                state = BL_State_FWLengthRes;
            }
            break;
            case BL_State_FWLengthRes: {
                if (comms_packets_available()) {
                    comms_read_packet(&packet);
                    fw_length = (
                                    packet.data[1]       |
                                    packet.data[2] << 8  |
                                    packet.data[3] << 16 |
                                    packet.data[4] << 24
                                );
                    if (is_fw_length_packet(&packet) && fw_length <= MAX_FW_LENGTH) {
                        state = BL_State_EraseApplication;
                    } else {
                        boot_loading_fail();
                    }
                } else {
                    check_for_timeout();
                }
            }
            break;
            case BL_State_EraseApplication: {
                bl_flash_erase_main_application();
                simple_timer_reset(&timer);
                state = BL_State_Receive_Firmware;
            }
            break;
            case BL_State_Receive_Firmware: {
                if (comms_packets_available()) {
                    comms_read_packet(&packet);
                    simple_timer_reset(&timer);

                    const uint8_t packet_length = (packet.length & 0x0F) + 1;
                    bl_flash_write(MAIN_APP_START_ADDRESS + bytes_written, packet.data, packet_length);
                    bytes_written += packet_length;

                    if (bytes_written >= fw_length) {
                        state = BL_State_Done;
                    }
                } else {
                    check_for_timeout();
                }
            }
            break;
            default: {
                state = BL_State_Sync;
            }
            break;
        }
    }
    system_delay(300);
    uart_teardown();
    uart_gpio_teardown();
    system_teardown();

    jump_to_application();
}
