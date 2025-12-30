#include "uart.h"
#include "stm32f429xx.h"

#define UART_FREQUENCY 21000000
#define UART_BAUD 115200
#define UART_DIVISOR 8 * (2 - 0)

static uint8_t data_buffer = 0U;
static bool data_available = false;

void usart3_isr(void) {
  const bool overrun_occured = (USART3->SR & USART_SR_ORE) !=0;
  const bool received_data = (USART3->SR & USART_SR_RXNE) !=0;

  if (received_data || overrun_occured) {
    data_buffer = (uint8_t)USART3->DR;
    data_available = true;
  }
}

void uart_setup(void) {
    /* Enable USART3 clock */
    RCC->APB1ENR |= (0b01 << 18);
    /* No DMA, no flow control, disable smart card mode, no half-duplex selection, normal power mode etc. */
    USART3->CR3 = 0;

    /** Reset the UART control register 1
     * Bit 12 = 0 : 1 Start bit, 8 Data bits, n Stop bit
     * Bit 10 = 0 : No parity bit
     * 
     */
    USART3->CR1 &= ~USART_CR1_UE;
    /* Set baud rate to 115200 
     * baud = fck /(8 * (2- OVER8)* usart_div)
     * usart_div = fck/(8 * (2- OVER8) * baud)
     * uint32_t usart_div = UART_FREQUENCY/(8 * (2- 0)*UART_BAUD) = 11 = 0xb
     * uint32_t usart_frac = (UART_FREQUENCY%(8 * (2- 0)*UART_BAUD))*16 = 6.2912 ~ 7 = 0x7 - Basically the remainder of usart_div above *16 rounded up
     * uint32_t brr = 0xb7
    */
    USART3->BRR |= 0xb7;

    NVIC_EnableIRQ(USART3_IRQn);

    /* Configure Uart Control Register 1 
     * USART enable
     * Transmitter enable
     * Receiver enable
     * Receiver interrupts enable
    */
    USART3->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
}

void uart_write(uint8_t *data, const uint32_t length) {
    for (int i =0; i < length; i++) {
        uart_write_byte(data[i]);
    }

}

void uart_write_byte(uint8_t data) {
    /* Wait until the data has been transferred into the shift register. */
	while ((USART3->SR & USART_SR_TXE) == 0);
    USART3->DR = data;
}

uint32_t uart_read(uint8_t *data, const uint32_t length) {
    if (length > 0 && data_available) {
        *data = data_buffer;
        data_available = false;
        return 1;
    }
    return 0;
}

uint8_t uart_read_byte(void) {
    data_available = false;
    return data_buffer;
}

bool uart_data_available(void) {
    return data_available;
}

