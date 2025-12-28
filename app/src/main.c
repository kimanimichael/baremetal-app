#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "bsp.h"
#include "uart.h"

int __io_putchar(int data) {
	uart_write((uint8_t *)&data, 1);
	return data;
}

int main(){
	BSP_init();
	uart_setup();
	uint8_t read_char = 0;

	uint32_t start = BSP_Tickr();
	while (true){
		if ((BSP_Tickr() - start) > 1000) {
			BSP_blueLedToggle();
			printf("LED toggled\n\r");
			printf("Float test: %.2f\n\r", 3.45);
			start = BSP_Tickr();
		}
		if (uart_data_available()) {
			uart_read(&read_char, 1);
			printf("%c\n\r", read_char + 1);
		}
	}
}

