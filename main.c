#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include "I2C_slave.h"
#include "uart.h"



// buffer used to convert integer to string
char buffer[3];

void init_uart(uint16_t baudrate){

	uint16_t UBRR_val = (F_CPU/16)/(baudrate-1);

	UBRR0H = UBRR_val >> 8;
	UBRR0L = UBRR_val;

	UCSR0B |= (1<<TXEN0) | (1<<RXEN0) | (1<<RXCIE0); // UART TX (Transmit - senden) einschalten
	UCSR0C |= (1<<USBS0) | (3<<UCSZ00); //Modus Asynchron 8N1 (8 Datenbits, No Parity, 1 Stopbit)
}

/*
void uart_putc(unsigned char c){

	while(!(UCSR0A & (1<<UDRE0))); // wait until sending is possible
	UDR0 = c; // output character saved in c
}

void uart_puts(char *s){
	while(*s){
		uart_putc(*s);
		s++;
	}
}
FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);
*/
int main(void){
	
	init_uart(57600);
	I2C_init(0x4); // initalize as slave with address 0x4
	
	// allow interrupts
	sei();
	uart_puts("AVR started0\n\r");
	_delay_ms(1000);
	while(1){
		// convert receiver buffer index 0 to character array and send it via UART

		/*uart_puts(" ");
		itoa(status[0], buffer, 10);
		uart_puts(buffer);
		_delay_ms(100);
		uart_puts(" ");
		itoa(status[1], buffer, 10);
		uart_puts(buffer);
		_delay_ms(100);
		uart_puts(" ");
*/		if (status[0]==1){
			status[0]=0;
			itoa(status[1], buffer, 10);
			uart_puts(buffer);
			_delay_ms(100);
			uart_puts("\n\r");
		}
/*		itoa(status[3], buffer, 10);
		uart_puts(buffer);
		_delay_ms(100);
		uart_puts(" ");
		itoa(status[4], buffer, 10);
		uart_puts(buffer);
		_delay_ms(100);
		uart_puts(" ");*/
		//itoa(rxbuffer[0], buffer, 10);
		//uart_puts(buffer);
		//_delay_ms(1000);
		//uart_puts("\n\r");
	}
	
	return 0;
}
