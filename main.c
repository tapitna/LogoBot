/* LogoBot
 * This program drives robot's motors. 
 * Based on the ATMEGA328
 * It use i2c to receive orders from the webserver running on ESP8266 07 type.
 * Based on the order received it will action the corresponding motor 
 * Orders have two component: the direction and how to move.
 * Direction can be forward, back, left or right. 
 * Type is: 1 step or continue until other order is received
 * hardware to control the motor is the Adafruit Motor/Stepper/Servo Shield for Arduino kit - v1.2
 * 
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with 
  this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

 * 
 * 
 */ 
#ifndef F_CPU
#define F_CPU 16000000UL
#endif


#ifdef SERIALDEBUG
#define DBG 1
#else
#define DBG 0
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include "I2C_slave.h"
#include "uart.h"
//change the values in main.h as desired. 
//remember to use the same i2c address here and in the web server
#include "main.h"  
//main file



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
	//these two values are defined in main.h
	init_uart(UARTSPEED);
	I2C_init(i2cADDR); 
	
	// allow interrupts
	sei();
	if (DBG ==1 ){
		uart_puts("AVR started0\n\r");
	}
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
			if (DBG==1) {uart_puts(buffer);}
			_delay_ms(100);
			if (DBG==1) {uart_puts("\n\r");}
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
