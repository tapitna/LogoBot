/*Puertos
*	PORTC	

*	pin23	PC0	HC165 CLOCK	O	pin2  tranparent
*	pin24	PC1	HC165 PLOAD	O	pin1  blue
*	pin25	PC2 	HC165 Q7	I	pin9  red
*	--pin26	PC3 	HC165 CLOCK Ena	O	pin15
*/


#define HC165_PORT   PORTC
#define HC165_DDR    DDRC
			     // IC	atmega
#define HC165_CLOCK PC1      //pin2	pin23
#define HC165_PLOAD PC2      //pin1	pin24
#define HC165_DATAPIN PC3    //pin9	pin35

#include <avr/io.h>
#include <util/delay.h>

//#define PORTCMASK 0b11011100
#define CLOCK	PC0
#define PLOAD PC1
#define DATAPIN	PC2
//#define DATA_WIDTH 9
//#define CLOCKENA	PC3

#define PORTCMASK ((0<<DATAPIN)|(1<<CLOCK)|(1<<PLOAD))
#define PULSE_WIDTH_USEC 5
#define DATA_WIDTH 8

