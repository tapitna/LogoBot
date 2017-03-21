/*
 * shiftdrv.c
 *
 * Created: 7/2/2014 10:41:49 PM
 *  Author: sixto
 */ 
/*Puertos
*	PORTC	

*	pin23	PC0	HC165 CLOCK	O	pin2  tranparent
*	pin24	PC1	HC165 PLOAD	O	pin1  blue
*	pin25	PC2 	HC165 Q7	I	pin9  red
*	--pin26	PC3 	HC165 CLOCK Ena	O	pin15
*/

#include "HC165_IN.h"

#include <string.h>
void shiftInitIN(void){
	//DDRC |= PORTCMASK;
	HC165_DDR |= (1 << CLOCK);
	HC165_DDR |= (1 << PLOAD);
	//input
	HC165_DDR &= ~(1 << DATAPIN);

	HC165_PORT &= ~(1 << CLOCK); //LOW
	HC165_PORT &= ~(1 << PLOAD); //LOW 
/*cargo entradas Q al registro*/
	//PORTC |= (1<<CLOCKENA); //HIGH
}

void shiftin(uint8_t *bytearray)
{
	
	//uint8_t bitVal=0;
	//char bytesVal=0;
	uint8_t i;

/* Inicializar 74HC165
* CLOCK LOW
*PLOAD HIGH
*/

	//parallel load to freeze the state of the data lines
	HC165_PORT &= ~(1 << PLOAD);	//LOW
	_delay_us(PULSE_WIDTH_USEC);
	HC165_PORT |= (1 << PLOAD);	//HIGH

	//PORTC &= ~(1<<CLOCKENA); //LOW

	
	 /* Loop to read each bit value from the serial out line
     * of the SN74HC165N.
    */
uint8_t currentbyte = 0;
for(i = 0; i <8; i++){


	//iterate through the bits in each registers
	currentbyte = 0;
	if (PINC & (1 << DATAPIN)){
	   currentbyte=1;
	}			

	memcpy(&bytearray[i], &currentbyte, 1);
	/* Set the corresponding bit in bytesVal.
        */
        //bytesVal |= (bitVal << ((DATA_WIDTH-1) - i));

        /* Pulse the Clock (rising edge shifts the next bit).
        */
	HC165_PORT |= (1 << CLOCK);  //HIGH
	_delay_us(PULSE_WIDTH_USEC);
	HC165_PORT &= ~(1 << CLOCK); // LOW
	_delay_us(PULSE_WIDTH_USEC);
    }
//return bytesVal;
//PORTC |= (1 << CLOCKENA);
}
uint8_t encoderIN(uint8_t encoderST, uint8_t *pulse)
{
//char asd[8];	

	//char bytesVal=0;
	uint8_t i;
/* Inicializar 74HC165
* CLOCK LOW
*PLOAD HIGH
*/
//parallel load to freeze the state of the data lines
HC165_PORT &= ~(1 << PLOAD);	//LOW
_delay_us(PULSE_WIDTH_USEC);
HC165_PORT |= (1 << PLOAD);	//HIGH

	 /* Loop to read each bit value from the serial out line
     * of the SN74HC165N.
    */
//uint8_t currentbit = 0;
for(i = 0; i <8; i++){
	//iterate through the bits in each registers
	//currentbit = 0;
	if (PINC & (1 << DATAPIN)){		//PIN is set
		//currentbit=1;
	   	if (encoderST & (1<<i) ){	
			//encoder is already 1m, then no change
		}else{
			pulse[i]=pulse[i]+1;	//incremenent the pulse counter for this sensor
			encoderST |= (1 << i);	//set the ecnoder to 1	
		}
	}else{
		encoderST &= ~(1 << i);	//set the ecnoder to 0, PIN is not set
	}		
	/* Pulse the Clock (rising edge shifts the next bit).
	       */
	HC165_PORT |= (1 << CLOCK);  //HIGH
	_delay_us(PULSE_WIDTH_USEC);
	HC165_PORT &= ~(1 << CLOCK); // LOW
	_delay_us(PULSE_WIDTH_USEC);
    }
return encoderST;
//PORTC |= (1 << CLOCKENA);
}
