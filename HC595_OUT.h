#include <stdio.h>
#include <avr/io.h>


#define HC595_PORT   PORTB
#define HC595_DDR    DDRB
#define HC595_DS_POS PB5      //(PB5)Data pin (DS) pin location		DIR_SER   
#define HC595_ST_CP_POS PB4      //(PB4) Store Clock (ST_CP) pin location RCK	DIR_LATCH 
#define HC595_SH_CP_POS PB0    //(PB3) Shift Clock (SH_CP) pin location SCK 	DIR_CLK   
/*
shield		ATMEGA
DIR_SER(D8) 	pin11 white
DIR_LATCH(D12) 	pin12 orange
DIR_CLK(D4)    	pin13 yellow
*/
extern void shiftInitOUT();
