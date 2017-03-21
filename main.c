#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "uart.h"
#include <stdlib.h>
#include "HC165_IN.h"
#include "HC595_OUT.h"
#include "I2C_slave.h"
#include "main.h"
#define UART_BAUD_RATE 57600	
//LEDS
#define LED_MSG DDRB
#define LED_PORT PORTB
#define LEDY PB0
#define LEDYON (LED_PORT|= (1<<LEDY) ); //on
#define LEDYOFF (LED_PORT&= ~(1<<LEDY)); //off
#define SENSORM3 7	//M3 sensor  (D0 position 6 on the hc165 byte)
#define SENSORM4 6  //M4 sensor	 (D1 position 7 on the hc165 byte)
#define SENSORM2_1 5 //down 
#define SENSORM2_2 4 //up
#define SENSORON 1
#define SENSOROFF 0
#define M3F	5	//motor 3 forward
#define M3B	6	//motor 3 back
#define M4F	7	//motor 4 forward
#define M4B 8	//motor 4 back
#define M2U 4
#define M2D 3

#define MSTOP 0
#define DON 1
#define DOFF 50

//WHEELS, tank track
#define	WB (WheelDir[M3F]|WheelDir[M4F]);	//forward
#define WF (WheelDir[M3B]|WheelDir[M4B]);	//backward
#define WL (WheelDir[M3F]|WheelDir[M4B]);					//right forward
#define WR (WheelDir[M3B]|WheelDir[M4F]);					//left forward
#define WRB WheelDir[M3B];					//right forward
#define WLB WheelDir[M4B];					//left forward
//ARM motor
#define AD WheelDir[M2U];					//arm up
#define AU WheelDir[M2D];					//arm down

#define INSTR_PER_US 16						// instructions per microsecond (depends on MCU clock, 16MHz current)
#define INSTR_PER_MS 16000					// instructions per millisecond (depends on MCU clock, 16MHz current)
#define MAX_RESP_TIME_MS 200				// timeout - max time to wait for low voltage drop (higher value increases measuring distance at the price of slower sampling)
#define DELAY_BETWEEN_TESTS_MS 20			// echo cancelling time between sampling

#define SONAR_TRIGGER_DDR DDRD
#define SONAR_TRIGGER_PORT PORTD
#define SONAR_TRIGGER_PIN PIND
//--define pin where SONAR sensor is connected
#define SONAR_TRIGGER_DQ 4	//PD4
//--
#define SONAR_TRIGGER_INPUT_MODE() SONAR_TRIGGER_DDR&=~(1<<SONAR_TRIGGER_DQ)
#define SONAR_TRIGGER_OUTPUT_MODE() SONAR_TRIGGER_DDR|=(1<<SONAR_TRIGGER_DQ)
#define SONAR_TRIGGER_LOW() SONAR_TRIGGER_PORT&=~(1<<SONAR_TRIGGER_DQ)
#define SONAR_TRIGGER_HIGH() SONAR_TRIGGER_PORT|=(1<<SONAR_TRIGGER_DQ)

volatile unsigned long result = 0;
volatile unsigned char up = 0;
volatile unsigned char running = 0;
volatile uint32_t timerCounter = 0;
volatile uint8_t PCOUNT1=0;
volatile uint8_t PCOUNT2=0;
volatile uint32_t PCOUNT3=0;
volatile uint8_t PCOUNT4=0;
SIGNAL(INT1_vect) 
{
        if (running) { //accept interrupts only when sonar was started

                if (up == 0) { // voltage rise, start time measurement
						PCOUNT1=1;
                        up = 1;
                        timerCounter = 0;
                        TCNT2 = 0; // reset timer counter 
                } else {
						PCOUNT2=1;
                        // voltage drop, stop time measurement
                        up = 0;
                        // convert from time to distance(millimeters): d = [ time_s * 340m/s ] / 2 = time_us/58
                        result = (timerCounter * 256 + TCNT2) / 58	; 
                        running = 0;
                }
        }
}

/**
        Sonar interfacing:
                1. Send high impulse to Trig input for minimum 10us
                2. Sonar automatically sends eight 40kHz inpulses
                3. Sonar rises high on Echo output and then after some time drops 
                   output to low (can take a while on long distances! - must include timeouts)
                4. Based on output time difference deltaT = lowT-highT calculate: 
                        distance = [ deltaT * sound_speed(340m/s) ] / 2
                5. Make a delay before starting the next cycle to compensate for late echoes
*/
// timer overflow interrupt, each time when timer value passes 255 value

SIGNAL(TIMER2_OVF_vect)
{
        if (up) {       // voltage rise was detected previously
        
                timerCounter++; // count the number of overflows
                // dont wait too long for the sonar end response, stop if time for measuring the distance exceeded limits
                uint32_t ticks = timerCounter * 256 + TCNT2;
				PCOUNT3=timerCounter;
                uint32_t max_ticks = (uint32_t)MAX_RESP_TIME_MS * INSTR_PER_MS; // this could be replaced with a value instead of multiplying

                if (ticks > max_ticks) {
		                PCOUNT4=1;			
                        // timeout
                        up = 0;          // stop counting timer values
                        running = 0; // ultrasound scan done
                        result = -1; // show that measurement failed with a timeout (could return max distance here if needed)
                }
        }
}

// generate an impulse for the Trig input (starts the sonar)
void sonar() {
        //PORTB = 0x00; // clear to zero for 1 us
		SONAR_TRIGGER_LOW();
        _delay_us(1);
                
        //PORTB = 0x01; // set high for 10us
		SONAR_TRIGGER_HIGH();
        running = 1;  // sonar launched
        _delay_us(10);
        
		SONAR_TRIGGER_LOW();
        //PORTB = 0x00; // clear  
}

int moveWheel( uint8_t direction,uint8_t WheelDir[], uint8_t sensorREAD[], uint8_t sensorOLD[], uint8_t steps, uint8_t pulsesFIN[]){
uint8_t pulsesM3=0;		//pulses on M3 wheel
uint8_t pulsesM4=0;		//pulses on M4 wheel
uint8_t pulses=0;

char buffer [3];
shiftWrite(direction);
while ((pulsesM3<steps)&&(pulsesM4<steps )){

    itoa (pulsesM3,buffer,10);
	uart_puts(buffer);
	itoa (pulsesM4,buffer,10);
	uart_puts(buffer);
	uart_puts("\r\n");
	//pulses are in SYNC, move the two wheels at the same time.
	if (steps>2){
		if (pulsesM3==pulsesM4){
			OCR0A=255;
			OCR0B=255;
		}
		if(pulsesM4>pulsesM3){
			//M4 is faster than M3, slowdown the M4 speed
			OCR0A=100;		
			uart_puts("slow M4\r\n");
		}
		if (pulsesM3>pulsesM4){
			//M3 is faster than M4, slowdown the M3 speed
			OCR0B=100;
			uart_puts("slow M3\r\n");
		}
	}
	
	
	//_delay_ms(DON);

	
		shiftin(sensorREAD);	//read the sensors values.


		if ( (sensorOLD[SENSORM3]==SENSOROFF) && (sensorREAD[SENSORM3]==SENSORON)){  //sensor changed from 0 to 1. Count a pulse for M3
			sensorOLD[SENSORM3]=SENSORON;
			pulsesM3++;
			pulses++;
		}
		if ( (sensorOLD[SENSORM3]==SENSORON) && (sensorREAD[SENSORM3]==SENSOROFF)){  //sensor changed from 1 to 0. Not count for M3
			sensorOLD[SENSORM3]=SENSOROFF;
		}
		if ( (sensorOLD[SENSORM4]==SENSOROFF) && (sensorREAD[SENSORM4]==SENSORON)){ //sensor changed from 0 to 1. Count a pulse for M4
			sensorOLD[SENSORM4]=SENSORON;
			pulsesM4++;
			pulses++;
		}
		if ( (sensorOLD[SENSORM4]==SENSORON) && (sensorREAD[SENSORM4]==SENSOROFF)){ //sensor changed from 1 to 0. Not count for M4
			sensorOLD[SENSORM4]=SENSOROFF;
		}
	
	//uart_puts("off\r\n");
	//shiftWrite(MSTOP);
	//_delay_us(DOFF);
}
pulsesFIN[0]=pulsesM3;
pulsesFIN[1]=pulsesM4;
shiftWrite(MSTOP);
return 0;
}
void Wait2()
{
   uint8_t i;
   for(i=0;i<50;i++)
   {
      _delay_loop_2(0);
      _delay_loop_2(0);
      _delay_loop_2(0);
   }

}
int main(void) {
//I2C init
I2C_init(0x4); // initalize as slave with address 0x4	

	char buffer [20];
uint8_t i=0;
	    // PD3 is now an input INT1
    DDRD &= ~(1<<PD3);
	    // PD6 and PD5 is now an output PWMs
    DDRD |= (1 << DDD6)|(1 << DDD5);
	//PDB1 Servo 1 CLIP
	//PDB2 Servo 2 ARM
	DDRB |= (1 << PB1)|(1 << PB2);;
    // set none-inverting mode
	TCCR0A |= (1 << COM0A1)|(1 << COM0B1);
	//PWM Phase Corrected mode 1 TOP 0xFF
    TCCR0A |= (1 << WGM00) ;
    //TCCR0B |= (1 << WGM02);
	/*PWM_frequency = clock_speed / (2 * Prescaller_value * TOP_value )
	 * prescaller=8  3.9Khz
	 * 			 64	490Hz
	 * 			256	122Hz
	*			1024 30Hz
	*/
	/*  OCR0A motor3 speed
	 * 	OCR0B motor4 speed
	 */
	OCR0A = 255;	//M3
	OCR0B = 255;	//M4
	
	//set prescaler to 1024= 30Hz and start the counter
	TCCR0B |= (1 << CS02)|(1 << CS00);	

   
uint8_t sensorIN[8]={0};	//sensors read value
uint8_t sensorOLD[8]={0};	//sensors status. To detect sensor change
LED_MSG |= (1<<LEDY); //output

uint8_t pulsesFIN[2]={0};

uint8_t WheelDir[9]={
0b00000000,	//stop	0
0b00000100,	//	 M1A F
0b00001000,	//   M1B B
0b00000010,	//	 M2A U	ARM UP
0b00010000,	//	 M2B D	ARM DOWN
0b00000001,	//	M3A
0b01000000,	//	M3B
0b00100000,	//	M4A
0b10000000	//	M4B

};



uint8_t direction=0;
uint8_t steps=0;
uart_init( UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU) );
uart_puts("Ready to start...");
uart_puts("\r\n");
_delay_ms(1000);

        // ------------------- ultrasonic init code --------------------
        SONAR_TRIGGER_OUTPUT_MODE();
		
		//  1.CONFIGURE INTERRUPT INT1
        // turn on interrupts for INT1, connect Echo to INT1
        //EICRA |= (1<<ISC10)|(1<<ISC11); //For rising edge
        EICRA |= (1<<ISC10);	//any logic change generate an int.
        EIMSK |= (1 << INT1);      // Turns on INT1
        
		// 2.CREATE Timer T2 to count seconds
        // setup 8 bit timer & enable interrupts, timer increments to 255 and interrupts on overflow
        TCCR2B = (0<<CS02)|(0<<CS01)|(1<<CS00); // select internal clock with no prescaling
        TCNT2 = 0; // reset counter to zero
        TIMSK2 = 1<<TOIE2; // enable timer interrupt
cli();
sei(); 

//EICRA |= ((1 << ISC11) | (1 << ISC10)); 
//EIMSK |= (1 << INT1);
        int numDelays = 0;
        
    for(;;){  /* main event loop */
			/*_delay_ms(100);
			uart_puts("P1:");
			itoa (PCOUNT1,buffer,10);
			uart_puts(buffer);
						uart_puts(" P2:");
			itoa (PCOUNT2,buffer,10);
			uart_puts(buffer);
						uart_puts(" P3:");
			//itoa (PCOUNT3,buffer,10);
			sprintf (buffer,"%lu", timerCounter);
			uart_puts(buffer);
						uart_puts(" P4:");
			itoa (PCOUNT4,buffer,10);
			uart_puts(buffer);
						uart_puts(" R:");
			if (result>1000){
				itoa (result,buffer,10);
				uart_puts(buffer);
			}*/
			_delay_ms(50);
			sprintf(buffer," Distance: %4.1lumm\r ", result);
			//itoa (result,buffer,10);
			uart_puts(buffer);
               if (running == 0) { // launch only when next iteration can happen
                
                        // configurable delay count
                        _delay_ms(1);
                        numDelays++;
                
                        // create a delay between tests, to compensate for old echoes
                        if (numDelays > DELAY_BETWEEN_TESTS_MS) 
						{ 
                                sonar(); // launch ultrasound measurement!
                                numDelays = 0;
                        }
                }
    }
    
shiftInitIN();	 //Initialise IN register
shiftInitOUT(); 	//Initialise OUT register

/*LEDYON;
_delay_ms(500);
LEDYOFF;
_delay_ms(200);*/
uart_puts("Ready to start...");
uart_puts("\r\n");
//_delay_ms(1000);
uart_puts("111");
uart_puts("\r\n");
//_delay_ms(100);
shiftin(sensorIN);
//init stored sensor values

/*
//shiftWrite (WheelDir[3]);
//_delay_ms(15000);
shiftin(sensorIN);	
while(1){
direction=AU;
while (sensorIN[SENSORM2_1]==0 ){
	for (i=0;i<8;i++){
	itoa (sensorIN[i],buffer,10);

	uart_puts(buffer);
	shiftin(sensorIN);	
	}
	uart_puts("D\r\n");
	for (i=3;i<5;i++){
	shiftWrite (direction);
	itoa (i,buffer,10);

	uart_puts(buffer);

	}
	uart_puts("\r\n");

}
shiftin(MSTOP);	
direction=AD;
while (sensorIN[SENSORM2_2]==0 ){
	for (i=0;i<8;i++){
	itoa (sensorIN[i],buffer,10);

	uart_puts(buffer);
	shiftin(sensorIN);	
	}
	uart_puts("U\r\n");
	for (i=3;i<5;i++){
	shiftWrite (direction);
	itoa (i,buffer,10);

	uart_puts(buffer);

	}
	uart_puts("\r\n");

}
}

uart_puts("Checking ARM motor\r\n");
	OCR1A = 255;	//M1
	OCR1B = 255;	//M2
direction=AD;
shiftWrite(direction);
		_delay_ms(3000);
direction=AU;
shiftWrite(direction);
_delay_ms(3000);




while (1 ){

	for (i=0;i<8;i++){
	itoa (sensorIN[i],buffer,10);

	uart_puts(buffer);
	shiftin(sensorIN);	
	}
	uart_puts("\r\n");
		_delay_ms(100);
}

uart_puts("Init ST sensors2\r\n");

*/

/*	
   //Configure TIMER1
   TCCR1A|=(1<<COM1A1)|(1<<COM1B1)|(1<<WGM11);        //NON Inverted PWM
   TCCR1B|=(1<<WGM13)|(1<<WGM12)|(1<<CS11)|(1<<CS10); //PRESCALER=64 MODE 14(FAST PWM)	
	 ICR1=4999;  //fPWM=50Hz (Period = 20ms Standard).
	 //OCR1A CLIP
	 //OCR1B ARM //190 MIN UP  180 degree (500 MAX DOWN)
		//CLIP 110 close
		//CLIP 500 open
  while(1)
   {

	OCR1B=300;
	      Wait2();
      OCR1A=110;   

      Wait2();


      OCR1B=180; //UP
      Wait2();
      OCR1A=500;  
      Wait2();

   }
  */ 

for (i=0;i<8;i++){
	itoa (sensorIN[i],buffer,10);
	uart_puts(buffer);
	shiftin(sensorIN);	
}
uart_puts("\r\n");

/*motor test*/
/*
shiftWrite(0);
shiftWrite(WheelDir[M3F]);
_delay_ms(2000);
shiftWrite(WheelDir[M3B]);
_delay_ms(2000);
shiftWrite(MSTOP);

shiftWrite(WheelDir[M4F]);
_delay_ms(2000);
shiftWrite(WheelDir[M4B]);
_delay_ms(2000);
shiftWrite(MSTOP);
*/
/*sensor tests*/
/*
while ((sensorIN[SENSORM3]==0)||(sensorIN[SENSORM4]==0)){
	shiftin(sensorIN);
	itoa (sensorIN[SENSORM3],buffer,10);
	uart_puts(buffer);	
	_delay_ms(100);
	itoa (sensorIN[SENSORM4],buffer,10);
	uart_puts(buffer);	
	_delay_ms(100);
	uart_puts("\n\r");
}

*/
uart_puts("Sync M3 \n\r");
//Move wheel until SENSOR is OFF 
while (sensorIN[SENSORM3]==SENSORON){
	shiftWrite(WheelDir[M3F]);
	shiftin(sensorIN);
 
}
shiftWrite(MSTOP);
uart_puts("Sync M4 \n\r");
while (sensorIN[SENSORM4]==SENSORON){
	shiftWrite(WheelDir[M4F]);
	shiftin(sensorIN);
} 

shiftWrite(MSTOP);

sensorOLD[SENSORM4]=sensorIN[SENSORM4]; //INIT SENSORS 
sensorOLD[SENSORM3]=sensorIN[SENSORM3]; 
for (i=0;i<8;i++){
	itoa (sensorIN[i],buffer,10);
	uart_puts(buffer);
	shiftin(sensorIN);	
}
uart_puts("\r\n");
uart_puts("x \n\r");


for (i=0;i<2;i++){
	itoa (pulsesFIN[i],buffer,10);
	uart_puts(buffer);
	uart_puts("\r\n");
}
while (1) {
			itoa(status[0], buffer, 10);
			uart_puts(buffer);
			uart_puts("..");
			itoa(status[1], buffer, 10);
			uart_puts(buffer);
			uart_puts("..");
			itoa(status[2], buffer, 10);
			uart_puts(buffer);
			uart_puts("..");
			itoa(status[3], buffer, 10);
			uart_puts(buffer);
			uart_puts("..");
			itoa(status[4], buffer, 10);
			uart_puts(buffer);
			_delay_ms(100);
			uart_puts("\n\r");
}

while (1){
		if (status[0]==1){
			status[0]=0;
			itoa(status[1], buffer, 10);
			uart_puts(buffer);
			_delay_ms(100);
			uart_puts("\n\r");
			switch (status[1]){
				case 2:
					direction=WB;
					steps=2;
				break;
				case 3:
					direction=WB;
					steps=10;
				break;
				case 4:
					direction=WF;
					steps=2;
				break;
				case 6:
					direction=WF;
					steps=10;
				break;
				case 8:
					direction=WR;
					steps=2;
				break;
				case 12:
					direction=WR;
					steps=10;
				break;
				case 16:
					direction=WL;
					steps=2;
				break;	
				case 24:
					direction=WL;
					steps=10;
				break;	
				default :
					i=1;				
				
			}
			moveWheel(direction, WheelDir, sensorIN, sensorOLD, steps,pulsesFIN);
			for (i=0;i<2;i++){
				itoa (pulsesFIN[i],buffer,10);
				uart_puts(buffer);
			}
			uart_puts("\r\n");
			
		}
	}
//direction=B;
//moveWheel(direction, WheelDir, sensorIN, sensorOLD, 20,pulsesFIN);


/*
while (pulsesRW<20){
	
	shiftWrite(WheelDir[3]);
	_delay_ms(2);
	shiftWrite(0);
	_delay_us(2);
	shiftin(sensorIN);
	if ( (sensorOLD[SENSORM2]==SENSOROFF) && (sensorIN[SENSORM2]==SENSORON)){
		sensorOLD[SENSORM2]=SENSORON;
		pulsesRW++;
	}
	if ( (sensorOLD[SENSORM2]==SENSORON) && (sensorIN[SENSORM2]==SENSOROFF)){
				sensorOLD[SENSORM2]=SENSOROFF;
	}
	
}
uart_puts("M2 ok \n\r");	
uart_puts("Sync M1 \n\r");
pulsesRW=0;
while (pulsesRW<20){
	
	shiftWrite(WheelDir[1]);
	_delay_ms(2);
	shiftWrite(0);
	_delay_us(2);
	shiftin(sensorIN);
	if ( (sensorOLD[SENSORM1]==SENSOROFF) && (sensorIN[SENSORM1]==SENSORON)){
		sensorOLD[SENSORM1]=SENSORON;
		pulsesRW++;
	}
	if ( (sensorOLD[SENSORM1]==SENSORON) && (sensorIN[SENSORM1]==SENSOROFF)){
				sensorOLD[SENSORM1]=SENSOROFF;
	}
	
}
uart_puts("M1 ok \n\r");	
_delay_ms(1000);
*/

/*itoa (pulsesM2,buffer,10);
uart_puts(buffer);
uart_puts("M2 \r\n");
itoa (pulsesM1,buffer,10);
uart_puts(buffer);
uart_puts("M1 \r\n");
for (i=0;i<8;i++){
	itoa (sensorIN[i],buffer,10);
	uart_puts(buffer);
	shiftin(sensorIN);	
}
uart_puts("\r\n");
*/
//_delay_ms(2000);
}

