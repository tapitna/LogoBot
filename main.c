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
#define DELAY_BETWEEN_TESTS_MS 100			// echo cancelling time between sampling

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
/*for a 16MHz clock
* T=1/16MHz=0.0625us
* Top Counter=255 so there are 255 clocks= 255*0.0625=15.94us
* To remove the comas:
* 0.0625us
* 15.94x10000=159400
* SoundSpeed=0.343mm/us
* distance=(o*15.94+c*0.0625)0.343/2
*			=(o*255+c)*0.0625*0.343/2
* 			=(o*255+c)*107/100
* Max Distante=2m=2000mm
* Sensor HARD LIMIT. If the object is 2000mm, then the pulse will take 5830.9us to go and another 5830.9to come back
* So the pulse will take 11661.8us in total (go and return)
* There is a overflow every 15.94us, so if overflow >732 times, then I assume there was a timeout and there is no object 
* in leess than 2m
* Limit=2000mm 
* 2000/0.343=5830.9
* 5830.9*2/15.94=732
*/
#define PULSE_TIMEOUT 732  //if there are more than this overflows, then the object is more than 2m, discard the measurement 
#define TIMER_DELAY 159400	//delay after each timer overflow.(us*10000)
#define INSTR_DELAY 625		//clock period (us*10000)
#define SOUND_SPEED 3430	//soundspeed mm/us x10000
/* There is an overflow every 15.94us, so after 4000 overflow (~64ms) I can send a new sample
 */
#define SAMPLE_DELAY 2000		//minimum delay between samples. 

//volatile uint32_t result = 0;
volatile unsigned char up = 0;
volatile unsigned char running = 0;
volatile uint32_t overflowCounter = 0; 
volatile uint8_t result = 0; 
volatile uint8_t currentCounter=0;
volatile uint32_t delayCounter=0;
volatile uint32_t distance_avg[5];
volatile uint8_t dist_index=0;
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

SIGNAL(INT1_vect) 
{
        if (running) { //accept interrupts only when sonar was started

                if (up == 0) { // voltage rise, start time measurement (low to high)
						// start timer
                        up = 1;
                        overflowCounter = 0;
                        currentCounter=0;
                        result=0;
                        TCNT2 = 0; // reset timer counter 
                } else {
                        // voltage drop, stop time measurement
                       //stop timer
                        currentCounter=TCNT2;
                        distance_avg[dist_index]=(overflowCounter*255+currentCounter)*107/10000;
                        up = 0;
                        running = 0;
						dist_index++;
						if (dist_index==3){  //i take 3 samples then calculate the average
							result=1;
							dist_index=0;
						}
							
                       
                }
        }
}

// timer overflow interrupt, each time when timer value passes 255 value

SIGNAL(TIMER2_OVF_vect)
{
        if (up) {       // The pulse has been sent, we are counting the time until the pulse is detected back
				delayCounter=0;
                overflowCounter++; // count the number of overflows
                // dont wait too long for the sonar end response, stop if time for measuring the distance exceeded limits
                if (overflowCounter>PULSE_TIMEOUT) {
                        // timeout
                        up = 0;          // stop counting timer values
                        running = 0; // ultrasound scan done
                        //overflowCounter = -1; // show that measurement failed with a timeout (could return max distance here if needed)
						distance_avg[dist_index]=2000;
						dist_index++;
						if (dist_index==3){
							result=1;
							dist_index=0;
						}
                }
        }else{
			delayCounter++;	
		}
}
//I only need the timer2 active when i send a sonar pulse, so activate or deactivate the timer when needed.
void start_TIMER2(void){
	TIMSK2 = 1<<TOIE2;
}
void stop_TIMER2(void){
	TIMSK2 &= ~(1<<TOIE2);
}
// generate an impulse for the Trig input (starts the sonar)
void sonar() {
		SONAR_TRIGGER_LOW();
        _delay_us(1);
		SONAR_TRIGGER_HIGH();
        running = 1;  // sonar launched
        _delay_us(10);
		SONAR_TRIGGER_LOW();
}
uint8_t radar_pulse(uint8_t distance_byte[]){
	
uint32_t distance = 0;
uint8_t response = 0;
	
char buffer [40];
//		sprintf(buffer," %lu \r\n", delayCounter);
//	uart_puts(buffer);	
if ((delayCounter>SAMPLE_DELAY)&&(up==0)){ //Check if I can send a new pulse. The delay between pulses is ~63ms
	delayCounter=0;
	sonar(); // launch ultrasound measurement!
}
		
	if (result==1){
		//distance=overflowCounter*255+currentCounter;
		//distance=distance*107/10000;
			/*distance is 4 bytes. So I get each byte as an array element for later transmission to the ESP8266
					* MSB ... LSB
					* 3 2 1 0 
					*/
		//distance=(distance_avg[0]+distance_avg[1]+distance_avg[2]+distance_avg[3]+distance_avg[4])/5;
		distance=(distance_avg[0]+distance_avg[1]+distance_avg[2])/3;
		distance_byte[0]=distance & 0xFF;  //LSB  
		distance_byte[1]=distance >>8;
		distance_byte[2]=distance >>16;
		distance_byte[3]=distance >>24; //MSB
		//sprintf(buffer," distance1: %lumm %lu %lu %lu\n\r", distance,distance_avg[0],distance_avg[1],distance_avg[2]);
		//uart_puts(buffer);
		response=1;
		result=0;
	}	
	if (result==2){
		distance=2000;
		distance_byte[0]=distance & 0xFF;  //LSB  
		distance_byte[1]=distance >>8;
		distance_byte[2]=distance >>16;
		distance_byte[3]=distance >>24; //MSB
		//sprintf(buffer," distance2: %lumm \n\r", distance);
		//uart_puts(buffer);
		response=1;
		result=0;
	}

   return response;
}
int scan(void){
	uint8_t distance_bytes[4];	//the distance cant take up to 4 bytes. 
	uint8_t k=0;
	char buffer [3];
		uart_puts("Oper: ");
		itoa(TWI_BUFF[TWI_OPER], buffer, 10);
			//_delay_ms(500);
			uart_puts(buffer);
			uart_puts("\n\r");
		if (TWI_BUFF[TWI_OPER]==TWI_TX){

		uart_puts("scan\n\r");
		k=0;
		while(OCR1A<=500){
			OCR1A=OCR1A+15;
			k++;
			while (radar_pulse(distance_bytes)==0){
			}
			TWI_BUFF[TWI_1B]=distance_bytes[0];
			TWI_BUFF[TWI_2B]=distance_bytes[1];
			TWI_BUFF[TWI_3B]=distance_bytes[2];
			TWI_BUFF[TWI_4B]=distance_bytes[3];
			TWI_BUFF[TWI_ANGLE]=k;
			//sprintf(buffer," distanceI: %d %d %d %d  %d  \n\r", k,distance_bytes[0],distance_bytes[1],distance_bytes[2],distance_bytes[3]);
			//uart_puts(buffer);
		}
		while(OCR1A>=200){
			OCR1A=OCR1A-15;
			k++;
			while (radar_pulse(distance_bytes)==0){
			}
			TWI_BUFF[TWI_1B]=distance_bytes[0];
			TWI_BUFF[TWI_2B]=distance_bytes[1];
			TWI_BUFF[TWI_3B]=distance_bytes[2];
			TWI_BUFF[TWI_4B]=distance_bytes[3];
			TWI_BUFF[TWI_ANGLE]=k;
			//sprintf(buffer," distanceI: %d %d %d %d  %d  \n\r", k,distance_bytes[0],distance_bytes[1],distance_bytes[2],distance_bytes[3]);
			//uart_puts(buffer);
		}
			uart_puts("TX2\n\r");

		}
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

/*********************MAIN**************/
int main(void) {
//I2C init
I2C_init(TWI_ADDRESS); // initalize as slave with address 0x4	

	char buffer [20];
uint8_t i=0;
	    // PD3 is now an input INT1
    DDRD &= ~(1<<PD3);
	    // PD6 and PD5 is now an output PWMs
    DDRD |= (1 << DDD6)|(1 << DDD5);
	//PDB1 Servo 1 Radar
	//PDB2 Servo 2 ARM
	DDRB |= (1 << PB1)|(1 << PB2);
	
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
        //--------------radar timer-----------------
		// 2.CREATE Timer T2 to count seconds
        // setup 8 bit timer & enable interrupts, timer increments to 255 and interrupts on overflow
        TIMSK2 &= ~(1<<TOIE2); //disable time while configuring 
        TCCR2B = (0<<CS02)|(0<<CS01)|(1<<CS00); // select internal clock with no prescaling
        TCNT2 = 0; // reset counter to zero
        //TIMSK2 = 1<<TOIE2; // enable timer interrupt
        
   //Configure TIMER1
   //OCR1B= ARM
   //OCR1A= radar servo
   TCCR1A|=(1<<COM1A1)|(1<<COM1B1)|(1<<WGM11);        //NON Inverted PWM
   TCCR1B|=(1<<WGM13)|(1<<WGM12)|(1<<CS11)|(1<<CS10); //PRESCALER=64 MODE 14(FAST PWM)	
	 ICR1=4999;  //fPWM=50Hz (Period = 20ms Standard).
				//F=50Hz (T*64*ICR1)=0.02s 
	/*The PWM freqquence is 50Hz, period 20ms
	 * Servo 1ms 0
	 * 		1.5ms 90
	 * 		2ms	180
	 */
	 //OCR1A CLIP
	 //OCR1B ARM //190 MIN UP  180 degree (500 MAX DOWN)
		//CLIP 110 close
		//CLIP 500 open
		//OCR1B=300;
		//disable ARM servo
		DDRB &= ~(1<<PB2);
		DDRB &= ~(1<<PB1);
cli();
sei(); 


//uint8_t sonar_result;
//uint32_t DIS=0;    

_delay_ms(1000);
shiftInitIN();	 //Initialize IN register
shiftInitOUT(); 	//Initialize OUT register







/*LEDYON;
_delay_ms(500);
LEDYOFF;
_delay_ms(200);*/
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



//test radar
start_TIMER2();
uint8_t k=0;
//enable servo1 
DDRB &= ~(1<<PB2);
DDRB |= (1 << PB1);
//move to center
OCR1A=200;
_delay_ms(1000);
//DDRB &= ~(1<<PB1);
start_TIMER2();
//for (i=0; i<10;i++){
//scan();


uart_puts("Started ");
_delay_ms(1000);

while (1){
	scan();
		if (TWI_BUFF[TWI_OPER]==TWI_RX){
			uart_puts("RX   ");
			TWI_BUFF[TWI_OPER]=TWI_IDLE;
			itoa(TWI_BUFF[MOTOR_DIR], buffer, 10);
			uart_puts(buffer);
			_delay_ms(100);
			uart_puts("\n\r");
			switch (TWI_BUFF[MOTOR_DIR]){
				case BACKSTEPS:
					direction=WB;
					steps=2;
				break;
				case BACKCONT:
					direction=WB;
					steps=10;
				break;
				case FORWARDSTEPS:
					direction=WF;
					steps=2;
				break;
				case FORWARDCONT:
					direction=WF;
					steps=10;
				break;
				case RIGHTSTEPS:
					direction=WR;
					steps=2;
				break;
				case RIGHTCONT:
					direction=WR;
					steps=10;
				break;
				case LEFTSTEPS:
					direction=WL;
					steps=2;
				break;	
				case LEFTCONT:
					direction=WL;
					steps=10;
				break;	
				default :
					i=1;				
				
			}
			//moveWheel(direction, WheelDir, sensorIN, sensorOLD, steps,pulsesFIN);
			//for (i=0;i<2;i++){
			//	itoa (pulsesFIN[i],buffer,10);
			//	uart_puts(buffer);
			//}
			//uart_puts("\r\n");
			
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

