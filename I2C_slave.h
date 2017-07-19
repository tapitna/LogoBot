#ifndef I2C_SLAVE_H
#define I2C_SLAVE_H

volatile uint8_t buffer_address;
//volatile uint8_t txbuffer[0xFF];
//volatile uint8_t rxbuffer[0xFF];
//TWI buffer for RX and TX
volatile uint8_t TWI_BUFF[7];
/*TWI_BUFF
 * 0: operation type RX or TX
 * 1: motor direction/1: distance 1st byte LSB
 * 2: distance 2nd byte
 * 3: distance 3rd byte
 * 4: distance 4th byte MSB
 * 5: angle
 */
#define TWI_ADDRESS 0x04
#define TWI_OPER 0
#define	MOTOR_DIR 1
#define TWI_1B 1	
#define TWI_2B 2
#define TWI_3B 3
#define TWI_4B 4
#define TWI_ANGLE 5
#define BACKSTEPS 2
#define BACKCONT  3
#define FORWARDSTEPS 4
#define FORWARDCONT 6
#define RIGHTSTEPS 8
#define RIGHTCONT 12
#define	LEFTSTEPS 16
#define LEFTCONT 24
#define TWI_RX 1
#define TWI_TX 2
#define TWI_IDLE 0


void I2C_init(uint8_t address);
void I2C_stop(void);
ISR(TWI_vect);

#endif // I2C_SLAVE_H
