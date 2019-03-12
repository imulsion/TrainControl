#include <avr/interrupt.h> //includes avr/io.h
#include <util/twi.h>
#include "i2c.h"

ISR(I2C_vect) //I2C interrupt handler
{
	cli();//disable interrupts
	uint8_t status = TWSR;

	switch(status)
	{
		case I2C_WRITING:
			//AVR is being written to over I2C, read the data
			globalData = TWDR;
			dataReady = DATA_WAITING;
			break;
		case I2C_READING:
			//computer has received data in TWDR and sent ACK, don't need to do anything
			break;
		default: //unexpected status code
			PORTB = status;//write error code to PORTB
			break;
	}

	TWCR |= I2C_INTERRUPT_CLEAR_MASK;//clear TWINT bit
	sei();//enable interrupts
}


int main (void)
{
	I2CInit();//initialise I2C interface
	return 0;
}

void I2CInit(void)
{
	DDRB = 0xFF; //report error codes on this port
	PORTB = 0x00;

	//enable global interrupts
	sei();
	//configure ports for I2C
	DDRC = PORTC_IO_CONFIG;
	PORTC = PORTC_IO_CONFIG; //enable internal pull-up for i2c

	//ensure bit rate register is cleared
	TWBR = 0x00;

	TWAR = I2C_ADDR;//assign I2C address to I2C address register

	TWCR = I2C_CONTROL_CONFIG; //set i2c configuration register
}
