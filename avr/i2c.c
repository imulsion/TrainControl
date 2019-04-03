#include <avr/interrupt.h> //includes avr/io.h

#include "i2c.h"

ISR(I2C_vect) //I2C interrupt handler
{
	cli();//disable interrupts
	uint8_t status = TWSR & I2C_PRESCALER_MASK;
	switch(status)
	{
		case I2C_STOP_OR_REPEATED_START:
			//repeated start or stop received, no action needed
			break;
		case I2C_ADDRESSED:
			//AVR has been addressed with SLA+W, wait for next interrupt
			break;
		case I2C_WRITING:
			//AVR is being written to over I2C, read the data
			globalData = TWDR;
			dataReady = DATA_WAITING;
			break;
		case I2C_READING:
			//computer has received data in TWDR and sent ACK, no action needed
			break;
		default: //unexpected status code
			PORTD = status;//write error code to PORTD
			break;
	}
	TWCR |= I2C_INTERRUPT_CLEAR_MASK;//clear TWINT bit
	sei();//enable interrupts
}


int main (void)
{
	I2CInit();//initialise I2C interface
	uint8_t ledStatus = 0U;
	sei();//enable global interrupts
	for(;;)
	{
		if((DATA_WAITING == dataReady) && (0xC5 == globalData))
		{
			PORTD = 0x11;
			if(0U == ledStatus)
			{
				ledStatus = 1U;
				PORTB |= PORTB_LED_ON_MASK;
			}
			else
			{
				ledStatus = 0U;
				PORTB &= PORTB_LED_OFF_MASK;
			}
			cli();
			dataReady = NOT_DATA_WAITING;
			sei();
		}
	}
	return 0;
}

void I2CInit(void)
{
	globalData = 0U;
	dataReady = NOT_DATA_WAITING;
	DDRD |= 0xFF; //report error codes on this port
	PORTD &= 0x00;
	
	DDRB |= 0xFF;
	PORTB &= 0x00;
	
	//ensure bit rate register is cleared
	TWBR &= 0x00;

	TWAR = I2C_ADDR;//assign I2C address to I2C address register

	TWCR = I2C_CONTROL_CONFIG; //set i2c configuration register
	
}
