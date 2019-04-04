#include <avr/interrupt.h> //includes avr/io.h
#include "i2c.h"


/*
 * Function: ISR(I2C_vect)
 *
 * I2C interrupt routine. Function called whenever the TWINT flag in I2C_CONTROL_REGISTER (I2C control register) is set (to 0) by hardware. This occurs when an I2C event occurs such as a START or STOP condition. The TWINT flag must then be cleared (set to 1) by software to allow the hardware to continue receiving events
 *
 * Parameter[in] I2C_vect 		Vector specifying the interrupt being serviced.
 */
ISR(I2C_vect) 
{
	cli();//disable interrupts while processing current interrupt

	uint8_t status = I2C_STATUS_REGISTER & I2C_PRESCALER_MASK;   //extract the event type from the I2C status register
	switch(status)
	{
		//repeated start or stop received, hardware will wait for next event, no software action required
		case I2C_STOP_OR_REPEATED_START:
			break;

		//AVR has been addressed with SLA+W, hardware will wait for next event, no software action required
		case I2C_ADDRESSED:
			break;


		//AVR is being written to over I2C, read the data
		case I2C_WRITING:
			globalData = I2C_DATA_REGISTER;//copy received data to global data variable
			dataReady = DATA_WAITING;
			break;

		//computer has received data in I2C_DATA_REGISTER and sent ACK, no action needed
		case I2C_READING:
			break;
			
                //unexpected status code
		default: 
			PORTB = status;//write error code to PORTB
			break;
	}

	I2C_CONTROL_REGISTER |= I2C_INTERRUPT_CLEAR_MASK;//clear TWINT bit so hardware can continue receiving events

	sei();//re-enable global interrupts
}


int main (void)
{
	I2CInit();//initialise I2C interface
	
	uint8_t ledStatus = LED_STATUS_OFF; //variable for storing current state of the LED
       	sei();//enable global interrupts 

	//main loop
        for(;;)
       	{
		//if data has been received and is valid
		if((DATA_WAITING == dataReady) && (VALID_DATA == globalData))
		{
			if(LED_STATUS_OFF == ledStatus)
			{
				ledStatus = LED_STATUS_ON;
				PORTB |= PORTB_LED_ON_MASK;
			}
			else
			{
				ledStatus = LED_STATUS_OFF;
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
	globalData = INITIAL_DATA;
	dataReady = NOT_DATA_WAITING;
	
	DDRB |= PORTB_DATA_DIRECTION;
	PORTB &= REGISTER_CLEAR_MASK;
	
	//ensure bit rate register is cleared
	I2C_BITRATE_REGISTER &= REGISTER_CLEAR_MASK;

	I2C_ADDRESS_REGISTER = I2C_ADDR;//assign I2C address to I2C address register

	I2C_CONTROL_REGISTER = I2C_CONTROL_CONFIG; //set i2c configuration register
	
}
