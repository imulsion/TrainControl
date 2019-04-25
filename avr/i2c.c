 
/*****************************************************************************************
 * 				          MACROS                                         *
 *****************************************************************************************/
#include <avr/interrupt.h> //includes avr/io.h
#include "i2c.h"


/*****************************************************************************************
 * 				        DEFINITIONS                                      *
 *****************************************************************************************/
//globals for accessing SectionT structs in functions other than main
volatile SectionT* sectionOnePtr;
volatile SectionT* sectionTwoPtr;

volatile uint8_t globalData;                               //used for receiving I2C data
volatile uint8_t dataReady;                                //flag to indicate data received

volatile I2CReadStateT readState = I2C_IDLE;


/*****************************************************************************************
 * 	               	        INTERRUPT SERVICE ROUTINES                               *
 *****************************************************************************************/

//NOTE: Interrupts do not have to be manually disabled and enabled during ISRs. The hardware automatically disables and enables interrupts
//at the start and end of all ISRs.

/*
 * I2C interrupt routine. Function called whenever the TWINT flag in I2C_CONTROL_REGISTER is set (to 0) by hardware. This occurs when an I2C 
 * event occurs such as a START or STOP condition. The TWINT flag must then be cleared (set to 1) by software to allow the hardware to 
 * continue receiving events
 *
 * Parameter[in] I2C_vect 		Vector specifying the interrupt being serviced.
 */
ISR(I2C_vect) 
{
	uint8_t status = I2C_STATUS_REGISTER & I2C_PRESCALER_MASK;   //extract the event type from the I2C status register
	switch(status)
	{
		//repeated start or stop received, hardware will wait for next event, no software action required
		case I2C_STOP_OR_REPEATED_START:
			break;

		//AVR has been addressed with SLA+W, hardware will wait for next event, no software action required
		case I2C_ADDRESSED_W:
			break;

		//AVR has been addressed with SLA+R, PC is requesting a status report
		case I2C_ADDRESSED_R:
			readState = I2C_SECTION_ONE_OCCUPANCY;
			I2C_DATA_REGISTER = sectionOnePtr->trainInSection;
			break;

		//Last byte sent received, ACK returned. Next byte should be sent
		case I2C_READING_ACK:
			//decide which byte to send next
			switch(readState)
			{
				case I2C_SECTION_ONE_OCCUPANCY:
					readState = I2C_SECTION_ONE_DCYCLE;
					I2C_DATA_REGISTER = sectionOnePtr->dutyCycle;
					break;
				case I2C_SECTION_ONE_DCYCLE:
					readState = I2C_SECTION_TWO_OCCUPANCY;
					I2C_DATA_REGISTER = sectionTwoPtr->trainInSection;
					break;
				case I2C_SECTION_TWO_OCCUPANCY:
					readState = I2C_SECTION_TWO_DCYCLE;
					I2C_DATA_REGISTER = sectionTwoPtr->dutyCycle;
					break;
				//if this case is executed, the master is asking for too many bytes of data. Generate a NACK to refuse to send more
				//NOTE: Master should have sent a NACK upon receipt of the previous byte, not an ACK. This is why this 
				//state is an error state.
				case I2C_SECTION_TWO_DCYCLE:
					readState = I2C_IDLE;
					I2C_CONTROL_REGISTER |= I2C_TWSTO_MASK;
					break;
				//only untested case is I2C_IDLE, it should be impossible to get here. Send NACK if this somehow executes
				default:
					readState = I2C_IDLE;
					I2C_CONTROL_REGISTER |= I2C_TWSTO_MASK;
					break;
			}
		break;
		//Last byte sent has been received, NACK returned. Transmission will terminate.
		case I2C_READING_NACK:
			//transmission will terminate, regardless of whether all the required bytes have been requested by master. 
			//NOTE: The slave doesn't care if the master doesn't get a full status report. If this is not handled at the master side
			// it may be possible for the master to receive corrupted or incomplete status data.
			
			readState = I2C_IDLE; //reset state machine
			break;

		//AVR is being written to over I2C, read the data
		case I2C_WRITING:
			globalData = I2C_DATA_REGISTER;//copy received data to global data variable
			dataReady = DATA_WAITING;
			break;

                //unexpected status code
		default: 
			PORTB = status;//write error code to PORTB
			break;
	}

	I2C_CONTROL_REGISTER |= I2C_INTERRUPT_CLEAR_MASK;//clear TWINT bit so hardware can continue receiving events
}

/*
 * External interrupt routine from the INT0 pin. Function called whenever there is a logical change on the INT0 pin, 
 * indicating a train has left or entered the section.
 *
 * Parameter[in] INT0_vect              Vector specifying the interrupt being serviced
 */ 
ISR(INT0_vect) 
{
	sectionOnePtr -> trainInSection = SECTION_OCCUPANCY_MASK & ~(sectionOnePtr -> trainInSection); //toggle occupancy status of the section
}

/*
 * External interrupt routine from the INT1 pin. Function called whenever there is a logical change on the INT1 pin, 
 * indicating a train has left or entered the section.
 *
 * Parameter[in] INT1_vect              Vector specifying the interrupt being serviced
 */ 
ISR(INT1_vect)
{
	sectionTwoPtr -> trainInSection = SECTION_OCCUPANCY_MASK & ~(sectionTwoPtr -> trainInSection); //toggle occupancy status of the section
}

/*****************************************************************************************
 * 				      MAIN FUNCTION                                      *
 *****************************************************************************************/
int main (void)
{
	SectionT sectionOne;
	SectionT sectionTwo;
	sectionOnePtr = &sectionOne;
	sectionTwoPtr = &sectionTwo;
	sectionOne.dutyCycle = PWM_INITIAL_DUTY_CYCLE;
	sectionOne.dutyCycle = PWM_INITIAL_DUTY_CYCLE;
	DeviceInit();//initialise internal registers and set up device	
	
       	sei();//enable global interrupts 

	//main loop
        for(;;)
       	{
		//if data has been received
		if(DATA_WAITING == dataReady)
		{
			cli(); //disable interrupts while dealing with interrupt modifiable data
			PWM_COUNTER_REGISTER = globalData;//change PWM duty cycle
			dataReady = NOT_DATA_WAITING;
			sei();
		}
	}
	return 0; //main
}

/*****************************************************************************************
 *          		          FUNCTION DEFINITIONS                                   *
 *****************************************************************************************/
void DeviceInit(void)
{
	//global variables
	globalData = INITIAL_DATA;
	dataReady = NOT_DATA_WAITING;
	
	//port registers and initial value setup 
	PORTB_DATA_DIRECTION_REGISTER |= PORTB_DATA_DIRECTION_VALUE;
	PORTB &= REGISTER_CLEAR_MASK;
	PORTD_DATA_DIRECTION_REGISTER |= PORTD_DATA_DIRECTION_VALUE;
	PORTD &= REGISTER_CLEAR_MASK;

	//PWM initialisation
	PWM_CONTROL_REGISTER_TWO = PWM_CONTROL_CONFIG; 
	PWM_COUNTER_REGISTER = PWM_INITIAL_DUTY_CYCLE;
	
	
	//I2C initialisation
	I2C_BITRATE_REGISTER &= REGISTER_CLEAR_MASK; //ensure bit rate register is cleared
	I2C_ADDRESS_REGISTER = I2C_ADDR;//assign I2C address to I2C address register
	I2C_CONTROL_REGISTER = I2C_CONTROL_CONFIG; //set i2c configuration register
	
	uint8_t pinStatus = PIND; //get section occupation status

	//set SectionT status based on section occupation
	if(INT0_SECTION_SENSE_MASK == (INT0_SECTION_SENSE_MASK & pinStatus))
	{
                sectionOnePtr->trainInSection = SECTION_OCCUPIED;
	}
	else
	{
		sectionOnePtr->trainInSection = SECTION_FREE;
	}
	if(INT1_SECTION_SENSE_MASK == (INT1_SECTION_SENSE_MASK & pinStatus))
	{
                sectionTwoPtr->trainInSection = SECTION_OCCUPIED;
	}
	else
	{
		sectionTwoPtr->trainInSection = SECTION_FREE;
	}

	//external interrupts
	MCU_CONTROL_REGISTER = MCU_CONFIG; //configure external interrupts
	INTERRUPT_CONTROL_REGISTER = INTERRUPT_CONTROL_CONFIG; //enable external interrupts
}
