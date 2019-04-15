#ifndef I2C_H //header guard
#define I2C_H

/*****************************************************************************************
 * 				          MACROS                                         *
 *****************************************************************************************/

//compile with -D SLAVE_ONE if compiling for first slave, SLAVE_TWO if compiling for second
#ifdef SLAVE_ONE
	#define I2C_ADDR 0x02U;//i2c slave address
#elif  SLAVE_TWO
	#define I2C_ADDR 0x04U;
#else
	#error "I2C address not defined! Please define either SLAVE_ONE or SLAVE_TWO with the -D flag"
#endif

//miscellaneous constant definitions
#define F_CPU			         8000000UL         //AVR Clock Frequency                               
#define DATA_WAITING			 1U                //flag to indicate whether or not data is waiting to be read
#define NOT_DATA_WAITING                 0U
#define REGISTER_CLEAR_MASK              0x00U             //mask for clearing a register        
#define INITIAL_DATA                     0U                //initial value of the I2C data storage variable    

//Port constant definitions
#define PORTB_DATA_DIRECTION_REGISTER    DDRB
#define PORTB_DATA_DIRECTION_VALUE       0xFFU             //PORTB pins data direction (all outputs)       
#define PORTD_DATA_DIRECTION_REGISTER    DDRD
#define PORTD_DATA_DIRECTION_VALUE       0xF3U             //PORTD data direction (all outputs except PD2 and PD3
#define INT0_SECTION_SENSE_MASK          0x04U             //mask to detect a train in the section connected to INT0
#define INT1_SECTION_SENSE_MASK          0x08U             //mask to detect a train in the section connected to INT1


//I2C constant definitions
#define I2C_vect			 TWI_vect          //AVR uses TWI (two-wire interface) to avoid I2C copyright. Create I2C alias to avoid confusion. 
#define I2C_STATUS_REGISTER              TWSR              //Alias for I2C status register
#define I2C_DATA_REGISTER                TWDR              //Alias for I2C data register
#define I2C_CONTROL_REGISTER             TWCR              //Alias for I2C control register
#define I2C_ADDRESS_REGISTER             TWAR              //Alias for I2C address register
#define I2C_BITRATE_REGISTER             TWBR              //Alias for I2C bit rate register
#define I2C_TWSTO_MASK                   0x10U             //Mask for setting TWSTO in TWCR in case of an error while sending data to the master
#define I2C_CONTROL_CONFIG		 0xC5U             //I2C control register configuration bits
#define I2C_INTERRUPT_CLEAR_MASK	 0x80U             //used for clearing I2C interrupt flag 
#define I2C_PRESCALER_MASK		 0xF8U             //mask for TWSR to remove prescaler bits
#define I2C_ADDRESSED_W			 0x60U             //status code for receiving own SLA+W and ACK returned
#define I2C_WRITING			 0x80U             //status code for receipt of data and ACK returned
#define I2C_ADDRESSED_R			 0xA8U             //status code for receipt of SLA+R and ACK returned
#define I2C_READING_ACK                  0xB8U             //status code for successful transmission from TWDR, ACK returned from computer
#define I2C_READING_NACK                 0xC0U             //status code for successful transmission from TWDR, NACK returned, communication will be terminated by master
#define I2C_STOP_OR_REPEATED_START	 0xA0U             //status code for receipt of I2C repeated start or stop condition

//PWM constant definitions
#define PWM_CONTROL_REGISTER_TWO         TCCR2             //Alias for 2nd PWM channel control register
#define PWM_PRESCALER                    1U                //Value for the PWM prescaler. Equation for PWM frequency is F = F_CPU/(510*PWM_PRESCALER)
#define PWM_CONTROL_CONFIG               0x61U             //PWM control register configuration bits 
#define PWM_COUNTER_REGISTER             OCR2              //Internal counter register used by the PWM hardware. The value written to this register determines the duty cycle of the PWM waveform
#define PWM_INITIAL_DUTY_CYCLE           128U              //Initial duty cycle value (50%)

//External interrupt definitions
#define MCU_CONTROL_REGISTER             MCUCR             //AVR main control register
#define INTERRUPT_CONTROL_REGISTER       GICR              //AVR interrupt control register
#define MCU_CONFIG                       0x05U             //generate external interrupts on any logical change on both INT pins
#define INTERRUPT_CONTROL_CONFIG         0xC0U             //enable external interrupts on both pins

//train occupancy definitions and masks
#define SECTION_OCCUPIED                 0x01U             //indicates an occupied section
#define SECTION_FREE                     0x00U             //indicates an unoccupied section
#define SECTION_OCCUPANCY_MASK           0x01U             //mask for toggling between SECTION_OCCUPIED and SECTION_FREE



/*****************************************************************************************
 * 				          TYPES                                          *
 *****************************************************************************************/

//struct for storing data about a section owned by the controller
typedef struct
{
	uint8_t trainInSection;
	uint8_t dutyCycle;
}SectionT; 

//enum describing a simple state machine for the correct ordering of transmission of status bytes to the computer
typedef enum
{
	I2C_IDLE,
	I2C_SECTION_ONE_OCCUPANCY,
	I2C_SECTION_ONE_DCYCLE,
	I2C_SECTION_TWO_OCCUPANCY,
	I2C_SECTION_TWO_DCYCLE,
}I2CReadStateT;


/*****************************************************************************************
 * 				       DEFINITIONS                                       *
 *****************************************************************************************/

 
/*****************************************************************************************
 * 			           FUNCTION PROTOTYPES                                   *
 *****************************************************************************************/
/*
 * I2C initialisation function. Sets up registers for I2C configuration with appropriate values. 
 *
 * No parameters or return value
 *
 */
void DeviceInit(void);


#endif //I2C_H
