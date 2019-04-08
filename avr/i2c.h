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


//I2C constant definitions
#define I2C_vect			 TWI_vect          //AVR uses TWI (two-wire interface) to avoid I2C copyright. Create I2C alias to avoid confusion. 
#define I2C_STATUS_REGISTER              TWSR              //Alias for I2C status register
#define I2C_DATA_REGISTER                TWDR              //Alias for I2C data register
#define I2C_CONTROL_REGISTER             TWCR              //Alias for I2C control register
#define I2C_ADDRESS_REGISTER             TWAR              //Alias for I2C address register
#define I2C_BITRATE_REGISTER             TWBR              //Alias for I2C bit rate register
#define I2C_CONTROL_CONFIG		 0xC5U             //I2C control register configuration bits
#define I2C_INTERRUPT_CLEAR_MASK	 0x80U             //used for clearing I2C interrupt flag 
#define I2C_PRESCALER_MASK		 0xF8U             //mask for TWSR to remove prescaler bits
#define I2C_ADDRESSED			 0x60U             //status code for receiving own SLA+W
#define I2C_WRITING			 0x80U             //status code for receipt of data and ACK returned
#define I2C_READING			 0xA8U             //status code for receipt of ACK after a write to the PC
#define I2C_STOP_OR_REPEATED_START	 0xA0U             //status code for receipt of I2C repeated start or stop condition

//PWM constant definitions
#define PWM_CONTROL_REGISTER_TWO         TCCR2             //Alias for 2nd PWM channel control register
#define PWM_PRESCALER                    1U                //Value for the PWM prescaler. Equation for PWM frequency is F = F_CPU/(510*PWM_PRESCALER)
#define PWM_CONTROL_CONFIG               0x71U             //PWM control register configuration bits 
#define PWM_COUNTER_REGISTER             TCNT2             //Internal counter register used by the PWM hardware. The value written to this register determines the duty cycle of the PWM waveform
#define PWM_INITIAL_DUTY_CYCLE           128U              //Initial duty cycle value (50%)

volatile uint8_t globalData;                               //used for receiving I2C data
volatile uint8_t dataReady;                                //flag to indicate data received

/*
 * Function: I2CInit
 *
 * I2C initialisation function. Sets up registers for I2C configuration with appropriate values. 
 *
 * No parameter and no return value
 *
 */
void DeviceInit(void);
