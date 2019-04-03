#define F_CPU 8000000UL
#define I2C_CONTROL_CONFIG 0xC5U //I2C control register configuration bits
#define I2C_vect TWI_vect //AVR uses TWI (two-wire interface) to avoid I2C copyright. Create alias for interrupt vector here
#define I2C_INTERRUPT_CLEAR_MASK 0x80U//used for clearing I2C interrupt flag 
#define I2C_PRESCALER_MASK 0xF8 //mask for TWSR to remove prescaler bits
#define I2C_ADDRESSED 0x60U //status code for receiving own SLA+W
#define I2C_WRITING 0x80 //status code for receipt of data and ACK returned
#define I2C_READING 0xA8U //status code for receipt of ACK after a write to the PC
#define I2C_STOP_OR_REPEATED_START 0xA0
#define PORTB_LED_ON_MASK 0x01
#define PORTB_LED_OFF_MASK 0x00
#define DATA_WAITING 1U
#define NOT_DATA_WAITING 0U
//compile with -D SLAVE_ONE if compiling for first slave, SLAVE_TWO if compiling for second
#ifdef SLAVE_ONE
	#define I2C_ADDR 0x02U;//i2c slave address
#elif  SLAVE_TWO
	#define I2C_ADDR 0x04U;
#else
	#error "I2C address not defined! Please define either SLAVE_ONE or SLAVE_TWO with the -D flag"
#endif



typedef enum
{
	I2C_OK,
	I2C_FAIL,
}I2C_STATUS;

volatile uint8_t globalData;//used for receiving I2C data
volatile uint8_t dataReady;//flag to indicate data received


void I2CInit(void);
