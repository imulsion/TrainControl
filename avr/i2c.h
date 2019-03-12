#define F_CPU 8000000UL
#define ERROR_PATTERN 0xAAU //error bit pattern is 0b10101010
#define PORTC_IO_CONFIG 0x3FU //enable input on PC4 and PC5 for internal pull-ups
#define I2C_CONTROL_CONFIG 0x45U //I2C control register configuration bits
#define GLOBAL_INTERRUPT_ENABLE
#define I2C_vect TWI_vect //AVR uses TWI (two-wire interface) to avoid I2C copyright. Create alias for interrupt vector here
#define I2C_INTERRUPT_CLEAR_MASK 0x80U//used for clearing I2C interrupt flag 
#define I2C_WRITING 0x60U //status code for receiving an I2C byte	
#define I2C_READING 0xA8U //status code for receipt of ACK after a write to the PC
#define DATA_WAITING 1U;
#define NOT_DATA_WAITING 0U;
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

uint8_t globalData = 0U;//used for receiving I2C data
uint8_t dataReady = NOT_DATA_WAITING;//flag to indicate data received


void I2CInit(void);
