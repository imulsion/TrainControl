#ifndef IMPLEMENTATION_H
#define IMPLEMENTATION_H

#include "ftd2xx.h"
#include <iostream>
#include <cstdint>
#include <string>           //C++ string library
#include <thread>           //C++11 thread library
#include <mutex>            //C++11 mutex library
#include <functional>       //needed for std::ref

namespace top
{
	constexpr uint8_t SLAVE_ADDR_ONE   = 0x01U
	,                 SLAVE_ADDR_TWO       = 0x02U 
	,                 SPEED_MIN            = 0U       //minimum valid duty cycle value
	,                 SPEED_MAX            = 100U     //maximum valid duty cycle value
	,                 DUTY_CYCLE_FRACTION  = 100U     //Used to normalise duty cycle input to a fraction between 0 and 1
	,                 AVR_COUNTER_MAX      = 255U     //Maximum value of AVR counter register
	,                 NUM_SECTIONS         = 4U       //total number of track sections
	,                 INITIAL_DUTY_CYCLE   = 128U     //initial duty cycle value
	,                 SLAVE_ONE            = 1U       //owner of SLAVE_ADDR_ONE
	,                 SLAVE_TWO            = 2U       //owner of SLAVE_ADDR_TWO 
	,                 STATUS_OCCUPIED      = 1U       //code for section occupied
	,                 STATUS_LENGTH        = 4U;      //length of status buffer

	const std::string IN_QUIT              = "q"       //Expected user input for a quit command
	,                 IN_RESET             = "r"       //Expected user input for a reset command
	,                 IN_HELP              = "help"    //Expected user input for a help command 
	,                 IN_STATUS            = "s";      //Expected user input for a status command
}

namespace implementation
{
	constexpr uint8_t    TXBUFFER_LENGTH                     = 64U                                   //length of transmission buffer
	,                    RXBUFFER_LENGTH                     = 64U                                   //length of reception buffer
		,                    COMMAND_REPEAT                      = 4U;                                   //number of times to repeat commands to the MPSSE to ensure they are received
	constexpr uint32_t	     DELAY_LENGTH                        = 1000000U;                       //used to create a delay
}
namespace mpsse_init
{
	//magic numbers for MPSSE initialisation
	constexpr uint32_t   DEVICE_HANDLE	                 = 0U                                    //numeric ID for the device handle. Only one FTDI device connected, so ID is 0
	,           	     USB_IN_TRANSFER		         = 65536U                                //maximum USB I/O transfer size
	,		     USB_OUT_TRANSFER		         = 65535U
	,		     READ_TIMEOUT		         = 5000U                                 //USB read/write timeout
	,		     WRITE_TIMEOUT		         = 5000U
	,		     LATENCY_TIMER		         = 16U                                   //USB latency timer in ms
	,		     BIT_MODE_MASK		         = 0x0U                                  //Mode mask for the FT_SetBitMode function
	,		     MODE_RESET			         = 0x0U                                  //Command to set chip mode to mode defined in EEPROM
	,		     MODE_MPSSE			         = 0x02U;                                //Command to set chip mode to MPSSE mode
	constexpr uint8_t    EVT_CHAR			         = 0U                                    //Event or error special characters (all are disabled)
	,		     ERR_CHAR			         = 0U
	,		     EVT_CHAR_EN		         = 0U
	,		     ERR_CHAR_EN		         = 0U;
}
namespace mpsse_config
{
	//magic numbers for MPSSE configuration
	constexpr uint8_t    DIVIDE_CONFIG 		         = 0x8BU                                 //command to clock divider configuration
	,		     ADAPTIVE_CLK_CONFIG	         = 0x97U                                 //command to configure adaptive clocking
	,		     THREE_PHASE_CLK_ENABLE 	         = 0x8CU                                 //command to enable three-phase clocking 
	,		     DRIVE_ZERO_ENABLE 		         = 0x9EU                                 //command to enable drive zero mode
	,		     DRIVE_ZERO_LOWER 		         = 0x07U                                 //on the i2c bus lines (ADBUS 2:0)
	,		     DRIVE_ZERO_UPPER 		         = 0x00U                                 //disable drive zero mode on ACBUS and ADBUS 7:3
	,		     DISABLE_LOOPBACK 		         = 0x85U;                                //command to disable internal loopback
	
	//magic numbers for MPSSE clock configuration
	constexpr uint16_t   CLK_DIVIDER 		         = 0x0050U;                              //value used to calculate clock divider high and low bytes. Do not change high/low bytes directly, change this value to change clock divider value
	constexpr uint8_t    CLOCK_DIVIDER_HIGH_BYTE 	         = (CLK_DIVIDER >> 8U) & 0xFFU           //clock divider high and low bytes
	,		     CLOCK_DIVIDER_LOW_BYTE              = CLK_DIVIDER & 0xFFU
	,		     CMD_SET_DIVIDER 		         = 0x86U;                                //command to set the clock divider value

}
namespace system_reset
{
	constexpr uint8_t    SET_AVR_RESET_LOW                   = 0xF7U;                                //ADBUS value that will pull TMS (brown) wire low allowing for AVR reset pulse
}
namespace i2c
{
	constexpr uint8_t    ADBUS_SET_DIRECTION_COMMAND 	 = 0x80U                                 //Commands to set the data direction of the ADBUS and ACBUS
	,                    ACBUS_SET_DIRECTION_COMMAND         = 0x82U
	,		     SET_LINES_HIGH 	                 = 0xFFU                                 //Value that when written to a bus will drive all output enabled pins high. Usually used as a default bus setting
	,                    ADBUS_LINE_CONFIG                   = 0xFBU                                 //Data direction values for the ADBUS and ACBUS
	,                    ACBUS_LINE_CONFIG                   = 0x40U; 

}

//magic numbers for I2C start
namespace start
{
	constexpr uint8_t    ADBUS_CLEAR_SDA	                 = 0xDDU                                 //Values for ADBUS that will pull SDA or SCL low
	,		     ADBUS_CLEAR_SCL 	                 = 0xDCU
	,		     ACBUS_DATA 		         = 0xBFU;                                //ACBUS data value for I2C start condition
}

//magic numbers for I2C stop
namespace stop
{
	constexpr uint8_t    ADBUS_CLEAR_I2C_BUS	         = 0xDCU                                 //Values for ADBUS that will clear the bus, set SCL, or set SDA.
	,		     ADBUS_SET_SCL	                 = 0xDDU
	,		     ADBUS_SET_SDA	                 = 0xDFU
	,		     ACBUS_DATA 		         = 0xFFU;                                //ACBUS data value for I2C stop condition
}

//magic numbers for reading a byte
namespace read
{
	constexpr uint8_t   READ_DATA_COMMAND                    = 0x20U                                 //Command to read data from the chip's internal buffer
	,		    READ_LENGTH 	                 = 0x00U                                 //Number of bytes the MPSSE should attempt to read (a length of 0 means one byte)
	,		    CMD_FALLING_CLK                         = 0x13U                                 //Command to clock bits out on falling clock edges 
	,		    NAK_BYTE 		                 = 0xFFU                                 //When reading over I2C, the master must send a NAK before issuing a STOP command 
	,                   ACK_BYTE                             = 0x00U                                 //ACK byte for multiple byte reads
	,		    ADBUS_DATA                         = 0xDEU                                 //When written to ADBUS, this value will pull SCL low while releasing SDA 
	,		    MPSSE_SEND_IMMEDIATE_COMMAND         = 0x87U                                 //Command that instructs the MPSSE to immediately flush its data buffer back to the host PC
	,                   MAX_QUEUE_LENGTH                     = 1U;                                   //Maximum number of bytes expected to be read 
	constexpr DWORD     BYTE_READ_TIMEOUT                    = 5000;                                  //Read timeout in ms
}

//magic numbers for writing a byte
namespace write
{
	constexpr uint8_t   WRITE_DATA_COMMAND	                 = 0x11U                                 //Command to write data to the I2C bus
	,		    WRITE_LENGTH                         = 0x00U                                 //Number of bytes to write (a length of 0 means 1 byte)
	,		    ADBUS_DATA		                 = 0xDEU                                 //When written to ADBUS, this value will pull SCL low while releasing SDA 
	,		    READ_ACK_COMMAND	                 = 0x22U                                 //Command to clock bits in MSB first and on the rising edge of the clock
	,		    READ_LENGTH		                 = 0x00U                                 //Number of bytes to read (this is used when the master is looking for an ACK bit from the slave)
	,		    MPSSE_SEND_IMMEDIATE_COMMAND         = 0x87U                                 //Command that instructs the MPSSE to immediately flush its data buffer back to the host PC
	,		    ACK_MASK 		                 = 0x01U                                 //Mask to extract ACK bit from data byte
	,		    ACK_VALUE		                 = 0x00U                                 //Value of the ACK bit (0) as opposed to NAK (1)
	,                   MAX_QUEUE_LENGTH                     = 1U;                                   //Maximum number of bytes expected to be read 
	constexpr DWORD     BYTE_WRITE_TIMEOUT	                 = 5000;                                  //Write timeout in ms
}

//magic numbers for addressing
namespace address
{
	constexpr uint8_t   DATA_SHIFT_AMOUNT                    = 1U                                    //Number of bits to shift address data by
	,		    DATA_READ_MASK	                 = 0x01U                                 //Mask for the I2C address when a read is performed
	,	            DATA_WRITE_MASK	                 = 0xFEU;                                //Mask for the I2C address when a read is performed
}

typedef struct
{
	uint8_t dutyCycle;
	bool occupied;
	uint8_t owner;
}SectionT;



//TODO: Function header comments
void GetUserInput(std::string& input, bool& flag, std::mutex& mutex);
void Run(std::string& input, bool& flag, std::mutex& mutex);
/* Function: CableInit
 *
 * Establishes a connection to the cable connected to a USB port and sets up the USB device parameters for communication with the chip.
 *
 * Parameter[out]: handle               Pointer to a FT_HANDLE used for accessing the device
 * Returns:        FT_STATUS            FT_STATUS enum indicating the error state of the function. If the function executes successfully it will return the value FT_OK
 */
FT_STATUS CableInit(FT_HANDLE* handle);

//TODO: Decide if verify mode needed 
#ifdef VERIFY_MODE
FT_STATUS MPSSEVerify();
#endif

/* Sends MPSSE configuration parameters to the on-board FT232H chip, setting up the chip for I2C communication
 *
 * Parameter[in] handle                 Pointer to a FT_HANDLE used for accessing the device
 * Returns:      FT_STATUS              FT_STATUS enum indicating the error state of the function. If the function executes successfully it will return the value FT_OK
 */
FT_STATUS MPSSEConfig(FT_HANDLE* handle);

/* Applies a pulse to the TMS (brown) line of the C232HM cable which is designed to reset all I2C slaves at the same time
 *
 * Parameter[in] handle                 Pointer to a FT_HANDLE used for accessing the device
 * Returns:      FT_STATUS              FT_STATUS enum indicating the error state of the function. If the function executes successfully it will return the value FT_OK
 */

FT_STATUS SystemReset(FT_HANDLE* handle);

/* Sets the I2C lines to idle mode, i.e. both SCL and SDA released by the master
 *
 * Parameter[in] handle                 Pointer to a FT_HANDLE used for accessing the device
 * Returns:      FT_STATUS              FT_STATUS enum indicating the error state of the function. If the function executes successfully it will return the value FT_OK
 */
FT_STATUS SetI2CIdle(FT_HANDLE* handle);


/* Generates an I2C START condition on the lines by pulling both SCL and SDA low. 
 *
 * Parameter[in] handle                 Pointer to a FT_HANDLE used for accessing the device
 * Returns:      FT_STATUS              FT_STATUS enum indicating the error state of the function. If the function executes successfully it will return the value FT_OK
 */
FT_STATUS SetI2CStart(FT_HANDLE* handle);


/* Generates an I2C STOP condition on the lines by releasing both I2C lines and allowing them to be pulled high 
 *
 * Parameter[in] handle                 Pointer to a FT_HANDLE used for accessing the device
 * Returns:      FT_STATUS              FT_STATUS enum indicating the error state of the function. If the function executes successfully it will return the value FT_OK
 */
FT_STATUS SetI2CStop(FT_HANDLE* handle);


/* Reads a byte of data that has been received over the I2C bus by the FT232H chip on the cable 
 *
 * Parameter[in]  handle                Pointer to a FT_HANDLE used for accessing the device
 * Parameter[out] result                Reference to a uint8_t that can be used to store the received value  
 * Parameter[out] success               Reference to a flag that indicates whether the operation was successful
 * Returns:       FT_STATUS             FT_STATUS enum indicating the error state of the function. If the function executes successfully it will return the value FT_OK
 */
FT_STATUS ReadByte(FT_HANDLE* handle,uint8_t& result,bool& success);

//TODO: Comment header
FT_STATUS ReadSequence(FT_HANDLE* handle, uint8_t bytesToRead, uint8_t* data, bool& success);

/* Instructs the FT232H chip to write a byte of data onto the I2C bus lines 
 *
 * Parameter[in]  handle                Pointer to a FT_HANDLE used for accessing the device
 * Parameter[in]  data                  The data to be written to the lines
 * Parameter[out] success               Reference to a flag that indicates whether the operation was successful
 * Returns:       FT_STATUS             FT_STATUS enum indicating the error state of the function. If the function executes successfully it will return the value FT_OK
 */
FT_STATUS WriteByte(FT_HANDLE* handle,uint8_t data,bool& success);


/* Wrapper for the WriteByte function that takes an I2C address and writes it to the bus, addressing a device on the bus. Should always be called before attempting to write data onto the bus 
 *
 * Parameter[in]  handle                Pointer to a FT_HANDLE used for accessing the device
 * Parameter[in]  data                  The address of the device it is desired to interact with  
 * Parameter[in]  readNWrite            A flag indicating whether the desired interaction is a WRITE to the slave or a READ from the slave
 * Parameter[out] success               Reference to a flag that indicates whether the operation was successful
 * Returns:       FT_STATUS             FT_STATUS enum indicating the error state of the function. If the function executes successfully it will return the value FT_OK
 */
FT_STATUS WriteAddr(FT_HANDLE* handle,uint8_t data,bool readNWrite,bool& success);


/* Frees the device from the USB port, allowing it to be unplugged and the USB port reused. Typically called at the end of the program before termination 
 *
 * Parameter[in] handle                 Pointer to a FT_HANDLE used for accessing the device
 * Returns:      FT_STATUS              FT_STATUS enum indicating the error state of the function. If the function executes successfully it will return the value FT_OK
 */
FT_STATUS FreePort(FT_HANDLE* handle);


#endif //IMPLEMENTATION_H
