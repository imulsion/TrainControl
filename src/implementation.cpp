#include "implementation.h"

namespace implementation
{
	const uint8_t        TXBUFFER_LENGTH                     = 64U                                   //length of transmission buffer
	,                    RXBUFFER_LENGTH                     = 64U                                   //length of reception buffer
        ,                    COMMAND_REPEAT                      = 4U;                                   //number of times to repeat commands to the MPSSE to ensure they are received
}
namespace mpsse_init
{
	//magic numbers for MPSSE initialisation
	const uint32_t       DEVICE_HANDLE	                 = 0U                                    //numeric ID for the device handle. Only one FTDI device connected, so ID is 0
	,           	     USB_IN_TRANSFER		         = 65536U                                //maximum USB I/O transfer size
	,		     USB_OUT_TRANSFER		         = 65535U
	,		     READ_TIMEOUT		         = 5000U                                 //USB read/write timeout
	,		     WRITE_TIMEOUT		         = 5000U
	,		     LATENCY_TIMER		         = 16U                                   //USB latency timer in ms
	,		     BIT_MODE_MASK		         = 0x0U                                  //Mode mask for the FT_SetBitMode function
	,		     MODE_RESET			         = 0x0U                                  //Command to set chip mode to mode defined in EEPROM
	,		     MODE_MPSSE			         = 0x02U;                                //Command to set chip mode to MPSSE mode
	const uint8_t        EVT_CHAR			         = 0U                                    //Event or error special characters (all are disabled)
	,		     ERR_CHAR			         = 0U
	,		     EVT_CHAR_EN		         = 0U
	,		     ERR_CHAR_EN		         = 0U;
}
namespace mpsse_config
{
	//magic numbers for MPSSE configuration
	const uint8_t        DIVIDE_CONFIG 		         = 0x8BU                                 //command to clock divider configuration
	,		     ADAPTIVE_CLK_CONFIG	         = 0x97U                                 //command to configure adaptive clocking
	,		     THREE_PHASE_CLK_ENABLE 	         = 0x8CU                                 //command to enable three-phase clocking 
	,		     DRIVE_ZERO_ENABLE 		         = 0x9EU                                 //command to enable drive zero mode
	,		     DRIVE_ZERO_LOWER 		         = 0x07U                                 //on the i2c bus lines (ADBUS 2:0)
	,		     DRIVE_ZERO_UPPER 		         = 0x00U                                 //disable drive zero mode on ACBUS and ADBUS 7:3
	,		     DISABLE_LOOPBACK 		         = 0x85U;                                //command to disable internal loopback
	
	//magic numbers for MPSSE clock configuration
	const uint16_t       CLK_DIVIDER 		         = 0x0027U;                              //value used to calculate clock divider high and low bytes. Do not change high/low bytes directly, change this value to change clock divider value
	const uint8_t        CLOCK_DIVIDER_HIGH_BYTE 	         = (CLK_DIVIDER >> 8U) & 0xFFU           //clock divider high and low bytes
	,		     CLOCK_DIVIDER_LOW_BYTE              = CLK_DIVIDER & 0xFFU
	,		     CMD_SET_DIVIDER 		         = 0x86U;                                //command to set the clock divider value

}
namespace system_reset
{
	const uint8_t        SET_AVR_RESET_LOW                   = 0xEFU;                                //ADBUS value that will pull GPIO3 (blue) wire low allowing for AVR reset pulse
}
namespace i2c
{
	const uint8_t        ADBUS_SET_DIRECTION_COMMAND 	 = 0x80U                                 //Commands to set the data direction of the ADBUS and ACBUS
	,                    ACBUS_SET_DIRECTION_COMMAND         = 0x82U
	,		     SET_LINES_HIGH 	                 = 0xFFU                                 //Value that when written to a bus will drive all output enabled pins high. Usually used as a default bus setting
	,                    ADBUS_LINE_CONFIG                   = 0xFBU                                 //Data direction values for the ADBUS and ACBUS
	,                    ACBUS_LINE_CONFIG                   = 0x40U; 

}

//magic numbers for I2C start
namespace start
{
	const uint8_t        ADBUS_CLEAR_SDA	                 = 0xDDU                                 //Values for ADBUS that will pull SDA or SCL low
	,		     ADBUS_CLEAR_SCL 	                 = 0xDCU
	,		     ACBUS_DATA 		         = 0xBFU;                                //ACBUS data value for I2C start condition
}

//magic numbers for I2C stop
namespace stop
{
	const uint8_t        ADBUS_CLEAR_I2C_BUS	         = 0xDCU                                 //Values for ADBUS that will clear the bus, set SCL, or set SDA.
	,		     ADBUS_SET_SCL	                 = 0xDDU
	,		     ADBUS_SET_SDA	                 = 0xDFU
	,		     ACBUS_DATA 		         = 0xFFU;                                //ACBUS data value for I2C stop condition
}

//magic numbers for reading a byte
namespace read
{
	const uint8_t       READ_DATA_COMMAND                    = 0x20U                                 //Command to read data from the chip's internal buffer
	,		    READ_LENGTH 	                 = 0x00U                                 //Number of bytes the MPSSE should attempt to read (a length of 0 means one byte)
	,		    SEND_NAK_CMD                         = 0x13U                                 //Command to clock bits out on falling clock edges 
	,		    NAK_BYTE 		                 = 0xFFU                                 //When reading over I2C, the master must send a NAK before issuing a STOP command 
	,		    ADBUS_DATA	                         = 0xDEU                                 //When written to ADBUS, this value will pull SCL low while releasing SDA 
	,		    MPSSE_SEND_IMMEDIATE_COMMAND         = 0x87U                                 //Command that instructs the MPSSE to immediately flush its data buffer back to the host PC
	,                   MAX_QUEUE_LENGTH                     = 1U;                                   //Maximum number of bytes expected to be read 
	DWORD	   	    BYTE_READ_TIMEOUT                    = 500;                                  //Read timeout in ms
}

//magic numbers for writing a byte
namespace write
{
	const uint8_t       WRITE_DATA_COMMAND	                 = 0x11U                                 //Command to write data to the I2C bus
	,		    WRITE_LENGTH                         = 0x00U                                 //Number of bytes to write (a length of 0 means 1 byte)
	,		    ADBUS_DATA		                 = 0xDEU                                 //When written to ADBUS, this value will pull SCL low while releasing SDA 
	,		    READ_ACK_COMMAND	                 = 0x22U                                 //Command to clock bits in MSB first and on the rising edge of the clock
	,		    READ_LENGTH		                 = 0x00U                                 //Number of bytes to read (this is used when the master is looking for an ACK bit from the slave)
	,		    MPSSE_SEND_IMMEDIATE_COMMAND         = 0x87U                                 //Command that instructs the MPSSE to immediately flush its data buffer back to the host PC
	,		    ACK_MASK 		                 = 0x01U                                 //Mask to extract ACK bit from data byte
	,		    ACK_VALUE		                 = 0x00U                                 //Value of the ACK bit (0) as opposed to NAK (1)
	,                   MAX_QUEUE_LENGTH                     = 1U;                                   //Maximum number of bytes expected to be read 
	DWORD               BYTE_WRITE_TIMEOUT	                 = 500;                                  //Write timeout in ms
}

//magic numbers for addressing
namespace address
{
	const uint8_t       DATA_SHIFT_AMOUNT                    = 1U                                    //Number of bits to shift address data by
	,		    DATA_READ_MASK	                 = 0x01U                                 //Mask for the I2C address when a read is performed
	,	            DATA_WRITE_MASK	                 = 0xFEU;                                //Mask for the I2C address when a read is performed
}

//initialise the i2c connection
FT_STATUS CableInit(FT_HANDLE* handle)
{
	FT_STATUS error;
	if(NULL ==  handle)
	{
		error = FT_INVALID_HANDLE;	
	}
	else
	{
		DWORD devices; //device list
		DWORD rxQueue; //receive queue - number of bytes in buffer
		DWORD rxBuffer; //receive buffer
		DWORD bytesRead; //how many bytes were read?
		error = FT_CreateDeviceInfoList(&devices); //create device information list


		if(FT_OK != error)
		{
			std::cout<<"Error during the creation of the device list. Code: "<<error<<std::endl;
		}
		else
		{
			std::cout<<"Number of devices: "<<devices<<std::endl; //print number of connected devices
			error = FT_Open(mpsse_init::DEVICE_HANDLE,handle); //open the device for interaction
			if(FT_OK != error)
			{
				std::cout<<"Error opening handle: "<<error<<std::endl;
			}
			else
			{
				
				error = FT_ResetDevice(*handle); //reset device to begin initialisation
				if(FT_OK != error)
				{
					std::cout<<"Error on reset: "<<error<<std::endl;
				}
				else
				{
					error = FT_GetQueueStatus(*handle,&rxQueue);
					if((FT_OK == error)&&(rxQueue > 0))
					{
						error = FT_Read(*handle,&rxBuffer,rxQueue,&bytesRead);
					}
					error |= FT_SetUSBParameters(*handle,mpsse_init::USB_IN_TRANSFER,mpsse_init::USB_OUT_TRANSFER);//USB request transfer sizes
					error |= FT_SetChars(*handle,mpsse_init::EVT_CHAR,mpsse_init::EVT_CHAR_EN,mpsse_init::ERR_CHAR,mpsse_init::ERR_CHAR_EN); //don't want to enable special transfer characters
					error |= FT_SetTimeouts(*handle,mpsse_init::READ_TIMEOUT,mpsse_init::WRITE_TIMEOUT);//set timeouts
					error |= FT_SetLatencyTimer(*handle,mpsse_init::LATENCY_TIMER);//set latency timer (default: 16ms)
					error |= FT_SetBitMode(*handle,mpsse_init::BIT_MODE_MASK,mpsse_init::MODE_RESET);
					error |= FT_SetBitMode(*handle,mpsse_init::BIT_MODE_MASK,mpsse_init::MODE_MPSSE);
					
					if(FT_OK != error)
					{
						std::cout<<"C232HM initialisation failed! Error code: "<<error<<std::endl;
					}
					else
					{
						std::cout<<"C232HM initialisation successful!"<<std::endl;
					}
				}
			}
		}
	}	
	return error;
}

#ifdef VERIFY_MODE
//TODO: Implement later (necessary?)
 FT_STATUS MPSSEVerify()
{
	DWORD bytesToSend = 0;
}
#endif 
//configure the MPSSE settings on chip
FT_STATUS MPSSEConfig(FT_HANDLE* handle)
{
	FT_STATUS error;
	if(NULL ==  handle)
	{
		error = FT_INVALID_HANDLE;	
	}
	else
	{
		DWORD bytesToSend = 0;
		DWORD bytesSent;
		uint8_t txBuffer[16];
		txBuffer[bytesToSend++] = mpsse_config::DIVIDE_CONFIG; //configure divide-by-5 for 60MHz master clock
		txBuffer[bytesToSend++] = mpsse_config::ADAPTIVE_CLK_CONFIG; //Set adaptive clocking state
		txBuffer[bytesToSend++] = mpsse_config::THREE_PHASE_CLK_ENABLE; //Enable 3-phase data clocking
		txBuffer[bytesToSend++] = mpsse_config::DRIVE_ZERO_ENABLE; //Enable drive-zero mode on the I2C lines
		txBuffer[bytesToSend++] = mpsse_config::DRIVE_ZERO_LOWER;//Enable drive-zero for lower port
		txBuffer[bytesToSend++] = mpsse_config::DRIVE_ZERO_UPPER;//not needed on upper port
		txBuffer[bytesToSend++] = mpsse_config::DISABLE_LOOPBACK;//disable internal loopback
		
		//write settings to chip
		error = FT_Write(*handle,txBuffer,bytesToSend,&bytesSent);
		if(FT_OK != error)
		{
			std::cout<<"Error: Write failed during configuration. Error code: "<<error<<std::endl;
		}
		else
		{
			//Configure clock dividers to set desired I2C frequency (100kHz)
			bytesToSend = 0;
			txBuffer[bytesToSend++] = mpsse_config::CMD_SET_DIVIDER;//Send command to set clock divider
			txBuffer[bytesToSend++] = mpsse_config::CLOCK_DIVIDER_LOW_BYTE;//Lower byte of divider value
			txBuffer[bytesToSend++] = mpsse_config::CLOCK_DIVIDER_HIGH_BYTE;//higher byte
			
			error = FT_Write(*handle,txBuffer,bytesToSend,&bytesSent);
			if(FT_OK != error)
			{
				std::cout<<"Error: Clock frequency could not be set. Error code: "<<error<<std::endl;
			}
			else
			{
				std::cout<<"MPSSE successfully configured!"<<std::endl;
			}
		}
	}
	return error;
}

FT_STATUS SystemReset(FT_HANDLE* handle)
{
	FT_STATUS error;
	if(NULL == handle)
	{
		return FT_INVALID_HANDLE;
	}
	else
	{
		DWORD bytesToSend = 0;
		DWORD bytesSent; //return param to confirm number of bytes successfully sent
		DWORD dwCount;
		uint8_t txBuffer[implementation::TXBUFFER_LENGTH];
		
		//write a 1 to the AVR reset line
		for(dwCount = 0; dwCount<implementation::COMMAND_REPEAT; dwCount++)
		{
			txBuffer[bytesToSend++] = i2c::ADBUS_SET_DIRECTION_COMMAND;
			txBuffer[bytesToSend++] = i2c::SET_LINES_HIGH;
			txBuffer[bytesToSend++] = i2c::ADBUS_LINE_CONFIG;
		}
		//pull AVR RESET low
		for(dwCount = 0; dwCount<implementation::COMMAND_REPEAT; dwCount++)
		{
			txBuffer[bytesToSend++] = i2c::ADBUS_SET_DIRECTION_COMMAND;
			txBuffer[bytesToSend++] = system_reset::SET_AVR_RESET_LOW;
			txBuffer[bytesToSend++] = i2c::ADBUS_LINE_CONFIG;
		}
		//return the line high 
		for(dwCount = 0; dwCount<implementation::COMMAND_REPEAT; dwCount++)
		{
			txBuffer[bytesToSend++] = i2c::ADBUS_SET_DIRECTION_COMMAND;
			txBuffer[bytesToSend++] = i2c::SET_LINES_HIGH;
			txBuffer[bytesToSend++] = i2c::ADBUS_LINE_CONFIG;
		}
		error = FT_Write(*handle,txBuffer,bytesToSend,&bytesSent);
		
	}
	return error;
}

//set I2C lines to idle mode
FT_STATUS SetI2CIdle(FT_HANDLE* handle)
{
	FT_STATUS error;
	if(NULL ==  handle)
	{
		error = FT_INVALID_HANDLE;	
	}
	else
	{
		DWORD bytesToSend = 0; 
		DWORD bytesSent;
		uint8_t txBuffer[implementation::TXBUFFER_LENGTH];

		//write bus values for an I2C idle state (both lines released)
		txBuffer[bytesToSend++] = i2c::ADBUS_SET_DIRECTION_COMMAND;
		txBuffer[bytesToSend++] = i2c::SET_LINES_HIGH;
		txBuffer[bytesToSend++] = i2c::ADBUS_LINE_CONFIG;
		txBuffer[bytesToSend++] = i2c::ACBUS_SET_DIRECTION_COMMAND;
		txBuffer[bytesToSend++] = i2c::SET_LINES_HIGH;
		txBuffer[bytesToSend++] = i2c::ACBUS_LINE_CONFIG;
		
		error = FT_Write(*handle,txBuffer,bytesToSend,&bytesSent);
		
	}
	return error;
	
}


FT_STATUS SetI2CStart(FT_HANDLE* handle)
{
	FT_STATUS error;
	if(NULL ==  handle)
	{
		error = FT_INVALID_HANDLE;	
	}
	else
	{
		DWORD bytesToSend = 0;
		DWORD bytesSent;	
		DWORD dwCount;
		uint8_t txBuffer[implementation::TXBUFFER_LENGTH];

		//pull SDA low
		for(dwCount=0; dwCount<implementation::COMMAND_REPEAT; dwCount++) 
		{
			txBuffer[bytesToSend++] = i2c::ADBUS_SET_DIRECTION_COMMAND; 
			txBuffer[bytesToSend++] = start::ADBUS_CLEAR_SDA; 
			txBuffer[bytesToSend++] = i2c::ADBUS_LINE_CONFIG; 
		}
		//pull SCL low 
		for(dwCount=0; dwCount<implementation::COMMAND_REPEAT; dwCount++) 
		{
			txBuffer[bytesToSend++] = i2c::ADBUS_SET_DIRECTION_COMMAND;
			txBuffer[bytesToSend++] = start::ADBUS_CLEAR_SCL; 
			txBuffer[bytesToSend++] = i2c::ADBUS_LINE_CONFIG; 
		}
                //write ACBUS data
		for(dwCount=0; dwCount<implementation::COMMAND_REPEAT; dwCount++) 
		{
			txBuffer[bytesToSend++] = i2c::ACBUS_SET_DIRECTION_COMMAND; 
			txBuffer[bytesToSend++] = start::ACBUS_DATA; 
			txBuffer[bytesToSend++] = i2c::ACBUS_LINE_CONFIG; 
		}
		error = FT_Write(*handle, txBuffer, bytesToSend, &bytesSent);
	}
	return error;
}

FT_STATUS SetI2CStop(FT_HANDLE* handle)
{
	FT_STATUS error;
	if(NULL ==  handle)
	{
		error = FT_INVALID_HANDLE;	
	}
	else
	{
		DWORD bytesToSend = 0; 
		DWORD dwCount;
		DWORD bytesSent;
		uint8_t txBuffer[implementation::TXBUFFER_LENGTH]; 
		
		//pull both I2C lines low
		for(dwCount=0; dwCount<implementation::COMMAND_REPEAT; dwCount++) 
		{
			txBuffer[bytesToSend++] = i2c::ADBUS_SET_DIRECTION_COMMAND; 
			txBuffer[bytesToSend++] = stop::ADBUS_CLEAR_I2C_BUS; 
			txBuffer[bytesToSend++] = i2c::ADBUS_LINE_CONFIG;
		}
		//release SCL
		for(dwCount=0; dwCount<implementation::COMMAND_REPEAT; dwCount++) 
		{
			txBuffer[bytesToSend++] = i2c::ADBUS_SET_DIRECTION_COMMAND; 
			txBuffer[bytesToSend++] = stop::ADBUS_SET_SCL; 
			txBuffer[bytesToSend++] = i2c::ADBUS_LINE_CONFIG;
		}
		//release SDA
		for(dwCount=0; dwCount<implementation::COMMAND_REPEAT; dwCount++) 
		{
			txBuffer[bytesToSend++] = i2c::ADBUS_SET_DIRECTION_COMMAND; 
			txBuffer[bytesToSend++] = stop::ADBUS_SET_SDA;
			txBuffer[bytesToSend++] = i2c::ADBUS_LINE_CONFIG;
		}
		//write ACBUS data
		txBuffer[bytesToSend++] = i2c::ACBUS_SET_DIRECTION_COMMAND; 
		txBuffer[bytesToSend++] = stop::ACBUS_DATA; 
		txBuffer[bytesToSend++] = i2c::ACBUS_LINE_CONFIG; 

		error = FT_Write(*handle, txBuffer, bytesToSend, &bytesSent);
	}
	return error;
}

FT_STATUS ReadByte(FT_HANDLE* handle,uint8_t& result,bool& success)
{
	FT_STATUS error;
	if(NULL ==  handle)
	{
		error = FT_INVALID_HANDLE;	
	}
	else
	{
		uint8_t txBuffer[implementation::TXBUFFER_LENGTH];
		uint8_t inputBuffer[implementation::RXBUFFER_LENGTH];
		DWORD bytesToSend = 0; 
		DWORD bytesRead;
		DWORD bytesSent;
		//read I2C byte
		txBuffer[bytesToSend++] = read::READ_DATA_COMMAND; 
		txBuffer[bytesToSend++] = read::READ_LENGTH; 
		txBuffer[bytesToSend++] = read::READ_LENGTH; 
		txBuffer[bytesToSend++] = read::SEND_NAK_CMD; 
		txBuffer[bytesToSend++] = read::READ_LENGTH; 
		txBuffer[bytesToSend++] = read::NAK_BYTE; 
		txBuffer[bytesToSend++] = i2c::ADBUS_SET_DIRECTION_COMMAND; 
		txBuffer[bytesToSend++] = read::ADBUS_DATA; 
		txBuffer[bytesToSend++] = i2c::ADBUS_LINE_CONFIG; 
		txBuffer[bytesToSend++] = read::MPSSE_SEND_IMMEDIATE_COMMAND; 
		error = FT_Write(*handle, txBuffer, bytesToSend, &bytesSent);

		DWORD queueLength = 0;
		DWORD readTimeout = 0;	
		

		error = FT_GetQueueStatus(*handle, &queueLength); //read the number of bytes in the queue

		//wait until timeout, an error, or a byte is waiting to be read
		while ((queueLength < read::MAX_QUEUE_LENGTH) && (FT_OK == error) && (readTimeout < read::BYTE_READ_TIMEOUT))
		{
			error = FT_GetQueueStatus(*handle, &queueLength); 
			readTimeout ++;
		}
		if ((FT_OK == error) && (readTimeout < read::BYTE_READ_TIMEOUT))
		{
			//read a byte if one is waiting
			error = FT_Read(*handle, &inputBuffer, queueLength, &bytesRead);
			result = inputBuffer[0]; 
			//read was successful
			success = true; 
		}
		else
		{
			//read failed due to an error or timeout
			success = false;
		}
	}
	return error;
}	

FT_STATUS WriteByte(FT_HANDLE* handle,uint8_t data,bool& success)
{
	FT_STATUS error;
	if(NULL ==  handle)
	{
		error = FT_INVALID_HANDLE;	
	}
	else
	{
		DWORD bytesToSend = 0;
		DWORD bytesSent;
		DWORD bytesRead;
		uint8_t txBuffer[implementation::TXBUFFER_LENGTH];
		uint8_t inputBuffer[implementation::RXBUFFER_LENGTH];

		//write an I2C byte to the lines
		txBuffer[bytesToSend++] = write::WRITE_DATA_COMMAND;
		txBuffer[bytesToSend++] = write::WRITE_LENGTH;
		txBuffer[bytesToSend++] = write::WRITE_LENGTH;
		txBuffer[bytesToSend++] = data;
		txBuffer[bytesToSend++] = i2c::ADBUS_SET_DIRECTION_COMMAND;
		txBuffer[bytesToSend++] = write::ADBUS_DATA;
		txBuffer[bytesToSend++] = i2c::ADBUS_LINE_CONFIG;
		txBuffer[bytesToSend++] = write::READ_ACK_COMMAND;
		txBuffer[bytesToSend++] = write::READ_LENGTH;
		txBuffer[bytesToSend++] = write::MPSSE_SEND_IMMEDIATE_COMMAND;

		error = FT_Write(*handle,txBuffer,bytesToSend,&bytesSent);


		DWORD queueLength;
		DWORD writeTimeout = 0;

		error = FT_GetQueueStatus(*handle,&queueLength);

		//wait for a timeout, a read error, or a byte to be read
		while((queueLength < write::MAX_QUEUE_LENGTH) && (FT_OK == error) && (writeTimeout < write::BYTE_WRITE_TIMEOUT))
		{
			error = FT_GetQueueStatus(*handle,&queueLength);
			writeTimeout++;
		}

		if((FT_OK == error) && (writeTimeout < write::BYTE_WRITE_TIMEOUT))
		{
			error = FT_Read(*handle,inputBuffer,queueLength,&bytesRead); 
			//test if an ACK bit was received
			if(write::ACK_VALUE == (inputBuffer[0] & write::ACK_MASK))
			{
				//ACK received, write successful
				success = true;
			}
			else
			{
				//NAK received, write failed
				success = false;
			}
		}
		else
		{
			//write failed due to an error or timeout
			success = false;
		}
	}
	return error;
}

FT_STATUS WriteAddr(FT_HANDLE* handle, uint8_t data, bool readNWrite, bool& success)
{
	uint8_t nData = data;
	if(readNWrite)
	{
		//cast to uint8_t to comply with MISRA C++ Rule 5-0-10
		//>shift the I2C address DATA_SHIFT_AMOUNT bits right and mask the R/!W bit
		//>this creates a valid I2C address (i.e. 7 address bits followed by the R/!W bit
		nData = static_cast<uint8_t>((nData << address::DATA_SHIFT_AMOUNT) | address::DATA_READ_MASK);
	}
	else
	{
		nData = static_cast<uint8_t>((nData << address::DATA_SHIFT_AMOUNT) & address::DATA_WRITE_MASK);
	}
	return WriteByte(handle,nData,success); 
}


FT_STATUS FreePort(FT_HANDLE* handle)
{
	FT_STATUS error;
	if(NULL ==  handle)
	{
		error = FT_INVALID_HANDLE;	
	}
	else
	{
		//reset chip mode to the mode specified in the chip EEPROM
		error = FT_SetBitMode(*handle,mpsse_init::BIT_MODE_MASK,mpsse_init::MODE_RESET);
		//close connection to the device
		error |= FT_Close(*handle);
		if(FT_OK != error)
		{
			std::cout<<"Error while terminating communication with device. Error code "<<error<<std::endl;
		}
		else
		{
			std::cout<<"Device disconnected successfully. Program will now terminate. "<<std::endl;
		}
	}
	return error;
}
