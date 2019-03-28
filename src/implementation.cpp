#include "implementation.h"

//magic number namespace

namespace magic
{
	//magic numbers for MPSSE initialisation
	const uint32_t       DEVICE_HANDLE		 = 0U
	,           	     USB_IN_TRANSFER		 = 65536U
  	,		     USB_OUT_TRANSFER		 = 65535U
  	,		     READ_TIMEOUT		 = 5000U
  	,		     WRITE_TIMEOUT		 = 5000U
  	,		     LATENCY_TIMER		 = 16U
  	,		     BIT_MODE_MASK		 = 0x0U
  	,		     MODE_RESET			 = 0x0U
  	,		     MODE_MPSSE			 = 0x02U;
	const uint8_t        EVT_CHAR			 = 0U
	,		     ERR_CHAR			 = 0U
	,		     EVT_CHAR_EN		 = 0U
	,		     ERR_CHAR_EN		 = 0U
	//magic numbers for MPSSE configuration
	,                    DIVIDE_DISABLE 		 = 0x8AU
        ,		     ADAPTIVE_CLK_DISABLE 	 = 0x97U
	,		     THREE_PHASE_CLK_ENABLE 	 = 0x8CU
	,		     DRIVE_ZERO_ENABLE 		 = 0x9EU
	,		     DRIVE_ZERO_LOWER 		 = 0x07U
	,		     DRIVE_ZERO_UPPER 		 = 0x00U
	,		     DISABLE_LOOPBACK 		 = 0x85U
	
	//magic numbers for MPSSE clock configuration
	,		     CLK_DIVIDER 		 = 0xCBU
	,		     CMD_SET_DIVIDER 		 = 0x86U
	,		     SET_DIVIDER_LOW 		 = CLK_DIVIDER & 0xFFU
	,		     SET_DIVIDER_HIGH 		 = (CLK_DIVIDER >> 8U) & 0xFFU;

	//magic numbers for I2C idle
	namespace idle
	{
		const uint8_t       ADBUS_DIRECTION 	 = 0x80U
		,		    SET_LINES_HIGH 	 = 0xFFU
		,		    ADBUS_LINE_CONFIG 	 = 0xFBU
		,		    ACBUS_DIRECTION 	 = 0x82U
		,		    ACBUS_LINE_CONFIG 	 = 0x40U;
	}

	//magic numbers for I2C start
	namespace start
	{
		const uint8_t       ADBUS_DIRECTION 	 = 0x80U
		,		    ADBUS_DATA 		 = 0xFDU
		,		    ADBUS_CLOCK 	 = 0xFCU
		,		    ADBUS_LINE_CONFIG 	 = 0xFBU
		,		    ACBUS_DIRECTION 	 = 0x82U
		,		    ACBUS_DATA 		 = 0xBFU
		,		    ACBUS_LINE_CONFIG 	 = 0x40U;
	}

	//magic numbers for I2C stop
	namespace stop
	{
		const uint8_t       ADBUS_DIRECTION 	 = 0x80U
		,		    ADBUS_DATA1		 = 0xFCU
		,		    ADBUS_DATA2		 = 0xFDU
		,		    ADBUS_DATA3		 = 0xFFU
		,		    ADBUS_LINE_CONFIG 	 = 0xFBU
		,		    ACBUS_DIRECTION 	 = 0x82U
		,		    ACBUS_DATA 		 = 0xFFU
		,		    ACBUS_LINE_CONFIG 	 = 0x40U;
	}

	//magic numbers for reading a byte
	namespace read
	{
		const uint8_t       READ_DATA_CMD        = 0x20
		,		    READ_LENGTH 	 = 0x00
		,		    SEND_NAK_CMD         = 0x13
		,		    NAK_BYTE 		 = 0xFF
		,		    ADBUS_DIRECTION      = 0x80
		,		    ADBUS_DATA	         = 0xFE
		,		    ADBUS_LINE_CONFIG    = 0xFB
		,		    MPSSE_SEND_IMMEDIATE = 0x87;
		DWORD	   	    BYTE_READ_TIMEOUT    = 500;
	}
	
	//magic numbers for writing a byte
	namespace write
	{
		const uint8_t       WRITE_DATA_CMD	 = 0x11
 		,		    WRITE_LENGTH         = 0x00
 		,		    ADBUS_DIRECTION      = 0x80
 		,		    ADBUS_DATA		 = 0xFE
 		,		    ADBUS_LINE_CONFIG	 = 0xFB
 		,		    READ_ACK_CMD	 = 0x22
 		,		    READ_LENGTH		 = 0x00
 		,		    MPSSE_SEND_IMMEDIATE = 0x87
 		,		    ACK_MASK 		 = 0x01
 		,		    ACK_VALUE		 = 0x00;
 		DWORD               BYTE_WRITE_TIMEOUT	 = 500;
	}

	//magic numbers for addressing
	namespace address
	{
		const uint8_t       DATA_SHIFT_AMOUNT    = 1;
		const uint8_t       DATA_READ_MASK	 = 0x01
		,	            DATA_WRITE_MASK	 = 0xFE;
	}
}

//initialise the i2c connection
FT_STATUS I2cInit(FT_HANDLE* handle)
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
			error = FT_Open(magic::DEVICE_HANDLE,handle); //open the device for interaction
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
					error |= FT_SetUSBParameters(*handle,magic::USB_IN_TRANSFER,magic::USB_OUT_TRANSFER);//USB request transfer sizes
					error |= FT_SetChars(*handle,magic::EVT_CHAR,magic::EVT_CHAR_EN,magic::ERR_CHAR,magic::ERR_CHAR_EN); //don't want to enable special transfer characters
					error |= FT_SetTimeouts(*handle,magic::READ_TIMEOUT,magic::WRITE_TIMEOUT);//set timeouts
					error |= FT_SetLatencyTimer(*handle,magic::LATENCY_TIMER);//set latency timer (default: 16ms)
					error |= FT_SetBitMode(*handle,magic::BIT_MODE_MASK,magic::MODE_RESET);
					error |= FT_SetBitMode(*handle,magic::BIT_MODE_MASK,magic::MODE_MPSSE);
					
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
		txBuffer[bytesToSend++] = magic::DIVIDE_DISABLE; //disable divide-by-5 for 60MHz master clock
		txBuffer[bytesToSend++] = magic::ADAPTIVE_CLK_DISABLE; //Disable adaptive clocking
		txBuffer[bytesToSend++] = magic::THREE_PHASE_CLK_ENABLE; //Enable 3-phase data clocking
		txBuffer[bytesToSend++] = magic::DRIVE_ZERO_ENABLE; //Enable drive-zero mode on the I2C lines
		txBuffer[bytesToSend++] = magic::DRIVE_ZERO_LOWER;//Enable drive-zero for lower port
		txBuffer[bytesToSend++] = magic::DRIVE_ZERO_UPPER;//not needed on upper port
		txBuffer[bytesToSend++] = magic::DISABLE_LOOPBACK;//disable internal loopback
		
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
			txBuffer[bytesToSend++] = magic::CMD_SET_DIVIDER;//Send command to set clock divider
			txBuffer[bytesToSend++] = magic::SET_DIVIDER_LOW;//Lower byte of divider value
			txBuffer[bytesToSend++] = magic::SET_DIVIDER_HIGH;//higher byte
			error = FT_Write(*handle,txBuffer,bytesToSend,&bytesSent);
			if(FT_OK != error)
			{
				std::cout<<"Error: Clock frequency could not be set. Error code: "<<error<<std::endl;
			}
			else
			{
				//Configure I2C data lines ready to send
				std::cout<<"MPSSE successfully configured!"<<std::endl;
			}
		}
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
		uint8_t txBuffer[6];
		txBuffer[bytesToSend++] = magic::idle::ADBUS_DIRECTION;
		txBuffer[bytesToSend++] = magic::idle::SET_LINES_HIGH;
		txBuffer[bytesToSend++] = magic::idle::ADBUS_LINE_CONFIG;
		txBuffer[bytesToSend++] = magic::idle::ACBUS_DIRECTION;
		txBuffer[bytesToSend++] = magic::idle::SET_LINES_HIGH;
		txBuffer[bytesToSend++] = magic::idle::ACBUS_LINE_CONFIG;

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
		uint8_t txBuffer[30];
		for(dwCount=0; dwCount < 4; dwCount++) 
		{
			txBuffer[bytesToSend++] = magic::start::ADBUS_DIRECTION; 
			txBuffer[bytesToSend++] = magic::start::ADBUS_DATA; 
			txBuffer[bytesToSend++] = magic::start::ADBUS_LINE_CONFIG; 
		}
		for(dwCount=0; dwCount < 4; dwCount++) 
		{
			txBuffer[bytesToSend++] = magic::start::ADBUS_DIRECTION;
			txBuffer[bytesToSend++] = magic::start::ADBUS_CLOCK; 
			txBuffer[bytesToSend++] = magic::start::ADBUS_LINE_CONFIG; 
		}
		txBuffer[bytesToSend++] = magic::start::ACBUS_DIRECTION; 
		txBuffer[bytesToSend++] = magic::start::ACBUS_DATA; 
		txBuffer[bytesToSend++] = magic::start::ACBUS_LINE_CONFIG; 
	
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
		uint8_t txBuffer[50];
		for(dwCount=0; dwCount<4; dwCount++) 
		{
			txBuffer[bytesToSend++] = magic::stop::ADBUS_DIRECTION; 
			txBuffer[bytesToSend++] = magic::stop::ADBUS_DATA1; 
			txBuffer[bytesToSend++] = magic::stop::ADBUS_LINE_CONFIG;
		}
		for(dwCount=0; dwCount<4; dwCount++) 
		{
			txBuffer[bytesToSend++] = magic::stop::ADBUS_DIRECTION; 
			txBuffer[bytesToSend++] = magic::stop::ADBUS_DATA2; 
			txBuffer[bytesToSend++] = magic::stop::ADBUS_LINE_CONFIG;
		}
		for(dwCount=0; dwCount<4; dwCount++) 
		{
			txBuffer[bytesToSend++] = magic::stop::ADBUS_DIRECTION; 
			txBuffer[bytesToSend++] = magic::stop::ADBUS_DATA3;
			txBuffer[bytesToSend++] = magic::stop::ADBUS_LINE_CONFIG;
		}
		txBuffer[bytesToSend++] = magic::stop::ACBUS_DIRECTION; 
		txBuffer[bytesToSend++] = magic::stop::ACBUS_DATA; 
		txBuffer[bytesToSend++] = magic::stop::ACBUS_LINE_CONFIG; 

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
		uint8_t txBuffer[15];
		uint8_t inputBuffer[20];
		DWORD bytesToSend = 0; 
		DWORD bytesRead;
		DWORD bytesSent;
		txBuffer[bytesToSend++] = magic::read::READ_DATA_CMD; 
		txBuffer[bytesToSend++] = magic::read::READ_LENGTH; 
		txBuffer[bytesToSend++] = magic::read::READ_LENGTH; 
		txBuffer[bytesToSend++] = magic::read::SEND_NAK_CMD; 
		txBuffer[bytesToSend++] = magic::read::READ_LENGTH; 
		txBuffer[bytesToSend++] = magic::read::NAK_BYTE; 
		txBuffer[bytesToSend++] = magic::read::ADBUS_DIRECTION; 
		txBuffer[bytesToSend++] = magic::read::ADBUS_DATA; 
		txBuffer[bytesToSend++] = magic::read::ADBUS_LINE_CONFIG; 
		txBuffer[bytesToSend++] = magic::read::MPSSE_SEND_IMMEDIATE; 
		error = FT_Write(*handle, txBuffer, bytesToSend, &bytesSent);

		DWORD queueLength = 0;
		DWORD readTimeout = 0;	
		

		error = FT_GetQueueStatus(*handle, &queueLength);
		while ((queueLength < 1) && (FT_OK == error) && (readTimeout < magic::read::BYTE_READ_TIMEOUT))
		{
			error = FT_GetQueueStatus(*handle, &queueLength); 
			readTimeout ++;
		}
		if ((FT_OK == error) && (readTimeout < magic::read::BYTE_READ_TIMEOUT))
		{
			error = FT_Read(*handle, &inputBuffer, queueLength, &bytesRead);
			result = inputBuffer[0]; 
			success = true; 
		}
		else
		{
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
		uint8_t txBuffer[20];
		uint8_t inputBuffer[5];
		txBuffer[bytesToSend++] = magic::write::WRITE_DATA_CMD;
		txBuffer[bytesToSend++] = magic::write::WRITE_LENGTH;
		txBuffer[bytesToSend++] = magic::write::WRITE_LENGTH;
		txBuffer[bytesToSend++] = data;
		txBuffer[bytesToSend++] = magic::write::ADBUS_DIRECTION;
		txBuffer[bytesToSend++] = magic::write::ADBUS_DATA;
		txBuffer[bytesToSend++] = magic::write::ADBUS_LINE_CONFIG;
		txBuffer[bytesToSend++] = magic::write::READ_ACK_CMD;
		txBuffer[bytesToSend++] = magic::write::READ_LENGTH;
		txBuffer[bytesToSend++] = magic::write::MPSSE_SEND_IMMEDIATE;

		error = FT_Write(*handle,txBuffer,bytesToSend,&bytesSent);


		DWORD queueLength;
		DWORD writeTimeout = 0;

		error = FT_GetQueueStatus(*handle,&queueLength);

		while((queueLength < 1) && (FT_OK == error) && (writeTimeout < magic::write::BYTE_WRITE_TIMEOUT))
		{
			error = FT_GetQueueStatus(*handle,&queueLength);
			writeTimeout++;
		}

		if((FT_OK == error) && (writeTimeout < magic::write::BYTE_WRITE_TIMEOUT))
		{
			error = FT_Read(*handle,inputBuffer,queueLength,&bytesRead); 
			if(magic::write::ACK_VALUE == (inputBuffer[0] & magic::write::ACK_MASK))
			{
				success = true;
			}
			else
			{
				success = false;
			}
		}
		else
		{
			success = false;
		}
	}
	return error;
}

//1st I2C phase - write address to devices to establish which device to talk to

FT_STATUS WriteAddr(FT_HANDLE* handle, uint8_t data, bool readNWrite, bool& success)
{
	uint8_t nData = data;
	if(readNWrite)
	{
		//cast to uint8_t to comply with MISRA C++ Rule 5-0-10
		nData = static_cast<uint8_t>((nData << magic::address::DATA_SHIFT_AMOUNT) | magic::address::DATA_READ_MASK));
	}
	else
	{
		nData = static_cast<uint8_t>((nData << magic::address::DATA_SHIFT_AMOUNT) & magic::address::DATA_WRITE_MASK));
	}
	return WriteByte(handle,nData,success); 
}
//close the connection to the active COM port
FT_STATUS FreePort(FT_HANDLE* handle)
{
	FT_STATUS error;
	if(NULL ==  handle)
	{
		error = FT_INVALID_HANDLE;	
	}
	else
	{
		error = FT_SetBitMode(*handle,magic::BIT_MODE_MASK,magic::MODE_RESET);
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
