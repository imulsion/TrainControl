#include "implementation.h"

int main()
{
	FT_HANDLE deviceHandle;
	FT_STATUS deviceStatus = I2cInit(&deviceHandle);
	char input = '\0';
	bool success = false;
	if(FT_OK != deviceStatus)
	{
		std::cout<<"Error encountered during initialisation, program terminated"<<std::endl;
	}
	else
	{
		/*
		//TODO: Code function if necessary
		//initStatus = MPSSEVerify();
		if(FT_OK != initStatus)
		{
			std::cout<<"Error: MPSSE mode did not verify successfully. Program terminated"<<std::endl;
		}
		*/
		deviceStatus = MPSSEConfig(&deviceHandle);
		if(FT_OK != deviceStatus)
		{
			std::cout<<"Error: MPSSE configuration failed. Program terminated."<<std::endl;
		}
		else
		{
			deviceStatus = SetI2CIdle(&deviceHandle);
		}
		std::cout<<"Press l to toggle LED, q to quit"<<std::endl;
		while('q' != input)
		{
			std::cin>>input;
			if('l' == input)
			{
				deviceStatus |= SetI2CStart(&deviceHandle);
				deviceStatus |= WriteAddr(&deviceHandle,0x02,true,success);
				if(success)
				{
					success = false;
					deviceStatus |= WriteByte(&deviceHandle,(uint8_t)input,success);
					deviceStatus |= SetI2CStop(&deviceHandle);
					if(!success || (FT_OK != deviceStatus))
					{
						std::cout<<"Error during transmission"<<std::endl;
					}
				}
			}
		}


	}
	
	deviceStatus = FT_OK;
	
	deviceStatus = FreePort(&deviceHandle);
	if(FT_OK != deviceStatus)
	{
		std::cout<<"Termination error. Press any key to continue..."<<std::endl;
		std::cin.get();
	}
	
	
	return 0;
}
