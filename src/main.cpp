#include "implementation.h"

//TODO: Magic numbers and some comments. Code will likely be radically changed soon though, so don't try too hard.
int main()
{
	FT_HANDLE deviceHandle;
	FT_STATUS deviceStatus = CableInit(&deviceHandle);
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
			if(FT_OK != deviceStatus)
			{
				std::cout<<"Error writing idle state, code: "<<deviceStatus<<std::endl;
			}
		}
		std::cout<<"Press l to toggle LED, q to quit"<<std::endl;
		while('q' != input)
		{
			std::cin>>input;
			if('l' == input)
			{
				deviceStatus |= SetI2CStart(&deviceHandle);
				deviceStatus |= WriteAddr(&deviceHandle,0x01,false,success);
				if(FT_OK != deviceStatus)
				{
					std::cout<<"Error writing address, code: "<<deviceStatus<<std::endl;
				}
				else
				{
					if(success)
					{
						success = false;
						deviceStatus |= WriteByte(&deviceHandle,0xC5,success);
						deviceStatus |= SetI2CStop(&deviceHandle);
						if(!success || (FT_OK != deviceStatus))
						{
							std::cout<<"Error during byte transmission"<<std::endl;
						}
					}
					else
					{
						std::cout<<"Error during address transmission. Code: "<<deviceStatus<<std::endl;
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
