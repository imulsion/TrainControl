#include "implementation.h"
#include <string>
//TODO: Magic numbers and some comments. Code will likely be radically changed soon though, so don't try too hard.

const uint8_t SLAVE_ADDR_ONE    = 0x01
,             SLAVE_ADDR_TWO    = 0x02; 


int main()
{
	FT_HANDLE deviceHandle;
	FT_STATUS deviceStatus = CableInit(&deviceHandle);
	bool success = false;
	uint8_t dutyCycle = 0U; 
        std::string input;
	std::string::const_iterator iterator;
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
		std::cout<<"Type a number between 0 and 100 to set the duty cycle of the PWM wave. Press q to quit."<<std::endl;
		while(true)
		{
		        getline(std::cin,input);
			if("q" == input)
			{
				break;
			}
			try
			{
				dutyCycle = std::stoi(input,NULL);
			}
			catch(const std::invalid_argument& e)
			{
				std::cerr<<"Error: Invalid input"<<std::endl;
				goto END;
			}
			if((0 <= dutyCycle) && (100 >= dutyCycle))
			{
				dutyCycle = static_cast<int>(256 * (static_cast<float>(dutyCycle) / 100));
				deviceStatus = SetI2CStart(&deviceHandle);
				deviceStatus |= WriteAddr(&deviceHandle,SLAVE_ADDR_ONE,false,success);
				if(success && (FT_OK == deviceStatus))
				{
					deviceStatus = WriteByte(&deviceHandle,dutyCycle,success);
					if(!success || (FT_OK != deviceStatus))
					{
						std::cout<<"Error during data transmission"<<std::endl;	
					}
					deviceStatus = SetI2CStop(&deviceHandle);
					deviceStatus |= SetI2CIdle(&deviceHandle);
					if(FT_OK != deviceStatus)
					{
						std::cout<<"An error occurred while attempting to transmit an I2C STOP condition on the bus lines. A system reset may be required."<<std::endl;
					}
				}	
				else
				{
					std::cout<<"Error during address transmission"<<std::endl;
				}

			}
			else
			{
				std::cout<<"Invalid input"<<std::endl;
			}
		END:
			;
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
