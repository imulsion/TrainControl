#include "implementation.h" //implementation specific header

int main(void)
{
	//threading variables
	bool inputFlag = false;
	std::string inputString = "";

	std::mutex inputMutex;
	std::thread inputThread(GetUserInput,std::ref(inputString),std::ref(inputFlag), std::ref(inputMutex)); 
	std::thread runThread(Run,std::ref(inputString),std::ref(inputFlag), std::ref(inputMutex));

	inputThread.detach();
	runThread.join();

	return 0; //end of main function
}



void GetUserInput(std::string& input, bool& flag, std::mutex& mutex)
{
	std::string userInput;
	while(true)
	{
		getline(std::cin,userInput); //wait for input
		mutex.lock();//wait for lock
		input= userInput;
		flag = true;
		mutex.unlock();
	}
}

void Run(std::string& input, bool& flag, std::mutex& mutex)
{
	FT_HANDLE deviceHandle;                                //Handle to the device 
	bool success = false;                                  //Flag to indicate success of data transmission functions
	uint8_t trainSpeed = 0U;                                //PWM duty cycle
	bool commsFlag = false;                                //flag for indicating user input waiting to be processed, essentially a copy of the mutex protected flag parameter
	std::string userInput = "";                            //local copy of mutex protected user input string

	SectionT sections[top::NUM_SECTIONS];
	uint8_t statusBuffer[implementation::RXBUFFER_LENGTH];

	for(uint8_t i = 0;i<top::NUM_SECTIONS;i++)
	{
		sections[i].dutyCycle = top::INITIAL_DUTY_CYCLE;	
	}
        sections[0].owner = top::SLAVE_ONE;
	sections[1].owner = top::SLAVE_ONE;
	sections[2].owner = top::SLAVE_TWO;
	sections[3].owner = top::SLAVE_TWO;

	sections[0].occupied = true;
	sections[1].occupied = false;
	sections[2].occupied = true;
	sections[3].occupied = false;

	FT_STATUS deviceStatus = CableInit(&deviceHandle);     //initialise cable
	if(FT_OK != deviceStatus)
	{
		std::cout<<"Error encountered during initialisation, program terminated"<<std::endl;
	}
	else
	{
#if 0
		//TODO: Code function if necessary
		//initStatus = MPSSEVerify();
		if(FT_OK != initStatus)
		{
			std::cout<<"Error: MPSSE mode did not verify successfully. Program terminated"<<std::endl;
		}
#endif
	        //configure on-chip MPSSE module	
		deviceStatus = MPSSEConfig(&deviceHandle);
		if(FT_OK != deviceStatus)
		{
			std::cout<<"Error: MPSSE configuration failed. Program terminated."<<std::endl;
		}
		else
		{
			//prepare bus for I2C communications
			deviceStatus = SetI2CIdle(&deviceHandle);
			if(FT_OK != deviceStatus)
			{
				std::cout<<"Error writing idle state, code: "<<deviceStatus<<std::endl;
			}
			else
			{
				std::cout<<"Enter a number between 0 and 99 to adjust the duty cycle of the PWM. Enter q to quit and r for a system reset. Enter help to display this message."<<std::endl;

			}
		}
	}	

	while(true)
	{
		if(mutex.try_lock()) //attempt to lock mutex
		{
			if(flag)
			{
				commsFlag = true; //user has entered new input which should be processed
				flag = false; 
				userInput = input;
			}
			mutex.unlock();
		}
		success = false;

		//gather status data
		deviceStatus = SetI2CStart(&deviceHandle);
		if(FT_OK == deviceStatus)
		{
			deviceStatus |= WriteAddr(&deviceHandle,top::SLAVE_ADDR_ONE,true,success);
			if((FT_OK == deviceStatus) && success)
			{
				success = false;
				deviceStatus |= ReadSequence(&deviceHandle,top::STATUS_LENGTH,statusBuffer,success); 	
				if(!success)
				{
					std::cout<<"Error: Read sequence failed, status buffer not updated."<<std::endl;
				}
				else
				{
					if(top::STATUS_OCCUPIED == statusBuffer[0])
					{
						sections[0].occupied = true;
					}
					else
					{
						sections[0].occupied = false;
					}
					sections[0].dutyCycle = statusBuffer[1];
					
					if(top::STATUS_OCCUPIED == statusBuffer[2])
					{
						sections[1].occupied = true;
					}
					else
					{
						sections[1].occupied = false;
					}
					sections[1].dutyCycle = statusBuffer[3];
				}
			}
			else
			{
				std::cout<<"Error: Address write failed when updating status buffer"<<std::endl;
			}
		}
		else
		{
			std::cout<<"Error: Could not start I2C communications when updating status buffer"<<std::endl;
		}

                deviceStatus = SetI2CStop(&deviceHandle);
		if(FT_OK != deviceStatus)
		{
			std::cout<<"Error while terminating communications with first slave"<<std::endl;
		}

		deviceStatus = SetI2CStart(&deviceHandle);
		if(FT_OK == deviceStatus)
		{
			deviceStatus |= WriteAddr(&deviceHandle,top::SLAVE_ADDR_TWO,true,success);
			if((FT_OK == deviceStatus) && success)
			{
				success = false;
				deviceStatus |= ReadSequence(&deviceHandle,top::STATUS_LENGTH,statusBuffer,success); 	
				if(!success)
				{
					std::cout<<"Error: Read sequence failed, status buffer not updated."<<std::endl;
				}
				else
				{
					if(top::STATUS_OCCUPIED == statusBuffer[0])
					{
						sections[2].occupied = true;
					}
					else
					{
						sections[2].occupied = false;
					}
					sections[0].dutyCycle = statusBuffer[1];
					
					if(top::STATUS_OCCUPIED == statusBuffer[2])
					{
						sections[3].occupied = true;
					}
					else
					{
						sections[3].occupied = false;
					}
					sections[1].dutyCycle = statusBuffer[3];
				}
			}
			else
			{
				std::cout<<"Error: Address write failed when updating status buffer"<<std::endl;
			}
		}
		else
		{
			std::cout<<"Error: Could not start I2C communications when updating status buffer"<<std::endl;
		}



		
		if(commsFlag) //use flag here to ensure mutex is locked for the shortest amount of time possible
		{	
			commsFlag = false;
			if(top::IN_HELP == userInput)
			{
				std::cout<<"Enter a number between 0 and 99 to adjust the speed of the trains. Enter q to quit, r for a system reset, and s for a status report. Enter help to display this message."<<std::endl;
			}
			else if(top::IN_RESET == userInput)
			{
				//reset AVRs
				deviceStatus = SystemReset(&deviceHandle);
				if(FT_OK != deviceStatus)
				{
					std::cout<<"Error while attempting to reset I2C slave devices"<<std::endl;
				}
			}
			else if(top::IN_QUIT == userInput)
			{
				break;
			}
			else if(top::IN_STATUS == userInput)
			{

				for(uint8_t i = 0;i<top::NUM_SECTIONS;i++)
				{
					std::cout<<"Section "<<i<<":"<<std::endl;
					std::cout<<"      Occupancy status: "+sections[i].occupied<<std::endl;
					std::cout<<"      Duty cycle:       "+sections[i].dutyCycle<<std::endl;
				}
			}
			else
			{
				try
				{
					//attempt to convert user input to an integer
					trainSpeed = std::stoi(input,NULL);
				}
				catch(const std::invalid_argument& e)
				{
					//input is NaN
					std::cerr<<"Error: Invalid input"<<std::endl;

					//one goto allowed per loop in MISRA
					goto END;
				}
				if((top::SPEED_MIN <= trainSpeed) && (top::SPEED_MAX >= trainSpeed))
				{
					//generate I2C START condition on the bus lines 
					deviceStatus = SetI2CStart(&deviceHandle);

					//write to first device
					deviceStatus |= WriteAddr(&deviceHandle,top::SLAVE_ADDR_ONE,false,success);
					if(success && (FT_OK == deviceStatus))
					{
						//if addressing was successful and device returned ACK, write the data to the device
						deviceStatus = WriteByte(&deviceHandle,trainSpeed,success);
						if(!success || (FT_OK != deviceStatus))
						{
							std::cout<<"Error during data transmission"<<std::endl;	
						}
					}	
					else
					{
						std::cout<<"Error during address transmission"<<std::endl;
					}
					//write to second device
					deviceStatus |= WriteAddr(&deviceHandle,top::SLAVE_ADDR_TWO,false,success);
					if(success && (FT_OK == deviceStatus))
					{
						//if addressing was successful and device returned ACK, write the data to the device
						deviceStatus = WriteByte(&deviceHandle,trainSpeed,success);
						if(!success || (FT_OK != deviceStatus))
						{
							std::cout<<"Error during data transmission"<<std::endl;	
						}
					}	
					else
					{
						std::cout<<"Error during address transmission"<<std::endl;
					}
					//end communications
					deviceStatus = SetI2CStop(&deviceHandle);
					deviceStatus |= SetI2CIdle(&deviceHandle);
					if(FT_OK != deviceStatus)
					{
						std::cout<<"An error occurred while attempting to transmit an I2C STOP condition on the bus lines. A system reset may be required."<<std::endl;
					}

				}
				else
				{
					std::cout<<"Invalid input"<<std::endl;
				}
				END:
					; //goto from catch statement, code jumps here if user enters an invalid input. Effectively a continue statement in the catch block, but goto is used because continue is only allowed in for loops when following MISRA guidelines


				//print status report
				for(uint8_t i = 0;i<top::NUM_SECTIONS;i++)
				{
					std::cout<<"Section "<<i<<":"<<std::endl;
					std::cout<<"      Occupancy status: "+sections[i].occupied<<std::endl;
					std::cout<<"      Duty cycle:       "+sections[i].dutyCycle<<std::endl;
				}
			}

		}
	}

	//disconnect device
	deviceStatus = FreePort(&deviceHandle);
	if(FT_OK != deviceStatus)
	{
		std::cout<<"Termination error. Press any key to continue..."<<std::endl;
		std::cin.get();
	}
}
