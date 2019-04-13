#include "implementation.h" //implementation specific header
#include <string>           //C++ string library
#include <thread>           //C++11 thread library
#include <mutex>            //C++11 mutex library
#include <functional>       //needed for std::ref



const uint8_t     SLAVE_ADDR_ONE       = 0x01      
,                 SLAVE_ADDR_TWO       = 0x02 
,                 DUTY_CYCLE_MIN       = 0U       //minimum valid duty cycle value
,                 DUTY_CYCLE_MAX       = 100U     //maximum valid duty cycle value
,                 DUTY_CYCLE_FRACTION  = 100U     //Used to normalise duty cycle input to a fraction between 0 and 1
,                 AVR_COUNTER_MAX      = 255U;    //Maximum value of AVR counter register

const std::string IN_QUIT              = "quit"       //Expected user input for a quit command
,                 IN_RESET             = "reset"       //Expected user input for a reset command
,                 IN_HELP              = "help";   //Expected user input for a help command 

void GetUserInput(std::string& input, bool& flag, std::mutex& mutex);
void Run(std::string& input, bool& flag, std::mutex& mutex);

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
	uint8_t dutyCycle = 0U;                                //PWM duty cycle
	bool commsFlag = false;                                //flag for indicating user input waiting to be processed, essentially a copy of the mutex protected flag parameter
	std::string userInput = "";                            //local copy of mutex protected user input string


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
		
		if(commsFlag) //use flag here to ensure mutex is locked for the shortest amount of time possible
		{	
			commsFlag = false;
			if(IN_HELP == userInput)
			{
				std::cout<<"Enter a number between 0 and 99 to adjust the duty cycle of the PWM. Enter q to quit and r for a system reset. Enter help to display this message."<<std::endl;
			}
			else if(IN_RESET == userInput)
			{
				//reset AVRs
				deviceStatus = SystemReset(&deviceHandle);
				if(FT_OK != deviceStatus)
				{
					std::cout<<"Error while attempting to reset I2C slave devices"<<std::endl;
				}
			}
			else if(IN_QUIT == userInput)
			{
				break;
			}
			else
			{
				try
				{
					//attempt to convert user input to an integer
					dutyCycle = std::stoi(input,NULL);
				}
				catch(const std::invalid_argument& e)
				{
					//input is NaN
					std::cerr<<"Error: Invalid input"<<std::endl;

					//one goto allowed per loop in MISRA
					goto END;
				}
				if((DUTY_CYCLE_MIN <= dutyCycle) && (DUTY_CYCLE_MAX >= dutyCycle))
				{
					//calculate AVR counter register value necessary to produce the required PWM duty cycle
					dutyCycle = static_cast<int>(AVR_COUNTER_MAX * (static_cast<float>(dutyCycle) / DUTY_CYCLE_FRACTION));

					//generate I2C START condition on the bus lines 
					deviceStatus = SetI2CStart(&deviceHandle);
					//I2C first phase: Address the required device
					deviceStatus |= WriteAddr(&deviceHandle,SLAVE_ADDR_ONE,false,success);
					if(success && (FT_OK == deviceStatus))
					{
						//if addressing was successful and device returned ACK, write the data to the device
						deviceStatus = WriteByte(&deviceHandle,dutyCycle,success);
						if(!success || (FT_OK != deviceStatus))
						{
							std::cout<<"Error during data transmission"<<std::endl;	
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
						std::cout<<"Error during address transmission"<<std::endl;
					}

				}
				else
				{
					std::cout<<"Invalid input"<<std::endl;
				}
				END:
					; //goto from catch statement, code jumps here if user enters an invalid input. Effectively a continue statement in the catch block, but goto is used because continue is only allowed in for loops when following MISRA guidelines
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
