#include "ftd2xx.h"
#include <iostream>
#include <cstdint>


FT_STATUS I2cInit(FT_HANDLE* handle);

//TODO: Decide if verify mode needed 
#ifdef VERIFY_MODE
FT_STATUS MPSSEVerify();
#endif

FT_STATUS MPSSEConfig(FT_HANDLE* handle);

FT_STATUS SetI2CIdle(FT_HANDLE* handle);
FT_STATUS SetI2CStart(FT_HANDLE* handle);
FT_STATUS SetI2CStop(FT_HANDLE* handle);
FT_STATUS ReadByte(FT_HANDLE* handle,uint8_t& result,bool& success);
FT_STATUS WriteByte(FT_HANDLE* handle,uint8_t data,bool& success);
FT_STATUS WriteAddr(FT_HANDLE* handle,uint8_t data,bool readNWrite,bool& success);

FT_STATUS FreePort(FT_HANDLE* handle);
