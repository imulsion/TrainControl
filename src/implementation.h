#include "ftd2xx.h"
#include <iostream>
#include <cstdint>

/* Function: CableInit
 *
 * Establishes a connection to the cable connected to a USB port and sets up the USB device parameters for communication with the chip.
 *
 * Parameter[out]: handle               Pointer to a FT_HANDLE used for accessing the device
 */
FT_STATUS CableInit(FT_HANDLE* handle);

//TODO: Decide if verify mode needed 
#ifdef VERIFY_MODE
FT_STATUS MPSSEVerify();
#endif

/* Function: MPSSEConfig
 *
 * Sends MPSSE configuration parameters to the on-board FT232H chip, setting up the chip for I2C communication
 *
 * Parameter[in] handle                 Pointer to a FT_HANDLE used for accessing the device
 */
FT_STATUS MPSSEConfig(FT_HANDLE* handle);

/* Function: SystemReset
 *
 * Applies a pulse to the TMS (brown) line of the C232HM cable which is designed to reset all I2C slaves at the same time
 *
 * Parameter[in] handle                 Pointer to a FT_HANDLE used for accessing the device
 */

FT_STATUS SystemReset(FT_HANDLE* handle);

/* Function: SetI2CIdle
 *
 * Sets the I2C lines to idle mode, i.e. both SCL and SDA released by the master
 *
 * Parameter[in] handle                 Pointer to a FT_HANDLE used for accessing the device
 */
FT_STATUS SetI2CIdle(FT_HANDLE* handle);


/* Function: SetI2CStart
 *
 * Generates an I2C START condition on the lines by pulling both SCL and SDA low. 
 *
 * Parameter[in] handle                 Pointer to a FT_HANDLE used for accessing the device
 */
FT_STATUS SetI2CStart(FT_HANDLE* handle);


/* Function: SetI2CStop
 *
 * Generates an I2C STOP condition on the lines by releasing both I2C lines and allowing them to be pulled high 
 *
 * Parameter[in] handle                 Pointer to a FT_HANDLE used for accessing the device
 */
FT_STATUS SetI2CStop(FT_HANDLE* handle);


/* Function: ReadByte
 *
 * Reads a byte of data that has been received over the I2C bus by the FT232H chip on the cable 
 *
 * Parameter[in]  handle                Pointer to a FT_HANDLE used for accessing the device
 * Parameter[out] result                Reference to a uint8_t that can be used to store the received value  
 * Parameter[out] success               Reference to a flag that indicates whether the operation was successful
 */
FT_STATUS ReadByte(FT_HANDLE* handle,uint8_t& result,bool& success);


/* Function: WriteByte
 *
 * Instructs the FT232H chip to write a byte of data onto the I2C bus lines 
 *
 * Parameter[in]  handle                Pointer to a FT_HANDLE used for accessing the device
 * Parameter[in]  data                  The data to be written to the lines
 * Parameter[out] success               Reference to a flag that indicates whether the operation was successful
 */
FT_STATUS WriteByte(FT_HANDLE* handle,uint8_t data,bool& success);


/* Function: WriteAddr
 *
 * Wrapper for the WriteByte function that takes an I2C address and writes it to the bus, addressing a device on the bus. Should always be called before attempting to write data onto the bus 
 *
 * Parameter[in]  handle                Pointer to a FT_HANDLE used for accessing the device
 * Parameter[in]  data                  The address of the device it is desired to interact with  
 * Parameter[in]  readNWrite            A flag indicating whether the desired interaction is a WRITE to the slave or a READ from the slave
 * Parameter[out] success               Reference to a flag that indicates whether the operation was successful
 */
FT_STATUS WriteAddr(FT_HANDLE* handle,uint8_t data,bool readNWrite,bool& success);


/* Function: FreePort
 *
 * Frees the device from the USB port, allowing it to be unplugged and the USB port reused. Typically called at the end of the program before termination 
 *
 * Parameter[in] handle                 Pointer to a FT_HANDLE used for accessing the device
 */
FT_STATUS FreePort(FT_HANDLE* handle);
