/*
 * MLX90640_I2C_Driver.cpp
 *
 *  Created on: Apr 14, 2024
 *      Author: AJ
 */
#include "MLX90640_I2C_Driver.h"
/* ## I2C HAL Callback Functions for Interrupt Mode ##
HAL_I2C_MasterTxCpltCallback(): callback when the master transmission process is complete.
HAL_I2C_MasterRxCpltCallback(): callback when the master reception process is complete.
HAL_I2C_SlaveTxCpltCallback(): callback when the slave transmission process is complete.
HAL_I2C_SlaveRxCpltCallback(): callback when the slave reception process is complete.
HAL_I2C_AddrCallback(): callback when the slave address matched with the received address.
HAL_I2C_ListenCpltCallback(): callback when the complete slave transfer is done, and the slave is set to listen mode.
HAL_I2C_ErrorCallback():  callback when an error occurs during the I2C communication.
*/
//create static pointer to I2C typedef, this is initialised in the main.c and passed to this header via MLX90640_I2CInit()
static I2C_HandleTypeDef *hi2cPtr; // Pointer to store the reference to hi2cX
static bool I2CPtrInit = false;

// Initialise  MLX90640_I2C_Driver.h with reference to hi2c1, this reference is stored and used to transmit and recieve data
void MLX90640_I2CInitRef(I2C_HandleTypeDef *hi2c) {
	if(hi2c != nullptr) {
		hi2cPtr = hi2c; // Store the reference to hi2c1
		I2CPtrInit = true;
		#ifdef _DEBUG_I2C_DRIVER
			printf("OK: I2C endpoint reference configured.\n\r");
		#endif
	} else {
		#ifdef _DEBUG_I2C_DRIVER
			printf("[Err] No I2C endpoint configured: I2CInitRef(&hi2cX) == NULL.\n\r");
		#endif
	}
}

//Check if device at given address is returning an ACK flag on the current I2C interface hi2cPtr reference
bool MLX90640_isConnected(uint8_t slaveAddr) {
    // Check if hi2cPtr reference is set and not NULL
    if (!I2CPtrInit) {
		#ifdef _DEBUG_I2C_DRIVER
    		printf("[Err] No I2C endpoint configured: I2CInitRef(&hi2cX) == NULL.\n\r");
		#endif
        return false; // I2C pointer reference not initialised, return error
    }
    uint16_t address = slaveAddr << 1; // Shift the address left by 1 to include the R/W bit

    // Attempt to start an I2C transmission to the device
    //DEFINITION: HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint32_t Trials, uint32_t Timeout)
    HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(hi2cPtr, address, 2, MLX90640_TIMEOUT_MS);
    if (status != HAL_OK) {
		#ifdef _DEBUG_I2C_DRIVER
    		printf("[Err] Device I2C isConnected test did not ACK: "); printHALStatus(status);
		#endif
        return false; // Sensor did not ACK
    } else {
		#ifdef _DEBUG_I2C_DRIVER
			printf("Device I2C isConnected test returned: "); printHALStatus(status);
		#endif
        return true; // Sensor detected
    }
}


/*Read a number of 16bit words (2x 8bit bytes per word) from startAddress of requested slave, store returned data to array.
- slaveAddr: hex address of I2C slave device
- startAddress: First address from memory to be read, MLX90640 note:
 > EEPROM in address range 0x2400 to 0x273F;
 > RAM is in address range 0x0400 to 0x073F;
- nMemAddressRead: Number of 16-bits words to be read from the MLX90640 memory.
- data: returned data storage.
Returns 0 if successful, -1 if error */
int MLX90640_I2CRead(uint8_t slaveAddr, uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *data) {
	// Example reading EEPROM: MLX90640_I2CRead(0x33, 0x2400, 832, *data);
    // Check if hi2cPtr reference is set and not NULL
	if (!I2CPtrInit) {
		#ifdef _DEBUG_I2C_DRIVER
        	printf("[Err]@I2CRead: No I2C endpoint configured: I2CInitRef(&hi2cX) == NULL.\n\r");
		#endif
        return -1; // I2C pointer reference not initialized, return error
    }
    uint16_t bytesRemaining = nMemAddressRead * 2; // 832 * 2 = 1664 bytes
    uint16_t currentAddress = startAddress; // Initialize current address to start

	#ifdef _DEBUG_I2C_DRIVER
		printf("[Info]@I2CRead: Requested address [0x%x]->[0x%x],%u word (%u bytes) remaining. Hex data:\n\r", startAddress, startAddress + (nMemAddressRead - 1), nMemAddressRead, bytesRemaining);
	#endif

    // Loop until all memory addresses are read
    while (bytesRemaining > 0) {
        // Determine the number of bytes to read in this iteration
        uint16_t bytesToRead = (bytesRemaining > I2C_BUFFER_LENGTH) ? I2C_BUFFER_LENGTH : bytesRemaining;

        // Receive the data from the device
        uint8_t rxData[I2C_BUFFER_LENGTH];
        HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2cPtr, slaveAddr << 1, currentAddress, I2C_MEMADD_SIZE_16BIT, rxData, bytesToRead, MLX90640_TIMEOUT_MS);
        if (status != HAL_OK) {
			#ifdef _DEBUG_I2C_DRIVER
        		printf("[Err]@I2CRead: NACK occurred during initial device comms, HAL_State: "); printHALStatus(status);
			#endif
            return -1; // Error in reception
        }

		#ifdef _DEBUG_I2C_DRIVER
        	printf("[%x]->[%x] %u of %u:\t", currentAddress, currentAddress + ((bytesToRead / 2) - 1), (bytesToRead / 2), (bytesRemaining / 2));
		#endif
        // Parse received data and store into array
        for (uint16_t x = 0; x < (bytesToRead / 2); x++) {
            data[(currentAddress - startAddress) + x] = (rxData[x * 2] << 8) | rxData[x * 2 + 1]; // Combine MSB and LSB
			#ifdef _DEBUG_I2C_DRIVER
				printf("%*x", 6, data[(currentAddress - startAddress) + x]); // Print received data
			#endif
        }
		#ifdef _DEBUG_I2C_DRIVER
        	printf("\n\r");
		#endif

        // Update counters and memory address
        bytesRemaining -= bytesToRead;
        currentAddress += (bytesToRead / 2);
    }
	#ifdef _DEBUG_I2C_DRIVER
		printf("\n\r");
	#endif
    return 0; // Success
}

/*Write a 16bit word (2x 8bit bytes per word) to writeAddress of provided slave, data is read back after write to verify success.
- slaveAddr: hex address of I2C slave device
- writeAddress: device address to write to
- data: 16bit word to write to memory location.
Returns 0 if successful, -1 if transmission error, -2 if read back data is mismatched to write request */
int MLX90640_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data) {
    // Check if hi2cPtr reference is set and not NULL
	if (!I2CPtrInit) {
		#ifdef _DEBUG_I2C_DRIVER
			printf("[Err]@I2CWrite: No I2C endpoint configured: I2CInitRef(&hi2cX) == NULL.\n\r");
		#endif
        return -1; // I2C pointer reference not initialized, return error
    }

	#ifdef _DEBUG_I2C_DRIVER
		printf("[Info]@I2CWrite: Requested [0x%x] write to address [0x%x].\n\r", data, writeAddress);
	#endif

	//convert write data to 8bit with corrected endianness
	uint8_t txData[2];
	txData[0] = (uint8_t)(data >> 8); // MSB
	txData[1] = (uint8_t)(data & 0xFF); // LSB

    // Transmit the data to the device
    HAL_StatusTypeDef status;
    //DEFINITION: HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
    status = HAL_I2C_Mem_Write(hi2cPtr, slaveAddr << 1, writeAddress, I2C_MEMADD_SIZE_16BIT, txData, sizeof(txData), MLX90640_TIMEOUT_MS);
    if (status != HAL_OK) {
		#ifdef _DEBUG_I2C_DRIVER
			printf("[Err]@I2CWrite: NACK occurred during transmission: "); printHALStatus(status);
		#endif
        return -1; // Error in transmission: NACK occurred during transmission
    }

    HAL_Delay(5); // Delay as per MLX documentation: "After each write at least 5ms delay is needed in order to writing process to take place."

    // Verify the write operation
    uint8_t rxData[2]; // Buffer to store received data; 1x 16bit word = 2x 8bit bytes
    status = HAL_I2C_Mem_Read(hi2cPtr, slaveAddr << 1, writeAddress, I2C_MEMADD_SIZE_16BIT, rxData, sizeof(rxData), MLX90640_TIMEOUT_MS);
    uint16_t dataCheck = (rxData[0] << 8) | rxData[1]; // Reconstruct uint16_t word from received uint8_t byte data
    if (status != HAL_OK || dataCheck != data) {
		#ifdef _DEBUG_I2C_DRIVER
			printf("[Err]@I2CWrite: After write command, mem data [%x] != [%x] write request: ", dataCheck, data); printHALStatus(status);
		#endif
        return -2; // Error: Data in memory is not the same as intended
    }
    return 0; // Success
}


/*Perform a general reset of all devices on I2C bus pointed to by hi2cPtr; reset condition in I2C specification is sending 0x06 to address 0x00
Return 0 if communication is successful, -1 if NACK occurred during communication.*/
int MLX90640_I2CGeneralReset(void) {
    // Check if hi2cPtr reference is set and not NULL
	if (!I2CPtrInit) {
		#ifdef _DEBUG_I2C_DRIVER
			printf("[Err]@I2CGeneralReset: No I2C endpoint configured: I2CInitRef(&hi2cX) == NULL.\n\r");
		#endif
        return -1; // I2C pointer reference not initialized, return error
    }

    uint8_t resetCommand = 0x06; // Send reset command (0x06) to address 0x00
    HAL_StatusTypeDef status  = HAL_I2C_Master_Transmit(hi2cPtr, 0x00, &resetCommand, sizeof(resetCommand), MLX90640_TIMEOUT_MS);
    if (status != HAL_OK) {
		#ifdef _DEBUG_I2C_DRIVER
			printf("[Err]@I2CGeneralReset: NACK on I2C reset command:"); printHALStatus(status);
		#endif
    	return -1; // Error: NACK occurred during transmission
    }
    // Reset successful
    return 0;
}

/*Helper function to interpret and print HAL status code to serial.*/
void printHALStatus(HAL_StatusTypeDef& status) {
    switch (status) {
        case HAL_OK:
        	printf("HAL_OK\n\r");
            break;
        case HAL_ERROR:
        	printf("HAL_ERROR\n\r");
        	break;
        case HAL_BUSY:
        	printf("HAL_BUSY\n\r");
            break;
        case HAL_TIMEOUT:
        	printf("HAL_TIMEOUT\n\r");
        	break;
        default:
        	printf("HAL_UNKNOWN\n\r");
        	break;
    }
}



