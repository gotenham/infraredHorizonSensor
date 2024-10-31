/*
 * MLX90640_I2C_Driver.h
 *
 *  Created on: Apr 14, 2024
 *      Author: AJ
 */
#ifndef SRC_MLX90640_I2C_DRIVER_H_
#define SRC_MLX90640_I2C_DRIVER_H_

#include <stm32l4xx_hal.h> //include I2C driver header for HAL comms functions
#include <stdio.h> //TBD DEBUG, DELETE INF LOOP
//#define _DEBUG_I2C_DRIVER //DEBUG FLAG, uncomment to enable module debugging

#define I2C_BUFFER_LENGTH 32 // TBD review STM32 buffer size
#define MLX90640_TIMEOUT_MS 5000 // I2C timeout in ms // HAL_MAX_DELAY

//NOTE: settings are reset after power down and should be reinitialised on powerup
/*Typedef different sensor operation modes */
enum class MLX90640_Mode : uint8_t {
	MLX90640_INTERLEAVED, //image read in interleaved lines (odd rows, even rows etc)
	MLX90640_CHESS // [default] image read in chess pattern (odd index, even index etc)
};

/*Typedef different sensor resolutions for pixel ADC reading */
enum class MLX90640_Resolution : uint8_t {
	MLX90640_ADC_16BIT,
	MLX90640_ADC_17BIT,
	MLX90640_ADC_18BIT, // [default]
	MLX90640_ADC_19BIT
};

/*Typedef different sensor supported refresh rates */
enum class MLX90640_RefreshRate : uint8_t {
	MLX90640_0_5HZ,
	MLX90640_1HZ,
	MLX90640_2HZ, // [default]
	MLX90640_4HZ,
	MLX90640_8HZ,
	MLX90640_16HZ,
	MLX90640_32HZ,
	MLX90640_64HZ
};

// Initialise  MLX90640_I2C_Driver.h with reference to hi2c1, this reference is stored and used to transmit and recieve data
void MLX90640_I2CInitRef(I2C_HandleTypeDef *hi2c);

//Check if device at given address is returning an ACK flag on the current I2C interface hi2cPtr reference
bool MLX90640_isConnected(uint8_t slaveAddr);

/*Read a number of 16bit words (2x 8bit bytes per word) from startAddress of requested slave, store returned data to array.*/
int MLX90640_I2CRead(uint8_t slaveAddr, uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *data);

/*Write a 16bit word (2x 8bit bytes per word) to writeAddress of provided slave, data is read back after write to verify success.*/
int MLX90640_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data);

/*Perform a general reset of all devices on I2C bus pointed to by hi2cPtr; reset condition in I2C specification is sending 0x06 to address 0x00*/
int MLX90640_I2CGeneralReset(void);

/*Helper function to interpret and print HAL status code to serial.*/
void printHALStatus(HAL_StatusTypeDef& status);

#endif /* SRC_MLX90640_I2C_DRIVER_H_ */
