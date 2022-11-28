/*
 * Camera.h
 *
 *  Created on: Oct 17, 2022
 *      Author: semba
 */

#ifndef INC_CAMERA_H_
#define INC_CAMERA_H_

#include "main.h"
#include "sensorRegs.h"

#define SPI_WRITE_MASK 0b10000000	// 0x80
#define SPI_READ_MASK 0b01111111	// 0x7f

#define I2C_DEVICE_ADDRESS 0x78		// sccb (i2c) address of ov5642 shifted to 7bits (OV5642 doc page 119 and 113)

void camInit(I2C_HandleTypeDef hi2c1, SPI_HandleTypeDef hspi1);
void snapPic(I2C_HandleTypeDef hi2c1, UART_HandleTypeDef huart2, SPI_HandleTypeDef hspi1);

HAL_StatusTypeDef wCamReg(I2C_HandleTypeDef hi2c1, uint16_t regID, uint16_t data);
HAL_StatusTypeDef wCamRegs(I2C_HandleTypeDef hi2c1, const struct sensor_reg regList[]);
void resetCam(I2C_HandleTypeDef hi2c1);

HAL_StatusTypeDef wCamRegSPI(SPI_HandleTypeDef hspi1, uint8_t addr, uint8_t data);
uint8_t rCamSPI(SPI_HandleTypeDef hspi1, uint8_t addr);
void spiStart();
void spiEnd();

void resetFifoFlags(SPI_HandleTypeDef hspi1);
void resetCapDoneFlag(SPI_HandleTypeDef hspi1);
void setCaptureCount(SPI_HandleTypeDef hspi1, uint8_t captureCount);
void startCapture(SPI_HandleTypeDef hspi1);
uint32_t getFifoLen(SPI_HandleTypeDef hspi1);


#endif /* INC_CAMERA_H_ */
