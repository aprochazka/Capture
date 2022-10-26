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

void camInit(I2C_HandleTypeDef hi2c1, SPI_HandleTypeDef hspi1);

int wCamReg(I2C_HandleTypeDef hi2c1, uint16_t regID, uint16_t data);
int wCamRegs(I2C_HandleTypeDef hi2c1, const struct sensor_reg regList[]);

int wCamRegSPI(SPI_HandleTypeDef hspi1, uint8_t addr, uint8_t data);
uint8_t rCamSPI(SPI_HandleTypeDef hspi1, uint8_t addr);

int setJPEG(I2C_HandleTypeDef hi2c1);

void snapPic(I2C_HandleTypeDef hi2c1, UART_HandleTypeDef huart2, SPI_HandleTypeDef hspi1);
#endif /* INC_CAMERA_H_ */
