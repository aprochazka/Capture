/*
 * Camera.c
 *
 *  Created on: Oct 17, 2022
 *      Author: semba
 */

#include "Camera.h"
#include "main.h"

uint8_t	Buf1[4096]={0}, Buf2[4096]={0};
uint8_t	*picbuf = 0;
uint32_t haveRev = 0;

//Initialize camera by reverse engineering demo code for weaker chip on official github
void camInit(I2C_HandleTypeDef hi2c1, SPI_HandleTypeDef hspi1){

	wCamReg(hi2c1, 0x3008, 0x80); // RESET CHIP
	wCamRegs(hi2c1, OV5642_QVGA_Preview);
	wCamRegs(hi2c1, OV5642_JPEG_Capture_QSXGA);
    wCamRegs(hi2c1, ov5642_320x240);
    wCamReg(hi2c1, 0x3818, 0xa8); //TIMING CONTROL - ENABLE COMPRESSION, THUMBNAIL MODE DISABLE, VERTICAL FLIP, MIRROR OFF
    wCamReg(hi2c1, 0x3621, 0x10); //REGISTER FOR CORRECT MIRROR FUNCTION
    wCamReg(hi2c1, 0x3801, 0xb0); //TIMING HORIZONTAL START - ALSO FOR MIRROR
    wCamReg(hi2c1, 0x4407, 0x04); // COMPRESSION CONTROL
	wCamRegSPI(hspi1, 0x03, 0x02); // SET VSYNC POLARITY TO ACTIVE LOW

}

//edit single register
int wCamReg(I2C_HandleTypeDef hi2c1, uint16_t regID, uint16_t data){
	HAL_StatusTypeDef ret;
	uint8_t buf[5];
	uint8_t Addr = 0x78; //sccb (i2c) address of ov5642 shifted to 7bits (OV5642 doc page 119 and 113)
	buf[0] = regID >> 8; // we want to get just the first 8 bits of address
	buf[1] = regID; //rest of register address
	buf[2] = data;
	ret = HAL_I2C_Master_Transmit(&hi2c1, Addr, buf, 3, HAL_MAX_DELAY);
	if(ret == HAL_OK) return(1);
	return(0);
};

//cam library equivalent - int wrSensorRegs16_8(const struct sensor_reg reglist[])
// write data to multiple registers
int wCamRegs(I2C_HandleTypeDef hi2c1, const struct sensor_reg regList[])
{
	  int err = 0;

	  uint16_t regID;
	  uint16_t regData;
	  const struct sensor_reg *nextReg = regList;

	  while ((regID != 0xffff) | (regData != 0xff))
	  {
	    regID =nextReg->reg;
	    regData = nextReg->val;
	    err = wCamReg(hi2c1, regID, regData);
	    nextReg++;
	  }
	  return err;
};

//equivalent to library - bus_write();
int wCamRegSPI(SPI_HandleTypeDef hspi1, uint8_t addr, uint8_t data){
	HAL_StatusTypeDef ret;
	uint8_t addrFormatted = addr | 0x80;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_Delay(1);

	//if these two transmits are rewritten to TransmitReceive() program HardFaults on return from this function
	ret = HAL_SPI_Transmit(&hspi1, (uint8_t *)&addrFormatted, 1, HAL_MAX_DELAY);
	if(ret != HAL_OK){
		return ret;
	}

	ret = HAL_SPI_Transmit(&hspi1, (uint8_t *)&data, 1, 100);
	if(ret != HAL_OK){
		return ret;
	}

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	return 0;

};

//read register
//library equivalent - uint8_t bus_read(int address)
uint8_t rCamSPI(SPI_HandleTypeDef hspi1, uint8_t addr){
	uint8_t addrMasked = addr & 0x7F;
	uint8_t empty = 0x00;
	uint8_t ret;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, &addrMasked, &ret, 1, HAL_MAX_DELAY);
	HAL_SPI_TransmitReceive(&hspi1, &empty, &ret, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	return ret;
}

//Set output type and resolution to low res jpeg
int setJPEG(I2C_HandleTypeDef hi2c1){

	//wrSensorRegs16_8(ov5642_320x240);
	wCamRegs(hi2c1, ov5642_320x240);

	return 1;
}

//try to get any capture data back from camera module
void snapPic(I2C_HandleTypeDef hi2c1, UART_HandleTypeDef huart2, SPI_HandleTypeDef hspi1){

			//set number of frames to capture to 1
			wCamRegSPI(hspi1, 0x1, 0x1);

			// test read number of frames to capture - working
			rCamSPI(hspi1, 0x1);

			//reset flags
			wCamRegSPI(hspi1, 0x04, 0x31);

			// set cam to test mode
			//wCamRegSPI(hspi1, 0x05, 0x1);

			// start capture
			wCamRegSPI(hspi1, 0x04, 0x2);

			//wait for capture done
			while(1){
				uint8_t regValue = rCamSPI(hspi1, 0x41);
				uint8_t captureDoneMask = 0x8;
				if(regValue & captureDoneMask) break;
			}

				uint32_t len1,len2,len3,len=0;
			 	// FIFO_SIZE1,2,3 - 0x42, 43, 44
				//len1 = read_reg(FIFO_SIZE1);
  	  	  	  	len1 = rCamSPI(hspi1, 0x42);

			 	//len2 = read_reg(FIFO_SIZE2);
  	  	  	  	len2 = rCamSPI(hspi1, 0x43);

  	  	  	  	//len3 = read_reg(FIFO_SIZE3) & 0x7f;
  	  	  	  	len3 = rCamSPI(hspi1, 0x44) & 0x7f;

  	  	  	  	len = ((len3 << 16) | (len2 << 8) | len1) & 0x07fffff;

			//sendlen = (length>=BUFFER_MAX_SIZE) ? BUFFER_MAX_SIZE : length;
			uint32_t sendLen = (len>=4096) ? 4096 : len;
			picbuf = Buf1;
			haveRev = 0;

			//DMA1_RX(picbuf, sendlen);

				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

				uint8_t BURST_FIFO_READ = 0x3d;

				HAL_SPI_TransmitReceive(&hspi1, &BURST_FIFO_READ, picbuf, 1, HAL_MAX_DELAY);

				//probably not correct usage of this function, but for this case it is working as we need -
				//(just sending clock pulses and saving responses)
				HAL_SPI_Receive_DMA(&hspi1, picbuf, sendLen);

				//while(hspi1.State != HAL_SPI_STATE_READY){;}
				HAL_Delay(1000); //delay to ensure full dma transmission

				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

				int bufLen = strlen((char*)picbuf);
				HAL_UART_Transmit(&huart2, picbuf, bufLen, HAL_MAX_DELAY);

				/*
				 * THE ERROR IS THAT WE ONLY RECEIVE THIS DATA FROM CHIP:
				 * 	ff d8 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f
					10 11 12 13 14 15 16 17 18 19 1a 1b 1c 1d 1e 1f
					20 21 22 23 24 25 26 27 28 29 2a 2b 2c 2d ff d9
					ff d8 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f
					10 11 12 13 14 15 16 17 18 19 1a 1b 1c 1d 1e 1f
					20 21 22 23 24 25 26 27 28 29 2a 2b 2c 2d ff d9
					ff d8 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f
					10 11 12 13 14 15 16 17 18 19 1a 1b 1c 1d 1e 1f
					20 21 22 23 24 25 26 27 28 29 2a 2b 2c 2d ff d9
					ff d8 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f
					10 11 12 13 14 15 16 17 18 19 1a 1b 1c 1d 1e 1f
					20 21 22 23 24 25 26 27 28 29 2a 2b 2c 2d ff d9

					WHICH HAS CORRECT JPEG START AND END BYTES BUT IS FOR SOME REASON DUMMY DATA
				 */
		}

