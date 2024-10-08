/*
 * DAC.h
 *
 *  Created on: May 8, 2024
 *      Author: rjned
 */
#include "cmsis_os2.h"

#ifndef INC_DAC_H_
#define INC_DAC_H_

#define DAC_1 				0x01
#define DAC_2				0x02

#define CH0					0x00
#define CH1					0x01
#define CH2					0x02
#define CH3					0x03
#define CH4					0x04
#define CH5					0x05
#define CH6					0x06
#define CH7					0x07

// DAC settings and registers
#define SYNC  				0x02
#define CONFIG  			0x03
#define TRIGGER 			0x05
#define BRDCAST 			0x06

// extern variables
extern SPI_HandleTypeDef hspi1;
extern osMutexId_t mutexSPIHandle;

// function
void write_DAC(uint8_t dac, uint8_t reg, uint16_t data);
uint16_t read_DAC(uint8_t dac, uint8_t reg);

#endif /* INC_DAC_H_ */
