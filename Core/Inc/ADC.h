/*
 * ADC.h
 *
 *  Created on: May 7, 2024
 *      Author: rjned
 */
#include "cmsis_os2.h"

#ifndef INC_ADC_H_
#define INC_ADC_H_

#define CH0					0x00
#define CH1					0x01
#define CH2					0x02
#define CH3					0x03
#define CH4					0x04
#define CH5					0x05
#define CH6					0x06
#define CH7					0x07

// ADC settings and registers
#define NOP					0x00
#define	READ  				0x10
#define	WRITE  				0x08
#define	SETBIT  			0x18
#define	CLEARBIT  			0x20

#define	SYSTEM_STATUS		0x00
#define	DATA_CFG			0x02
#define	CHANNEL_SEL			0x11

// extern variables
extern SPI_HandleTypeDef hspi1;

// function
void write_ADC(uint8_t reg, uint8_t data);
uint8_t read_ADC(uint8_t reg);
uint8_t check_ADC(void);
float ADC_get(uint8_t channel);
void ADC_change_polarity(void);
// read multiply ADC channels
void ADC_mult(uint8_t cnt, uint8_t sel_ch[], float volt[]);

#endif /* INC_ADC_H_ */
