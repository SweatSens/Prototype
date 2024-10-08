/*
 * DAC.c
 *
 *  Created on: May 8, 2024
 *      Author: rjned
 */

// includes
#include "main.h"


void write_DAC(uint8_t dac, uint8_t reg, uint16_t data){

	uint8_t DAC_tx_buff[3] = {(0x0F&reg), (uint16_t)(data>>8), (uint16_t)(data&0x00FF)};
	//uint8_t DAC_rx_buff[3];
	if(dac == 1){
		// write data to DAC1
		HAL_GPIO_WritePin(GPIOA, DAC1_CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1, DAC_tx_buff, 3, 100);
		HAL_GPIO_WritePin(GPIOA, DAC1_CS_Pin, GPIO_PIN_SET);

		// read data from DAC1 (optional)
		// HAL_GPIO_WritePin(GPIOA, DAC1_CS_Pin, GPIO_PIN_RESET);
		// HAL_SPI_Receive(&hspi1, DAC_rx_buff, 3, 100);
		// HAL_GPIO_WritePin(GPIOA, DAC1_CS_Pin, GPIO_PIN_SET);
	}
	else if(dac == 2){
		// write data to DAC2
		HAL_GPIO_WritePin(GPIOB, DAC2_CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1, DAC_tx_buff, 3, 100);
		HAL_GPIO_WritePin(GPIOB, DAC2_CS_Pin, GPIO_PIN_SET);

		// read data from DAC2 (optional)
		// HAL_GPIO_WritePin(GPIOB, DAC2_CS_Pin, GPIO_PIN_RESET);
		// HAL_SPI_Receive(&hspi1, DAC_rx_buff, 3, 100);
		// HAL_GPIO_WritePin(GPIOB, DAC2_CS_Pin, GPIO_PIN_SET);
}

	return;
}

uint16_t read_DAC(uint8_t dac, uint8_t reg){

	uint8_t DAC_tx_buff[3] = {((0x0F&reg)|(1<<7)), 0x00, 0x00};
	uint8_t DAC_rx_buff[3];
	if(dac == 1){
		// write data to DAC1
		HAL_GPIO_WritePin(GPIOA, DAC1_CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1, DAC_tx_buff, 3, 100);
		HAL_GPIO_WritePin(GPIOA, DAC1_CS_Pin, GPIO_PIN_SET);

		// read data from DAC1
		HAL_GPIO_WritePin(GPIOA, DAC1_CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Receive(&hspi1, DAC_rx_buff, 3, 100);
		HAL_GPIO_WritePin(GPIOA, DAC1_CS_Pin, GPIO_PIN_SET);
	}
	else if(dac == 2){
		// write data to DAC2
		HAL_GPIO_WritePin(GPIOB, DAC2_CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1, DAC_tx_buff, 3, 100);
		HAL_GPIO_WritePin(GPIOB, DAC2_CS_Pin, GPIO_PIN_SET);

		// read data from DAC2
		HAL_GPIO_WritePin(GPIOB, DAC2_CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Receive(&hspi1, DAC_rx_buff, 3, 100);
		HAL_GPIO_WritePin(GPIOB, DAC2_CS_Pin, GPIO_PIN_SET);
	}
	uint16_t result  = (uint16_t)((DAC_rx_buff[1] << 8) + DAC_rx_buff[2]);

	return result;
}

