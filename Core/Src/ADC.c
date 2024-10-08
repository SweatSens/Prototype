/*
 * ADC.c
 *
 *  Created on: May 7, 2024
 *      Author: rjned
 */

// includes
#include "main.h"


void write_ADC(uint8_t reg, uint8_t data){
	uint8_t ADC_tx_buff[3] = {WRITE, reg, data};
	HAL_GPIO_WritePin(GPIOA, ADC_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, ADC_tx_buff, 3, 100);
	HAL_GPIO_WritePin(GPIOA, ADC_CS_Pin, GPIO_PIN_SET);

	return;
}

uint8_t read_ADC(uint8_t reg){

	uint8_t ADC_tx_buff[3] = {READ, reg, 0x00};
	uint8_t ADC_rx_buff[3];

	// write data to ADC
	HAL_GPIO_WritePin(GPIOA, ADC_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, ADC_tx_buff, 3, 100);
	HAL_GPIO_WritePin(GPIOA, ADC_CS_Pin, GPIO_PIN_SET);

	// read data from ADC
	HAL_GPIO_WritePin(GPIOA, ADC_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Receive(&hspi1, ADC_rx_buff, 3, 100);
	HAL_GPIO_WritePin(GPIOA, ADC_CS_Pin, GPIO_PIN_SET);

	return ADC_rx_buff[0];
}

uint8_t check_ADC(void){

	if(read_ADC(SYSTEM_STATUS) == 0x81){
		return 1;
	}
	return 0;
}

void ADC_change_polarity(void){

	uint8_t REG_DATA_CGF = read_ADC(DATA_CFG);
	write_ADC(DATA_CFG, (REG_DATA_CGF|(1<<2)) );

	return;
}

float ADC_get(uint8_t channel){

	uint8_t ADC_tx_buff[3];
	uint8_t ADC_rx_buff[3];

	ADC_tx_buff[0] = WRITE;
	ADC_tx_buff[1]= CHANNEL_SEL;
	ADC_tx_buff[2] = channel;

	HAL_GPIO_WritePin(GPIOA, ADC_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, ADC_tx_buff, 3, 100);
	// nothing
	HAL_GPIO_WritePin(GPIOA, ADC_CS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, ADC_CS_Pin, GPIO_PIN_RESET);
	// nothing (conversion is happening)
	HAL_GPIO_WritePin(GPIOA, ADC_CS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, ADC_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Receive(&hspi1, ADC_rx_buff, 3, 100);
	HAL_GPIO_WritePin(GPIOA, ADC_CS_Pin, GPIO_PIN_SET);

	uint16_t result = (ADC_rx_buff[0] << 8) + ADC_rx_buff[1];
	float val = (float)result * 33/655360;

	return val;
}

// to read the voltage of different changes the ADC must change from channel before conversion
void ADC_mult(uint8_t cnt, uint8_t sel_ch[], float volt[]){
	uint16_t result = 0;
	uint8_t ADC_tx_buff[3];
	uint8_t ADC_rx_buff[3];
	ADC_tx_buff[0] = WRITE;
	ADC_tx_buff[1]= CHANNEL_SEL;

	if(cnt > 0){
		ADC_tx_buff[2] = sel_ch[0];
		HAL_GPIO_WritePin(GPIOA, ADC_CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1, ADC_tx_buff, 3, 100);
		// nothing to read
		HAL_GPIO_WritePin(GPIOA, ADC_CS_Pin, GPIO_PIN_SET);
	}
	if(cnt > 1){
		ADC_tx_buff[2] = sel_ch[1];
		HAL_GPIO_WritePin(GPIOA, ADC_CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1, ADC_tx_buff, 3, 100);
		// nothing to read (conversion is happening)
		HAL_GPIO_WritePin(GPIOA, ADC_CS_Pin, GPIO_PIN_SET);
	}
	// data is ready but also new data is loaded
	for(int8_t i=0;i<(cnt-2);i++){
		uint8_t ch = (uint8_t)(i+2);
		ADC_tx_buff[2] = sel_ch[ch];
		HAL_GPIO_WritePin(GPIOA, ADC_CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Receive(&hspi1, ADC_rx_buff, 3, 100);
		HAL_SPI_Transmit(&hspi1, ADC_tx_buff, 3, 100);
		HAL_GPIO_WritePin(GPIOA, ADC_CS_Pin, GPIO_PIN_SET);
		result = (ADC_rx_buff[0] << 8) + ADC_rx_buff[1];
		uint8_t k = (uint8_t)i;
		volt[k] = (float)result * 33/655360;
	}
	if(cnt > 1){
		HAL_GPIO_WritePin(GPIOA, ADC_CS_Pin, GPIO_PIN_RESET);
		// nothing to write
		HAL_SPI_Receive(&hspi1, ADC_rx_buff, 3, 100);
		HAL_GPIO_WritePin(GPIOA, ADC_CS_Pin, GPIO_PIN_SET);
		result = (ADC_rx_buff[0] << 8) + ADC_rx_buff[1];
		volt[cnt-2] = (float)result * 33/655360;
	}
	if(cnt == 1){
		HAL_GPIO_WritePin(GPIOA, ADC_CS_Pin, GPIO_PIN_RESET);
		// nothing to write
		// nothing to read
		HAL_GPIO_WritePin(GPIOA, ADC_CS_Pin, GPIO_PIN_SET);
	}
	if(cnt > 0){
		HAL_GPIO_WritePin(GPIOA, ADC_CS_Pin, GPIO_PIN_RESET);
		// nothing to write
		HAL_SPI_Receive(&hspi1, ADC_rx_buff, 3, 100);
		HAL_GPIO_WritePin(GPIOA, ADC_CS_Pin, GPIO_PIN_SET);
		result = (ADC_rx_buff[0] << 8) + ADC_rx_buff[1];
		volt[cnt-1] = (float)result * 33/655360;
	}

	return;
}


