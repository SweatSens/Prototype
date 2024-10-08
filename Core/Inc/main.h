/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CLI.h"
#include "ADC.h"
#include "DAC.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */


#define V1			0x00
#define V2			0x01
#define V3			0x02
#define V4			0x03
#define P1			0x04
#define TEMP		0x05

#define IND			0x00
#define SIM		 	0x01

// structs of all the types of measurements
struct pH{
  uint8_t on;
  float t_interval;
  float t_run;
};
struct CV{
	float t_equilibration;
	float E_begin;
	float E_vertex_1;
	float E_vertex_2;
	float E_step;
	float scan_rate;
};
struct DPV{
	float t_equilibration;
	float E_begin;
	float E_end;
	float E_step;
	float E_pulse;
	float t_pulse;
	float Scan_rate;
};
struct SWV{
	float t_equilibration;
	float E_begin;
	float E_end;
	float E_step;
	float Amplitude;
	float Frequency;
};
// parameters contain both the technique and if there needs to be simultaneously measured
struct Parameters{
	uint8_t ch_brdcst_on;
	uint8_t technique;
	struct CV cv;
	struct DPV dpv;
	struct SWV swv;
};
// is the overall struct for voltammetry
struct Channels{
	uint8_t on;
	uint8_t broadcast;
	struct Parameters parameters[4];
};

// struct of message queue
typedef struct {
	float val;
	float val_x;
	float val_rf;
	uint32_t tick;
	uint8_t channel;
} MSGQUEUE_OBJ_t;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOA
#define VCP_TX_Pin GPIO_PIN_2
#define VCP_TX_GPIO_Port GPIOA
#define TempAlert_Pin GPIO_PIN_3
#define TempAlert_GPIO_Port GPIOA
#define ADC_CS_Pin GPIO_PIN_4
#define ADC_CS_GPIO_Port GPIOA
#define DAC2_CS_Pin GPIO_PIN_0
#define DAC2_CS_GPIO_Port GPIOB
#define DAC2_DIV_Pin GPIO_PIN_1
#define DAC2_DIV_GPIO_Port GPIOB
#define DAC1_LAC_Pin GPIO_PIN_8
#define DAC1_LAC_GPIO_Port GPIOA
#define DAC1_CS_Pin GPIO_PIN_11
#define DAC1_CS_GPIO_Port GPIOA
#define DAC2_LAC_Pin GPIO_PIN_12
#define DAC2_LAC_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_15
#define VCP_RX_GPIO_Port GPIOA
#define DAC1_GAIN_Pin GPIO_PIN_3
#define DAC1_GAIN_GPIO_Port GPIOB
#define DAC1_DIV_Pin GPIO_PIN_4
#define DAC1_DIV_GPIO_Port GPIOB
#define DAC1_RST_Pin GPIO_PIN_5
#define DAC1_RST_GPIO_Port GPIOB
#define DAC2_RST_Pin GPIO_PIN_6
#define DAC2_RST_GPIO_Port GPIOB
#define DAC2_GAIN_Pin GPIO_PIN_7
#define DAC2_GAIN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
