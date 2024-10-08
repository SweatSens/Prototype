/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* Definitions for CLI */
osThreadId_t CLIHandle;
const osThreadAttr_t CLI_attributes = {
  .name = "CLI",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Temp */
osThreadId_t TempHandle;
const osThreadAttr_t Temp_attributes = {
  .name = "Temp",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for V1 */
osThreadId_t V1Handle;
const osThreadAttr_t V1_attributes = {
  .name = "V1",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for V2 */
osThreadId_t V2Handle;
const osThreadAttr_t V2_attributes = {
  .name = "V2",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for V3 */
osThreadId_t V3Handle;
const osThreadAttr_t V3_attributes = {
  .name = "V3",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for V4 */
osThreadId_t V4Handle;
const osThreadAttr_t V4_attributes = {
  .name = "V4",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for P1 */
osThreadId_t P1Handle;
const osThreadAttr_t P1_attributes = {
  .name = "P1",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for VSIM */
osThreadId_t VSIMHandle;
const osThreadAttr_t VSIM_attributes = {
  .name = "VSIM",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Print */
osThreadId_t PrintHandle;
const osThreadAttr_t Print_attributes = {
  .name = "Print",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for mutexSPI */
osMutexId_t mutexSPIHandle;
const osMutexAttr_t mutexSPI_attributes = {
  .name = "mutexSPI"
};
/* Definitions for mutexPrint */
osMutexId_t mutexPrintHandle;
const osMutexAttr_t mutexPrint_attributes = {
  .name = "mutexPrint"
};
/* Definitions for semV1 */
osSemaphoreId_t semV1Handle;
const osSemaphoreAttr_t semV1_attributes = {
  .name = "semV1"
};
/* Definitions for semV2 */
osSemaphoreId_t semV2Handle;
const osSemaphoreAttr_t semV2_attributes = {
  .name = "semV2"
};
/* Definitions for semV3 */
osSemaphoreId_t semV3Handle;
const osSemaphoreAttr_t semV3_attributes = {
  .name = "semV3"
};
/* Definitions for semV4 */
osSemaphoreId_t semV4Handle;
const osSemaphoreAttr_t semV4_attributes = {
  .name = "semV4"
};
/* Definitions for semTemp */
osSemaphoreId_t semTempHandle;
const osSemaphoreAttr_t semTemp_attributes = {
  .name = "semTemp"
};
/* Definitions for semP1 */
osSemaphoreId_t semP1Handle;
const osSemaphoreAttr_t semP1_attributes = {
  .name = "semP1"
};
/* Definitions for semVSIM */
osSemaphoreId_t semVSIMHandle;
const osSemaphoreAttr_t semVSIM_attributes = {
  .name = "semVSIM"
};
/* Definitions for semReturn */
osSemaphoreId_t semReturnHandle;
const osSemaphoreAttr_t semReturn_attributes = {
  .name = "semReturn"
};
/* USER CODE BEGIN PV */
uint8_t UART_tx_flag = 0;
uint8_t UART_rx_flag = 0;
uint8_t UART_rx[1];
uint8_t UART_rx_buff[100];
uint8_t UARTcnt = 0;
struct Channels channels;
struct pH p1;

// message queue
osMessageQueueId_t queueHandle;
const osMessageQueueAttr_t queue_attributes = {
  .name = "queue"
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
void StartCLI(void *argument);
void StartTemp(void *argument);
void StartV1(void *argument);
void StartV2(void *argument);
void StarV3(void *argument);
void StartV4(void *argument);
void StartP1(void *argument);
void StartVSIM(void *argument);
void StartPrint(void *argument);

/* USER CODE BEGIN PFP */
void SPI1_change_polarity(void);
void voltammerty(uint8_t channel, uint8_t set, uint8_t DAC_Vin, uint8_t DAC_Vw);
void ADC_get_send(uint8_t setting, uint8_t channel, float dac, uint32_t tick);
void DAC_set(uint8_t setting, uint8_t channel, float voltage);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	// ADC change polarity SPI for DAC's
	ADC_change_polarity();
	SPI1_change_polarity();
	check_ADC();
	// disable internal reference DAC's
	write_DAC(DAC_1, CONFIG, 0x0100);
	write_DAC(DAC_2, CONFIG, 0x0100);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of mutexSPI */
  mutexSPIHandle = osMutexNew(&mutexSPI_attributes);

  /* creation of mutexPrint */
  mutexPrintHandle = osMutexNew(&mutexPrint_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of semV1 */
  semV1Handle = osSemaphoreNew(1, 0, &semV1_attributes);

  /* creation of semV2 */
  semV2Handle = osSemaphoreNew(1, 0, &semV2_attributes);

  /* creation of semV3 */
  semV3Handle = osSemaphoreNew(1, 0, &semV3_attributes);

  /* creation of semV4 */
  semV4Handle = osSemaphoreNew(1, 0, &semV4_attributes);

  /* creation of semTemp */
  semTempHandle = osSemaphoreNew(1, 0, &semTemp_attributes);

  /* creation of semP1 */
  semP1Handle = osSemaphoreNew(1, 0, &semP1_attributes);

  /* creation of semVSIM */
  semVSIMHandle = osSemaphoreNew(1, 0, &semVSIM_attributes);

  /* creation of semReturn */
  semReturnHandle = osSemaphoreNew(7, 7, &semReturn_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  queueHandle = osMessageQueueNew(20, sizeof(MSGQUEUE_OBJ_t), &queue_attributes);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of CLI */
  CLIHandle = osThreadNew(StartCLI, NULL, &CLI_attributes);

  /* creation of Temp */
  TempHandle = osThreadNew(StartTemp, NULL, &Temp_attributes);

  /* creation of V1 */
  V1Handle = osThreadNew(StartV1, NULL, &V1_attributes);

  /* creation of V2 */
  V2Handle = osThreadNew(StartV2, NULL, &V2_attributes);

  /* creation of V3 */
  V3Handle = osThreadNew(StarV3, NULL, &V3_attributes);

  /* creation of V4 */
  V4Handle = osThreadNew(StartV4, NULL, &V4_attributes);

  /* creation of P1 */
  P1Handle = osThreadNew(StartP1, NULL, &P1_attributes);

  /* creation of VSIM */
  VSIMHandle = osThreadNew(StartVSIM, NULL, &VSIM_attributes);

  /* creation of Print */
  PrintHandle = osThreadNew(StartPrint, NULL, &Print_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10909CEC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ADC_CS_Pin|DAC1_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DAC2_CS_Pin|DAC2_DIV_Pin|DAC1_GAIN_Pin|DAC1_DIV_Pin
                          |DAC2_GAIN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DAC1_LAC_Pin|DAC2_LAC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DAC1_RST_Pin|DAC2_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : TempAlert_Pin */
  GPIO_InitStruct.Pin = TempAlert_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TempAlert_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ADC_CS_Pin DAC1_LAC_Pin DAC1_CS_Pin DAC2_LAC_Pin */
  GPIO_InitStruct.Pin = ADC_CS_Pin|DAC1_LAC_Pin|DAC1_CS_Pin|DAC2_LAC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DAC2_CS_Pin DAC2_DIV_Pin DAC1_GAIN_Pin DAC1_DIV_Pin
                           DAC1_RST_Pin DAC2_RST_Pin DAC2_GAIN_Pin */
  GPIO_InitStruct.Pin = DAC2_CS_Pin|DAC2_DIV_Pin|DAC1_GAIN_Pin|DAC1_DIV_Pin
                          |DAC1_RST_Pin|DAC2_RST_Pin|DAC2_GAIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// this function gets the value from the
void ADC_get_send(uint8_t setting, uint8_t channel, float dac, uint32_t tick){
	MSGQUEUE_OBJ_t msg;

	if(setting == 0){
		uint8_t sel_ch[2] = {channel*2,channel*2+1};	// channel for both ADC of one voltammerty channel
		float volt[2];
		osMutexAcquire(mutexSPIHandle, osWaitForever);
		ADC_mult(2,sel_ch,volt);	// two as first argument otherwise nothing is received
		osMutexRelease(mutexSPIHandle);
		// send data
		msg.channel = channel;
		msg.tick = tick;
		msg.val_x = dac;
		msg.val = volt[0];
		msg.val_rf = volt[1];
	    osMessageQueuePut(queueHandle, &msg, 0U, 0U);
	}
	else{
		// for simultaneously mode use the sel_ch to see the channels that are selected (in binary)
		uint8_t cnt = 0;
		uint8_t sel_ch[8] = {0,0,0,0,0,0,0,0};
		uint8_t ch[8] = {0,0,0,0};
		for(uint8_t i=0;i<4;i++){
			if(channel&0x01){
				ch[cnt] = i;
				sel_ch[cnt*2] = i*2;		// two ADC for one voltammerty channel. Thus, 8 channels
				sel_ch[cnt*2+1] = i*2+1;
				cnt += 1;
			}
			channel = channel>>1;
		}
		float volt[cnt*2];
		osMutexAcquire(mutexSPIHandle, osWaitForever);
		ADC_mult(cnt*2,sel_ch,volt);
		osMutexRelease(mutexSPIHandle);
		// send data
		for(uint8_t i=0;i<cnt;i++){
			switch(ch[i]){
				case V1:
					msg.channel = V1;
					break;
				case V2:
					msg.channel = V2;
					break;
				case V3:
					msg.channel = V3;
					break;
				case V4:
					msg.channel = V4;
					break;
			}
			msg.tick = tick;
			msg.val_x = dac;
			msg.val = volt[i*2];
			msg.val_rf = volt[i*2+1];
		    osMessageQueuePut(queueHandle, &msg, 0U, 0U);
		}
	}

	return;
}

void DAC_set(uint8_t setting, uint8_t channel, float voltage){
	uint8_t dac = 0;
	uint16_t result = (uint16_t)(voltage/33.0*655360.0);

	if(setting == 0){
		if(channel > CH3){
			channel -= 4;
			dac = DAC_2;
		}
		else{
			dac = DAC_1;
		}
		uint8_t reg = channel + 8;

		osMutexAcquire(mutexSPIHandle, osWaitForever);
		write_DAC(dac, reg, result);
		osMutexRelease(mutexSPIHandle);
	}
	else if(setting == 1){
		osMutexAcquire(mutexSPIHandle, osWaitForever);
		write_DAC(DAC_1, SYNC, (channel<<8));
		write_DAC(DAC_2, SYNC, (channel<<4));
		write_DAC(DAC_1, BRDCAST, result);
		write_DAC(DAC_2, BRDCAST, result);
		osMutexRelease(mutexSPIHandle);
	}

	return;
}

void voltammerty(uint8_t channel, uint8_t set, uint8_t DAC_Vin, uint8_t DAC_Vw){
	// get only one channel
	uint8_t chnnl = channel;
	if(set == SIM){
		uint8_t ch = channel;
		for(uint8_t i=0;i<4;i++){
			if(ch&0x01){
				channel = i;
			}
			ch = ch>>1;
		}
	}
	float E = 0;
	float vin = 0;
	uint8_t equilibration = 0;
	uint32_t tick;
	uint32_t wait;
	uint32_t start_tick = osKernelGetTickCount();
	switch(channels.parameters[channel].technique){
		case 1:
			// variables CV
			if(channels.parameters[channel].cv.E_vertex_1 < 0.0){
				if(channels.parameters[channel].cv.E_begin >= 0.0){
					E = channels.parameters[channel].cv.E_begin + channels.parameters[channel].cv.E_vertex_1*-2.0;
				}
				else{
					E = channels.parameters[channel].cv.E_vertex_1*-2.0 - channels.parameters[channel].cv.E_begin*-1.0;
				}
				vin = channels.parameters[channel].cv.E_vertex_1*-2.0;
			}
			else{
				E = 0.0;
				vin = 0.0;
			}
			DAC_set(set, DAC_Vw, 0.0);
			DAC_set(set, DAC_Vin, 0.0);
			float rate = channels.parameters[channel].cv.E_step/channels.parameters[channel].cv.scan_rate*1000;
			float rise = (channels.parameters[channel].cv.E_vertex_2 - channels.parameters[channel].cv.E_begin)/channels.parameters[channel].cv.E_step;
			uint32_t end_loop_rise = (uint32_t)rise;
			float fall = (channels.parameters[channel].cv.E_vertex_2 - channels.parameters[channel].cv.E_vertex_1)/channels.parameters[channel].cv.E_step;
			uint32_t end_loop_fall = (uint32_t)fall;
			// start CV
			osDelay(channels.parameters[channel].cv.t_equilibration*1000);
			// loops for rise and fall CV
			for(uint32_t i=0;i<end_loop_rise;i++){
				E = E + channels.parameters[channel].cv.E_step;
				DAC_set(set, DAC_Vw, E);
				osDelay(rate);
				ADC_get_send(set, chnnl,(E-vin), (osKernelGetTickCount()-start_tick));
			}
			for(uint32_t i=0;i<end_loop_fall;i++){
				E = E - channels.parameters[channel].cv.E_step;
				DAC_set(set, DAC_Vw, E);
				osDelay(rate);
				ADC_get_send(set, chnnl, (E-vin), (osKernelGetTickCount()-start_tick));
			}
			E = 0;
			DAC_set(set, DAC_Vw, E);
			DAC_set(set, DAC_Vin, 0.0);
			break;
		case 2:
			// variables DPV
			if(channels.parameters[channel].dpv.E_begin < 0.0){
				E = channels.parameters[channel].dpv.E_begin*-1.0;
				vin = channels.parameters[channel].dpv.E_begin*-2.0;

			}
			else{
				E = channels.parameters[channel].dpv.E_begin;
				vin = 0.0;
			}
			DAC_set(set, DAC_Vw, E);
			DAC_set(set, DAC_Vin, vin);
			float t_low = (channels.parameters[channel].dpv.E_step/channels.parameters[channel].dpv.Scan_rate - channels.parameters[channel].dpv.t_pulse) * 1000;
			float end_DPV = (channels.parameters[channel].dpv.E_end-channels.parameters[channel].dpv.E_begin)/channels.parameters[channel].dpv.E_step;
			uint32_t end_loop_DPV = (uint32_t)end_DPV;
			// start DPV
			for(uint32_t i=0;i<(end_loop_DPV+1);i++){
				if(equilibration == 0){
					osDelay(channels.parameters[channel].dpv.t_equilibration*1000);
					equilibration = 1;
				}
				E = E + channels.parameters[channel].dpv.E_pulse;
				DAC_set(set, DAC_Vw, E);
				osDelay(channels.parameters[channel].dpv.t_pulse*1000-1);
				tick = osKernelGetTickCount();
				wait = tick + 1;
				ADC_get_send(set, chnnl, (E-vin), (tick-start_tick));
				osDelayUntil(wait);
				E = E - channels.parameters[channel].dpv.E_pulse + channels.parameters[channel].dpv.E_step;
				DAC_set(set, DAC_Vw, E);
				osDelay(t_low-1);
				tick = osKernelGetTickCount();
				wait = tick + 1;
				ADC_get_send(set, chnnl, (E-vin), (tick-start_tick));
				osDelayUntil(wait);
			}
			E = 0;
			DAC_set(set, DAC_Vw, E);
			DAC_set(set, DAC_Vin, 0.0);
			equilibration = 0;
			break;
		case 3:
			// variables SWV
			if(channels.parameters[channel].swv.E_begin < 0.0){
				E = channels.parameters[channel].swv.E_begin*-1.0;
				vin = channels.parameters[channel].swv.E_begin*-2.0;
			}
			else{
				E = channels.parameters[channel].swv.E_begin;
				vin = 0.0;
			}
			DAC_set(set, DAC_Vw, E);
			DAC_set(set, DAC_Vin, vin);
			float period = 1/(2*channels.parameters[channel].swv.Frequency)*1000;
			float end_SWV = (channels.parameters[channel].swv.E_end-channels.parameters[channel].swv.E_begin)/channels.parameters[channel].swv.E_step;
			uint32_t end_loop_SWV = (uint32_t)end_SWV;
			// start SWV
			for(uint32_t i=0;i<(end_loop_SWV+1);i++){
				if(equilibration == 0){
					osDelay(channels.parameters[channel].swv.t_equilibration*1000);
					equilibration = 1;
					E = E - channels.parameters[channel].swv.Amplitude;	// to start
				}
				E = E + channels.parameters[channel].swv.E_step + 2*channels.parameters[channel].swv.Amplitude;
				DAC_set(set, DAC_Vw, E);
				osDelay(period-1);
				tick = osKernelGetTickCount();
				wait = tick + 1;
				ADC_get_send(set, chnnl, (E-vin), (tick-start_tick));
				osDelayUntil(wait);
				E = E - 2*channels.parameters[channel].swv.Amplitude;
				DAC_set(set, DAC_Vw, E);
				osDelay(period-1);
				tick = osKernelGetTickCount();
				wait = tick + 1;
				ADC_get_send(set, chnnl, (E-vin), (tick-start_tick));
				osDelayUntil(wait);
			}
			E = 0;
			DAC_set(set, DAC_Vw, E);
			DAC_set(set, DAC_Vin, 0.0);
			equilibration = 0;
			break;
	}
	return;
}

void SPI1_change_polarity(void){
  // change polarity for ADC and DAC (SPI)
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

}
void clear_UART_rx_buff(void){
	// clear UART rx buffer
	for(uint8_t i=0;i<100;i++){
		UART_rx_buff[i] = 0;
	}
	return;
}
// interrupt(s)
void HAL_UART_TxCpltCallback(UART_HandleTypeDef * huart){
	UART_tx_flag = 0;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart){
	// put received data in UART_Rx_buff
	HAL_UART_Receive_DMA(&huart2, UART_rx, 1);
	UART_rx_buff[UARTcnt] = UART_rx[0];
	// check for new line and reset incrementer for UART_Rx_buff
	if(UART_rx_buff[UARTcnt] == 10){
		UART_rx_flag = 1;
		UARTcnt = 0;
	}
	else{
		UARTcnt += 1;
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartCLI */
/**
  * @brief  Function implementing the CLI thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartCLI */
void StartCLI(void *argument)
{
  /* USER CODE BEGIN 5 */
	// set uart DMA receive
	HAL_UART_Receive_DMA(&huart2, UART_rx, 1);

	// set default settings of the prototype
	p1.t_interval = 0.1;
	p1.t_run = 1.0;

	channels.parameters[0].technique = 3.0;
	channels.parameters[0].swv.t_equilibration = 5;
	channels.parameters[0].swv.E_begin = -0.5;
	channels.parameters[0].swv.E_end = 1.0;
	channels.parameters[0].swv.Frequency = 10;
	channels.parameters[0].swv.E_step = 0.005;
	channels.parameters[0].swv.Amplitude = 0.025;

//	channels.parameters[0].technique = 2.0;
//	channels.parameters[0].dpv.t_equilibration = 5;
//	channels.parameters[0].dpv.E_begin = -0.5;
//	channels.parameters[0].dpv.E_end = 1.2;
//	channels.parameters[0].dpv.Scan_rate = 0.05;
//	channels.parameters[0].dpv.E_step = 0.005;
//	channels.parameters[0].dpv.E_pulse = 0.03;
//	channels.parameters[0].dpv.t_pulse = 0.015;

//	channels.parameters[0].technique = 1.0;
//	channels.parameters[0].cv.t_equilibration = 5;
//	channels.parameters[0].cv.E_begin = 0.5;
//	channels.parameters[0].cv.E_vertex_2 = 1.0;
//	channels.parameters[0].cv.E_vertex_1 = -0.5;
//	channels.parameters[0].cv.E_step  = 0.01;
//	channels.parameters[0].cv.scan_rate = 1.0;

	channels.parameters[1].technique = 0.0;
	channels.parameters[2].technique = 0.0;
	channels.parameters[3].technique = 0.0;


  /* Infinite loop */
  for(;;)
  {
	// check all if all tasks are done
	for(uint8_t i=0;i<7;i++){
		osSemaphoreAcquire(semReturnHandle, osWaitForever);
	}

	// use uart (with mutex because message queue task also use uart)
	osMutexAcquire(mutexPrintHandle, osWaitForever);
	// wait for input
	clear_UART_rx_buff();	// wait until user type someting in CLI and clear it to show menu
	UART_rx_flag = 0;
	while(UART_rx_flag != 1);
	clear_UART_rx_buff();
	UART_rx_flag = 0;
	UART_menu();			// run menu in CLI
	// release the tasks to execute
	osMutexRelease(mutexPrintHandle);

	osSemaphoreRelease(semTempHandle);
	osSemaphoreRelease(semV1Handle);
	osSemaphoreRelease(semV2Handle);
	osSemaphoreRelease(semV3Handle);
	osSemaphoreRelease(semV4Handle);
	osSemaphoreRelease(semP1Handle);
	osSemaphoreRelease(semVSIMHandle);

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTemp */
/**
* @brief Function implementing the Temp thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTemp */
void StartTemp(void *argument)
{
  /* USER CODE BEGIN StartTemp */
	uint8_t I2C_addrss = 0b10010000;	// address of temp sensor
	uint8_t I2C_tx_data[1];
	uint8_t I2C_rx_data[2];
	int16_t I2C_temp = 0;
	float temp = 0.0;
	MSGQUEUE_OBJ_t msg;
	/* Infinite loop */
	for(;;)
	{
	    osSemaphoreAcquire(semTempHandle, osWaitForever);
		// get temperature
		I2C_tx_data[0] = 0x01;
		HAL_I2C_Master_Transmit(&hi2c1, I2C_addrss, I2C_tx_data, 1, 100);
		HAL_I2C_Master_Receive(&hi2c1, I2C_addrss, I2C_rx_data, 2, 100);
		if((I2C_rx_data[1]&0x20) == 0x20){	// check if conversion is done
			I2C_tx_data[0] = 0x00;
			HAL_I2C_Master_Transmit(&hi2c1, I2C_addrss, I2C_tx_data, 1, 100);
			HAL_I2C_Master_Receive(&hi2c1, I2C_addrss, I2C_rx_data, 2, 100);
			I2C_temp = (I2C_rx_data[0]<<8) + I2C_rx_data[1];
			temp = (float)I2C_temp * 0.0078125;
			// send data
			msg.channel = TEMP;
			msg.val = temp;
		    osMessageQueuePut(queueHandle, &msg, 0U, 0U);
		}
	    osSemaphoreRelease(semReturnHandle);
	}

  /* USER CODE END StartTemp */
}

/* USER CODE BEGIN Header_StartV1 */
/**
* @brief Function implementing the V1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartV1 */
void StartV1(void *argument)
{
  /* USER CODE BEGIN StartV1 */
	/* Infinite loop */
	for(;;)
	{
		// voltammerty channel 1 in individual mode
		osSemaphoreAcquire(semV1Handle, osWaitForever);
		if(channels.on == ON && channels.broadcast == 0 && channels.parameters[0].technique != 0){
			voltammerty(V1, IND, CH0, CH1);
		}
		osSemaphoreRelease(semReturnHandle);
	}
  /* USER CODE END StartV1 */
}

/* USER CODE BEGIN Header_StartV2 */
/**
* @brief Function implementing the V2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartV2 */
void StartV2(void *argument)
{
  /* USER CODE BEGIN StartV2 */
  /* Infinite loop */
  for(;;)
  {
	// voltammerty channel 2 in individual mode
	osSemaphoreAcquire(semV2Handle, osWaitForever);
	if(channels.on == ON && channels.broadcast == 0 && channels.parameters[1].technique != 0){
		voltammerty(V2, IND, CH3, CH2);
	}
	osSemaphoreRelease(semReturnHandle);
  }
  /* USER CODE END StartV2 */
}

/* USER CODE BEGIN Header_StarV3 */
/**
* @brief Function implementing the V3 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StarV3 */
void StarV3(void *argument)
{
  /* USER CODE BEGIN StarV3 */
  /* Infinite loop */
  for(;;)
  {
	// voltammerty channel 3 in individual mode
	osSemaphoreAcquire(semV3Handle, osWaitForever);
	if(channels.on == ON && channels.broadcast == 0 && channels.parameters[2].technique != 0){
		voltammerty(V3, IND, CH4, CH5);
	}
	osSemaphoreRelease(semReturnHandle);
  }
  /* USER CODE END StarV3 */
}

/* USER CODE BEGIN Header_StartV4 */
/**
* @brief Function implementing the V4 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartV4 */
void StartV4(void *argument)
{
  /* USER CODE BEGIN StartV4 */
  /* Infinite loop */
  for(;;)
  {
	// voltammerty channel 4 in individual mode
	osSemaphoreAcquire(semV4Handle, osWaitForever);
	if(channels.on == ON && channels.broadcast == 0 && channels.parameters[3].technique != 0){
		voltammerty(V4, IND, CH7, CH6);
	}
	osSemaphoreRelease(semReturnHandle);
  }
  /* USER CODE END StartV4 */
}

/* USER CODE BEGIN Header_StartP1 */
/**
* @brief Function implementing the P1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartP1 */
void StartP1(void *argument)
{
  /* USER CODE BEGIN StartP1 */
	MSGQUEUE_OBJ_t msg;
	float volt = 0.0;
	uint32_t tick, wait = 0;
  /* Infinite loop */
  for(;;)
  {
	// potential channel for measuring pH
	osSemaphoreAcquire(semP1Handle, osWaitForever);
	if(p1.on == ON){		// check if potentiostat is needs to measure
		HAL_ADC_Start(&hadc1);
		// calculate the end time for the for loop
		float end = p1.t_run/p1.t_interval;
		uint32_t end_loop = (uint32_t) end;
		uint32_t start_tick = osKernelGetTickCount();
		for(uint32_t i=0;i<end_loop;i++){
			tick = osKernelGetTickCount();
			wait = tick + (uint32_t)(1000*p1.t_interval); // get time and run next code and wait until time is done
			HAL_ADC_PollForConversion(&hadc1, 100);	// get data ADC
			uint32_t val_ADC_raw = HAL_ADC_GetValue(&hadc1);
			float val_ADC = (float)val_ADC_raw;
			volt = val_ADC*33/40960;
			// send potential calculated from ADC
			msg.channel = P1;
			msg.tick = (tick - start_tick);
			msg.val = volt;
		    osMessageQueuePut(queueHandle, &msg, 0U, 0U);
			osDelayUntil(wait);
		}
		HAL_ADC_Stop(&hadc1);
	}
	osSemaphoreRelease(semReturnHandle);
  }
  /* USER CODE END StartP1 */
}

/* USER CODE BEGIN Header_StartVSIM */
/**
* @brief Function implementing the VSIM thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartVSIM */
void StartVSIM(void *argument)
{
  /* USER CODE BEGIN StartVSIM */
	uint8_t channel = 0;
	uint8_t DAC_Vin = 0;
	uint8_t DAC_Vw = 0;
  /* Infinite loop */
  for(;;)
  {
	// simultaneously measure the voltammerty channels by setting the the dac channels high for all channels that are measured
	osSemaphoreAcquire(semVSIMHandle, osWaitForever);
	if(channels.on == ON && channels.broadcast == 1){
		for(uint8_t i=0;i<4;i++){
			if(channels.parameters[i].ch_brdcst_on == 1){
				channel = channel | (1<<i);
				switch(i){
					case 0:
						DAC_Vin = DAC_Vin | (1<<CH0);
						DAC_Vw = DAC_Vw | (1<<CH1);
						break;
					case 1:
						DAC_Vin = DAC_Vin | (1<<CH3);
						DAC_Vw = DAC_Vw | (1<<CH2);
						break;
					case 2:
						DAC_Vin = DAC_Vin | (1<<CH4);
						DAC_Vw = DAC_Vw | (1<<CH5);
						break;
					case 3:
						DAC_Vin = DAC_Vin | (1<<CH7);
						DAC_Vw = DAC_Vw | (1<<CH6);
						break;
				}
			}
		}
		voltammerty(channel, SIM, DAC_Vin, DAC_Vw);
		channel = 0;
		DAC_Vin = 0;
		DAC_Vw = 0;
	}
	osSemaphoreRelease(semReturnHandle);
  }
  /* USER CODE END StartVSIM */
}

/* USER CODE BEGIN Header_StartPrint */
/**
* @brief Function implementing the Print thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPrint */
void StartPrint(void *argument)
{
  /* USER CODE BEGIN StartPrint */
	MSGQUEUE_OBJ_t msg;
	osStatus_t status;
  /* Infinite loop */
  for(;;)
  {
	// get data from message queue and send data by UART
	while(osMessageQueueGetCount(queueHandle) != 0){
		osMutexAcquire(mutexPrintHandle, osWaitForever);
		// if message run
		status = osMessageQueueGet(queueHandle, &msg, NULL, 0U);
			if(status == osOK){
				// the pH meter, temperature and all the voltammerty channels have a own message
				if(msg.channel == P1){
					while(UART_tx_flag != 0);
					HAL_UART_Transmit_DMA(&huart2, (uint8_t*)"P1, ", 4);
					UART_tx_flag = 1;
					uint8_t buff_tick[10];
					uint8_t size_tick = float_to_ASCII((float)msg.tick, buff_tick);
					while(UART_tx_flag != 0);
					HAL_UART_Transmit_DMA(&huart2, buff_tick, size_tick);
					UART_tx_flag = 1;
					while(UART_tx_flag != 0);
					HAL_UART_Transmit_DMA(&huart2, (uint8_t*)", ", 2);
					UART_tx_flag = 1;
					uint8_t buff_val[10];
					uint8_t size_val = float_to_ASCII(msg.val, buff_val);
					while(UART_tx_flag != 0);
					HAL_UART_Transmit_DMA(&huart2, buff_val, size_val);
					UART_tx_flag = 1;
					while(UART_tx_flag != 0);
					HAL_UART_Transmit_DMA(&huart2, (uint8_t*)" V\n", 3);
					UART_tx_flag = 1;
				}
				if(msg.channel == TEMP){
					while(UART_tx_flag != 0);
					HAL_UART_Transmit_DMA(&huart2, (uint8_t*)"TEMP, ", 6);
					UART_tx_flag = 1;
					uint8_t buff_val[10];
					uint8_t size_val = float_to_ASCII(msg.val, buff_val);
					while(UART_tx_flag != 0);
					HAL_UART_Transmit_DMA(&huart2, buff_val, size_val);
					UART_tx_flag = 1;
					while(UART_tx_flag != 0);
					HAL_UART_Transmit_DMA(&huart2, (uint8_t*)" C\n", 3);
					UART_tx_flag = 1;
				}
				// check which voltammerty channel is in the message and send appropriate message accordingly
				if(msg.channel == V1 || msg.channel == V2 || msg.channel == V3 || msg.channel == V4){
					while(UART_tx_flag != 0);
					switch(msg.channel){
						case V1:
							HAL_UART_Transmit_DMA(&huart2, (uint8_t*)"V1, ", 4);
							break;
						case V2:
							HAL_UART_Transmit_DMA(&huart2, (uint8_t*)"V2, ", 4);
							break;
						case V3:
							HAL_UART_Transmit_DMA(&huart2, (uint8_t*)"V3, ", 4);
							break;
						case V4:
							HAL_UART_Transmit_DMA(&huart2, (uint8_t*)"V4, ", 4);
							break;
					}
					UART_tx_flag = 1;

					uint8_t buff_tick[10];
					uint8_t size_tick = float_to_ASCII((float)msg.tick, buff_tick);
					while(UART_tx_flag != 0);
					HAL_UART_Transmit_DMA(&huart2, buff_tick, size_tick);
					UART_tx_flag = 1;
					while(UART_tx_flag != 0);
					HAL_UART_Transmit_DMA(&huart2, (uint8_t*)", ", 2);
					UART_tx_flag = 1;
					uint8_t buff_val_x[10];
					uint8_t size_val_x = float_to_ASCII(msg.val_x, buff_val_x);
					while(UART_tx_flag != 0);
					HAL_UART_Transmit_DMA(&huart2, buff_val_x, size_val_x);
					UART_tx_flag = 1;
					while(UART_tx_flag != 0);
					HAL_UART_Transmit_DMA(&huart2, (uint8_t*)", ", 2);
					UART_tx_flag = 1;
					uint8_t buff_val[10];
					uint8_t size_val = float_to_ASCII(msg.val, buff_val);
					while(UART_tx_flag != 0);
					HAL_UART_Transmit_DMA(&huart2, buff_val, size_val);
					UART_tx_flag = 1;
					while(UART_tx_flag != 0);
					HAL_UART_Transmit_DMA(&huart2, (uint8_t*)", ", 2);
					UART_tx_flag = 1;
					uint8_t buff_val_rf[10];
					uint8_t size_val_rf = float_to_ASCII(msg.val_rf, buff_val_rf);
					while(UART_tx_flag != 0);
					HAL_UART_Transmit_DMA(&huart2, buff_val_rf, size_val_rf);
					UART_tx_flag = 1;
					while(UART_tx_flag != 0);
					HAL_UART_Transmit_DMA(&huart2, (uint8_t*)"\n", 1);
					UART_tx_flag = 1;
				}
			}
			osMutexRelease(mutexPrintHandle);
	}
	osDelay(1);

  }
  /* USER CODE END StartPrint */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
