/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define USING_STOPMODE 0


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

uint8_t answerFromSPI[10];

#define ACC_BUFFER_SIZE 50
uint8_t acc_buffer_main[ACC_BUFFER_SIZE];
uint16_t acc_buffer_reference[ACC_BUFFER_SIZE];

RTC_TimeTypeDef myTime;
RTC_AlarmTypeDef myAlarm;

int RTC_EVENT = 0;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void setBoardLED(int state){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, !state);
}
int getBoardLED(){
	return HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc){
	RTC_EVENT = 1;

	myTime.Hours = 0;
	myTime.Minutes = 0;
	myTime.Seconds = 0;
	HAL_RTC_SetTime(hrtc, &myTime, RTC_FORMAT_BIN);

	myTime.Hours = 0;
	myTime.Minutes = 0;
	myTime.Seconds = 2;
	myAlarm.AlarmTime = myTime;

	HAL_RTC_SetAlarm_IT(hrtc, &myAlarm, RTC_FORMAT_BIN);

}


void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi){
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, 1); //Chip select high -> acc deselected
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi){

}

void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef *hspi){

}

void acc_cmd(uint8_t* data, uint8_t length){

}

long tss(int data){
	static int counter = 0;
	static int ringbuffer[50];
	if(counter < 50-1){counter++;}else{counter = 0;}

	ringbuffer[counter] = data;
	long squared_sum = 0; //long max = +2,147,483,647, max for 50 values = 838'860'800
	for(int i = 0; i < 50-1; i++){
		squared_sum += ringbuffer[i]*ringbuffer[i];
	}
	return squared_sum;
}

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
  MX_RTC_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

/*
	HAL_SPI_Transmit(&hspi2, (uint8_t *)0x0A, 1, 100);
	HAL_SPI_Transmit(&hspi2, (uint8_t *)0x20, 1, 100);
	HAL_SPI_Transmit(&hspi2, (uint8_t *)0xFA, 1, 100);

	HAL_SPI_Transmit(&hspi2, (uint8_t *)0x0A, 1, 100);
	HAL_SPI_Transmit(&hspi2, (uint8_t *)0x21, 1, 100);
	HAL_SPI_Transmit(&hspi2, (uint8_t *)0x00, 1, 100);

	HAL_SPI_Transmit(&hspi2, (uint8_t *)0x0A, 1, 100);
	HAL_SPI_Transmit(&hspi2, (uint8_t *)0x25, 1, 100);
	HAL_SPI_Transmit(&hspi2, (uint8_t *)0x1E, 1, 100);

	HAL_SPI_Transmit(&hspi2, (uint8_t *)0x0A, 1, 100);
	HAL_SPI_Transmit(&hspi2, (uint8_t *)0x27, 1, 100);
	HAL_SPI_Transmit(&hspi2, (uint8_t *)0x3F, 1, 100);

	HAL_SPI_Transmit(&hspi2, (uint8_t *)0x0A, 1, 100);
	HAL_SPI_Transmit(&hspi2, (uint8_t *)0x2B, 1, 100);
	HAL_SPI_Transmit(&hspi2, (uint8_t *)0x40, 1, 100);

	HAL_SPI_Transmit(&hspi2, (uint8_t *)0x0A, 1, 100);
	HAL_SPI_Transmit(&hspi2, (uint8_t *)0x2D, 1, 100);
	HAL_SPI_Transmit(&hspi2, (uint8_t *)0x0A, 1, 100);
*/

	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin,1); //Accelerometer !CS high to disable
	hspi2.Init.NSS = SPI_NSS_SOFT;

	HAL_DBGMCU_EnableDBGStopMode();

	setBoardLED(0);

	myTime.Hours = 0;
	myTime.Minutes = 0;
	myTime.Seconds = 0;
	HAL_RTC_SetTime(&hrtc, &myTime, RTC_FORMAT_BIN);

	myTime.Hours = 0;
	myTime.Minutes = 0;
	myTime.Seconds = 2;

	myAlarm.AlarmTime = myTime;

	//HAL_RTC_SetAlarm_IT(&hrtc, &myAlarm, RTC_FORMAT_BIN);

	#if USING_STOPMODE == 1
	for(int i = 0; i < 15; i++){ //7.5s to connect to debugger
		setBoardLED(1);
		HAL_Delay(250);
		setBoardLED(0);
		HAL_Delay(250);
	}
	#endif

	uint8_t id_read[3] = {0x0B, 0x00, 0x00}; //write, reg 0, dummy

	HAL_SPI_Receive_IT(&hspi2, answerFromSPI, 3);

	HAL_SPI_Transmit_IT(&hspi2, id_read, 3);

	//Test SPI
	while(1){


		//HAL_SPI_TransmitReceive(&hspi2, id_read, answerFromSPI, 3, 100);
		HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, 0);
		HAL_SPI_TransmitReceive_IT(&hspi2, id_read, answerFromSPI, 3);


		HAL_Delay(10);
		//HAL_SPI_Transmit_IT(&hspi2, id_read, 3);

		if(answerFromSPI[2] == 0xAD){
			setBoardLED(1);
		}else{
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		}
	}


	//Test Stop Mode
	while(1){

		if(RTC_EVENT){
			RTC_EVENT = 0;
			setBoardLED(1);
			HAL_Delay(1);
			setBoardLED(0);

			//Put in stop mode
		#if USING_STOPMODE == 1
			HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
		#endif

		}else{
			setBoardLED(1);
			HAL_Delay(1);
			setBoardLED(0);
			HAL_Delay(5);
		}


	}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
    
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date 
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 0x1;
  DateToUpdate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_CS_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI2_CS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
