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

//AD COMMANDS
#define READ 			0x0A
#define WRITE			0x0B
#define READ_FIFO		0x0D
//AD REGISTERS
#define DEVID_AD 		0x00
#define DEVID_MST 		0x01
#define PARTID			0x02
#define REVID			0x03
#define XDATA			0x08
#define YDATA			0x09
#define ZDATA			0x0A
#define STATUS			0x0B
#define FIFO_ENTRIES_L	0x0C
#define FIFO_ENTRIES_H	0x0D
#define XDATA_L			0x0E
#define XDATA_H			0x0F
#define YDATA_L			0x10
#define YDATA_H			0x11
#define ZDATA_L			0x12
#define ZDATA_H			0x13
#define TEMP_L			0x14
#define TEMP_H			0x15
#define SOFT_RESET		0x1F
#define THRESH_ACT_L	0x20
#define THRESH_ACT_H	0x21
#define TIME_ACT		0x22
#define THRESH_INACT_L	0x23
#define THRESH_INACT_H	0x24
#define TIME_INACT_L	0x25
#define TIME_INACT_H	0x26
#define ACT_INACT_CTL	0x27
#define FIFO_CONTROL	0x28
#define FIFO_SAMPLES	0x29
#define INTMAP1			0x2A
#define INTMAP2			0x2B
#define FILTER_CTL		0x2C
#define POWER_CTL		0x2D
#define SELF_TEST		0x2E

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */

typedef struct AccelerometerStatus {
	unsigned int ERR_USER_REGS :1;
	unsigned int AWAKE :1;
	unsigned int INACT :1;
	unsigned int ACT :1;
	unsigned int FIFO_OVERRUN :1;
	unsigned int FIFO_WATERMARK :1;
	unsigned int FIFO_READY :1;
	unsigned int DATA_READY :1;
} Acc_Status;

Acc_Status ADXL;

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

void parseAccStatus(uint8_t status_byte){
	ADXL.ERR_USER_REGS = 	(status_byte >> 7) & 0x01;
	ADXL.AWAKE = 			(status_byte >> 6) & 0x01;
	ADXL.INACT = 			(status_byte >> 5) & 0x01;
	ADXL.ACT = 				(status_byte >> 4) & 0x01;
	ADXL.FIFO_OVERRUN = 	(status_byte >> 3) & 0x01;
	ADXL.FIFO_WATERMARK = 	(status_byte >> 2) & 0x01;
	ADXL.FIFO_READY = 		(status_byte >> 1) & 0x01;
	ADXL.DATA_READY = 		(status_byte >> 0) & 0x01;
}



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

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == 0){
		//A0 acc interrupt activity detected
		setBoardLED(1);
	}
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi){
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, 1); //Chip select high -> acc deselected
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi){

}

void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef *hspi){

}

void initAcc(){
#define length 30
	uint8_t dummy_read[length];
	uint8_t commands[length/3][3] = { //1. reg, 2. value, ...
		WRITE,
		THRESH_ACT_L, //activity threshold 250mg
		0xFA, //one sample -> direct detection

		WRITE,
		THRESH_ACT_H,
		0x00,

		WRITE,
		THRESH_INACT_L, //2 inact. threshold 150mg
		0x96,

		WRITE,
		THRESH_INACT_H,
		0x00,

		WRITE,
		TIME_INACT_L, // inact. timer 30sampl.
		0x20,

		WRITE,
		TIME_ACT, // act. timer 0x20 sampl.
		0x20,

		WRITE,
		ACT_INACT_CTL, //4 motion detect loop mode
		0x3F,

		WRITE,
		INTMAP1, //INT1 pin
		0x10, //Activity mapped to INT1 pin

		WRITE,
		FILTER_CTL, //
		0x03, //100Hz, 2g measurement, no filtering, low power

		WRITE,
		POWER_CTL, //6 Begin measur. wakeup mode
		0b00001010

	};
	for(int i = 0; i < length/3; i++){
		HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, 0);
		uint8_t current_cmd[3] = {commands[i][0], commands[i][1], commands[i][2]};
		HAL_SPI_TransmitReceive(&hspi2, current_cmd, dummy_read, 3, 10);
		HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, 1); //use if not using interrupts

	}

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

	initAcc();



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  //Test SPI / Acc
	  	while(1){
	  		uint8_t status_read[3] = {0x0B, 0x0B, 0x00}; //write, reg 0, dummy
	  		HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, 0);
	  		HAL_SPI_TransmitReceive(&hspi2, status_read, answerFromSPI, 3, 10);
	  		HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, 1); //use if not using interrupts
	  		uint8_t status = answerFromSPI[2];

	  		parseAccStatus(status);
	  		if(ADXL.ERR_USER_REGS == 1){
	  			//while(1){
	  				//ERROR, ACC NOT INITIALIZED
	  			//}
	  		}
	  		/*if(ADXL.DATA_READY){
	  			uint8_t x_read[3] = {0x0B, 0x0E, 0x00}; //write, reg 0, dummy
	  			HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, 0);
	  			HAL_SPI_TransmitReceive(&hspi2, x_read, answerFromSPI, 3, 10);
	  			HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, 1); //use if not using interrupts
	  			uint8_t x = answerFromSPI[2];
	  			if(x > 128){
	  				setBoardLED(1);
	  			}else{
	  				setBoardLED(0);
	  			}
	  		}*/

	  		if(ADXL.ACT){
				setBoardLED(1);
			}
	  		else{
	  		//if(ADXL.INACT){
	  			HAL_Delay(100);
				setBoardLED(0);
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

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
