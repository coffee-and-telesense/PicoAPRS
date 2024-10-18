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
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "gpio.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
//#include "wavetable.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	INIT,
	TRIGGER_MEASUREMENT,
	WAIT_FOR_MEASUREMENT,
	READ_DATA,
	PROCESS_DATA,
	PAUSE,
	IDLE
} State;

State currentState = INIT;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AHT20_ADDR 0x38 << 1 // AHT20 I2C address (shifted for HAL compatibility)
#define AHT20_DATA_SIZE 6
#define AHT20_STATE_SIZE 1
#define AHT20_Power_ON_Delay_Prescale 72
#define AHT20_Init_Delay_Prescale 7
#define AHT20_Trigger_Measurement_Prescale 58
#define DAC_BUFFER_SIZE 128
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t data[AHT20_DATA_SIZE];
uint8_t state[AHT20_STATE_SIZE];
volatile uint32_t adcValue = 0;
float temperature, humidity;
int flag = 0;
char rx_buf[22];
char i2c_buf[32];
char buf[5];
char handshakeCmd[] = "AT+DMOCONNECT\r\n";
char configCmd[] = "AT+DMOSETGROUP=0,144.1100,144.1100,0000,4,0000\r\n";
char scanCmd[] = "S+152.1250\r\n";
char setFilter[] = "AT+SETFILTER=0,0,0\r\n";
char expectedResponse[17] = "+DMOCONNECT:0\r\n\0";
char response[17]; // Buffer to store the response
int t_flag = 0;

const uint32_t sine_wave_array[32] = { 2047, 1648, 1264, 910, 600, 345, 156, 39,
		0, 39, 156, 345, 600, 910, 1264, 1648, 2048, 2447, 2831, 3185, 3495,
		3750, 3939, 4056, 4095, 4056, 3939, 3750, 3495, 3185, 2831, 2447 };

int input[] = { 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0 };
int i = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void DRA818V_Init(void);
static void AHT20_Init(void);
static void AHT20_triggerMeasurement(void);
static void AHT20_recieveData(void);
static int AHT20_measurementStatus(void);
static int AHT20_aprsConvert(void);
static void enterSleepMode(void);
static void enterTimedSleep(TIM_HandleTypeDef *htim);
static void transmitMessage(const char *msg);
static void configureTimer(TIM_HandleTypeDef *htim, uint32_t prescaler);

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
  MX_ADC_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_RTC_Init();
  MX_DAC_Init();
  MX_TIM6_Init();
  MX_USART8_UART_Init();
  MX_TIM14_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_ADC_Init();
	MX_I2C1_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */
	//transmitMessage("\n-----------------------------------\n\r");
	//transmitMessage("Checking Initial Voltage level...\n\r");
	//transmitMessage("-----------------------------------\n\n\r");
	HAL_ADC_Start_IT(&hadc);
	//transmitMessage("Voltage Level = ");
	//sprintf(buf,"%lu\n\r", adcValue);
	//HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 1000);

	//transmitMessage("\n-----------------------------------\n\r");
	//transmitMessage("BEGIN SENSOR FSM\n\r");
	//transmitMessage("-----------------------------------\n\n\r");

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	while (1) {
		switch (currentState) {
		case INIT:
			// transmitMessage("Current State: INIT\n\r");

			AHT20_Init();
			currentState = TRIGGER_MEASUREMENT;
			break;

		case TRIGGER_MEASUREMENT:

			// transmitMessage("Current State: TRIGGER_MEASURMENT\n\r");
			HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);

			AHT20_triggerMeasurement();

			currentState = WAIT_FOR_MEASUREMENT;
			break;

		case WAIT_FOR_MEASUREMENT:

			// transmitMessage("Current State: WAIT_FOR_MEASUREMENT\n\r");

			if (!AHT20_measurementStatus()) {
				currentState = READ_DATA;
			}
			break;

		case READ_DATA:

			//transmitMessage("Current State: READ_DATA\n\r");

			AHT20_recieveData();
			currentState = PROCESS_DATA;
			break;

		case PROCESS_DATA:

			//transmitMessage("Current State: PROCESS_DATA\n\r");

			if (!AHT20_aprsConvert()) {
				currentState = PAUSE;
			}
			break;

		case PAUSE:

			//transmitMessage("Current State: PAUSE\n\r");
			//transmitMessage("Button Press will change states.\n\r");

			// DRA818V_Init();

			HAL_TIM_Base_Start(&htim6);
			HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*) sine_wave_array,
					32, DAC_ALIGN_12B_R);

			//HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
			//HAL_TIM_Base_Start_IT(&htim14);

			while (1) {

				//transmitMessage("Begin APRS transmission\r\n");

				while (i < 18) {

					HAL_TIM_Base_Start_IT(&htim7);
					while (flag == 0) {
						asm("nop");
						//do nothing
					}

					i++;

					flag = 0;

				}

				HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
				i = 0;
				transmitMessage("DMA off\r\n");
				HAL_Delay(1000);
				HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1,
						(uint32_t*) sine_wave_array, 32, DAC_ALIGN_12B_R);
				/* transmitMessage("PTT off\r\n");
				 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
				 HAL_DAC_Stop(&hdac, DAC_CHANNEL_1);
				 HAL_Delay(1000);
				 transmitMessage("PTT on\r\n");
				 HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)sine_wave_array, 32, DAC_ALIGN_12B_R);


				 //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
				 HAL_Delay(500);
				 */
			}

			enterSleepMode();

			currentState = IDLE;

			break;

		case IDLE:

			transmitMessage("Current State: IDLE\n\r");
			MX_RTC_Init();
			transmitMessage("\n-----------------------------------\n\r");
			transmitMessage("\n***********  ATTENTION!  **********\n\r");
			transmitMessage("\nDevice in IDLE will enter standby!\n\r");
			transmitMessage("Button Press will change states.\n\r");
			transmitMessage("\n-----------------------------------\n\n\r");
			enterSleepMode();

			currentState = TRIGGER_MEASUREMENT;

			break;

		default:

			//transmitMessage("Current State: UNHANDLED\n\r");
			enterSleepMode();

			break;
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14
                              |RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_SYSCLK;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* USER CODE BEGIN 4 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {

}




//Timer interrupt callback function
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	if (htim->Instance == TIM7) {

		flag = 1;
	}

}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac) {

	if (input[i] == 0) {
		// Stop the timer
		//HAL_TIM_Base_Stop(&htim6);

		// Set the new prescaler value
		__HAL_TIM_SET_PRESCALER(&htim6, 341);

		// Restart the timer
		//HAL_TIM_Base_Start(&htim6);
	} else {
		// Stop the timer
		// HAL_TIM_Base_Stop(&htim6);

		// Set the new prescaler value
		__HAL_TIM_SET_PRESCALER(&htim6, 625);

		// Restart the timer
		// HAL_TIM_Base_Start(&htim6);
	}

	HAL_DAC_Start_DMA(hdac, DAC_CHANNEL_1, (uint32_t*) sine_wave_array, 32,
			DAC_ALIGN_12B_R);

}

//Initialize DRA818V

static void DRA818V_Init(void) {

	// Assuming rx_buf is an array with a fixed size

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);

	while (1) {
		memset(response, 0, sizeof(response));
		HAL_UART_Transmit(&huart8, (uint8_t*) handshakeCmd,
				sizeof(handshakeCmd), 1000);
		HAL_Delay(10);
		// Receive response

		HAL_UART_Receive(&huart8, (uint8_t*) response, sizeof(response), 1000);

		HAL_Delay(10);

		HAL_UART_Transmit(&huart2, (uint8_t*) response, sizeof(response), 1000);

		t_flag = strcmp(response, expectedResponse);
		if (!t_flag) {
			break;
		} else {

		}

	}

	memset(response, 0, sizeof(response));
	transmitMessage("Scanning 144.390....\r\n");
	HAL_UART_Transmit(&huart8, (uint8_t*) scanCmd, sizeof(scanCmd), 1000);
	HAL_Delay(10);
	HAL_UART_Receive(&huart8, (uint8_t*) response, sizeof(response), 1000);
	HAL_UART_Transmit(&huart2, (uint8_t*) response, sizeof(response), 1000);

	memset(response, 0, sizeof(response));
	// Send group setting command

	transmitMessage("AT+DMOSETGROUP=0,144.390,144.390,0000,4,0000\r\n");
	HAL_UART_Transmit(&huart8, (uint8_t*) configCmd, sizeof(configCmd), 1000);
	HAL_Delay(10);
	HAL_UART_Receive(&huart8, (uint8_t*) response, sizeof(response), 1000);
	// HAL_Delay(10);
	HAL_UART_Transmit(&huart2, (uint8_t*) response, sizeof(response), 1000);

	transmitMessage("AT+SETFILTER=0,0,0\r\n");
	HAL_UART_Transmit(&huart8, (uint8_t*) setFilter, sizeof(setFilter), 1000);
	HAL_Delay(10);
	HAL_UART_Receive(&huart8, (uint8_t*) response, sizeof(response), 1000);
	// HAL_Delay(10);
	HAL_UART_Transmit(&huart2, (uint8_t*) response, sizeof(response), 1000);

	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

}

// Initialize the AHT20 sensor
static void AHT20_Init(void) {

	uint8_t initCmd[] = { 0xBE, 0x08, 0x00 }; // Initialization command
	transmitMessage("Entering Sleep for 100ms\n\r");
	configureTimer(&htim3, AHT20_Power_ON_Delay_Prescale);
	enterTimedSleep(&htim3);

	HAL_I2C_Master_Transmit_IT(&hi2c1, AHT20_ADDR, initCmd, sizeof(initCmd));

	configureTimer(&htim3, AHT20_Init_Delay_Prescale);
	enterTimedSleep(&htim3);

	transmitMessage("INIT COMPLETE\n\r");
}

// Trigger a measurement on the AHT20 sensor
static void AHT20_triggerMeasurement(void) {

	uint8_t triggerCmd[] = { 0xAC, 0x33, 0x00 }; // Trigger measurement command

	HAL_I2C_Master_Transmit_IT(&hi2c1, AHT20_ADDR, triggerCmd,
			sizeof(triggerCmd));
}

// Receive data from the AHT20 sensortriangle wave generation stm32f09 hal
static void AHT20_recieveData(void) {
	HAL_I2C_Master_Receive_IT(&hi2c1, AHT20_ADDR, data, AHT20_DATA_SIZE);
}

// Check the measurement status of the AHT20 sensor
static int AHT20_measurementStatus(void) {

	transmitMessage("Entering Sleep for 80ms\n\r");

	configureTimer(&htim3, AHT20_Trigger_Measurement_Prescale);

	enterTimedSleep(&htim3);

	return 0;
}

// Convert the raw data from AHT20 into applicable units
static int AHT20_aprsConvert(void) {
	uint32_t hum = (data[1] << 12) | (data[2] << 4) | (data[3] >> 4);
	uint32_t temp = ((data[3] & 0x0F) << 16) | (data[4] << 8) | data[5];

	// Convert to actual values
	humidity = hum * 100.0 / 0x100000;
	temperature = temp * 200.0 / 0x100000 - 50;

	transmitMessage("\n-----------------------------------\n\r");
	transmitMessage("%Humidity = ");
	sprintf(i2c_buf, "%f\r\n", humidity); // Convert float to string and store in buffer
	HAL_UART_Transmit(&huart2, (uint8_t*) i2c_buf, strlen(i2c_buf), 1000);
	transmitMessage("Temperature C = ");
	sprintf(i2c_buf, "%f\r\n", temperature); // Convert float to string and store in buffer
	HAL_UART_Transmit(&huart2, (uint8_t*) i2c_buf, strlen(i2c_buf), 1000);
	transmitMessage("-----------------------------------\n\n\r");
	return 0;
}

void transmitMessage(const char *message) {
	HAL_UART_Transmit(&huart2, (uint8_t*) message, strlen(message), 1000);
}

void configureTimer(TIM_HandleTypeDef *htim, uint32_t prescaler) {
	__HAL_TIM_DISABLE(htim);
	__HAL_TIM_SET_PRESCALER(htim, prescaler);
	HAL_TIM_Base_Init(htim);
	__HAL_TIM_ENABLE(htim);
}

void enterSleepMode(void) {
	transmitMessage("Entering Sleep Mode\n\r");
	HAL_SuspendTick();
	HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
	HAL_ResumeTick();
	transmitMessage("Exiting Sleep Mode\n\r");
}

void enterTimedSleep(TIM_HandleTypeDef *htim) {
	HAL_TIM_Base_Start_IT(htim);

	HAL_SuspendTick();
	/* Request to enter SLEEP mode */
	HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
	/* Resume Tick interrupt if disabled prior to sleep mode entry */
	HAL_ResumeTick();

	HAL_TIM_Base_Stop_IT(htim);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if (hadc->Instance == ADC1) {

		adcValue = HAL_ADC_GetValue(hadc);
		transmitMessage("ADC Conversion Complete\n\r");

	}
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c1) {

	if (hi2c1->Instance == I2C1) {

	}

}

/* USER CODE BEGIN 4 */
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc) {
	RTC_AlarmTypeDef sAlarm;
	HAL_RTC_GetAlarm(hrtc, &sAlarm, RTC_ALARM_A, FORMAT_BIN);
	if (sAlarm.AlarmTime.Seconds > 58) {
		sAlarm.AlarmTime.Seconds = 10;
	} else {
		sAlarm.AlarmTime.Seconds = sAlarm.AlarmTime.Seconds + 10;
	}
	while (HAL_RTC_SetAlarm_IT(hrtc, &sAlarm, FORMAT_BIN) != HAL_OK) {
	}

	if (currentState == IDLE) {
		// Clear the Wake-up flag
		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
		transmitMessage("Entering Standby Mode\n\r");
		// Enter Standby mode
		HAL_PWR_EnterSTANDBYMode();
	}

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
