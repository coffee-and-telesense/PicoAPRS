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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h>
#include <string.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes*/
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    INIT,
    TRIGGER_MEASUREMENT,
    WAIT_FOR_MEASUREMENT,
    READ_DATA,
    PROCESS_DATA,
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

char i2c_buf[32];
char buf[5];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void AHT20_Init(void);
static void AHT20_triggerMeasurement(void);
static void AHT20_recieveData(void);
static int AHT20_measurementStatus(void);
static int AHT20_aprsConvert(void);
static void enterSleepMode(void);
static void enterTimedSleep(TIM_HandleTypeDef* htim);
static void transmitMessage(const char* msg);
static void configureTimer(TIM_HandleTypeDef* htim, uint32_t prescaler);

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
  MX_USART2_UART_Init();
  MX_ADC_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */
  transmitMessage("\n-----------------------------------\n\r");
  transmitMessage("Checking Initial Voltage level...\n\r");
  transmitMessage("-----------------------------------\n\n\r");
  HAL_ADC_Start_IT(&hadc);
  transmitMessage("Voltage Level = ");
  sprintf(buf,"%lu\n\r", adcValue);
  HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 1000);

  transmitMessage("\n-----------------------------------\n\r");
  transmitMessage("BEGIN SENSOR FSM\n\r");
  transmitMessage("-----------------------------------\n\n\r");

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	          switch (currentState) {
	              case INIT:
	            	  transmitMessage("Current State: INIT\n\r");

	            	  AHT20_Init();
	                  currentState = TRIGGER_MEASUREMENT;
	                  break;

	              case TRIGGER_MEASUREMENT:

	            	  transmitMessage("Current State: TRIGGER_MEASURMENT\n\r");


	                  AHT20_triggerMeasurement();

	                  currentState = WAIT_FOR_MEASUREMENT;
	                  break;

	              case WAIT_FOR_MEASUREMENT:

	            	  transmitMessage("Current State: WAIT_FOR_MEASUREMENT\n\r");

	                  if (!AHT20_measurementStatus()) {
	                      currentState = READ_DATA;
	                  }
	                  break;

	              case READ_DATA:

	            	  transmitMessage("Current State: READ_DATA\n\r");

	                  AHT20_recieveData();
	                  currentState = PROCESS_DATA;
	                  break;

	              case PROCESS_DATA:

	            	  transmitMessage("Current State: PROCESS_DATA\n\r");

	                  if (!AHT20_aprsConvert()) {
	                      currentState = IDLE;
	                  }
	                  break;


	              case IDLE:

	            	  transmitMessage("Current State: IDLE\n\r");
	            	  enterSleepMode();

	            	  currentState = TRIGGER_MEASUREMENT;

	                  break;

	              default:

	            	  transmitMessage("Current State: UNHANDLED\n\r");
	            	  enterSleepMode();

	                  break;
	          }

    /* USER CODE END WHILE */
  }
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */


// Initialize the AHT20 sensor
static void AHT20_Init(void) {

	uint8_t initCmd[] = {0xBE, 0x08, 0x00}; // Initialization command
	transmitMessage("Entering Sleep for 100ms\n\r");
	configureTimer(&htim3, AHT20_Power_ON_Delay_Prescale);
	enterTimedSleep(&htim3);

	HAL_I2C_Master_Transmit_IT(&hi2c1, AHT20_ADDR, initCmd, sizeof(initCmd));

    configureTimer(&htim3,AHT20_Init_Delay_Prescale);
    enterTimedSleep(&htim3);

    transmitMessage("INIT COMPLETE\n\r");
}

// Trigger a measurement on the AHT20 sensor
static void AHT20_triggerMeasurement(void) {

		uint8_t triggerCmd[] = {0xAC, 0x33, 0x00}; // Trigger measurement command

	 	  HAL_I2C_Master_Transmit_IT(&hi2c1, AHT20_ADDR, triggerCmd, sizeof(triggerCmd));
}

// Receive data from the AHT20 sensor
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
	  HAL_UART_Transmit(&huart2, (uint8_t*)i2c_buf, strlen(i2c_buf), 1000);
	  transmitMessage("Temperature C = ");
	  sprintf(i2c_buf, "%f\r\n", temperature); // Convert float to string and store in buffer
	  HAL_UART_Transmit(&huart2, (uint8_t*)i2c_buf, strlen(i2c_buf), 1000);
	  transmitMessage("-----------------------------------\n\n\r");
    return 0;
}


void transmitMessage(const char* message) {
  HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), 1000);
}

void configureTimer(TIM_HandleTypeDef* htim, uint32_t prescaler) {
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

void enterTimedSleep(TIM_HandleTypeDef* htim) {
    HAL_TIM_Base_Start_IT(htim);

    HAL_SuspendTick();
    /* Request to enter SLEEP mode */
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
    /* Resume Tick interrupt if disabled prior to sleep mode entry */
    HAL_ResumeTick();

    HAL_TIM_Base_Stop_IT(htim);
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
    {

    	 adcValue = HAL_ADC_GetValue(hadc);
    	 transmitMessage("ADC Conversion Complete\n\r");


    }
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c1) {

    if (hi2c1->Instance == I2C1) {


    	}


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_SYSCLK;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
