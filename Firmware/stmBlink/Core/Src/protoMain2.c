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
#include "usart.h"
#include "gpio.h"
#include <stdio.h>
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AHT20_ADDR 0x38 << 1 // AHT20 I2C address (shifted for HAL compatibility)
#define AHT20_DATA_SIZE 6
#define AHT20_STATE_SIZE 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t data[AHT20_DATA_SIZE];
uint8_t state[AHT20_STATE_SIZE];
volatile uint32_t adcValue = 0;
double temperature, humidity;
int flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void AHT20_Init(void);
static void AHT20_TriggerMeasurement(void);
static void AHT20_checkStatus(void);
static void AHT20_RecieveData(void);



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
  AHT20_Init();
  /* USER CODE BEGIN 2 */
 //AHT20_Init();
 // Initialize AHT20 sensor

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  HAL_SuspendTick();
 	  /* Request to enter SLEEP mfode */
  HAL_ADC_Start_IT(&hadc);
  HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
 	   /* Resume Tick interrupt if disabled prior to sleep mode entry */
  HAL_ResumeTick();


  char buf[5];
 	  	  char* msg1 = "Checking Power Supply Voltage Level...\n\r";
 	  	  char* msg2 = "Acceptable Voltage Level. V = ";
 	  	  char* msg3 = "Insufficient Voltage. Returning to Standby\n\r";
 	  	  char* msg4 = "Reading Sensor Data\n\r";

  HAL_UART_Transmit(&huart2, (uint8_t*)msg1, strlen(msg1), 1000);

  if(adcValue >= 500){


	  	 sprintf(buf,"%lu\n\r", adcValue);
	 	/* USER CODE END WHILE */
	 	 HAL_UART_Transmit(&huart2, (uint8_t*)msg2, strlen(msg2), 1000);
	 	 HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 1000); //blocking call for debug purposes
	 	 HAL_UART_Transmit(&huart2, (uint8_t*)msg4, strlen(msg4), 1000);

  	  	  while (1)
  	  	  {

  	  		AHT20_TriggerMeasurement();
  	  		AHT20_RecieveData();

    /* USER CODE BEGIN 3 */
  	  	  }

  	  }

  	  else {

  		HAL_UART_Transmit(&huart2, (uint8_t*)msg3, strlen(msg3), 1000);
  		  /* USER CODE END 3 */
  	  }

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
    {

    	 adcValue = HAL_ADC_GetValue(hadc);

    }
}

void AHT20_Init() {


    uint8_t initCmd[] = {0xBE, 0x08, 0x00}; // Initialization command
    uint8_t rstCmd[] = {0xBA, 0x08, 0x00}; // Initialization command

    HAL_I2C_Master_Transmit_IT(&hi2c1, AHT20_ADDR, rstCmd, sizeof(rstCmd));
    HAL_Delay(40);

    HAL_I2C_Master_Transmit_IT(&hi2c1, AHT20_ADDR, initCmd, sizeof(initCmd));
    HAL_Delay(40);



}

void AHT20_TriggerMeasurement() {

    uint8_t triggerCmd[] = {0xAC, 0x33, 0x00}; // Trigger measurement command
    HAL_I2C_Master_Transmit_IT(&hi2c1, AHT20_ADDR, triggerCmd, sizeof(triggerCmd));

}

//recursive status check function
void AHT20_checkStatus(){

	 	 	 uint8_t statusCmd[] = {0x71, 0x00, 0x00};
	 	 	 HAL_SuspendTick();
	    	 HAL_I2C_Master_Transmit_IT(&hi2c1, AHT20_ADDR, statusCmd, sizeof(statusCmd));
	    	 HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
	    	 HAL_I2C_Master_Receive_IT(&hi2c1, AHT20_ADDR, state, AHT20_STATE_SIZE);
	    	 HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
	    	    if((state[0] & 0x80) == 0){

	    	    	HAL_I2C_Master_Receive_IT(&hi2c1, AHT20_ADDR, state, AHT20_STATE_SIZE);
	    	    	HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);

	    	    }

	    	 HAL_ResumeTick();

}

void AHT20_RecieveData(){

	HAL_SuspendTick();
	HAL_I2C_Master_Receive_IT(&hi2c1, AHT20_ADDR, data, AHT20_DATA_SIZE);
	HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
	HAL_ResumeTick();


}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c1) {

    if (hi2c1->Instance == I2C1) {


    	}


}


void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c1) {
	char i2c_buf[32];

    if (hi2c1->Instance == I2C1) {


    	AHT20_checkStatus(); //recursive call to check status


    	   	  uint32_t hum = (data[1] << 12) | (data[2] << 4) | (data[3] >> 4);
    	      uint32_t temp = ((data[3] & 0x0F) << 16) | (data[4] << 8) | data[5];

    	      // Convert to actual values
    	      humidity = hum * 100.0 / 0x100000;
    	      temperature = temp * 200.0 / 0x100000 - 50;


    		  sprintf(i2c_buf, "%f\r\n", humidity); // Convert float to string and store in buffer
    		  HAL_UART_Transmit(&huart2, (uint8_t*)i2c_buf, strlen(i2c_buf), 1000);
    		  sprintf(i2c_buf, "%f\r\n", temperature); // Convert float to string and store in buffer
    		  HAL_UART_Transmit(&huart2, (uint8_t*)i2c_buf, strlen(i2c_buf), 1000);
    	// Check if data is valid
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
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
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
