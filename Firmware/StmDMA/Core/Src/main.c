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
#include "dac.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdlib.h>
#include <stdint.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BIT_STREAM_LEN 128
#define STEPS 32
#define PERIOD 1200
#define SAMPLE_FREQ (STEPS*PERIOD)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
const uint32_t sine_wave_array[32] = {2047, 1648, 1264, 910, 600,  345,
                   156, 39,  0,  39,  156,  345,
                   600, 910, 1264, 1648, 2048, 2447,
                   2831, 3185, 3495, 3750, 3939, 4056,
                   4095, 4056, 3939, 3750, 3495, 3185,
                   2831, 2447};


int input_1[] = {0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1
			  ,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1
			  ,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1
};

int input[] ={0,1,1,1,1,1,1,0,0,0,0,1,0,1,0,1,0,1,0,1,0,0,0,1,0,1,1,0,0,1,0,1,0,0,0,1,
		0,1,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,1,1,1,0,1,1,0,1,0,0,1,0,1,
		1,1,1,0,0,1,0,1,0,0,0,0,0,1,0,0,0,1,0,1,0,1,0,0,0,1,1,0,0,1,0,0,0,0,0,0,1,0,
		1,0,0,0,0,1,1,0,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,1,0,0,0,1,0,0,1,0,1,0,1,0,0,1,
		1,0,0,0,1,1,0,1,1,0,0,0,1,1,0,1,1,0,1,1,1,1,0,1,1,0,0,0,0,0,0,1,0,0,1,1,1,0,
		1,0,1,0,1,1,1,1,0,1,1,0,0,1,0,0,1,1,1,0,0,0,1,1,0,1,1,0,0,0,1,0,0,1,1,0,1,1,
		1,1,0,0,0,1,0,1,0,0,1,1,0,0,0,1,1,1,1,1,1,0
};
int i = 0;
uint32_t errorValue;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint32_t* audioCPFSK(int* bitstream, int bitstreamLength, int samplingFreq);

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
  MX_DAC_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */


  uint32_t* data = audioCPFSK(input, BIT_STREAM_LEN, SAMPLE_FREQ);


  HAL_TIM_Base_Start(&htim6);
  if(HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)data, BIT_STREAM_LEN*STEPS, DAC_ALIGN_12B_R)== HAL_OK){
  }
  else {
	  errorValue = HAL_DAC_GetError(&hdac);
  }

  while(1){




	  /* if(input[i] == 0){
		  // Stop the timer
		     HAL_TIM_Base_Stop(&htim6);

		     // Set the new prescaler value
		     __HAL_TIM_SET_PRESCALER(&htim6, 625);

		     // Restart the timer
		     HAL_TIM_Base_Start(&htim6);
	  }
	  else {
		  // Stop the timer
		  		     HAL_TIM_Base_Stop(&htim6);

		  		     // Set the new prescaler value
		  		     __HAL_TIM_SET_PRESCALER(&htim6, 341);

		  		     // Restart the timer
		  		     HAL_TIM_Base_Start(&htim6);
	  }


	  if(i<8){
		  i++;
	  }
	  else{
	  i = 0;
	  }

	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_value);
	if (dac_value < 4095) {
	dac_value++;
	} else {
	dac_value=0;
	}
	HAL_Delay(1);
*/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
uint32_t* audioCPFSK(int* bitstream, int bitstreamLength, int samplingFreq) {
    int fCenter = 1700;
    int fDelta = 500;
    int bitrate = 1200;
    int steps = samplingFreq / bitrate;
    int totalSamples = bitstreamLength * steps;
    //uint32_t dummy[16384] = {0};

    size_t size = (totalSamples * sizeof(uint32_t));

    uint32_t* result = (uint32_t*)malloc(size);
    // Allocate memory for the output array
    //float* y = (float*)malloc(totalSamples * sizeof(float));



    if(result == NULL){
    	return NULL;
    }

    //Convert bitstream to NRZ format
    for (int i = 0; i < bitstreamLength; i++) {
        bitstream[i] = bitstream[i] * 2 - 1;
    }


    result[0] = 0;
    float m = 0;

    for (int i = 1; i < totalSamples; i++) {
        // Interpolate the bitstream to steps points per bit
        int index = (i + steps - 1) / steps;
        int indexPrev = (i - 1 + steps - 1) / steps;

        // Integration of the bitstream with trapezoidal rule
        m += (bitstream[indexPrev - 1] + bitstream[index - 1]) / 2.0;

        // FM Modulation
        result[i] = (uint32_t)(2047.5*(1+(cos(2 * M_PI * i * (fCenter / (double)samplingFreq) - 2 * M_PI * m * (fDelta / (double)samplingFreq)))));

        //result[i] = ((result[i]+1) * 2047);

    }


    return result;
}
    // Set the output length
   // *outputLength = totalSamples;




void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac) {


	//HAL_DAC_Start_DMA(hdac, DAC_CHANNEL_1, (uint32_t*) sine_wave_array, 32,DAC_ALIGN_12B_R);

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
