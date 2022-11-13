/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "string.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void proveriBroj(char *ispravanBroj, char **unetiBroj)
{

	if(strlen(*unetiBroj) == 2)
	{
		if(strcmp(ispravanBroj,*unetiBroj) == 0)
		{
			HAL_GPIO_WritePin(GPIOA, dobar_Pin, 0);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOA, los_Pin, 0);
		}
	}

	else if (strlen(*unetiBroj) > 2)
	{
		strcpy(*unetiBroj,"");
		GPIOB->ODR = 0xFE;
		HAL_GPIO_WritePin(GPIOA, dobar_Pin, 1);
		HAL_GPIO_WritePin(GPIOA, los_Pin, 1);
		return;
	}
	else
	{
		return;
	}
}



void proveri_unos (char *ispravanBroj, char **unetiBroj)
{
	/* Make ROW 1 HIGH and all other ROWs LOW */
	HAL_GPIO_WritePin (GPIOA, red1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin (GPIOA, red2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin (GPIOA, red3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin (GPIOA, red4_Pin, GPIO_PIN_RESET);


	if ((HAL_GPIO_ReadPin (GPIOA, kol1_Pin)))   // if the Col 1 is low
	{
		//1
		GPIOB->ODR = ~0x0C;
		while(HAL_GPIO_ReadPin (GPIOA, kol1_Pin));
		strcat(*unetiBroj,"1");
		proveriBroj(ispravanBroj, unetiBroj);
		return;
	}

	if ((HAL_GPIO_ReadPin (GPIOA, kol2_Pin)))   // if the Col 2 is low
	{
		//2
		HAL_GPIO_WritePin (GPIOA, red1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin (GPIOA, red2_Pin, GPIO_PIN_SET);
		GPIOB->ODR = ~0xB6;
		while(HAL_GPIO_ReadPin (GPIOA, kol2_Pin));
		strcat(*unetiBroj,"2");
		proveriBroj(ispravanBroj, unetiBroj);
		HAL_GPIO_WritePin (GPIOA, red1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin (GPIOA, red2_Pin, GPIO_PIN_RESET);
		return;
	}

	if ((HAL_GPIO_ReadPin (GPIOB, kol3_Pin)))   // if the Col 3 is low
	{
		//3
		GPIOB->ODR = ~0x9E;
		while(HAL_GPIO_ReadPin (GPIOB, kol3_Pin));
		strcat(*unetiBroj,"3");
		proveriBroj(ispravanBroj, unetiBroj);
		return;
	}



	/* Make ROW 2 LOW and all other ROWs HIGH */
	HAL_GPIO_WritePin (GPIOA, red1_Pin, GPIO_PIN_RESET);  //Pull the R1 low
	HAL_GPIO_WritePin (GPIOA, red2_Pin, GPIO_PIN_SET);  // Pull the R2 High
	HAL_GPIO_WritePin (GPIOA, red3_Pin, GPIO_PIN_RESET);  // Pull the R3 High
	HAL_GPIO_WritePin (GPIOA, red4_Pin, GPIO_PIN_RESET);  // Pull the R4 High



	if ((HAL_GPIO_ReadPin (GPIOA, kol1_Pin)))   // if the Col 1 is low
	{
		//4
		GPIOB->ODR = ~0xCC;
		while(HAL_GPIO_ReadPin (GPIOA, kol1_Pin));
		strcat(*unetiBroj,"4");
		proveriBroj(ispravanBroj, unetiBroj);
		return;

	}

	if ((HAL_GPIO_ReadPin (GPIOA, kol2_Pin)))   // if the Col 2 is low
	{
		GPIOB->ODR = ~0xDA;
		while(HAL_GPIO_ReadPin (GPIOA, kol2_Pin));
		strcat(*unetiBroj,"5");
		proveriBroj(ispravanBroj, unetiBroj);
		return;
	}

	if ((HAL_GPIO_ReadPin (GPIOB, kol3_Pin)))   // if the Col 3 is low
	{
		//6
		GPIOB->ODR = ~0xFA;
		while(HAL_GPIO_ReadPin (GPIOB, kol3_Pin));
		strcat(*unetiBroj,"6");
		proveriBroj(ispravanBroj, unetiBroj);
		return;
	}



	/* Make ROW 3 LOW and all other ROWs HIGH */
	HAL_GPIO_WritePin (GPIOA, red1_Pin, GPIO_PIN_RESET);  //Pull the R1 low
	HAL_GPIO_WritePin (GPIOA, red2_Pin, GPIO_PIN_RESET);  // Pull the R2 High
	HAL_GPIO_WritePin (GPIOA, red3_Pin, GPIO_PIN_SET);  // Pull the R3 High
	HAL_GPIO_WritePin (GPIOA, red4_Pin, GPIO_PIN_RESET);  // Pull the R4 High

	if ((HAL_GPIO_ReadPin (GPIOA, kol1_Pin)))   // if the Col 1 is low
	{
		//7
		GPIOB->ODR = ~0x0E;
		while(HAL_GPIO_ReadPin (GPIOA, kol1_Pin));
		strcat(*unetiBroj,"7");
		proveriBroj(ispravanBroj, unetiBroj);
		return;
	}

	if ((HAL_GPIO_ReadPin (GPIOA, kol2_Pin)))   // if the Col 2 is low
	{
		//8
		GPIOB->ODR = ~0xFD;
		while(HAL_GPIO_ReadPin (GPIOA, kol2_Pin));
		strcat(*unetiBroj,"8");
		proveriBroj(ispravanBroj, unetiBroj);
		return;
	}

	if ((HAL_GPIO_ReadPin (GPIOB, kol3_Pin)))   // if the Col 3 is low
	{
		//9
		GPIOB->ODR = ~0xDE;
		while(HAL_GPIO_ReadPin (GPIOB, kol3_Pin));
		strcat(*unetiBroj,"9");
		proveriBroj(ispravanBroj, unetiBroj);
		return;
	}




	/* Make ROW 4 LOW and all other ROWs HIGH */
	HAL_GPIO_WritePin (GPIOA, red1_Pin, GPIO_PIN_RESET);  //Pull the R1 low
	HAL_GPIO_WritePin (GPIOA, red2_Pin, GPIO_PIN_RESET);  // Pull the R2 High
	HAL_GPIO_WritePin (GPIOA, red3_Pin, GPIO_PIN_RESET);  // Pull the R3 High
	HAL_GPIO_WritePin (GPIOA, red4_Pin, GPIO_PIN_SET);  // Pull the R4 High

	if ((HAL_GPIO_ReadPin (GPIOA, kol1_Pin)))   // if the Col 1 is low
	{
		while(HAL_GPIO_ReadPin (GPIOA, kol1_Pin));
		return;
	}

	if ((HAL_GPIO_ReadPin (GPIOA, kol2_Pin)))   // if the Col 2 is low
	{
		while(HAL_GPIO_ReadPin (GPIOA, kol2_Pin));
		return;
	}

	if ((HAL_GPIO_ReadPin (GPIOB, kol3_Pin)))   // if the Col 3 is low
	{

		while(HAL_GPIO_ReadPin (GPIOB, kol3_Pin));
		return;
	}

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
  /* USER CODE BEGIN 2 */
  char ispravanBroj[2] = "73";
  char *unetiBroj;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  proveri_unos(ispravanBroj, &unetiBroj);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, dobar_Pin|los_Pin, GPIO_PIN_SET);

  HAL_GPIO_WritePin(GPIOA, red1_Pin|red2_Pin|red3_Pin|red4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, a_Pin|b_Pin|c_Pin|d_Pin
                          |e_Pin|f_Pin|g_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : kol1_Pin kol2_Pin */
  GPIO_InitStruct.Pin = kol1_Pin|kol2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : red1_Pin red2_Pin red3_Pin red4_Pin
                           dobar_Pin los_Pin */
  GPIO_InitStruct.Pin = red1_Pin|red2_Pin|red3_Pin|red4_Pin
                          |dobar_Pin|los_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : kol3_Pin */
  GPIO_InitStruct.Pin = kol3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(kol3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : a_Pin b_Pin c_Pin d_Pin
                           e_Pin f_Pin g_Pin */
  GPIO_InitStruct.Pin = a_Pin|b_Pin|c_Pin|d_Pin
                          |e_Pin|f_Pin|g_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
