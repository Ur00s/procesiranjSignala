/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

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
#define kol1_Pin GPIO_PIN_0
#define kol1_GPIO_Port GPIOA
#define kol2_Pin GPIO_PIN_1
#define kol2_GPIO_Port GPIOA
#define red1_Pin GPIO_PIN_3
#define red1_GPIO_Port GPIOA
#define red2_Pin GPIO_PIN_4
#define red2_GPIO_Port GPIOA
#define red3_Pin GPIO_PIN_5
#define red3_GPIO_Port GPIOA
#define red4_Pin GPIO_PIN_6
#define red4_GPIO_Port GPIOA
#define kol3_Pin GPIO_PIN_0
#define kol3_GPIO_Port GPIOB
#define a_Pin GPIO_PIN_1
#define a_GPIO_Port GPIOB
#define b_Pin GPIO_PIN_2
#define b_GPIO_Port GPIOB
#define dobar_Pin GPIO_PIN_8
#define dobar_GPIO_Port GPIOA
#define los_Pin GPIO_PIN_9
#define los_GPIO_Port GPIOA
#define c_Pin GPIO_PIN_3
#define c_GPIO_Port GPIOB
#define d_Pin GPIO_PIN_4
#define d_GPIO_Port GPIOB
#define e_Pin GPIO_PIN_5
#define e_GPIO_Port GPIOB
#define f_Pin GPIO_PIN_6
#define f_GPIO_Port GPIOB
#define g_Pin GPIO_PIN_7
#define g_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
