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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define touch1_Pin GPIO_PIN_1
#define touch1_GPIO_Port GPIOA
#define touch1_EXTI_IRQn EXTI1_IRQn
#define touch2_Pin GPIO_PIN_2
#define touch2_GPIO_Port GPIOA
#define touch2_EXTI_IRQn EXTI2_IRQn
#define touch3_Pin GPIO_PIN_3
#define touch3_GPIO_Port GPIOA
#define touch3_EXTI_IRQn EXTI3_IRQn
#define touch4_Pin GPIO_PIN_4
#define touch4_GPIO_Port GPIOA
#define touch4_EXTI_IRQn EXTI4_IRQn
#define touch5_Pin GPIO_PIN_5
#define touch5_GPIO_Port GPIOA
#define touch5_EXTI_IRQn EXTI9_5_IRQn
#define touch6_Pin GPIO_PIN_6
#define touch6_GPIO_Port GPIOA
#define touch6_EXTI_IRQn EXTI9_5_IRQn
#define LED6_Pin GPIO_PIN_12
#define LED6_GPIO_Port GPIOB
#define LED5_Pin GPIO_PIN_13
#define LED5_GPIO_Port GPIOB
#define LED4_Pin GPIO_PIN_14
#define LED4_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_15
#define LED3_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_8
#define LED2_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_9
#define LED1_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
