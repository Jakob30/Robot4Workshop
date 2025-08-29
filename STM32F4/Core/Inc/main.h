/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define LED_yellow_Pin GPIO_PIN_0
#define LED_yellow_GPIO_Port GPIOC
#define LED_green_Pin GPIO_PIN_1
#define LED_green_GPIO_Port GPIOC
#define MOT_EN_3_Pin GPIO_PIN_2
#define MOT_EN_3_GPIO_Port GPIOC
#define STEP_3_Pin GPIO_PIN_3
#define STEP_3_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define MOT_EN_2_Pin GPIO_PIN_6
#define MOT_EN_2_GPIO_Port GPIOA
#define STEP_2_Pin GPIO_PIN_7
#define STEP_2_GPIO_Port GPIOA
#define MOT_EN_5_Pin GPIO_PIN_4
#define MOT_EN_5_GPIO_Port GPIOC
#define LED_red_Pin GPIO_PIN_0
#define LED_red_GPIO_Port GPIOB
#define MOT_EN_4_Pin GPIO_PIN_1
#define MOT_EN_4_GPIO_Port GPIOB
#define STEP_4_Pin GPIO_PIN_2
#define STEP_4_GPIO_Port GPIOB
#define MOT_EN_1_Pin GPIO_PIN_12
#define MOT_EN_1_GPIO_Port GPIOB
#define STEP_1_Pin GPIO_PIN_13
#define STEP_1_GPIO_Port GPIOB
#define DIR_1_Pin GPIO_PIN_14
#define DIR_1_GPIO_Port GPIOB
#define STEP_5_Pin GPIO_PIN_8
#define STEP_5_GPIO_Port GPIOC
#define DIR_5_Pin GPIO_PIN_9
#define DIR_5_GPIO_Port GPIOC
#define DIR_2_Pin GPIO_PIN_8
#define DIR_2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define DIR_3_Pin GPIO_PIN_11
#define DIR_3_GPIO_Port GPIOC
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define DIR_4_Pin GPIO_PIN_4
#define DIR_4_GPIO_Port GPIOB
#define DIAG_3_Pin GPIO_PIN_9
#define DIAG_3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
