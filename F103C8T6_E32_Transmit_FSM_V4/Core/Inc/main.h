/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#define PIN_LED_Pin GPIO_PIN_13
#define PIN_LED_GPIO_Port GPIOC
#define PIN_POWER_DISPLAY_Pin GPIO_PIN_15
#define PIN_POWER_DISPLAY_GPIO_Port GPIOC
#define PIN_M0_Pin GPIO_PIN_5
#define PIN_M0_GPIO_Port GPIOA
#define PIN_M1_Pin GPIO_PIN_6
#define PIN_M1_GPIO_Port GPIOA
#define PIN_AUX_Pin GPIO_PIN_7
#define PIN_AUX_GPIO_Port GPIOA
#define PIN_AUX_EXTI_IRQn EXTI9_5_IRQn
#define PIN_BUTTON_10_Pin GPIO_PIN_10
#define PIN_BUTTON_10_GPIO_Port GPIOB
#define PIN_BUTTON_10_EXTI_IRQn EXTI15_10_IRQn
#define PIN_BUTTON_11_Pin GPIO_PIN_11
#define PIN_BUTTON_11_GPIO_Port GPIOB
#define PIN_BUTTON_11_EXTI_IRQn EXTI15_10_IRQn
#define PIN_BUTTON_12_Pin GPIO_PIN_12
#define PIN_BUTTON_12_GPIO_Port GPIOB
#define PIN_BUTTON_12_EXTI_IRQn EXTI15_10_IRQn
#define PIN_BUTTON_13_Pin GPIO_PIN_13
#define PIN_BUTTON_13_GPIO_Port GPIOB
#define PIN_BUTTON_13_EXTI_IRQn EXTI15_10_IRQn
#define PIN_BUTTON_14_Pin GPIO_PIN_14
#define PIN_BUTTON_14_GPIO_Port GPIOB
#define PIN_BUTTON_14_EXTI_IRQn EXTI15_10_IRQn
#define PIN_BUTTON_DEMOLITION_Pin GPIO_PIN_15
#define PIN_BUTTON_DEMOLITION_GPIO_Port GPIOB
#define PIN_BUTTON_DEMOLITION_EXTI_IRQn EXTI15_10_IRQn
#define PIN_BUTTON_ARM_Pin GPIO_PIN_8
#define PIN_BUTTON_ARM_GPIO_Port GPIOA
#define PIN_BUTTON_ARM_EXTI_IRQn EXTI9_5_IRQn
#define PIN_TX_Pin GPIO_PIN_9
#define PIN_TX_GPIO_Port GPIOA
#define PIN_RX_Pin GPIO_PIN_10
#define PIN_RX_GPIO_Port GPIOA
#define PIN_BUTTON_TEST_Pin GPIO_PIN_9
#define PIN_BUTTON_TEST_GPIO_Port GPIOB
#define PIN_BUTTON_TEST_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
