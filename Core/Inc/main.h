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
#define GREEN_Pin GPIO_PIN_13
#define GREEN_GPIO_Port GPIOC
#define WRITE_LED_Pin GPIO_PIN_14
#define WRITE_LED_GPIO_Port GPIOC
#define READ_LED_Pin GPIO_PIN_15
#define READ_LED_GPIO_Port GPIOC
#define AD0_Pin GPIO_PIN_0
#define AD0_GPIO_Port GPIOA
#define AD1_Pin GPIO_PIN_1
#define AD1_GPIO_Port GPIOA
#define AD2_Pin GPIO_PIN_2
#define AD2_GPIO_Port GPIOA
#define AD3_Pin GPIO_PIN_3
#define AD3_GPIO_Port GPIOA
#define AD4_Pin GPIO_PIN_4
#define AD4_GPIO_Port GPIOA
#define AD5_Pin GPIO_PIN_5
#define AD5_GPIO_Port GPIOA
#define AD6_Pin GPIO_PIN_6
#define AD6_GPIO_Port GPIOA
#define AD7_Pin GPIO_PIN_7
#define AD7_GPIO_Port GPIOA
#define WE_Pin GPIO_PIN_0
#define WE_GPIO_Port GPIOB
#define OE_Pin GPIO_PIN_1
#define OE_GPIO_Port GPIOB
#define CE_Pin GPIO_PIN_2
#define CE_GPIO_Port GPIOB
#define IN1_Pin GPIO_PIN_10
#define IN1_GPIO_Port GPIOB
#define IO4_Pin GPIO_PIN_12
#define IO4_GPIO_Port GPIOB
#define IO5_Pin GPIO_PIN_13
#define IO5_GPIO_Port GPIOB
#define IO6_Pin GPIO_PIN_14
#define IO6_GPIO_Port GPIOB
#define IO7_Pin GPIO_PIN_15
#define IO7_GPIO_Port GPIOB
#define AD8_Pin GPIO_PIN_8
#define AD8_GPIO_Port GPIOA
#define AD9_Pin GPIO_PIN_9
#define AD9_GPIO_Port GPIOA
#define AD10_Pin GPIO_PIN_10
#define AD10_GPIO_Port GPIOA
#define AD11_Pin GPIO_PIN_11
#define AD11_GPIO_Port GPIOA
#define AD12_Pin GPIO_PIN_12
#define AD12_GPIO_Port GPIOA
#define IN2_Pin GPIO_PIN_15
#define IN2_GPIO_Port GPIOA
#define IO0_Pin GPIO_PIN_4
#define IO0_GPIO_Port GPIOB
#define IO1_Pin GPIO_PIN_5
#define IO1_GPIO_Port GPIOB
#define IO2_Pin GPIO_PIN_6
#define IO2_GPIO_Port GPIOB
#define IO3_Pin GPIO_PIN_7
#define IO3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
