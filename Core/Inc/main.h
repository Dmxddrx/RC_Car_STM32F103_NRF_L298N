/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#define IN1_Left_Pin GPIO_PIN_0
#define IN1_Left_GPIO_Port GPIOB

#define IN2_Left_Pin GPIO_PIN_1
#define IN2_Left_GPIO_Port GPIOB

#define IN3_Left_Pin GPIO_PIN_10
#define IN3_Left_GPIO_Port GPIOB

#define IN4_Left_Pin GPIO_PIN_11
#define IN4_Left_GPIO_Port GPIOB

#define IN1_Right_Pin GPIO_PIN_12
#define IN1_Right_GPIO_Port GPIOB

#define IN2_Right_Pin GPIO_PIN_13
#define IN2_Right_GPIO_Port GPIOB

#define IN3_Right_Pin GPIO_PIN_14
#define IN3_Right_GPIO_Port GPIOB

#define IN4_Right_Pin GPIO_PIN_15
#define IN4_Right_GPIO_Port GPIOB


/* USER CODE BEGIN Private defines */
// TIM handles
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

// SPI handle (if using NRF24L01 or other SPI device)
extern SPI_HandleTypeDef hspi1;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
