#ifndef __MAIN_H
#define __MAIN_H
// Header guard:

#ifdef __cplusplus
extern "C" {
#endif

/* ======================= Includes ======================= */

// Main STM32 HAL header:
// - Core definitions
// - Peripheral structures
// - HAL APIs
#include "stm32f1xx_hal.h"

// Interrupt handler declarations:
// - EXTI
// - SysTick
// - Peripheral IRQ prototypes
#include "stm32f1xx_it.h"

/* ======================= HAL Hooks ======================= */

// Called by HAL after timer initialization
// Used to configure GPIO alternate functions for TIM PWM pins
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);


/* ======================= Exported Functions ======================= */

// Central error handler for unrecoverable faults
void Error_Handler(void);

/* ======================= GPIO Pin Definitions ======================= */
// Motor driver direction control pins

// -------- Left Motor Group --------
#define IN1_Left_Pin GPIO_PIN_0
#define IN1_Left_GPIO_Port GPIOB

#define IN2_Left_Pin GPIO_PIN_1
#define IN2_Left_GPIO_Port GPIOB

#define IN3_Left_Pin GPIO_PIN_10
#define IN3_Left_GPIO_Port GPIOB

#define IN4_Left_Pin GPIO_PIN_11
#define IN4_Left_GPIO_Port GPIOB

// -------- Right Motor Group --------
#define IN1_Right_Pin GPIO_PIN_12
#define IN1_Right_GPIO_Port GPIOB

#define IN2_Right_Pin GPIO_PIN_13
#define IN2_Right_GPIO_Port GPIOB

#define IN3_Right_Pin GPIO_PIN_14
#define IN3_Right_GPIO_Port GPIOB

#define IN4_Right_Pin GPIO_PIN_15
#define IN4_Right_GPIO_Port GPIOB


// TIM handles
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

// SPI handle (For NRF24L01)
extern SPI_HandleTypeDef hspi1;

#ifdef __cplusplus
}
#endif

#endif
