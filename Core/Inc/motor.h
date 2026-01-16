#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f1xx_hal.h"

typedef enum {
    MOTOR_LEFT_TOP=0,
    MOTOR_LEFT_BOTTOM,
    MOTOR_RIGHT_TOP,
    MOTOR_RIGHT_BOTTOM
} Motor_TypeDef;

typedef enum {
    MOTOR_STOP=0,
    MOTOR_FORWARD,
    MOTOR_BACKWARD
} Motor_Direction;

void Motor_Init(void);
void Motor_SetSpeed(Motor_TypeDef motor,uint8_t speed);
void Motor_SetDirection(Motor_TypeDef motor,Motor_Direction dir);
void Motor_Stop(Motor_TypeDef motor);
void Motor_MoveAll(Motor_Direction dir,uint8_t speed);
void Motor_RunDirection(int16_t angle,uint8_t speed);

#endif
