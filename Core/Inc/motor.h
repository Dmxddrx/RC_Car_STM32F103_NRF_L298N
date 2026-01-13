#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f1xx_hal.h"

// Motor identifiers
typedef enum {
    MOTOR_LEFT_TOP = 0,
    MOTOR_LEFT_BOTTOM,
    MOTOR_RIGHT_TOP,
    MOTOR_RIGHT_BOTTOM
} Motor_TypeDef;

// Motor direction
typedef enum {
    MOTOR_STOP = 0,
    MOTOR_FORWARD,
    MOTOR_BACKWARD
} Motor_Direction;

// Initialize all motors
void Motor_Init(void);

// Set speed (0-100%) for a motor
void Motor_SetSpeed(Motor_TypeDef motor, uint8_t speed);

// Set direction of a motor
void Motor_SetDirection(Motor_TypeDef motor, Motor_Direction dir);

// Stop a motor
void Motor_Stop(Motor_TypeDef motor);

// Move all motors in one direction
void Motor_MoveAll(Motor_Direction dir, uint8_t speed);

#endif /* __MOTOR_H */
