#include "motor.h"

// --- PWM timer handles (defined in main.c) ---
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

// --- Motor pin definition ---
typedef struct {
    GPIO_TypeDef* IN1_Port;
    uint16_t      IN1_Pin;
    GPIO_TypeDef* IN2_Port;
    uint16_t      IN2_Pin;
    TIM_HandleTypeDef* htim;
    uint32_t      Channel; // PWM Channel for EN pin
} Motor_PinDef;

// Motors configuration
Motor_PinDef motors[4] = {
    // LEFT TOP
    {GPIOB, GPIO_PIN_10, GPIOB, GPIO_PIN_11, &htim1, TIM_CHANNEL_2},   // ENB - A11 (TIM1 CH2)
    // LEFT BOTTOM
    {GPIOB, GPIO_PIN_0,  GPIOB, GPIO_PIN_1,  &htim1, TIM_CHANNEL_1},   // ENA - A10 (TIM1 CH1)
    // RIGHT TOP
    {GPIOB, GPIO_PIN_12, GPIOB, GPIO_PIN_13, &htim1, TIM_CHANNEL_3},   // ENA - A8  (TIM1 CH3)
    // RIGHT BOTTOM
    {GPIOB, GPIO_PIN_14, GPIOB, GPIO_PIN_15, &htim1, TIM_CHANNEL_4}    // ENB - A9  (TIM1 CH4)
};

// --- Initialize all motors ---
void Motor_Init(void)
{
    for(int i=0;i<4;i++)
    {
        // Start PWM
        HAL_TIM_PWM_Start(motors[i].htim, motors[i].Channel);

        // Stop motors
        Motor_Stop((Motor_TypeDef)i);
    }
}

// --- Set motor speed (0-100%) ---
void Motor_SetSpeed(Motor_TypeDef motor, uint8_t speed)
{
    if(speed > 100) speed = 100;
    uint32_t pulse = (__HAL_TIM_GET_AUTORELOAD(motors[motor].htim) + 1) * speed / 100;
    __HAL_TIM_SET_COMPARE(motors[motor].htim, motors[motor].Channel, pulse);
}

// --- Set motor direction ---
void Motor_SetDirection(Motor_TypeDef motor, Motor_Dir_
