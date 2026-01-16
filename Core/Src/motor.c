#include "motor.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

// Motor struct
typedef struct {
    GPIO_TypeDef* IN1_Port;
    uint16_t IN1_Pin;
    GPIO_TypeDef* IN2_Port;
    uint16_t IN2_Pin;
    TIM_HandleTypeDef* htim;
    uint32_t Channel;
} Motor_PinDef;

Motor_PinDef motors[4] = {
    {GPIOB, GPIO_PIN_10, GPIOB, GPIO_PIN_11, &htim1, TIM_CHANNEL_2}, // LEFT_TOP
    {GPIOB, GPIO_PIN_0,  GPIOB, GPIO_PIN_1,  &htim1, TIM_CHANNEL_1}, // LEFT_BOTTOM
    {GPIOB, GPIO_PIN_12, GPIOB, GPIO_PIN_13, &htim1, TIM_CHANNEL_3}, // RIGHT_TOP
    {GPIOB, GPIO_PIN_14, GPIOB, GPIO_PIN_15, &htim1, TIM_CHANNEL_4}  // RIGHT_BOTTOM
};

static const int8_t motorDirSign[4] = {-1, -1, 1, 1};

void Motor_Init(void)
{
    for (int i=0;i<4;i++) {
        HAL_TIM_PWM_Start(motors[i].htim, motors[i].Channel);
        Motor_Stop((Motor_TypeDef)i);
    }
}

void Motor_SetSpeed(Motor_TypeDef motor, uint8_t speed)
{
    if(speed>100) speed=100;
    uint32_t pulse = (__HAL_TIM_GET_AUTORELOAD(motors[motor].htim)+1)*speed/100;
    __HAL_TIM_SET_COMPARE(motors[motor].htim, motors[motor].Channel, pulse);
}

void Motor_SetDirection(Motor_TypeDef motor, Motor_Direction dir)
{
    if(dir==MOTOR_STOP) {
        HAL_GPIO_WritePin(motors[motor].IN1_Port,motors[motor].IN1_Pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motors[motor].IN2_Port,motors[motor].IN2_Pin,GPIO_PIN_RESET);
        return;
    }

    Motor_Direction realDir = dir;
    if(motorDirSign[motor]<0)
        realDir = (dir==MOTOR_FORWARD)?MOTOR_BACKWARD:MOTOR_FORWARD;

    if(realDir==MOTOR_FORWARD) {
        HAL_GPIO_WritePin(motors[motor].IN1_Port,motors[motor].IN1_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(motors[motor].IN2_Port,motors[motor].IN2_Pin,GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(motors[motor].IN1_Port,motors[motor].IN1_Pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motors[motor].IN2_Port,motors[motor].IN2_Pin,GPIO_PIN_SET);
    }
}

void Motor_Stop(Motor_TypeDef motor)
{
    Motor_SetDirection(motor,MOTOR_STOP);
    __HAL_TIM_SET_COMPARE(motors[motor].htim,motors[motor].Channel,0);
}

// Move robot by angle (8 directions)
void Motor_RunDirection(int16_t angle, uint8_t speed)
{
    switch(angle) {
        case 0:     Motor_MoveAll(MOTOR_FORWARD,speed); break;
        case 45:    Motor_SetDirection(MOTOR_LEFT_TOP,MOTOR_FORWARD);
                    Motor_SetDirection(MOTOR_LEFT_BOTTOM,MOTOR_STOP);
                    Motor_SetDirection(MOTOR_RIGHT_TOP,MOTOR_STOP);
                    Motor_SetDirection(MOTOR_RIGHT_BOTTOM,MOTOR_FORWARD);
                    Motor_SetSpeed(MOTOR_LEFT_TOP,speed);
                    Motor_SetSpeed(MOTOR_RIGHT_BOTTOM,speed);
                    Motor_SetSpeed(MOTOR_LEFT_BOTTOM,0);
                    Motor_SetSpeed(MOTOR_RIGHT_TOP,0);
                    break;
        case 90:    Motor_SetDirection(MOTOR_LEFT_TOP,MOTOR_FORWARD);
                    Motor_SetDirection(MOTOR_LEFT_BOTTOM,MOTOR_BACKWARD);
                    Motor_SetDirection(MOTOR_RIGHT_TOP,MOTOR_BACKWARD);
                    Motor_SetDirection(MOTOR_RIGHT_BOTTOM,MOTOR_FORWARD);
                    Motor_SetSpeed(MOTOR_LEFT_TOP,speed);
                    Motor_SetSpeed(MOTOR_LEFT_BOTTOM,speed);
                    Motor_SetSpeed(MOTOR_RIGHT_TOP,speed);
                    Motor_SetSpeed(MOTOR_RIGHT_BOTTOM,speed);
                    break;
        case 135:   Motor_SetDirection(MOTOR_LEFT_TOP,MOTOR_STOP);
                    Motor_SetDirection(MOTOR_LEFT_BOTTOM,MOTOR_BACKWARD);
                    Motor_SetDirection(MOTOR_RIGHT_TOP,MOTOR_BACKWARD);
                    Motor_SetDirection(MOTOR_RIGHT_BOTTOM,MOTOR_STOP);
                    Motor_SetSpeed(MOTOR_LEFT_BOTTOM,speed);
                    Motor_SetSpeed(MOTOR_RIGHT_TOP,speed);
                    Motor_SetSpeed(MOTOR_LEFT_TOP,0);
                    Motor_SetSpeed(MOTOR_RIGHT_BOTTOM,0);
                    break;
        case 180:   Motor_MoveAll(MOTOR_BACKWARD,speed); break;
        case 225:   Motor_SetDirection(MOTOR_LEFT_TOP,MOTOR_BACKWARD);
                    Motor_SetDirection(MOTOR_LEFT_BOTTOM,MOTOR_STOP);
                    Motor_SetDirection(MOTOR_RIGHT_TOP,MOTOR_STOP);
                    Motor_SetDirection(MOTOR_RIGHT_BOTTOM,MOTOR_BACKWARD);
                    Motor_SetSpeed(MOTOR_LEFT_TOP,speed);
                    Motor_SetSpeed(MOTOR_RIGHT_BOTTOM,speed);
                    Motor_SetSpeed(MOTOR_LEFT_BOTTOM,0);
                    Motor_SetSpeed(MOTOR_RIGHT_TOP,0);
                    break;
        case 270:   Motor_SetDirection(MOTOR_LEFT_TOP,MOTOR_BACKWARD);
                    Motor_SetDirection(MOTOR_LEFT_BOTTOM,MOTOR_FORWARD);
                    Motor_SetDirection(MOTOR_RIGHT_TOP,MOTOR_FORWARD);
                    Motor_SetDirection(MOTOR_RIGHT_BOTTOM,MOTOR_BACKWARD);
                    Motor_SetSpeed(MOTOR_LEFT_TOP,speed);
                    Motor_SetSpeed(MOTOR_LEFT_BOTTOM,speed);
                    Motor_SetSpeed(MOTOR_RIGHT_TOP,speed);
                    Motor_SetSpeed(MOTOR_RIGHT_BOTTOM,speed);
                    break;
        case 315:   Motor_SetDirection(MOTOR_LEFT_TOP,MOTOR_STOP);
                    Motor_SetDirection(MOTOR_LEFT_BOTTOM,MOTOR_FORWARD);
                    Motor_SetDirection(MOTOR_RIGHT_TOP,MOTOR_FORWARD);
                    Motor_SetDirection(MOTOR_RIGHT_BOTTOM,MOTOR_STOP);
                    Motor_SetSpeed(MOTOR_LEFT_BOTTOM,speed);
                    Motor_SetSpeed(MOTOR_RIGHT_TOP,speed);
                    Motor_SetSpeed(MOTOR_LEFT_TOP,0);
                    Motor_SetSpeed(MOTOR_RIGHT_BOTTOM,0);
                    break;
        default:    Motor_MoveAll(MOTOR_STOP,0); break;
    }
}

void Motor_MoveAll(Motor_Direction dir,uint8_t speed)
{
    for(int i=0;i<4;i++) {
        Motor_SetDirection((Motor_TypeDef)i,dir);
        Motor_SetSpeed((Motor_TypeDef)i,speed);
    }
}
