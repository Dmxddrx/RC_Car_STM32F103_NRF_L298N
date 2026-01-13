#include "general.h"

// Robot_Move: move in a specific direction with given speed
void General_Run(int16_t angle, uint8_t speed)
{
    // Normalize angle to 0-360
    while(angle < 0) angle += 360;
    while(angle >= 360) angle -= 360;

    switch(angle)
    {
        case 0:     // Forward
            Motor_MoveAll(MOTOR_FORWARD, speed);
            break;
        case 180:   // Backward
            Motor_MoveAll(MOTOR_BACKWARD, speed);
            break;
        case 270:   // Left
            Motor_SetDirection(MOTOR_LEFT_TOP, MOTOR_BACKWARD);
            Motor_SetDirection(MOTOR_LEFT_BOTTOM, MOTOR_FORWARD);
            Motor_SetDirection(MOTOR_RIGHT_TOP, MOTOR_FORWARD);
            Motor_SetDirection(MOTOR_RIGHT_BOTTOM, MOTOR_BACKWARD);

            Motor_SetSpeed(MOTOR_LEFT_TOP, speed);
            Motor_SetSpeed(MOTOR_LEFT_BOTTOM, speed);
            Motor_SetSpeed(MOTOR_RIGHT_TOP, speed);
            Motor_SetSpeed(MOTOR_RIGHT_BOTTOM, speed);
            break;
        case 90:    // Right
            Motor_SetDirection(MOTOR_LEFT_TOP, MOTOR_FORWARD);
            Motor_SetDirection(MOTOR_LEFT_BOTTOM, MOTOR_BACKWARD);
            Motor_SetDirection(MOTOR_RIGHT_TOP, MOTOR_BACKWARD);
            Motor_SetDirection(MOTOR_RIGHT_BOTTOM, MOTOR_FORWARD);

            Motor_SetSpeed(MOTOR_LEFT_TOP, speed);
            Motor_SetSpeed(MOTOR_LEFT_BOTTOM, speed);
            Motor_SetSpeed(MOTOR_RIGHT_TOP, speed);
            Motor_SetSpeed(MOTOR_RIGHT_BOTTOM, speed);
            break;

        case 45:
            Motor_SetDirection(MOTOR_LEFT_TOP, MOTOR_FORWARD);
            Motor_SetDirection(MOTOR_LEFT_BOTTOM, MOTOR_STOP);
            Motor_SetDirection(MOTOR_RIGHT_TOP, MOTOR_STOP);
            Motor_SetDirection(MOTOR_RIGHT_BOTTOM, MOTOR_FORWARD);

            Motor_SetSpeed(MOTOR_LEFT_TOP, speed);
            Motor_SetSpeed(MOTOR_RIGHT_BOTTOM, speed);
            Motor_SetSpeed(MOTOR_LEFT_BOTTOM, 0);
            Motor_SetSpeed(MOTOR_RIGHT_TOP, 0);
            break;
        case 225:
            Motor_SetDirection(MOTOR_LEFT_TOP, MOTOR_BACKWARD);
            Motor_SetDirection(MOTOR_LEFT_BOTTOM, MOTOR_STOP);
            Motor_SetDirection(MOTOR_RIGHT_TOP, MOTOR_STOP);
            Motor_SetDirection(MOTOR_RIGHT_BOTTOM, MOTOR_BACKWARD);

            Motor_SetSpeed(MOTOR_LEFT_TOP, speed);
            Motor_SetSpeed(MOTOR_RIGHT_BOTTOM, speed);
            Motor_SetSpeed(MOTOR_LEFT_BOTTOM, 0);
            Motor_SetSpeed(MOTOR_RIGHT_TOP, 0);
            break;
        case 135:
            Motor_SetDirection(MOTOR_LEFT_TOP, MOTOR_STOP);
            Motor_SetDirection(MOTOR_LEFT_BOTTOM, MOTOR_FORWARD);
            Motor_SetDirection(MOTOR_RIGHT_TOP, MOTOR_FORWARD);
            Motor_SetDirection(MOTOR_RIGHT_BOTTOM, MOTOR_STOP);

            Motor_SetSpeed(MOTOR_LEFT_BOTTOM, speed);
            Motor_SetSpeed(MOTOR_RIGHT_TOP, speed);
            Motor_SetSpeed(MOTOR_LEFT_TOP, 0);
            Motor_SetSpeed(MOTOR_RIGHT_BOTTOM, 0);
            break;
        case 315:   // -135deg equivalent
            Motor_SetDirection(MOTOR_LEFT_TOP, MOTOR_STOP);
            Motor_SetDirection(MOTOR_LEFT_BOTTOM, MOTOR_BACKWARD);
            Motor_SetDirection(MOTOR_RIGHT_TOP, MOTOR_BACKWARD);
            Motor_SetDirection(MOTOR_RIGHT_BOTTOM, MOTOR_STOP);

            Motor_SetSpeed(MOTOR_LEFT_BOTTOM, speed);
            Motor_SetSpeed(MOTOR_RIGHT_TOP, speed);
            Motor_SetSpeed(MOTOR_LEFT_TOP, 0);
            Motor_SetSpeed(MOTOR_RIGHT_BOTTOM, 0);
            break;

        default:
            // Unknown angle → stop
            Robot_Stop();
            break;
    }
}

// Stop all motors
void Robot_Stop(void)
{
    Motor_MoveAll(MOTOR_STOP, 0);
}
