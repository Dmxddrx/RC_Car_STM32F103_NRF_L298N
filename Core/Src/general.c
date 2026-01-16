#include "general.h"
#include "nrf24_rx.h"
#include "motor.h"
#include "led.h"

// Direction mapping for STM32 motors
static const int16_t directionAngles[9] = {
    -1, 0, 45, 90, 225, 180, 135, 270, 315
};

void General_Run(void)
{
    ControlPacket pkt;

    if (!NRF24_DataAvailable()) {
    	 Motor_MoveAll(MOTOR_STOP, 0);
        return;
    }

    if (!NRF24_Read(&pkt)) {
    	 Motor_MoveAll(MOTOR_STOP, 0);
        return;
    }

    int16_t angle = directionAngles[pkt.direction];
    if (angle < 0) {
    	 Motor_MoveAll(MOTOR_STOP, 0);
        return;
    }

    uint8_t speed = (pkt.speed * 100) / 70; // scale to 0-100%
    Motor_RunDirection(angle, speed);
}
