#include "general.h"
#include "nrf24_rx.h"
#include "motor.h"
#include "led.h"

extern LED_HandleTypeDef statusLED;

// Direction mapping for STM32 motors
static const int16_t directionAngles[9] = {
    -1, 0, 45, 90, 225, 180, 135, 270, 315
};

void General_Run(void)
{
    ControlPacket pkt;

    // ---------- NRF not responding ----------
    if (!NRF24_DataAvailable()) {
    	 Motor_MoveAll(MOTOR_STOP, 0);
    	 statusLED.state = LED_STATE_BLINK_SLOW;  // slow blink
		 LED_Update(&statusLED);
        return;
    }

    // ---------- Read packet ----------
    if (!NRF24_Read(&pkt)) {
    	 Motor_MoveAll(MOTOR_STOP, 0);
    	 statusLED.state = LED_STATE_BLINK_FAST;  // indicate error reading
		 LED_Update(&statusLED);
        return;
    }

    // Packet received → fast blink
    statusLED.state = LED_STATE_STEADY;
    LED_Update(&statusLED);

    // ---------- Determine angle ----------
    int16_t angle = directionAngles[pkt.direction];
    if (angle < 0) {
    	 Motor_MoveAll(MOTOR_STOP, 0);
         statusLED.state = LED_STATE_STEADY;  // invalid direction
         LED_Update(&statusLED);
        return;
    }

    // ---------- Run motor ----------
    uint8_t speed = (pkt.speed * 100) / 70; // scale to 0-100%
    Motor_RunDirection(angle, speed);
}
