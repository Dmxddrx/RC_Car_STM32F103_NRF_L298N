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
    LED_State nextLedState = LED_STATE_OFF;

    // ---------- NRF not connected ----------
    if (!NRF24_IsConnected())
    {
        Motor_MoveAll(MOTOR_STOP, 0);
        nextLedState = LED_STATE_BLINK_SLOW;
    }

    // ---------- NRF connected but idle ----------
    else if (!NRF24_DataAvailable())
    {
        Motor_MoveAll(MOTOR_STOP, 0);
        nextLedState = LED_STATE_HEARTBEAT;
    }

    // ---------- RX error ----------
    else if (!NRF24_Read(&pkt))
    {
        Motor_MoveAll(MOTOR_STOP, 0);
        nextLedState = LED_STATE_BLINK_FAST;
    }

    // ---------- Valid packet ----------
    else
    {
        int16_t angle = directionAngles[pkt.direction];

        if (angle < 0)
        {
            Motor_MoveAll(MOTOR_STOP, 0);
            nextLedState = LED_STATE_BLINK_FAST;
        }
        else
        {
            uint8_t speed = (pkt.speed * 100) / 70;
            Motor_RunDirection(angle, speed);

            // Motor running → steady ON
            nextLedState = LED_STATE_STEADY;

            // OPTIONAL: brief RX indication
            // nextLedState = LED_STATE_DOUBLE_BLINK;
        }
    }

    // ---------- Apply LED state once ----------
    statusLED.state = nextLedState;
    LED_Update(&statusLED);
}

