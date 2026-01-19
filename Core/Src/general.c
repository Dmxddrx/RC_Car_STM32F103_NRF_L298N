#include "general.h"
#include "nrf24_rx.h"
#include "motor.h"
#include "led.h"

#define MANUAL_TEST_MODE   1   // set to 0 to restore NRF control

extern LED_HandleTypeDef statusLED;

// Direction mapping for STM32 motors
static const int16_t directionAngles[9] = {
    -1, 	// index 0	- Stop
	180, 	// index 1	- Backward
	225, 	// index 2	- Backward - Left
	270, 	// index 3	- Left
	315, 	// index 4	- Forward - Left
	0, 		// index 5	- Forward
	45, 	// index 6	- Forward - Right
	90, 	// index 7	- Right
	135		// index 8	- Backward - Right
};


void General_Run(void)
{
	#if MANUAL_TEST_MODE
		// ===== MANUAL MOTOR TEST (REMOTE OFF) =====
		int16_t angle = directionAngles[8];   // 2nd direction (45°)
		uint8_t speed = 40;                   // 40%

		Motor_RunDirection(angle, speed);

		statusLED.state = LED_STATE_STEADY;
		LED_Update(&statusLED);

		return;   // bypass NRF logic completely
	#endif


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

