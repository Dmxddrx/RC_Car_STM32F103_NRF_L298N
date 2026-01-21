#include "general.h"
#include "nrf24_rx.h"
#include "motor.h"
#include "led.h"
#include "oled.h"

#define MANUAL_TEST_MODE   1   // set to 0 to restore NRF control
#define OLED_DEBUG_MODE   1   // 1 = show NRF packets on OLED

extern LED_HandleTypeDef statusLED;


#if OLED_DEBUG_MODE
	#include <stdio.h>

	static char oledLine1[20];
	static char oledLine2[20];
	static uint8_t last_dir = 0xFF;
	static uint8_t last_spd = 0xFF;
#endif


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

    ControlPacket *pkt = NRF24_GetLatest();
    LED_State nextLedState = LED_STATE_OFF;


	#if MANUAL_TEST_MODE
		// ===== MANUAL MOTOR TEST (REMOTE OFF) =====
		int16_t angle = directionAngles[0];   // 2nd direction (45°)
		uint8_t speed = 40;                   // 40%

		Motor_RunDirection(angle, speed);

		statusLED.state = LED_STATE_STEADY;
		LED_Update(&statusLED);

		return;   // bypass NRF logic completely
	#endif


	#if OLED_DEBUG_MODE

		uint8_t st   = NRF24_ReadStatus();
		uint8_t fifo = NRF24_ReadFIFO();
		uint8_t ch   = NRF24_ReadChannel();

		OLED_ClearArea(0, 0, 128, 40);

		// ---------- RF STATUS (top) ----------
		snprintf(oledLine1, sizeof(oledLine1),
				 "ST:%02X RX:%d TX:%d",
				 st,
				 (st >> 6) & 1,
				 (st >> 5) & 1);

		snprintf(oledLine2, sizeof(oledLine2),
				 "FIFO:%02X IRQ:%lu",
				 fifo,
				 nrfIrqCount);

		OLED_Print(0, 0, oledLine1);
		OLED_Print(0,10, oledLine2);

		char line3[20];
		snprintf(line3,sizeof(line3),"CH:%d",ch);
		OLED_Print(0,20,line3);

		// ---------- PACKET INFO (bottom) ----------
		if (pkt == NULL)
		{
			OLED_Print(0, 30, "PKT: NULL");
		}
		else
		{
			uint8_t dir = pkt->direction;
			uint8_t spd = pkt->speed;

			if (dir > 8 || spd > 70)
			{
				snprintf(oledLine1, sizeof(oledLine1),
						 "BAD D:%d S:%d", dir, spd);
				OLED_Print(0, 30, oledLine1);
			}
			else
			{
				snprintf(oledLine1, sizeof(oledLine1),
						 "DIR:%d SPD:%d", dir, spd);
				OLED_Print(0, 30, oledLine1);

				last_dir = dir;
				last_spd = spd;
			}
		}

		OLED_Update();

	#endif



    if(!NRF24_IsConnected()){
        Motor_MoveAll(MOTOR_STOP, 0);
        nextLedState = LED_STATE_BLINK_SLOW;
    }
    else if(pkt == NULL){
        Motor_MoveAll(MOTOR_STOP, 0);
        nextLedState = LED_STATE_HEARTBEAT;
    }
    else{
        int16_t angle = directionAngles[pkt->direction];
        if(angle < 0){
            Motor_MoveAll(MOTOR_STOP, 0);
            nextLedState = LED_STATE_OFF;
        } else {
            uint8_t speed = (pkt->speed * 100) / 70;
            Motor_RunDirection(angle, speed);
            nextLedState = LED_STATE_STEADY;
        }
    }

    // ---------- Apply LED state once ----------
    statusLED.state = nextLedState;
    LED_Update(&statusLED);
}

