// ======================= Includes =======================
#include "general.h"	// Core application logic
#include "nrf24_rx.h"	// - NRF24 receiver
#include "motor.h"		// - Motor driver
#include "led.h"		// - Status LED handling
#include "oled.h"		// - OLED for debugging

// ======================= Build-Time Modes =======================

// MANUAL_TEST_MODE:
// 1 → Ignore NRF, force fixed motor movement (debug / hardware test)
// 0 → Normal NRF-controlled operation
#define MANUAL_TEST_MODE   0

// OLED_DEBUG_MODE:
// 1 → Show NRF status + packets on OLED
// 0 → OLED disabled (saves time & I2C traffic)
#define OLED_DEBUG_MODE   1   // 1 = show NRF packets on OLED

// Status LED handle defined in main.c
extern LED_HandleTypeDef statusLED;

// ======================= OLED Debug Support =======================
#if OLED_DEBUG_MODE
	#include <stdio.h>

	static char oledLine1[20];
	static char oledLine2[20];
	static uint8_t last_dir = 0xFF;
	static uint8_t last_spd = 0xFF;
#endif


	// ======================= Direction Mapping =======================
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

static bool nrfPresent = false;

// ======================= Packet Timeout =======================
static uint32_t lastPacketTick = 0;
static int16_t lastAngle = 0;
static uint8_t lastSpeed = 0;
#define PACKET_TIMEOUT_MS 200  // 200 ms timeout

// ======================= Initialization =======================
void General_Init(void)
{
    nrfPresent = NRF24_IsConnected();

    if(!nrfPresent)
    {
        Motor_MoveAll(MOTOR_STOP, 0);
        statusLED.state = LED_STATE_ERROR_PULSE; // hardware fault
        LED_Update(&statusLED);
    }
}


// ======================= Main Control Loop =======================

// - NRF packet reading
// - Motor control logic
// - LED status updates
// - Optional OLED debugging
void General_Run(void)
{

	// Get pointer to most recent NRF packet (non-blocking)
    ControlPacket *pkt = NRF24_GetLatest();

    // Desired LED state for this cycle
    LED_State nextLedState = LED_STATE_OFF;


    // ======================= Manual Motor Test =======================
	#if MANUAL_TEST_MODE
    	// Forces constant movement without NRF
		int16_t angle = directionAngles[5];   // Direction
		uint8_t speed = 40;                   // 40%

		Motor_RunDirection(angle, speed);

		statusLED.state = LED_STATE_STEADY;
		LED_Update(&statusLED);

		return;   // bypass NRF logic completely
	#endif


	// ======================= OLED Debug Output =======================
	#if OLED_DEBUG_MODE

		// Read low-level NRF diagnostics
		uint8_t st   = NRF24_ReadStatus();
		uint8_t fifo = NRF24_ReadFIFO();
		uint8_t ch   = NRF24_ReadChannel();

		OLED_ClearArea(0, 0, 128, 64);

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

		OLED_Print(0,20, oledLine1);
		OLED_Print(0,30, oledLine2);

		char line3[20];
		snprintf(line3, sizeof(line3), "CH:%d", ch);
		OLED_Print(0,40,line3);

		OLED_Print(56, 40, (pkt != NULL) ? "PKT:1" : "PKT:0");
	    OLED_Print(0, 0, NRF24_IsConnected() ? "NRF:1" : "NRF:0");
	    OLED_Print(58, 0, Motor_IsReady() ? "MOTOR:1" : "MOTOR:0");

		// ---------- PACKET INFO (bottom) ----------
		if (pkt == NULL)
		{
			OLED_Print(0, 50, "PKT: NULL");
		}
		else
		{
			uint8_t dir = pkt->direction;
			uint8_t spd = pkt->speed;

			if (dir > 8 || spd > 70)
			{
				snprintf(oledLine1, sizeof(oledLine1),
						 "BAD D:%d S:%d", dir, spd);
				OLED_Print(0, 50, oledLine1);
			}
			else
			{
				snprintf(oledLine1, sizeof(oledLine1),
						 "DIR:%d SPD:%d", dir, spd);
				OLED_Print(0, 50, oledLine1);

				last_dir = dir;
				last_spd = spd;
			}
		}

		OLED_Update();

	#endif

	// ---------- NRF & Packet Handling ----------

	uint32_t now = HAL_GetTick(); //returns the system tick in milliseconds

	if(!nrfPresent)
	{
		Motor_MoveAll(MOTOR_STOP, 0);
		nextLedState = LED_STATE_ERROR_PULSE;
	}
    else if(pkt != NULL)
    {
        // Valid packet → update last command
        lastAngle = directionAngles[pkt->direction];
        lastSpeed = (pkt->speed * 100) / 70;
        lastPacketTick = now;

        if(lastAngle < 0)
        {
            Motor_MoveAll(MOTOR_STOP, 0);
            nextLedState = LED_STATE_OFF;
        }
        else
        {
            Motor_RunDirection(lastAngle, lastSpeed);
            nextLedState = LED_STATE_STEADY;
        }
    }
    else
    {
        // No packet → stop immediately
        Motor_MoveAll(MOTOR_STOP, 0);
        nextLedState = LED_STATE_BLINK_SLOW;

        // Soft NRF reset only if timeout exceeded
        if(now - lastPacketTick > PACKET_TIMEOUT_MS)
        {
            NRF24_Init();
            lastPacketTick = now;
        }
    }

	// ---------- Apply LED state ----------
	statusLED.state = nextLedState;
	LED_Update(&statusLED);
}
