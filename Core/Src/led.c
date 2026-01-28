#include "led.h"

#define LED_BLINK_SLOW_INTERVAL 900  // ms
#define LED_BLINK_FAST_INTERVAL 100  // ms

#define LED_DOUBLE_BLINK_INTERVAL  120
#define LED_DOUBLE_BLINK_PAUSE     800

#define LED_HEARTBEAT_ON_TIME       80
#define LED_HEARTBEAT_PERIOD      1000

#define LED_ERROR_BLINK_INTERVAL   120
#define LED_ERROR_PAUSE           1500

#define TRIPLE_BLINK_INTERVAL 120
#define TRIPLE_BLINK_PAUSE    800
#define SOS_DOT               100
#define SOS_DASH              300
#define SOS_GAP               200


void LED_Init(LED_HandleTypeDef *led, GPIO_TypeDef *port, uint16_t pin)
{
    led->port = port;
    led->pin = pin;
    led->state = LED_STATE_OFF;
    led->lastToggle = HAL_GetTick();
    led->outputState = 0;

    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET); // LED OFF
}

void LED_Update(LED_HandleTypeDef *led)
{
    uint32_t now = HAL_GetTick();
    //static uint8_t phase = 0;
	static uint8_t blinkCount = 0;

    switch (led->state)
    {
        case LED_STATE_OFF:
            HAL_GPIO_WritePin(led->port, led->pin, GPIO_PIN_RESET);
            led->outputState = 0;
            break;

        case LED_STATE_STEADY:
            HAL_GPIO_WritePin(led->port, led->pin, GPIO_PIN_SET);
            led->outputState = 1;
            break;

        case LED_STATE_BLINK_SLOW:
            if (now - led->lastToggle >= LED_BLINK_SLOW_INTERVAL)
            {
                led->lastToggle = now;
                led->outputState ^= 1;
                HAL_GPIO_WritePin(
                    led->port,
                    led->pin,
                    led->outputState ? GPIO_PIN_SET : GPIO_PIN_RESET
                );
            }
            break;

        case LED_STATE_BLINK_FAST:
            if (now - led->lastToggle >= LED_BLINK_FAST_INTERVAL)
            {
                led->lastToggle = now;
                led->outputState ^= 1;
                HAL_GPIO_WritePin(
                    led->port,
                    led->pin,
                    led->outputState ? GPIO_PIN_SET : GPIO_PIN_RESET
                );
            }
            break;

        case LED_STATE_DOUBLE_BLINK:
			if (now - led->lastToggle >= LED_DOUBLE_BLINK_INTERVAL)
			{
				led->lastToggle = now;
				led->outputState ^= 1;
				HAL_GPIO_WritePin(led->port, led->pin,
					led->outputState ? GPIO_PIN_SET : GPIO_PIN_RESET);

				if (++blinkCount >= 4) {   // ON-OFF-ON-OFF
					blinkCount = 0;
					led->lastToggle = now + LED_DOUBLE_BLINK_PAUSE;
				}
			}
			break;

        case LED_STATE_HEARTBEAT:
			if (now - led->lastToggle >= LED_HEARTBEAT_PERIOD)
			{
				led->lastToggle = now;
				HAL_GPIO_WritePin(led->port, led->pin, GPIO_PIN_SET);
			}
			else if (now - led->lastToggle >= LED_HEARTBEAT_ON_TIME)
			{
				HAL_GPIO_WritePin(led->port, led->pin, GPIO_PIN_RESET);
			}
			break;

        case LED_STATE_ERROR_PULSE:
			if (now - led->lastToggle >= LED_ERROR_BLINK_INTERVAL)
			{
				led->lastToggle = now;
				led->outputState ^= 1;
				HAL_GPIO_WritePin(led->port, led->pin,
					led->outputState ? GPIO_PIN_SET : GPIO_PIN_RESET);

				if (++blinkCount >= 6) { // 3 blinks
					blinkCount = 0;
					led->lastToggle = now + LED_ERROR_PAUSE;
					HAL_GPIO_WritePin(led->port, led->pin, GPIO_PIN_RESET);
				}
			}
			break;

        case LED_STATE_TRIPLE_BLINK:
            if(now - led->lastToggle >= TRIPLE_BLINK_INTERVAL)
            {
                led->lastToggle = now;
                led->outputState ^= 1;
                HAL_GPIO_WritePin(led->port, led->pin,
                    led->outputState ? GPIO_PIN_SET : GPIO_PIN_RESET);

                if(++blinkCount >= 6){ // ON-OFF-ON-OFF-ON-OFF = 3 blinks
                    blinkCount = 0;
                    led->lastToggle = now + TRIPLE_BLINK_PAUSE;
                    HAL_GPIO_WritePin(led->port, led->pin, GPIO_PIN_RESET);
                }
            }
            break;

    }
}
