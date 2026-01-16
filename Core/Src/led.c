#include "led.h"

#define LED_BLINK_SLOW_INTERVAL 500  // ms
#define LED_BLINK_FAST_INTERVAL 100  // ms

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
    }
}
