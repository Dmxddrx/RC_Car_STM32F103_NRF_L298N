#include "led.h"

#define LED_BLINK_SLOW_INTERVAL 500  // ms
#define LED_BLINK_FAST_INTERVAL 100  // ms

void LED_Init(LED_HandleTypeDef *led, TIM_HandleTypeDef *htim, uint32_t channel, uint8_t brightness)
{
    led->htim = htim;
    led->channel = channel;
    led->brightness = brightness;
    led->state = LED_STATE_OFF;
    led->lastToggle = HAL_GetTick();

    HAL_TIM_PWM_Start(htim, channel);
    __HAL_TIM_SET_COMPARE(htim, channel, 0); // LED off initially
}

void LED_Update(LED_HandleTypeDef *led)
{
    uint32_t now = HAL_GetTick();

    switch(led->state)
    {
        case LED_STATE_OFF:
            __HAL_TIM_SET_COMPARE(led->htim, led->channel, 0);
            break;

        case LED_STATE_STEADY:
            __HAL_TIM_SET_COMPARE(led->htim, led->channel, led->brightness);
            break;

        case LED_STATE_BLINK_SLOW:
            if(now - led->lastToggle >= LED_BLINK_SLOW_INTERVAL)
            {
                led->lastToggle = now;
                uint32_t current = __HAL_TIM_GET_COMPARE(led->htim, led->channel);
                __HAL_TIM_SET_COMPARE(led->htim, led->channel, (current>0)?0:led->brightness);
            }
            break;

        case LED_STATE_BLINK_FAST:
            if(now - led->lastToggle >= LED_BLINK_FAST_INTERVAL)
            {
                led->lastToggle = now;
                uint32_t current = __HAL_TIM_GET_COMPARE(led->htim, led->channel);
                __HAL_TIM_SET_COMPARE(led->htim, led->channel, (current>0)?0:led->brightness);
            }
            break;
    }
}
