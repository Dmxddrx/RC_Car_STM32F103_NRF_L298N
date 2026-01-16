#ifndef __LED_H
#define __LED_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

typedef enum {
    LED_STATE_OFF = 0,
    LED_STATE_STEADY,
    LED_STATE_BLINK_SLOW,
    LED_STATE_BLINK_FAST
} LED_State;

typedef struct {
    TIM_HandleTypeDef *htim; // timer handle
    uint32_t channel;        // timer channel
    uint8_t brightness;      // PWM brightness
    LED_State state;         // current state
    uint32_t lastToggle;     // last toggle time
} LED_HandleTypeDef;

void LED_Init(LED_HandleTypeDef *led, TIM_HandleTypeDef *htim, uint32_t channel, uint8_t brightness);
void LED_Update(LED_HandleTypeDef *led);

#endif
