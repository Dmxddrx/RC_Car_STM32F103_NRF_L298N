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
    GPIO_TypeDef *port;
    uint16_t pin;
    LED_State state;
    uint32_t lastToggle;
    uint8_t outputState;   // 0 = OFF, 1 = ON
} LED_HandleTypeDef;

void LED_Init(LED_HandleTypeDef *led, GPIO_TypeDef *port, uint16_t pin);
void LED_Update(LED_HandleTypeDef *led);

#endif
