#ifndef __GENERAL_H
#define __GENERAL_H

#include "stm32f1xx_hal.h"
#include "motor.h"
#include "nrf24_rx.h"


// Move robot in a specific direction (angle in degrees)
void General_Run(int16_t angle, uint8_t speed);

// Stop robot
void Robot_Stop(void);

#endif /* __GENERAL_H */
