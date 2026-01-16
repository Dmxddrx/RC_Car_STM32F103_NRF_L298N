#ifndef __NRF24_RX_H
#define __NRF24_RX_H

#include "stm32f1xx_hal.h"
#include <stdbool.h>

typedef struct {
    uint8_t direction;
    uint8_t speed;
} ControlPacket;

#define NRF_R_REGISTER      0x00
#define NRF_W_REGISTER      0x20
#define NRF_R_RX_PAYLOAD    0x61
#define NRF_FLUSH_RX        0xE2
#define NRF_NOP             0xFF

#define CONFIG              0x00
#define EN_AA               0x01
#define EN_RXADDR           0x02
#define RF_CH               0x05
#define RF_SETUP            0x06
#define STATUS              0x07
#define RX_ADDR_P0          0x0A
#define RX_PW_P0            0x11

void NRF24_Init(void);
bool NRF24_DataAvailable(void);
bool NRF24_Read(ControlPacket *pkt);
bool NRF24_IsConnected(void);

#endif
