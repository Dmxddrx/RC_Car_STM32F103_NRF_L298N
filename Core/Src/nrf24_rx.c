#include "nrf24_rx.h"

extern SPI_HandleTypeDef hspi1;

#define NRF_CE_PORT   GPIOA
#define NRF_CE_PIN    GPIO_PIN_4
#define NRF_CSN_PORT  GPIOA
#define NRF_CSN_PIN   GPIO_PIN_3

#define FIFO_STATUS 0x17

static const uint8_t rxAddress[5] = {'C','A','R','0','1'};

static void CSN_Low(void)  { HAL_GPIO_WritePin(NRF_CSN_PORT,NRF_CSN_PIN,GPIO_PIN_RESET); }
static void CSN_High(void) { HAL_GPIO_WritePin(NRF_CSN_PORT,NRF_CSN_PIN,GPIO_PIN_SET); }
static void CE_Low(void)   { HAL_GPIO_WritePin(NRF_CE_PORT,NRF_CE_PIN,GPIO_PIN_RESET); }
static void CE_High(void)  { HAL_GPIO_WritePin(NRF_CE_PORT,NRF_CE_PIN,GPIO_PIN_SET); }

static uint8_t SPI_RW(uint8_t data) {
    uint8_t rx;
    HAL_SPI_TransmitReceive(&hspi1,&data,&rx,1,HAL_MAX_DELAY);
    return rx;
}

static void NRF_WriteReg(uint8_t reg,uint8_t val){
    CSN_Low();
    SPI_RW(NRF_W_REGISTER|reg);
    SPI_RW(val);
    CSN_High();
}

static uint8_t NRF_ReadReg(uint8_t reg){
    uint8_t val;
    CSN_Low();
    SPI_RW(NRF_R_REGISTER|reg);
    val=SPI_RW(NRF_NOP);
    CSN_High();
    return val;
}

void NRF24_Init(void){
    CE_Low();
    HAL_Delay(5); // Power-on reset

    NRF_WriteReg(CONFIG,0x0B); // PWR_UP=1, PRIM_RX=1
    HAL_Delay(2); // ✅ REQUIRED (>=1.5ms)

    NRF_WriteReg(EN_AA,0x01);
    NRF_WriteReg(EN_RXADDR,0x01);
    NRF_WriteReg(RF_CH,108);
    NRF_WriteReg(RF_SETUP,0x06);
    NRF_WriteReg(RX_PW_P0,sizeof(ControlPacket));

    CSN_Low();
    SPI_RW(NRF_W_REGISTER|RX_ADDR_P0);
    for(int i=0;i<5;i++) SPI_RW(rxAddress[i]);
    CSN_High();

    CSN_Low();
    SPI_RW(NRF_FLUSH_RX);
    CSN_High();

    CE_High();
}

bool NRF24_DataAvailable(void){
	uint8_t fifo = NRF_ReadReg(FIFO_STATUS);
	return !(fifo & 0x01); // RX_EMPTY = 0
}

bool NRF24_Read(ControlPacket *pkt){
    if(!NRF24_DataAvailable()) return false;

    CSN_Low();
    SPI_RW(NRF_R_RX_PAYLOAD);
    uint8_t *p = (uint8_t*)pkt;
    for(int i=0;i<sizeof(ControlPacket);i++) p[i]=SPI_RW(NRF_NOP);
    CSN_High();

    NRF_WriteReg(STATUS,0x40);
    return true;
}

bool NRF24_IsConnected(void){
	NRF_WriteReg(RF_CH, 108);
	return (NRF_ReadReg(RF_CH) == 108);
}
