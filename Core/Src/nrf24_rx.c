#include "nrf24_rx.h"
#include "stm32f1xx_hal.h"

extern SPI_HandleTypeDef hspi1;

#define NRF_CE_PORT   GPIOA
#define NRF_CE_PIN    GPIO_PIN_4
#define NRF_CSN_PORT  GPIOA
#define NRF_CSN_PIN   GPIO_PIN_3
#define NRF_IRQ_PORT  GPIOA
#define NRF_IRQ_PIN   GPIO_PIN_0  // IRQ pin now on PA0

#define FIFO_STATUS 0x17

static const uint8_t rxAddress[5] = {'C','A','R','0','1'};

// Flag set by IRQ
volatile bool nrfPacketAvailable = false;
volatile uint32_t nrfIrqCount = 0;


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
    val = SPI_RW(NRF_NOP);
    CSN_High();
    return val;
}

// ---------- Latest received packet ----------
static volatile bool pktAvailable = false;
static ControlPacket latestPkt;

void NRF24_Init(void){
    CE_Low();
    HAL_Delay(5); // Power-on reset

    // PWR_UP=1, PRIM_RX=1
    NRF_WriteReg(CONFIG, 0x0F);
    HAL_Delay(2); // >=1.5ms required

    NRF_WriteReg(EN_AA, 0x00);
    NRF_WriteReg(EN_RXADDR, 0x01);
    NRF_WriteReg(RF_CH, 108);
    NRF_WriteReg(RF_SETUP, 0x06);
    NRF_WriteReg(RX_PW_P0, sizeof(ControlPacket));

    CSN_Low();
    SPI_RW(NRF_W_REGISTER | RX_ADDR_P0);
    for(int i=0; i<5; i++) SPI_RW(rxAddress[i]);
    CSN_High();

    CSN_Low();
    SPI_RW(NRF_FLUSH_RX);
    CSN_High();

    NRF_WriteReg(STATUS, 0x70);     // clear all IRQ flags

    CE_High();

    // -------- Configure PA0 IRQ --------
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = NRF_IRQ_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;  // NRF IRQ is active low
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(NRF_IRQ_PORT, &GPIO_InitStruct);

    // Enable EXTI0 interrupt
    HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

// Called from EXTI0_IRQHandler in stm32f1xx_it.c
void NRF24_HandleIRQ(void) {

	nrfIrqCount++;
    // Read the payload directly
    CSN_Low();
    SPI_RW(NRF_R_RX_PAYLOAD);
    uint8_t *p = (uint8_t*)&latestPkt;
    for(int i=0; i<sizeof(ControlPacket); i++) p[i] = SPI_RW(NRF_NOP);
    CSN_High();

    // Clear RX_DR | TX_DS | MAX_RT
    NRF_WriteReg(STATUS, 0x70);

    CSN_Low();
    SPI_RW(NRF_FLUSH_RX);
    CSN_High();

    // Mark packet available
    pktAvailable = true;
}

// ---------- Fully event-driven API ----------

// Returns pointer to the latest packet if available, NULL otherwise
ControlPacket* NRF24_GetLatest(void){
    if(pktAvailable){
        pktAvailable = false;
        return (ControlPacket*)&latestPkt;
    }
    return NULL;
}

bool NRF24_IsConnected(void){
    NRF_WriteReg(RF_CH, 108);
    return (NRF_ReadReg(RF_CH) == 108);
}

uint8_t NRF24_ReadStatus(void)
{
    CSN_Low();
    uint8_t status = SPI_RW(NRF_NOP);
    CSN_High();
    return status;
}

uint8_t NRF24_ReadFIFO(void)
{
    return NRF_ReadReg(FIFO_STATUS);
}

uint8_t NRF24_ReadChannel(void)
{
    return NRF_ReadReg(RF_CH);
}

