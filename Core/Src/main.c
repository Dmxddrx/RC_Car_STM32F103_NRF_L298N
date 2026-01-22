// ======================= Includes =======================

#include "main.h"		// HAL + CMSIS core definitions, startup code, IRQ handlers
#include "general.h"	// Main application logic
#include "led.h"		// LED indicator
#include "motor.h"		// Motor control (PWM + direction pins)
#include "nrf24_rx.h"	// NRF24L01 RX driver (SPI + IRQ based reception)
#include "oled.h"		// OLED display driver (I2C | 128x64 px)



// ======================= Peripheral Handles =======================

I2C_HandleTypeDef hi2c1;	// I2C1 handle (used for OLED display)
SPI_HandleTypeDef hspi1;	// SPI1 handle (used for NRF24L01)
TIM_HandleTypeDef htim1;	// TIM1 handle (Master timer for motor PWM)
TIM_HandleTypeDef htim2;	// TIM2 handle (Slave timer synced to TIM1)

LED_HandleTypeDef statusLED;	// Status LED handle (PC13)


// ======================= Function Prototypes =======================
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);


// ======================= Main Entry =======================
int main(void)
{

  // Initialize HAL library:
  // - Flash interface
  // - SysTick
  // - NVIC priority grouping
  HAL_Init();

  // Enable AFIO clock (needed for remapping & EXTI)
  __HAL_RCC_AFIO_CLK_ENABLE();

  // Disable JTAG but keep SWD
  // Frees PB3, PB4, PA15 for GPIO use
  __HAL_AFIO_REMAP_SWJ_NOJTAG();   // Disable JTAG, keep SWD

  // Configure system clock to 72 MHz (HSE + PLL)
  SystemClock_Config();

  MX_GPIO_Init();	// Initialize all GPIO pins
  MX_SPI1_Init();	// Initialize SPI1 (NRF24)
  MX_TIM1_Init();	// Initialize TIM1 (PWM Master)
  MX_TIM2_Init();	// Initialize TIM2 (PWM Slave)
  MX_I2C1_Init();	// Initialize I2C1 (OLED)

  Motor_Init();			// Initialize motor driver (PWM + direction pins)
  NRF24_Init();			// Initialize NRF24L01 receiver
  OLED_Init(&hi2c1);	// Initialize OLED with I2C handle

  // Initialize status LED on PC13
  LED_Init(&statusLED, GPIOC, GPIO_PIN_13);
  statusLED.state = LED_STATE_STEADY;

#define GPIO_TEST_MODE   0 	// Compile-time GPIO test switch


  // ======================= Main Loop =======================
  while (1)
  {
	  // Main robot logic:
	  // - Read NRF data
	  // - Compute direction & speed
	  // - Drive motors
	  General_Run();

	  // Update LED state machine
	  LED_Update(&statusLED);

	  // Small delay to reduce CPU load
	  HAL_Delay(10);


	#if GPIO_TEST_MODE
	  // Simple GPIO toggle test for PC13
	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	  HAL_Delay(500);
	#endif

  }
}


// ======================= System Clock =======================
void SystemClock_Config(void)
{
  //RCC | Reset and Clock Control.
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  // Use external crystal (HSE | High speed external clock)
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;

  // HSE divider = 1
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;

  // Enable internal RC (used for safety)
  // Internal RC refers to the HSI (High-Speed Internal) clock source.
  // HSI is an RC oscillator circuit (Resistor-Capacitor)
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;

  // Enable PLL
  // PLL stands for Phase-Locked Loop.
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;	// PLL source = HSE
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;			// PLL multiplier = 9 → 8MHz × 9 = 72MHz

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  // Configure AHB, APB1, APB2 clocks
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;

  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}


// ======================= I2C1 Init (OLED) =======================
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;		// Fast mode I2C (400kHz)
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}


// ======================= SPI1 Init (NRF24) ======================
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;			// SPI master mode
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;	// Full duplex
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;		// 8-bit data

  // SPI mode 0 (CPOL=0, CPHA=0)
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;

  hspi1.Init.NSS = SPI_NSS_SOFT;								// Software NSS control
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;		// SPI clock = PCLK / 16
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}


// ======================= TIM1 Init (PWM Master) =======================
// TIM1 is used as the MASTER PWM timer.
// It generates PWM signals for motors and also provides a TRGO trigger
// that synchronizes TIM2 (slave).
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;

  // Prescaler = 71 → Timer clock = 72 MHz / (71 + 1) = 1 MHz
  htim1.Init.Prescaler = 71;

  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;

  // Auto-reload value → PWM period = 1000 ticks
  // PWM frequency = 1 MHz / 1000 = 1 kHz
  htim1.Init.Period = 999;

  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

  // Initialize base timer
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  // Use internal clock (no external trigger)
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  // Configure TIM1 as MASTER:
  // TRGO = Update event → triggers slave timer (TIM2)
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;

  // Enable master-slave mode
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  // ---------------- PWM Channel Configuration ----------------
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim1);

}


// ======================= TIM2 Init (PWM Slave) =======================
// TIM2 is synchronized to TIM1 using internal trigger (ITR0).
// It resets its counter whenever TIM1 updates, keeping both PWMs aligned.
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  // ---------------- Slave Mode Configuration ----------------
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  // ---------------- PWM Channels ----------------
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim2);

}


// ======================= GPIO + EXTI + PWM Pins =======================
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // GPIO Ports Clock Enable
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // Set initial output level for PC13 (LED OFF)
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  // Set initial output level for PA3, PA4
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  // Set initial output level for motor direction pins
  HAL_GPIO_WritePin(GPIOB, IN1_Left_Pin|IN2_Left_Pin|IN3_Left_Pin|IN4_Left_Pin
                          |IN1_Right_Pin|IN2_Right_Pin|IN3_Right_Pin|IN4_Right_Pin, GPIO_PIN_RESET);

  // PC13 → Status LED output
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  // PA3, PA4 → NRF pins (CSN, CE)
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Motor direction control pins : IN1_Left_Pin IN2_Left_Pin IN3_Left_Pin IN4_Left_Pin
                           IN1_Right_Pin IN2_Right_Pin IN3_Right_Pin IN4_Right_Pin */
  GPIO_InitStruct.Pin = IN1_Left_Pin|IN2_Left_Pin|IN3_Left_Pin|IN4_Left_Pin
                          |IN1_Right_Pin|IN2_Right_Pin|IN3_Right_Pin|IN4_Right_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  //---------- NRF24 IRQ pin: PB5 as input with EXTI ----------
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING; // NRF IRQ active LOW
  GPIO_InitStruct.Pull = GPIO_PULLUP;          // required pull-up
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  //Enable EXTI5 interrupt (EXTI lines 5–9 share one IRQ)
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  // ---------------- TIM2 PWM Output Pins ----------------
  // PA0 → TIM2_CH1
  // PA1 → TIM2_CH2
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

void Error_Handler(void)
{
	// Disable all interrupts
  __disable_irq();

  // Stay here forever (debug breakpoint friendly)
  while (1)
  {
  }
}
#ifdef USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
