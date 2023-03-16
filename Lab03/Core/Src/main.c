/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi2;

TSC_HandleTypeDef htsc;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI2_Init(void);
static void MX_TSC_Init(void);
static void MX_USB_PCD_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();
	
	__HAL_RCC_GPIOC_CLK_ENABLE(); // Enable peripheral GPIOC clock (for LEDs)
	
	// Configuration struct to pass to the initialization function
	GPIO_InitTypeDef initStr = {GPIO_PIN_8 | GPIO_PIN_9,
															GPIO_MODE_OUTPUT_PP,
															GPIO_SPEED_FREQ_LOW,
															GPIO_NOPULL};
	
	HAL_GPIO_Init(GPIOC, &initStr); // Initialize all of the LED pins in the main function
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET); // Set the green LED (PC9) high	
			
															
															
	// Look up the alternate functions of the red (PC6) and blue (PC7) LEDs 
	//   by following examples 3.2 and 3.3 -- AF0->  PC6: TIM3_CH1,  PC7: TIM3_CH2
	// Configure the LED pins to alternate function mode, and select the 
	//   appropriate function number in alternate function registers
		
	// Set the pins to alternate function mode in the MODER register (write bits 10)
	GPIOC->MODER &= ~(1<<12); // Clear bit 12 of MODER6 for PC6
	GPIOC->MODER |=  (1<<13); // Set   bit 13 of MODER6 for PC6				
	GPIOC->MODER &= ~(1<<14); // Clear bit 14 of MODER7 for PC7
	GPIOC->MODER |=  (1<<15); // Bit   bit 15 of MODER7 for PC7
															
	// Set the pins to push-pull output type in the OTYPER register (write bit 0)
	GPIOC->OTYPER &= ~(1<<6); // Clear bit 6 of OTYPER6 for PC6
	GPIOC->OTYPER &= ~(1<<7); // Clear bit 7 of OTYPER7 for PC7			
															
	// Set the pins to low speed in the OSPEEDR register (write bits x0)
	GPIOC->OSPEEDR &= ~(1<<12); // Clear bit 12 of OSPEEDR6 for PC6 (don't care bit 13)
	GPIOC->OSPEEDR &= ~(1<<14); // Clear bit 14 of OSPEEDR7 for PC7 (don't care bit 15)
															
	// Set to no pull-up/down resistors in the PUPDR register (write bits 00)
	GPIOC->PUPDR &= ~(1<<12); // Clear bit 12 of PUPDR6 for PC6
	GPIOC->PUPDR &= ~(1<<13); // Clear bit 13 of PUPDR6 for PC6
	GPIOC->PUPDR &= ~(1<<14); // Clear bit 14 of PUPDR7 for PC7
	GPIOC->PUPDR &= ~(1<<15); // Clear bit 15 of PUPDR7 for PC7
/*
	// Configure LED pins to alternate function mode (write bits 0000)
	GPIOC->AFR[0] &= ~(1<<24);  // Clear bit 24 to select AF0 in AFRL for PC6
	GPIOC->AFR[0] &= ~(1<<25);  // Clear bit 25 to select AF0 in AFRL for PC6
	GPIOC->AFR[0] &= ~(1<<26);  // Clear bit 26 to select AF0 in AFRL for PC6
	GPIOC->AFR[0] &= ~(1<<27);  // Clear bit 27 to select AF0 in AFRL for PC6
	GPIOC->AFR[0] &= ~(1<<28);  // Clear bit 28 to select AF0 in AFRL for PC7
	GPIOC->AFR[0] &= ~(1<<29);  // Clear bit 29 to select AF0 in AFRL for PC7
	GPIOC->AFR[0] &= ~(1<<30);  // Clear bit 30 to select AF0 in AFRL for PC7
	GPIOC->AFR[0] &= ~(1<<31);  // Clear bit 31 to select AF0 in AFRL for PC7
*/
	GPIOC->AFR[0] |= 0x0000 << GPIO_AFRL_AFRL6_Pos;  // Select AF0 on PC6 in AFRL for TIM3_CH1
	GPIOC->AFR[0] |= 0x0000 << GPIO_AFRL_AFRL7_Pos;  // Select AF0 on PC7 in AFRL for TIM3_CH2										

															

	// For TIM2 -- Interrupt at 4 Hz with input clock of 8 MHz.
	// 4 Hz -> T = 1/4 = 0.25s = 250ms period.
	// 8Mhz / 8000 = 1kHz. 1kHz->1ms timer count. 
	// 1ms * 250 = 250s period. PSC = 8000-1. ARR = 250.

	// Enable the timer 2 peripheral (TIM2) in the RCC
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; 															
		
	// Configure the timer to trigger an update event (UEV) at 4 Hz 
	TIM2->PSC = 7999; // (8000-1) 8Mhz clock / 8000 = 1kHz (1 ms timer count)
	TIM2->ARR = 250;    // 1 ms * 250 = 250ms period
	
	// Configure the timer to generate an interrupt on the UEV event
	TIM2->DIER |= (1<<0); // Set bit 0 of DIER to enable update interrupt
	//TIM2->DIER |= TIM_DIER_UIE;
	
	// Configure and enable/start the timer
	TIM2->CR1 |= (1<<0); // Set bit 0 of CR1 to enable timer
	//TIM2->CR1 |= TIM_CR1_CEN;
	
	// Set up the timer’s interrupt handler and enable in the NVIC
	NVIC_EnableIRQ(TIM2_IRQn);
	NVIC_SetPriority (TIM2_IRQn, 2); // Set priority of EXTI0 to 2
	
	
	
	// For TIM3 -- Interrupt at 800 Hz with input clock of 8 MHz.
	// 800 Hz -> T = 1/8 = 0.125s = 1250s period.
	// 8Mhz / 8 = 1MHz. 1MHz->1s timer count. 
	// 1s * 1205 = 1250s period. PSC = 8000-1. ARR = 1250. 
	
	// Enable the timer 3 peripheral (TIM3) in the RCC
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	
	// Configure the timer to trigger an update event (UEV) at 800 Hz 
	TIM3->PSC = 7;  // (8-1) 8Mhz clock / 8 =  1MHz (1000ms = 1s timer count)
	TIM3->ARR = 1250; // 1s * 1250 = 1250s period	
	
	// Use the Capture/Compare Mode Register 1 (CCMR1) register  
	//   to configure the output channels to PWM mode
	// For the CC1S[1:0] and CC2S[1:0] bit fields; set channels to output (wite bits 00)
	TIM3->CCMR1 &= ~(1<<0);  // Clear bit 0 of CC1S[1:0] for TIM3_CCMR1	
	TIM3->CCMR1 &= ~(1<<1);  // Clear bit 1 of CC1S[1:1] for TIM3_CCMR1		
	TIM3->CCMR1 &= ~(1<<8);  // Clear bit 8 of CC2S[1:0] for TIM3_CCMR1	
	TIM3->CCMR1 &= ~(1<<9);  // Clear bit 9 of CC2S[1:1] for TIM3_CCMR1	
	
	// For the OC1M[2:0] bit field; set output channel 1 to PWM Mode 2 (write bits 111)
	TIM3->CCMR1 |=  (1<<4);  // Set bit 4 of OC1M[2:0] for TIM3_CCMR1
	TIM3->CCMR1 |=  (1<<5);  // Set bit 5 of OC1M[2:1] for TIM3_CCMR1
	TIM3->CCMR1 |=  (1<<6);  // Set bit 6 of OC1M[2:2] for TIM3_CCMR1
	
	// Use the OC2M[2:0] bit field to set channel 2 to PWM Mode 1 (write bits 110)
	TIM3->CCMR1 &= ~(1<<12); // Clear bit 12 of OC2M[2:0] for TIM3_CCMR1
	TIM3->CCMR1 |=  (1<<13); // Set   bit 13 of OC2M[2:1] for TIM3_CCMR1
	TIM3->CCMR1 |=  (1<<14); // Set   bit 14 of OC2M[2:2] for TIM3_CCMR1
	
	// Enable the output compare preload for both channels OC1PE & OC2PE (write bit 1)
	TIM3->CCMR1 |=  (1<<3);  // Set bit 3  of OC1PE for TIM3_CCMR1
	TIM3->CCMR1 |=  (1<<11); // Set bit 11 of OC2PE for TIM3_CCMR1
	
	// Set the output enable bits for channels 1 & 2 in the CCER register (write bit 1)
	TIM3->CCER |=  (1<<0);  // Set bit 0 of CC1E for TIM3_CCR1
	TIM3->CCER |=  (1<<4);  // Set bit 4 of CC1E for TIM3_CCR1
	
	// Set the capture/compare registers (CCRx) for both channels to 20% of your ARR value
	//TIM3->CCR1 = 250; 
	//TIM3->CCR2 = 250;
	TIM3->CCR1 = 1200; 
	TIM3->CCR2 = 10;  
	
	// Configure and enable/start the timer
	TIM3->CR1 |= (1<<0); // Set bit 0 of CR1 to enable timer
	//TIM3->CR1 |= TIM_CR1_CEN;
	

  while (1)
  {

  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x20303E5D;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TSC Initialization Function
  * @param None
  * @retval None
  */
static void MX_TSC_Init(void)
{

  /* USER CODE BEGIN TSC_Init 0 */

  /* USER CODE END TSC_Init 0 */

  /* USER CODE BEGIN TSC_Init 1 */

  /* USER CODE END TSC_Init 1 */

  /** Configure the TSC peripheral
  */
  htsc.Instance = TSC;
  htsc.Init.CTPulseHighLength = TSC_CTPH_2CYCLES;
  htsc.Init.CTPulseLowLength = TSC_CTPL_2CYCLES;
  htsc.Init.SpreadSpectrum = DISABLE;
  htsc.Init.SpreadSpectrumDeviation = 1;
  htsc.Init.SpreadSpectrumPrescaler = TSC_SS_PRESC_DIV1;
  htsc.Init.PulseGeneratorPrescaler = TSC_PG_PRESC_DIV4;
  htsc.Init.MaxCountValue = TSC_MCV_8191;
  htsc.Init.IODefaultMode = TSC_IODEF_OUT_PP_LOW;
  htsc.Init.SynchroPinPolarity = TSC_SYNC_POLARITY_FALLING;
  htsc.Init.AcquisitionMode = TSC_ACQ_MODE_NORMAL;
  htsc.Init.MaxCountInterrupt = DISABLE;
  htsc.Init.ChannelIOs = TSC_GROUP1_IO3|TSC_GROUP2_IO3|TSC_GROUP3_IO2;
  htsc.Init.ShieldIOs = 0;
  htsc.Init.SamplingIOs = TSC_GROUP1_IO4|TSC_GROUP2_IO4|TSC_GROUP3_IO3;
  if (HAL_TSC_Init(&htsc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TSC_Init 2 */

  /* USER CODE END TSC_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, NCS_MEMS_SPI_Pin|EXT_RESET_Pin|LD3_Pin|LD6_Pin
                          |LD4_Pin|LD5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : NCS_MEMS_SPI_Pin EXT_RESET_Pin LD3_Pin LD6_Pin
                           LD4_Pin LD5_Pin */
  GPIO_InitStruct.Pin = NCS_MEMS_SPI_Pin|EXT_RESET_Pin|LD3_Pin|LD6_Pin
                          |LD4_Pin|LD5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : MEMS_INT1_Pin MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT1_Pin|MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
