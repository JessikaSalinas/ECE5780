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

// Set LEDs depending on the output ADC value parameter
// For 8-bit resolution (2^8 = 256), threshold values set between this top amount
void Read_ADC_Set_LEDs(int adcValue){
	int threshold1 = 58;
	int threshold2 = 124;
	int threshold3 = 190;
	int threshold4 = 256; 
	// Set arbitrary values < than max of 256
	
	// LEDs light up sequentially orange(0°)->red(90°)->green(180°)->blue(270°)
	if ( (adcValue > 0) && (adcValue < threshold1) ) {
		GPIOC->BSRR |= (1<<8);   // Set orange LED (PC8) high
	}
	else if ( (adcValue > threshold1) && (adcValue < threshold2) ) {
		GPIOC->BSRR |= (1<<6);   // Set red LED (PC6) high
	}
	else if ( (adcValue > threshold2) && (adcValue < threshold3) ) {
		GPIOC->BSRR |= (1<<9);   // Set green LED (PC9) high	
	}
	else if ( (adcValue > threshold3) && (adcValue < threshold4) ) {
		GPIOC->BSRR |= (1<<7);   // Set blue LED (PC7) high	
	}
	else { // If adcValue < 0, turn off all LEDs
		GPIOC->BSRR |=  (1<<22); // Reset bit 22 of BSRR6 for PC6 (low)
		GPIOC->BSRR |=  (1<<23); // Reset bit 23 of BSRR7 for PC7 (low)
		GPIOC->BSRR |=  (1<<24); // Reset bit 24 of BSRR8 for PC8 (low)
		GPIOC->BSRR |=  (1<<25); // Reset bit 25 of BSRR9 for PC9 (low)
	}
}

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

	RCC->AHBENR  |= RCC_AHBENR_GPIOAEN; // Enable peripheral GPIOA clock
  RCC->AHBENR  |= RCC_AHBENR_GPIOCEN; // Enable peripheral GPIOC clock
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // Enable peripheral ADC1  clock
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;  // Enable peripheral DAC1  clock
	
	// -Setup LED pins-
																																																																						
	// Set the LED pins to general-purpose output mode in the MODER register (write bits 01)
	GPIOC->MODER |=  (1<<12);  // Set   bit 12 of MODER6 for PC6
	GPIOC->MODER &= ~(1<<13);  // Clear bit 13 of MODER6 for PC6
	GPIOC->MODER |=  (1<<14);  // Set   bit 14 of MODER7 for PC7
	GPIOC->MODER &= ~(1<<15);  // Clear bit 15 of MODER7 for PC7
	GPIOC->MODER |=  (1<<16);  // Set   bit 16 of MODER8 for PC8
	GPIOC->MODER &= ~(1<<17);  // Clear bit 17 of MODER8 for PC8
	GPIOC->MODER |=  (1<<18);  // Set   bit 18 of MODER9 for PC9
	GPIOC->MODER &= ~(1<<19);  // Clear bit 19 of MODER9 for PC9
	
	// Set the LED pins to push-pull output type in the OTYPER register (write bit 0)
	GPIOC->OTYPER &= ~(1<<6);  // Clear bit 6 of OTYPER6 for PC6
	GPIOC->OTYPER &= ~(1<<7);  // Clear bit 7 of OTYPER7 for PC7	
	GPIOC->OTYPER &= ~(1<<8);  // Clear bit 8 of OTYPER8 for PC8	
	GPIOC->OTYPER &= ~(1<<9);  // Clear bit 9 of OTYPER9 for PC9	
	
	// Set the LED pins to low speed in the OSPEEDR register (write bits x0)
	GPIOC->OSPEEDR &= ~(1<<12); // Clear bit 12 of OSPEEDR6 for PC6 (don't care bit 13)
	GPIOC->OSPEEDR &= ~(1<<14); // Clear bit 14 of OSPEEDR7 for PC7 (don't care bit 15)
	GPIOC->OSPEEDR &= ~(1<<16); // Clear bit 16 of OSPEEDR8 for PC8 (don't care bit 17)
	GPIOC->OSPEEDR &= ~(1<<18); // Clear bit 18 of OSPEEDR9 for PC9 (don't care bit 19)
	
	// Set the LED pins to no pull-up/down resistors in the PUPDR register (write bits 00)
	GPIOC->PUPDR &= ~(1<<12);  // Clear bit 12 of PUPDR6 for PC6
	GPIOC->PUPDR &= ~(1<<13);  // Clear bit 13 of PUPDR6 for PC6
	GPIOC->PUPDR &= ~(1<<14);  // Clear bit 14 of PUPDR7 for PC7
	GPIOC->PUPDR &= ~(1<<15);  // Clear bit 15 of PUPDR7 for PC7
	GPIOC->PUPDR &= ~(1<<16);  // Clear bit 16 of PUPDR8 for PC8
	GPIOC->PUPDR &= ~(1<<17);  // Clear bit 17 of PUPDR8 for PC8
	GPIOC->PUPDR &= ~(1<<18);  // Clear bit 18 of PUPDR9 for PC9
	GPIOC->PUPDR &= ~(1<<19);  // Clear bit 19 of PUPDR9 for PC9
	
	// -Setup ADC/DAC pins- 
	
	// ADC input  -> PC0 -> ADC_IN10
	// DAC output -> PA4 -> DAC_OUT1
	
	// Set the pins to analog mode in the MODER register (write bits 11)
	GPIOC->MODER |=  (1<<0); // Set  bit 0 of MODER0 for PC0
	GPIOC->MODER |=  (1<<1); // Set  bit 1 of MODER0 for PC0	
	GPIOA->MODER |=  (1<<8); // Set  bit 8 of MODER4 for PA4
	GPIOA->MODER |=  (1<<9); // Set  bit 9 of MODER4 for PA4
	
	// Set the pins to no pull-up/down resistor in the PUPDR register (write bits 00)
	GPIOC->PUPDR &= ~(1<<0); // Clear bit 0 of PUPDR0 for PC0
	GPIOC->PUPDR &= ~(1<<1); // Clear bit 1 of PUPDR0 for PC0
	GPIOA->PUPDR &= ~(1<<8); // Clear bit 8 of PUPDR4 for PA4
	GPIOA->PUPDR &= ~(1<<9); // Clear bit 9 of PUPDR4 for PA4
	
	// Connect the output (center pin) of a potentiometer to the input pin (PC0).
	// The other two pins of the potentiometer should be conencted to 3V and GND. --Done
	
	// Set ADC data resolution to 8 bits in the CFGR1 register (write bits 10)
	ADC1->CFGR1 &= ~(1<<3);  // Clear bit 3 of CFGR1 for PC0
	ADC1->CFGR1 |=  (1<<4);  // Set   bit 4 of CFGR1 for PC0
	
	// Set ADC continuous conversion mode in the CONT register (write bit 1)
	ADC1->CFGR1 |= (1<<13);  // Set   bit 13 of CFGR1 for PC0
	
	// Set ADC hardware triggers disabled mode in the CFGR1 register (write bits 00)
	ADC1->CFGR1 &= ~(1<<10); // Clear bit 10 of CFGR1 for PC0
	ADC1->CFGR1 &= ~(1<<11); // Clear bit 11 of CFGR1 for PC0
	
	// Select the pin's channel for ADC conversion in CHSELR register (write bit 1)
	ADC1->CHSELR |= (1<<10); // Set bit 10 of CHSELR for PC0 (ADC_IN10)
	
	// Connect the oscilliscope probe or channel 0 of the logic analyzer to pin PA4. --**TODO**
	
	// -Lab 06 Checkoff 1: Read ADC values (from potentiometer) & set LEDs-
/*	
	// ADC calibration procedure:
	// 1. Ensure that ADEN = 0 and DMAEN = 0.
	// 2. Set ADCAL = 1.
	// 3. Wait until ADCAL = 0.
	// 4. The calibration factor can be read from bits 6:0 of ADC_DR
	if ( (ADC1->CR & 0x00) && (ADC1->CFGR1 & 0x00) ) {	// Check that ADEN=0 & DMAEN=0, respectively 
		ADC1->CR |= (1<<31);                              // Set bit 31 of CR for ADCAL
		while(! (ADC1->CR & 0x1F) ) {};                   // Wait until ADCAL = 0; 0x1F=31
	}
	
	// ADC enable procedure:
	// 1. Clear the ADRDY bit in ADC_ISR register by programming this bit to 1.
	// 2. Set ADEN = 1 in the ADC_CR register.
	// 3. Wait until ADRDY = 1 in the ADC_ISR register and continue to write ADEN = 1.
	ADC1->ISR |= (1<<0);  					// Set bit 0 of ISR to clear ADRDY
	ADC1->CR  |= (1<<0);  					// Set bit 0 of CR for ADEN
	while(! (ADC1->CR & 0x1F) ){};  // Wait until ADRDY = 1
	ADC1->CR  |= (1<<0);						// Continue to write ADEN = 1
		
	ADC1->CR |= ADC_CR_ADSTART; // Start ADC
	
	// In the main application loop, read the ADC data register and
	//   turn on/off LEDs depending on the value.
	int adcValue = 0;
	while (1) {
		adcValue = ADC1->DR;
		Read_ADC_Set_LEDs(adcValue); // Function to set LEDs
	}
*/	
	
	// -Lab 06 Checkoff 2: Generate a viewable analog waveform-
	
	// Set the used DAC channel to software trigger mode.
	DAC->SWTRIGR |= (1<<0);  // Set bit 0 for SWTRIG1
	
	// Enable the DAC1 channel.
	DAC->CR |= (1<<0);  // Set bit 0 in CR for DAC channel 1 enable
		
	// Copy one of the wave-tables into your application.	
	// Sine Wave: 8-bit, 32 samples/cycle
	const uint8_t sine_table[32] = {127,151,175,197,216,232,244,251,254,251,244,
	232,216,197,175,151,127,102,78,56,37,21,9,2,0,2,9,21,37,56,78,102};
//	// Triangle Wave: 8-bit, 32 samples/cycle
//	const uint8_t triangle_table[32] = {0,15,31,47,63,79,95,111,127,142,158,174,
//	190,206,222,238,254,238,222,206,190,174,158,142,127,111,95,79,63,47,31,15};
//	// Sawtooth Wave: 8-bit, 32 samples/cycle
//	const uint8_t sawtooth_table[32] = {0,7,15,23,31,39,47,55,63,71,79,87,95,103,
//	111,119,127,134,142,150,158,166,174,182,190,198,206,214,222,230,238,246};

	// In the main application loop, use an index variable to write the next value 
	//   in the wave-table to the appropriate DAC data register.
	// Use a 1ms delay between updating the DAC to new values.
	while (1) {	
		for (int i = 0; i < 32; i++) {
			HAL_Delay(1);  // 1ms delay	
			// Update values in the DAC channel 1 8-bit right-aligned register
			DAC->DHR8R1 = sine_table[i];	
		}			
	}	

} // End of main





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
