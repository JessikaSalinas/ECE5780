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


volatile int RX_Data1 = 0;
volatile int RX_Data2 = 0;
volatile int RX_Flag_Has_First_Char = 0;
volatile int RX_Flag_Got_Both_Chars = 0;


// Transmit data function
void UART3_Transmit (int toSend) 
{	
	// Check and wait on the USART status flag that indicates the transmit register is empty.
	if (USART3->ISR & (1<<7)) {
		USART3->TDR = toSend;
	}	
}
	

// Receive 1 value function
void UART3_Receive1 (int toGet) 
{	
	// --For first checkoff--
	// Toggle the correct LED whenever the character matching the first letter of the color is pressed
	// For ex, typing ‘r’ into the terminal would toggle the red LED.	
	
	if (toGet == 0x72) { 			// If 'r' for red LED (0x72=r)
		GPIOC->BSRR |= (1<<6);  // Set red LED (PC6) high		
	}
	else if (toGet == 0x62) { // If 'b' for blue LED (0x62=b)
		GPIOC->BSRR |= (1<<7);  // Set blue LED (PC7) high	
	}
	else if (toGet == 0x6F) { // If 'o' for orange LED (0x6F=o)
		GPIOC->BSRR |= (1<<8);  // Set orange LED (PC8) high	
	}
	else if (toGet == 0x67) { // If 'g' for green LED (0x67=g)
		GPIOC->BSRR |= (1<<9);  // Set green LED (PC9) high	
	}
	else {
		int count = 0;
		char* errorMsg = "Error. Try again.\n"; 	
		while (errorMsg[count] != '\0') { 
			HAL_Delay(50);	
			UART3_Transmit(errorMsg[count]); // Send error msg
			count++;
		}
		count = 0;
	}
}


// Receive 2 values function
void UART3_Receive2 (int toGet1, int toGet2)
{
	// --For second checkoff--
	// Your command parser must now accept two character commands.
	// The first character is a letter matching the one of the LED colors.
	// The first character is a letter matching the one of the LED colors.
	//  ‘0’ turns off the LED
	//  ‘1’ turns on the LED
	//  ‘2’ toggles the LED
	// On a successful command, print a message about which command was recognized.
	UART3_Transmit(' ');
		
	if (toGet1 == 0x72) { 			    // If 'r' for red LED (0x72=r)
		UART3_Transmit('r');
		
		if (toGet2 == 0x30) {			    // If '0' for red LED off (0x30=0)
			GPIOC->BSRR |= (1<<22);     // Set red LED (PC6) low	
			HAL_Delay(1);
			UART3_Transmit(0);					// Msg back 'r0'
		}
		else if (toGet2 == 0x31) {    // If '1' for red LED off (0x31=1)
			GPIOC->BSRR |= (1<<6);      // Set red LED (PC6) high	
			HAL_Delay(1);
			UART3_Transmit(1);					// Msg back 'r1'
		}
		else if (toGet2 == 0x32) {    // If '2' for red LED togle (0x32=2)
			int count = 0;		
			while (count < 10000000) {  // Turn on red pin for 1000ms
				GPIOC->BSRR |= (1<<6);    // Set red LED (PC6) high
				count++;
			}
			GPIOC->BSRR |= (1<<22);     // Set red LED (PC6) low
			HAL_Delay(1);
			UART3_Transmit(2);					// Msg back 'r2'
		}
		
	}
	else if (toGet1 == 0x62) {      // If 'b' for blue LED (0x62=b)
		UART3_Transmit('b');
		
		if (toGet2 == 0x30) {			    // If '0' for blue LED off (0x30=0)
			GPIOC->BSRR |= (1<<23);     // Set blue LED (PC7) low	
			HAL_Delay(1);
			UART3_Transmit(0);					// Msg back 'b0'
		}
		else if (toGet2 == 0x31) {    // If '1' for blue LED off (0x31=1)
			GPIOC->BSRR |= (1<<7);      // Set blue LED (PC7) high	
			HAL_Delay(1);
			UART3_Transmit(1);					// Msg back 'b1'
		}
		else if (toGet2 == 0x32) {    // If '2' for blue LED togle (0x32=2)
			int count = 0;		
			while (count < 10000000) {  // Turn on blue pin for 1000ms
				GPIOC->BSRR |= (1<<7);    // Set blue LED (PC7) high
				count++;
			}
			GPIOC->BSRR |= (1<<23);     // Set blue LED (PC7) low
			HAL_Delay(1);
			UART3_Transmit(2);					// Msg back 'b2'
		}
			
	}
	else if (toGet1 == 0x6F) {      // If 'o' for orange LED (0x6F=o)
		UART3_Transmit('o');
		
		if (toGet2 == 0x30) {			    // If '0' for orange LED off (0x30=0)
			GPIOC->BSRR |= (1<<24);     // Set orange LED (PC8) low	
			HAL_Delay(1);
			UART3_Transmit(0);					// Msg back 'o0'
		}
		else if (toGet2 == 0x31) {    // If '1' for orange LED off (0x31=1)
			GPIOC->BSRR |= (1<<8);      // Set orange LED (PC8) high
			HAL_Delay(1);
			UART3_Transmit(1);					// Msg back 'o1'			
		}
		else if (toGet2 == 0x32) {    // If '2' for orange LED togle (0x32=2)
			int count = 0;		
			while (count < 10000000) {  // Turn on orange pin for 1000ms
				GPIOC->BSRR |= (1<<8);    // Set orange LED (PC8) high
				count++;
			}
			GPIOC->BSRR |= (1<<24);     // Set orange LED (PC8) low
			HAL_Delay(1);
			UART3_Transmit(2);					// Msg back 'o2'
		}
		
	}
	else if (toGet1 == 0x67) {      // If 'g' for green LED (0x67=g)
		UART3_Transmit('g');
		
		if (toGet2 == 0x30) {			    // If '0' for green LED off (0x30=0)
			GPIOC->BSRR |= (1<<25);     // Set green LED (PC9) low
			HAL_Delay(1);
			UART3_Transmit(0);					// Msg back 'g0'
		}
		else if (toGet2 == 0x31) {    // If '1' for green LED off (0x31=1)
			GPIOC->BSRR |= (1<<9);      // Set green LED (PC9) high	
			HAL_Delay(1);
			UART3_Transmit(1);					// Msg back 'g1'
		}
		else if (toGet2 == 0x32) {    // If '2' for green LED togle (0x32=2)
			int count = 0;		
			while (count < 10000000) {  // Turn on green pin for 1000ms
				GPIOC->BSRR |= (1<<9);    // Set green LED (PC9) high
				count++;
			}
			GPIOC->BSRR |= (1<<25);     // Set green LED (PC9) low
			HAL_Delay(1);
			UART3_Transmit(2);					// Msg back 'g2'
		}
		
	}
	else {	
		int count = 0;
		char* errorMsg = "Error. Try again.\n"; 	
		while (errorMsg[count] != '\0') { 
			HAL_Delay(50);	
			UART3_Transmit(errorMsg[count]); // Send error msg
			count++;
		}
		count = 0;
	}
	
}


/**
  * @brief This function handles System peripheral interrupts.
  */
void USART3_4_IRQHandler(void)
{	
	// Interrupt handler that saves received data as it arrives.
	// Within the handler, save the receive register’s value into a global variable.	
	// Within the handler set a global variable as a flag indicating new data.
	
	if (USART3->ISR & 0x20) {        // Check data ready flags	
		if (RX_Flag_Has_First_Char == 1) {
			
			RX_Data2 = USART3->RDR; 			// Receive char 2 from putty, clear flag
			RX_Flag_Has_First_Char = 0;		// Clear since we got second char
			RX_Flag_Got_Both_Chars = 1;	  // Indicate we got both chars
			UART3_Transmit(RX_Data2);			// Test - send back char 2
		}
		else {
			RX_Data1 = USART3->RDR; 			// Receive char 1 from putty, clear flag	
			RX_Flag_Has_First_Char = 1;		// Indicate we have first char now
			UART3_Transmit(RX_Data1);			// Test - send back char 1
		}			
	}
}


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	// PuTTY Configuration:
	// Connection Type: Serial, Serial line: COM3, Speed (Baud Rate): 115200
	
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

	//__HAL_RCC_GPIOC_CLK_ENABLE();       // Enable peripheral GPIOC clock 
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;    // Enable peripheral GPIOC clock (for LEDs)
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN; // Enable peripheral GPIOC clock (for USART3)
																																																																						
	// Set the LED pins to general-purpose output mode in the MODER register (write bits 01)
	GPIOC->MODER |=  (1<<12); // Set   bit 12 of MODER6 for PC6
	GPIOC->MODER &= ~(1<<13); // Clear bit 13 of MODER6 for PC6
	GPIOC->MODER |=  (1<<14); // Set   bit 14 of MODER7 for PC7
	GPIOC->MODER &= ~(1<<15); // Clear bit 15 of MODER7 for PC7
	GPIOC->MODER |=  (1<<16); // Set   bit 16 of MODER8 for PC8
	GPIOC->MODER &= ~(1<<17); // Clear bit 17 of MODER8 for PC8
	GPIOC->MODER |=  (1<<18); // Set   bit 18 of MODER9 for PC9
	GPIOC->MODER &= ~(1<<19); // Clear bit 19 of MODER9 for PC9
	
	// Set the LED pins to push-pull output type in the OTYPER register (write bit 0)
	GPIOC->OTYPER &= ~(1<<6); // Clear bit 6 of OTYPER6 for PC6
	GPIOC->OTYPER &= ~(1<<7); // Clear bit 7 of OTYPER7 for PC7	
	GPIOC->OTYPER &= ~(1<<8); // Clear bit 8 of OTYPER8 for PC8	
	GPIOC->OTYPER &= ~(1<<9); // Clear bit 9 of OTYPER9 for PC9	
	
	// Set the LED pins to low speed in the OSPEEDR register (write bits x0)
	GPIOC->OSPEEDR &= ~(1<<12); // Clear bit 12 of OSPEEDR6 for PC6 (don't care bit 13)
	GPIOC->OSPEEDR &= ~(1<<14); // Clear bit 14 of OSPEEDR7 for PC7 (don't care bit 15)
	GPIOC->OSPEEDR &= ~(1<<16); // Clear bit 16 of OSPEEDR8 for PC8 (don't care bit 17)
	GPIOC->OSPEEDR &= ~(1<<18); // Clear bit 18 of OSPEEDR9 for PC9 (don't care bit 19)
	
	// Set the LED pins to no pull-up/down resistors in the PUPDR register (write bits 00)
	GPIOC->PUPDR &= ~(1<<12); // Clear bit 12 of PUPDR6 for PC6
	GPIOC->PUPDR &= ~(1<<13); // Clear bit 13 of PUPDR6 for PC6
	GPIOC->PUPDR &= ~(1<<14); // Clear bit 14 of PUPDR7 for PC7
	GPIOC->PUPDR &= ~(1<<15); // Clear bit 15 of PUPDR7 for PC7
	GPIOC->PUPDR &= ~(1<<16); // Clear bit 16 of PUPDR8 for PC8
	GPIOC->PUPDR &= ~(1<<17); // Clear bit 17 of PUPDR8 for PC8
	GPIOC->PUPDR &= ~(1<<18); // Clear bit 18 of PUPDR9 for PC9
	GPIOC->PUPDR &= ~(1<<19); // Clear bit 19 of PUPDR9 for PC9
	
	// Using the chip datasheet, locate pins that connect to TX/RX signals on USART peripherals. 
	// --> USART3_TX = PC4 (connects to RX on serial converter)
	// --> USART3_RX = PC5 (connects to TX on serial converter)
	
	// Configure GPIOC pins PC4 and PC5 to alternative function mode
  GPIOC->MODER &= ~(GPIO_MODER_MODER4_Msk | GPIO_MODER_MODER5_Msk);
  GPIOC->MODER |=  (MODE_AF << GPIO_MODER_MODER4_Pos) | (MODE_AF << GPIO_MODER_MODER5_Pos);	
	
	// Configure USART3 TX/RX pins to alternate function mode (write bits 0001)
  GPIOC->AFR[0] &= ~(GPIO_AFRL_AFSEL4 | GPIO_AFRL_AFSEL5); 										 // Clear AF#
  GPIOC->AFR[0] |=  (1 << GPIO_AFRL_AFSEL4_Pos) | (1 << GPIO_AFRL_AFSEL5_Pos); // Select AF1
  
  // Set the Baud rate for communication to be 115200 bits/second.
  USART3->BRR = HAL_RCC_GetHCLKFreq() / 115200;
	
	// Enable and set the USART interrupt priority in the NVIC.
	NVIC_EnableIRQ(USART3_4_IRQn);      // Enable interrupt for USART3
	NVIC_SetPriority(USART3_4_IRQn, 0); // Set priority of USART3 to 0
	
  USART3->CR1 |= USART_CR1_TE | USART_CR1_RE; // Enable transmitter and receiver
  USART3->CR1 |= USART_CR1_RXNEIE; 						// Enable receive register not empty interrupt
  USART3->CR1 |= USART_CR1_UE; 								// Enable USART3

	// Print a command prompt such as “CMD?” when waiting for user input.
	char* cmdMsg = "CMD?\n"; 
	for (int i = 0; i < 5; i++) {
		HAL_Delay(10);             // 10ms delay	
		UART3_Transmit(cmdMsg[i]); // Send the message
	}


  /* Infinite loop */
  while (1)
  {
		// Implement some code in the infinite while loop of the main function that calls the character transmit
		//   function with a character constant. Feel free to simply loop with delay.
//		int red = 'r';
//		UART3_Transmit(red);
//		HAL_Delay(100);
			
		// --For first checkoff--
		// Check and wait on the USART status flag that indicates the receive (read) register is not empty.
//		int received_data = 0;
//		if (USART3->ISR & 0x20) {  
//			received_data = (uint8_t)(USART3->RDR); // Receive data, clear flag
//			UART3_Receive1(received_data); 					// Use received data to toggle appropriate LEDs
//			UART3_Transmit(received_data);
//		}
		
		// --For second checkoff--
		// Rewrite your main application to check and act on the flag and data variables you set in the interrupt handler.	
		int local_data1 = 0;
		int local_data2 = 0;
		
		//__disable_irq(); // -Actual- critical section; disable IRQ interrupts to prevent conflict (short section!)
		if (RX_Flag_Got_Both_Chars) {
			
			local_data1 = RX_Data1;     // Local copy of char 1
			local_data2 = RX_Data2;     // Local copy of char 2	
			RX_Flag_Got_Both_Chars = 0; //Clear flag
			//__enable_irq();             // Re-enable IRQ interrupts
			
			if (RX_Data1 == 0x72 || RX_Data1 == 0x62 ||RX_Data1 == 0x6F ||RX_Data1 == 0x67) { // If we have r,b,o or g
				if (RX_Data2 == 0x30 || RX_Data2 == 0x31 || RX_Data2 == 0x32) {                 // If we have 0,1 or 2
					
					UART3_Receive2(local_data1, local_data2);  // Use received data to toggle appropriate LEDs
				}	
				else {
					int count = 0;
					char* errorMsg = "Error. Only '0' '1' '2'.\n"; 	
					while (errorMsg[count] != '\0') { 
						HAL_Delay(50);	
						UART3_Transmit(errorMsg[count]); // Send error msg
						count++;
					}
					count = 0;
				}
			}
			else {
				int count = 0;
				char* errorMsg = "Error. Only 'r' 'b' 'o' 'g'.\n"; 	
				while (errorMsg[count] != '\0') { 
					HAL_Delay(50);	
					UART3_Transmit(errorMsg[count]); // Send error msg
					count++;
				}
				count = 0;
			}
			
		}	
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
