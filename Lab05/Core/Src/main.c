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
#include <stdio.h>

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

// Transmit data function (transmits a char)
void UART_Transmit_Char (char toSend){
	if (USART3->ISR & (1<<7)) {
			HAL_Delay(1);
			USART3->TDR = toSend;
		}	
}


// Transmit data function (transmits a char array)
void UART_Transmit_Array (char* array){
	int count = 0;	
	while(array[count] != 0){
		UART_Transmit_Char(array[count]);
		HAL_Delay(1);
		count++;
	}
}


// Initialize I2C for master/slave read/write requests
void Initialize_I2C_TXRX(int SADD, int NBYTES, int setReadWrite){
	UART_Transmit_Array("Initialize_I2C_TXRX\r\n");
	I2C2->CR2 &= ~(I2C_CR2_SADD_Msk   | 
								 I2C_CR2_NBYTES_Msk | 
						     I2C_CR2_RD_WRN_Msk);         // Clear slave adr, num bytes & rd/wr bit	
	
	I2C2->CR2 |= (SADD << 1) | (NBYTES << 16);	// Set slave address & transmit 1 byte
	
	if (setReadWrite == 0) {										// If setReadWrite == 0 -> write
		I2C2->CR2 &= ~I2C_CR2_RD_WRN; 						// Set RD_WRN bit to write
		UART_Transmit_Array("Set Write\r\n");
	}
	else if (setReadWrite == 1) { 					  	// If setReadWrite == 1 -> read
		I2C2->CR2 |=  I2C_CR2_RD_WRN; 						// Set RD_WRN bit to read
		UART_Transmit_Array("Set Read\r\n");
	}	
	
	I2C2->CR2 |= (1 << 13);											// Set I2C start bit
}


// Checks the TXIS and NAKF flags for I2C transmit
// Return int shows polling status:
// 0 - Error
// 1 - TXIS flag is set
// 2 - Keep polling
int Poll_I2C_TX_Status(){
	UART_Transmit_Array("Poll_I2C_TX_Status\r\n");

	//Check the TXIS Flag - The address frame completed successfully, and the peripheral
	// is requesting new data to be written into the transmit data (TXDR) register
	if((I2C2->ISR & (1 << 1)) == (1 << 1)){
		UART_Transmit_Array("Checked TXIS -- all good\r\n");
		return 1;  // Indicate TXIS set
	}
	
	//Check the NACKF Flag - This flag indicates that the slave device did not acknowledge the address frame.
	// There is likely a configuration issue; the current transaction has been aborted.
	else if((I2C2->ISR & (1 << 4)) == (1 << 4)){
		UART_Transmit_Array("Checked NACKF - wiring or config is wrong\r\n");
		I2C2->ICR |= (1 << 4);  // Clear the NACKF flag
		return 0;								// Indicate NACKF was set
	}
	return 2;									// Indicate keep polling
}


// Checks the RXNE and NAKF flags for I2C transmit
// Return int shows polling status:
// 0 - Error
// 1 - RXNE flag is set
// 2 - Keep polling
int Poll_I2C_RX_Status(){
	UART_Transmit_Array("Poll_I2C_RX_Status\r\n");
	
	//Poll on the Receive Data Register Not Empty (RXNE)
	if(I2C2->ISR & (1 << 2)){
		UART_Transmit_Array("Checked RXNE -- all good\r\n");
		return 1;  // Indicate RXNE set
	}
	//Poll on the Not Acknowledge Receive Flag (NACKF) 
	else if(I2C2->ISR & (1 << 4)){
		UART_Transmit_Array("Checked NACKF - there was an ack error\r\n");
		return 0;  // Indicate NACKF was set
	}
	return 2;    // Indicate keep polling
}


// Checks the TC flag if transfer completed successfully
// Return int shows polling status:
// 0 - Error
// 1 - TC flag is set
// 2 - Keep polling
int Poll_I2C_TC_Status(){
	UART_Transmit_Array("Poll_I2C_TC_Status\r\n");
	
	//Poll on the Transfer Complete flag (TC)
	if(I2C2->ISR & (1 << 6)){
		UART_Transmit_Array("Checked TC - transfer complete\r\n");
		return 1;  // Indicate TC set  
	}
	return 2;    // Indicate keep polling
}


// Initializes I2C to write address for X high/low bits register
// Initializes I2C for read (from gyro)
// Reads the X sense values from OUT_X_H and OUT_X_L registors
// Combines the high and low bits into 16-bit, then casts to short to give pos/neg
// Returns the full X value as a short
short I2C_Gyro_Read_X() {
	UART_Transmit_Array("\r\n\r\n>>>I2C_Gyro_Read_X\r\n");
	
	uint16_t received = 0;
	uint16_t xData = 0;
	
	// Initialize I2C write transmit
	int SADD = 0x69;      // SADD = slave address
	int NBYTES = 1;       // NBYTES = # of bytes to transmit
	int setReadWrite = 0; // setReadWrite = 0/1 for write/read, respectively
	int pollCheck = 2;    // 0=error, 1=all good, 2=keep trying
	Initialize_I2C_TXRX(SADD, NBYTES, setReadWrite);  // write
	
	while (pollCheck == 2 || pollCheck == 0){
		pollCheck = Poll_I2C_TX_Status();  // Get TX status		
		if(pollCheck == 0){                // If error
			Initialize_I2C_TXRX(SADD, NBYTES, setReadWrite);
		}
	}
	pollCheck = 2;
	
	// Write the address of the OUT_X_H register into the I2C transmit register (TXDR).
	I2C2->TXDR = 0x29;  // 0x29 = OUT_X_H address: high bits of X-axis angular rate data
	UART_Transmit_Array("0x29\r\n");
	
	// Wait for TC flag
	while(pollCheck == 2 || pollCheck == 0){
		pollCheck = Poll_I2C_TC_Status();
	} 
	
	// Read the high 8-bits of the X axis
	setReadWrite = 1;  // setReadWrite = 0/1 for write/read, respectively 
	Initialize_I2C_TXRX(SADD, NBYTES, setReadWrite);  // read
			
	pollCheck = 2;
	while(pollCheck == 2 || pollCheck == 0){
		pollCheck = Poll_I2C_RX_Status();  // Get RX status	
		if(pollCheck == 0){                // If error
			Initialize_I2C_TXRX(SADD, NBYTES, setReadWrite);
		}
	}
	
	// Receive 1st 8 bits (high X bits)
	received = I2C2->RXDR;    // (uint16_t)Get data from I2C
	xData |= (received << 8); // (uint16_t)Shift received bits into the first 8 bits
	received = 0;						  // Clear previously received
	UART_Transmit_Array("Got X first 8 bits\r\n");
	
	// Now get 2nd 8 bits (low X bits)
	setReadWrite = 0;
	Initialize_I2C_TXRX(SADD, NBYTES, setReadWrite);  // write
	
	while (pollCheck == 2 || pollCheck == 0){
		pollCheck = Poll_I2C_TX_Status();  // Get TX status		
		if(pollCheck == 0){                // If error
			Initialize_I2C_TXRX(SADD, NBYTES, setReadWrite);
		}
	}	
	pollCheck = 2;
	
	// Write the address of the OUT_X_L register into the I2C transmit register (TXDR).
	I2C2->TXDR = 0x28;  // 0x28 = OUT_X_L address: low bits of X-axis angular rate data
	UART_Transmit_Array("0x28\r\n");
	
	// Wait for TC flag
	while(pollCheck == 2 || pollCheck == 0){
		pollCheck = Poll_I2C_TC_Status();
	} 
	
	// Now read the low 8-bits of the X axis
	setReadWrite = 1;  // setReadWrite = 0/1 for write/read, respectively 
	Initialize_I2C_TXRX(SADD, NBYTES, setReadWrite);  // read
			
	pollCheck = 2;
	while(pollCheck == 2 || pollCheck == 0){
		pollCheck = Poll_I2C_RX_Status();  // Get RX status	
		if(pollCheck == 0){                // If error
			Initialize_I2C_TXRX(SADD, NBYTES, setReadWrite);
		}
	}
	
	// Receive 2st 8 bits (low X bits)
	received = I2C2->RXDR;          // (uint16_t)Get data from I2C
	xData |= received;              // (uint16_t)Put received low bits into the last 8 bits
	short xValue = (short)xData;    // Cast full X-axis value into a short (to get +/- value)
	
	// Test message 
	char array[50];
	int receivedData = (int)xValue;		
	sprintf(array, "%i", receivedData);
	UART_Transmit_Array("X-value:  ");
	UART_Transmit_Array(array);
	UART_Transmit_Array("\r\n");
	
	return xValue;
}


// Initializes I2C to write address for Y high/low bits register
// Initializes I2C for read (from gyro)
// Reads the Y sense values from OUT_Y_H and OUT_Y_L registors
// Combines the high and low bits into 16-bit, then casts to short to give pos/neg
// Returns the full Y value as a short 
short I2C_Gyro_Read_Y() {
	UART_Transmit_Array("\r\n\r\n>>>I2C_Gyro_Read_Y\r\n");
	
	uint16_t received = 0;
	uint16_t yData = 0;
	
	// Initialize I2C write transmit
	int SADD = 0x69;      // SADD = slave address
	int NBYTES = 1;       // NBYTES = # of bytes to transmit
	int setReadWrite = 0; // setReadWrite = 0/1 for write/read, respectively
	int pollCheck = 2;    // 0=error, 1=all good, 2=keep trying
	Initialize_I2C_TXRX(SADD, NBYTES, setReadWrite);  // write
	
	while (pollCheck == 2 || pollCheck == 0){
		pollCheck = Poll_I2C_TX_Status();  // Get TX status		
		if(pollCheck == 0){                // If error
			Initialize_I2C_TXRX(SADD, NBYTES, setReadWrite);
		}
	}
	pollCheck = 2;
	
	// Write the address of the OUT_Y_H register into the I2C transmit register (TXDR).
	I2C2->TXDR = 0x2B;  // 0x2B = OUT_Y_H address: high bits of Y-axis angular rate data
	UART_Transmit_Array("0x2B\r\n");
	
	// Wait for TC flag
	while(pollCheck == 2 || pollCheck == 0){
		pollCheck = Poll_I2C_TC_Status();
	} 
	
	// Read the high 8-bits of the Y axis
	setReadWrite = 1;  // setReadWrite = 0/1 for write/read, respectively 
	Initialize_I2C_TXRX(SADD, NBYTES, setReadWrite);  // read
			
	pollCheck = 2;
	while(pollCheck == 2 || pollCheck == 0){
		pollCheck = Poll_I2C_RX_Status();  // Get RX status	
		if(pollCheck == 0){                // If error
			Initialize_I2C_TXRX(SADD, NBYTES, setReadWrite);
		}
	}
	
	// Receive 1st 8 bits (high Y bits)
	received = I2C2->RXDR;   // Get data from I2C
	yData |= (received << 8); // Shift received bits into the first 8 bits
	received = 0;						 // Clear previously received
	UART_Transmit_Array("Got Y first 8 bits\r\n");
	
	// Now get 2nd 8 bits (low Y bits)
	setReadWrite = 0;
	Initialize_I2C_TXRX(SADD, NBYTES, setReadWrite);  // write
	
	while (pollCheck == 2 || pollCheck == 0){
		pollCheck = Poll_I2C_TX_Status();  // Get TX status		
		if(pollCheck == 0){                // If error
			Initialize_I2C_TXRX(SADD, NBYTES, setReadWrite);
		}
	}	
	pollCheck = 2;
	
	// Write the address of the OUT_Y_L register into the I2C transmit register (TXDR).
	I2C2->TXDR = 0x2A;  // 0x2A = OUT_Y_L address: low bits of Y-axis angular rate data
	UART_Transmit_Array("0x2A\r\n");
	
	// Wait for TC flag
	while(pollCheck == 2 || pollCheck == 0){
		pollCheck = Poll_I2C_TC_Status();
	} 
	
	// Now read the low 8-bits of the X axis
	setReadWrite = 1;  // setReadWrite = 0/1 for write/read, respectively 
	Initialize_I2C_TXRX(SADD, NBYTES, setReadWrite);  // read
			
	pollCheck = 2;
	while(pollCheck == 2 || pollCheck == 0){
		pollCheck = Poll_I2C_RX_Status();  // Get RX status	
		if(pollCheck == 0){                // If error
			Initialize_I2C_TXRX(SADD, NBYTES, setReadWrite);
		}
	}
	
	// Receive 2st 8 bits (low Y bits)
	received = I2C2->RXDR;         // Get data from I2C
	yData |= received;              // Put received low bits into the last 8 bits
	short yValue = (short)yData;   // Cast full Y-axis value into a short (to get +/- value)
	
	// Test message 
	char array[50];
	int receivedData = (int)yValue;		
	sprintf(array, "%i", receivedData);
	UART_Transmit_Array("Y-value:  ");
	UART_Transmit_Array(array);
	UART_Transmit_Array("\r\n");
	
	return yValue;
}


// Set the LEDs based on the gyro +/- X and Y axis values
void Gyro_Set_LEDs(short X, short Y) {
	// Check Y-axis
	if (Y > 0) {
		GPIOC->BSRR |= (1<<6);   // Set red LED (PC6) high
	}
	else if (Y < 0) {
		GPIOC->BSRR |= (1<<7);   // Set blue LED (PC7) high
	}
	
	// Check X-axis
	if (X > 0) {
		GPIOC->BSRR |= (1<<8);   // Set orange LED (PC8) high
	}
	else if (X < 0) {
		GPIOC->BSRR |= (1<<9);   // Set green LED (PC9) high
	}
}





/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
	 /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

	// Enable the GPIOB, GPIOC and I2C2 peripheral in the RCC.
	RCC->AHBENR  |= RCC_AHBENR_GPIOBEN;   // Enable peripheral GPIOB  clock
	RCC->AHBENR  |= RCC_AHBENR_GPIOCEN;   // Enable peripheral GPIOC  clock 
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;   // Enable peripheral I2C2   clock
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN; // Enable peripheral USART3 clock
	
	
	// - Setup for LEDs and UART (for testing) -
	
	
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
	
	// Configure GPIOC pins PC4 and PC5 to alternative function mode
  GPIOC->MODER &= ~(GPIO_MODER_MODER4_Msk | GPIO_MODER_MODER5_Msk);
  GPIOC->MODER |=  (MODE_AF << GPIO_MODER_MODER4_Pos) | (MODE_AF << GPIO_MODER_MODER5_Pos);	
	
	// Configure USART3 TX/RX pins to alternate function mode (write bits 0001)
  GPIOC->AFR[0] &= ~(GPIO_AFRL_AFSEL4 | GPIO_AFRL_AFSEL5); 										 // Clear AF#
  GPIOC->AFR[0] |=  (1 << GPIO_AFRL_AFSEL4_Pos) | (1 << GPIO_AFRL_AFSEL5_Pos); // Select AF1
  
  // Set the Baud rate for communication to be 115200 bits/second.
  USART3->BRR = HAL_RCC_GetHCLKFreq() / 115200;

  USART3->CR1 |= USART_CR1_TE | USART_CR1_RE; // Enable transmitter and receiver
  USART3->CR1 |= USART_CR1_UE; 								// Enable USART3
	
	
	// - Setup for I2C - 
	
	
	// Set PB11 to alt func mode, open-drain output type, and select I2C_SDA as its alt function.
	GPIOB->MODER  &= ~(1<<22);  // Clear bit 22 of MODER11  for PB11           (alt func: 10)
	GPIOB->MODER  |=  (1<<23);  // Set   bit 23 of MODER11  for PB11           (alt func: 10)
	GPIOB->OTYPER |=  (1<<11);  // Set   bit 11 of OTYPER11 for PB11           (open-drain: 1)
	GPIOB->AFR[1] |=  (1<<12);  // Set   bit 12 to select AF1 in AFRH for PB11 (alt func: 0001)
	GPIOB->AFR[1] &= ~(1<<13);  // Clear bit 13 to select AF1 in AFRH for PB11 (alt func: 0001)
	GPIOB->AFR[1] &= ~(1<<14);  // Clear bit 14 to select AF1 in AFRH for PB11 (alt func: 0001)
	GPIOB->AFR[1] &= ~(1<<15);  // Clear bit 15 to select AF1 in AFRH for PB11 (alt func: 0001)	
	
	// Set PB13 to alt func mode, open-drain output type, and select I2C_SCL as its alt func.
	GPIOB->MODER  &= ~(1<<26);  // Clear bit 26 of MODER13  for PB13           (alt func: 10)
	GPIOB->MODER  |=  (1<<27);  // Set   bit 27 of MODER13  for PB13           (alt func: 10)
	GPIOB->OTYPER |=  (1<<13);  // Set   bit 13 of OTYPER13 for PB13           (open-drain: 1)
	GPIOB->AFR[1] |=  (1<<20);  // Set   bit 20 to select AF5 in AFRH for PB13 (alt func: 0101)
	GPIOB->AFR[1] &= ~(1<<21);  // Clear bit 21 to select AF5 in AFRH for PB13 (alt func: 0101)
	GPIOB->AFR[1] |=  (1<<22);  // Set   bit 22 to select AF5 in AFRH for PB13 (alt func: 0101)
	GPIOB->AFR[1] &= ~(1<<23);  // Clear bit 23 to select AF5 in AFRH for PB13 (alt func: 0101)	
	
	// Set PB14 to ouput mode, push-pull output type, and initialize/set the pin high.
	GPIOB->MODER  |=  (1<<28);  // Set   bit 28 of MODER14  for PB14           (outp mode: 01)
	GPIOB->MODER  &= ~(1<<29);  // Clear bit 29 of MODER14  for PB14           (outp mode: 01)
	GPIOB->OTYPER &= ~(1<<14);  // Clear bit 14 of OTYPER14 for PB14           (push-pull: 0)
	GPIOB->BSRR   |=  (1<<14);  // Set pin PB14 high	                         (high: 1)	
	
	// Set PC0 to ouput mode, push-pull output type, and initialize/set the pin high.
	GPIOC->MODER  |=  (1<<0);   // Set   bit 0 of MODER0 for PC0               (outp mode: 01)
	GPIOC->MODER  &= ~(1<<1);   // Clear bit 1 of MODER0 for PC0               (outp mode: 01)
	GPIOC->OTYPER &= ~(1<<0);   // Clear bit 0 of OTYPER0 for PC0              (push-pull: 0)
	GPIOC->BSRR   |=  (1<<0);   // Set pin PC0 high	                           (high: 1)	
		
	// Set the parameters in the TIMINGR register to use 100 kHz standard-mode I2C.
	I2C2->TIMINGR |= (
			(1    << I2C_TIMINGR_PRESC_Pos) |
			(0x13 << I2C_TIMINGR_SCLL_Pos)  | 
			(0xF  << I2C_TIMINGR_SCLH_Pos)  |
			(0x2  << I2C_TIMINGR_SDADEL_Pos)|
			(0x4  << I2C_TIMINGR_SCLDEL_Pos));
	
	// Set to not autoend at end of I2C communication
	I2C2->CR2 &= ~I2C_CR2_AUTOEND;
	
	// Enable the I2C peripheral using the PE bit in the CR1 register.
	 I2C2->CR1 |= I2C_CR1_PE;   
	 
	 
	/* ---Lab 05 Checkoff 1--- */
	 
/* 
	// - I2C Write - 

	// Initialize I2C write transmit
	int SADD = 0x69;      // SADD = slave address
	int NBYTES = 1;       // NBYTES = # of bytes to transmit
	int setReadWrite = 0; // setReadWrite = 0/1 for write/read, respectively
	int pollCheck = 2;    // 0=error, 1=all good, 2=keep trying
	
	// Wait until either of the TXIS (TX register empty/ready) or NACKF (Slave not acknowledge) flags are set.
	// If the NACKF flag is set, the slave did not respond to the address frame (wiring or config error).
	// Continue if the TXIS flag is set.
	Initialize_I2C_TXRX(SADD, NBYTES, setReadWrite);
	
	while (pollCheck == 2 || pollCheck == 0){
		pollCheck = Poll_I2C_TX_Status();  // Get TX status
		
		if(pollCheck == 0){                // If error
			Initialize_I2C_TXRX(SADD, NBYTES, setReadWrite);
		}
	}
	UART_Transmit_Array("Write WHO_AM_I address and set TC flag\r\n");
	pollCheck = 2;
	
	// Write the address of the WHO_AM_I register into the I2C transmit register (TXDR).
	I2C2->TXDR = 0x0F;  // 0x0F = 15 = address of WHO_AM_I register
	
	// Wait for TC flag
	while(pollCheck == 2 || pollCheck == 0){
		pollCheck = Poll_I2C_TC_Status();
	}  
	 
	// - I2C Read -
				
	// Reload the CR2 register with the same parameters as before, but set the RD_WRN bit to 
	//   indicate a read operation & set start bit again for restart condition.
	char array[50];		 // array for msg sending
	setReadWrite = 1;  // setReadWrite = 0/1 for write/read, respectively 
	Initialize_I2C_TXRX(SADD, NBYTES, setReadWrite);
			
	pollCheck = 2;
	while(pollCheck == 2 || pollCheck == 0){
		pollCheck = Poll_I2C_RX_Status();  // Get RX status	
		if(pollCheck == 0){                // If error
			Initialize_I2C_TXRX(SADD, NBYTES, setReadWrite);
		}
	}
	
	int receivedData = I2C2->RXDR & 0xFF;		
	sprintf(array, "%i", receivedData);
	UART_Transmit_Array(array);
	UART_Transmit_Array("\r\n");
	if(receivedData == 0xD3){
		UART_Transmit_Array("Success! Received value 0xD3 from the WHO_AM_I register.\r\n");
	}
	UART_Transmit_Array("\r\n");
*/		
	
	/* ---Lab 05 Checkoff 2--- */
	 

	// Enable the X and Y sensing axes in the CTRL_REG1 register.	
	// Set the sensor into "normal or sleep-mode" using the PD bit in the CTRL_REG1 register.	
	// All other bits in the CTRL_REG1 register should be set to 0. These place the device in 
	//   the default low-speed mode.
	int SADD = 0x69;      // SADD = slave address (for the I3G4250D gyro)
	int NBYTES = 2;       // NBYTES = # of bytes to transmit
	int setReadWrite = 0; // setReadWrite = 0/1 for write/read, respectively
	int pollCheck = 2;    // 0=error, 1=all good, 2=keep polling
	
	Initialize_I2C_TXRX(SADD, NBYTES, setReadWrite);  // Initialize write
	
	while (pollCheck == 2 || pollCheck == 0){
		pollCheck = Poll_I2C_TX_Status();  // Get TX status	
		if(pollCheck == 0){                // If error
			Initialize_I2C_TXRX(SADD, NBYTES, setReadWrite);
		}
	}
	pollCheck = 2;
	UART_Transmit_Array("Write CTRL_REG1 and set TC flag\r\n");
	
	// 1st byte: Write the address of the CTRL_REG1 register into the I2C transmit register (TXDR).
	I2C2->TXDR = 0x20;           // 0x20 = address of CTRL_REG1 register
	UART_Transmit_Array("0x20 ");
	
	HAL_Delay(1);
	// Wait for TC flag
//	while(pollCheck == 2 || pollCheck == 0){pollCheck = Poll_I2C_TC_Status();}
	
	// 2nd byte: Enable X & Y sensing axis, enable normal/sleep mode
	I2C2->TXDR = 0x0B; 					// 0x0B = 00001011 = 11
	UART_Transmit_Array("0x0B \r\n\r\n");  
	
	// Wait for TC flag
	while(pollCheck == 2 || pollCheck == 0){pollCheck = Poll_I2C_TC_Status();}
	
	// Initialize the gyroscope sensor to read the X and Y axes.
	// Read and save the value of the X and Y axis data registers every 100 ms.
	// You will need to assemble the 16-bit measured value from the two data registers for each axis.	
	short X = 0;
	short Y = 0;
	while (1) {
		X = I2C_Gyro_Read_X();  // Read X sense axis and store its value
		Y = I2C_Gyro_Read_Y();  // Read Y sense axis and store its value
		HAL_Delay(100);         // wait 100ms		
		
		// Use the four LEDs to indicate whether each measured axis is positive or negative.
		// Because of measurement noise and to prevent the lghts from triggering due to small vibrations,
		//   set a minimum threshold before changing the active LED.
		// Design your application such that the LED nearest the direction of rotation lights up.
		// (E.g., when the board is rotated/tilted in the positive x-axis the orange LED turns on
		Gyro_Set_LEDs(X, Y);        // Set the LEDs according to +/-X & +/-Y
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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