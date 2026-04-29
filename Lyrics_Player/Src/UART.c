/**************************************************************************************************************
 *                  File Name       :UART.c
 *                  Overview        :UART functionality.
 *                  Authors         :Wenze Pan
 **************************************************************************************************************
 *                  Copyright @ As Applicable
 **************************************************************************************************************
 *
 * Further notes and any specific details go here.
 *
 **************************************************************************************************************/


/****************************************************/
/*Required Header Files */
#include <stdio.h>
#include "stm32f103xb.h"
#include "GlobalDefs.h"
#include <math.h>

/****************************************************/
/*Local only definitions */

/***************************/
/* Enumerations */

/**************************/
/*Structure types */

/*********************************************/
/* Global variable references */

/*********************************************/
/* Local only variable declaration */
uint32_t Tr_len;
char* Data;
uint32_t Uart_index=0;


/*********************************************
 * @brief UART_Init
 * Initialise UART1
 * @param uint32_t baudrate
 * @retval None
 */
void UART_Init(uint32_t ul_BaudRate)
{
	//Enable clock for USART1
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN; // Clock is at 72Mhz

	// Baud rate calculation
	// USARTDivider = Fclk / (8 x (2 - Over8) x BaudRate)
	// USARTDivider = 72000000 / (8 x (2 - Over8) x BaudRate)
	// Then take fractional part * 16 and whole parts as mantissa
	// USART1->BRR = (Mantissa << 4) | Fractional
	float USARTDivider = (float)APB2_CLOCK_FRQ / (8 * 2 * ul_BaudRate);
	uint32_t Mantissa = floor(USARTDivider);
	uint8_t uc_Fract = (uint8_t)((USARTDivider - Mantissa)*16);
	USART1->BRR = (Mantissa << 4) | uc_Fract;

	//Hardware flow control disabled by default.
	//8 bit , 1 start bit 1 stop bit , no parity are also defaults.
	//CR1 - Enable USART1
	USART1->CR1 |= USART_CR1_UE;

	//Enable Tx and Rx
	USART1->CR1 |= USART_CR1_TE ;

	//Enable interrupts for USART1...
	NVIC_EnableIRQ(USART1_IRQn);

	//Enable Tx Interrupt within the USART
	USART1->CR1 |= USART_CR1_TCIE;
}

/*********************************************
 * @brief USART1_IRQHandler
 * Handles all USART1 Interrupts
 * @param None
 * @retval None
 */
void USART1_IRQHandler(void)
{
	// Clear the interrupt
	USART1->SR &= ~USART_SR_TC;

	// check if index reach the length
	if(Uart_index<Tr_len)
	{
		// transmit next byte
		USART1->DR = Data[Uart_index];
		// increase the index
		Uart_index++;
	}
	else
	{
		// clear the index for next message
		Uart_index = 0;
	}
}

/*********************************************
 * @brief Transmit
 * Transmit data from uart
 * @param uint8_t* data, uint32_t length
 * @retval None
 */
void Transmit(char* data,uint32_t length)
{
	// no transmit if length is 0
	if(length==0)
	{
		return;
	}
	// set the length of the message and index and transmit the first byte ,let interrupt do the rest of the job
	Tr_len = length;
	Data = data;
	USART1->DR = Data[Uart_index];
	Uart_index++;
}




