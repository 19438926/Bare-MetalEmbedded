/**************************************************************************************************************
 *                  File Name       :USART.c
 *                  Overview        :USART functionality.
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
#include "stm32f429xx.h"
#include "math.h"

#include "GlobalDefs.h"
#include  "USART.h"



/****************************************************/
/*Local only definitions */
#define END_BAND_SIZE     250

/***************************/
/* Enumerations */

/**************************/
/*Structure types */

/*********************************************/
/* Global variable references */

/*********************************************/
/* Local only variable declaration */

/*********************************************
 * @brief USART_Init
 * Initialise USART1 on PA9 and PA10.
 * @param None
 * @retval None
 */
void USART_Init(uint32_t ul_BaudRate)
{
	//Enable clock for USART1
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN; // Clock is at 72Mhz

	// Baud rate calculation
	// USARTDivider = Fclk / (8 x (2 - Over8) x BaudRate)
	// USARTDivider = 72000000 / (8 x (2 - Over8) x BaudRate)
	// Then take fractional part * 16 and whole parts as mantissa
	// USART1->BRR = (Mantissa << 4) | Fractional
	float USARTDivider = (float)SYS_CLOCK_FRQ / (8 * 2 * ul_BaudRate);
	uint32_t Mantissa = floor(USARTDivider);
	uint8_t uc_Fract = (uint8_t)((USARTDivider - Mantissa)*16);
	USART1->BRR = (Mantissa << 4) | uc_Fract;

	//Hardware flow control disabled by default.
	//8 bit , 1 start bit 1 stop bit , no parity are also defaults.

	//CR1 - Enable USART1
	USART1->CR1 |= USART_CR1_UE;

	//Enable Tx and Rx
	USART1->CR1 |= USART_CR1_TE | USART_CR1_RE;
}
/*********************************************
 * @brief USART_Process
 * Handles processing of USART communications data (Tx/Rx)
 * @param None
 * @retval None
 */
void USART_Process(void)
{
	// Echo back 1 char at a time
	// Anything received?
	if (USART1->SR & USART_SR_RXNE)
	{
		//Transmit back
		USART1->DR = USART1->DR;
	}
}


