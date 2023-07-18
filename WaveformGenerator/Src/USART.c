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

#include "SysTick.h"
#include "GlobalDefs.h"
#include  "USART.h"



/****************************************************/
/*Local only definitions */


/***************************/
/* Enumerations */
typedef enum
{
	RxIdle = 0,
	Receiving,
	RxOverflow,
	RxMsgCompleted,
	Transmitting,
}eUSART_STATE;

/**************************/
/*Structure types */

/*********************************************/
/* Global variable references */

/*********************************************/
/* Local only variable declaration */
eUSART_STATE USART_State = RxIdle;
uint8_t RxData[64];
uint32_t RxDataCount = 0;
uint8_t *p_TxData;
uint32_t TxDataCount;

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
	// Receive whole messages before processing them.
	static uint64_t ull_Timestamp;//Used for detecting end of message(i.e. No more Rx for a period).

	//Action here depends on current state of port
	switch(USART_State)
	{
	case RxIdle:
		//Default state, waiting for a message to start coming in.

		//Has the first byte of a message been received?
		if(USART1->SR & USART_SR_RXNE)
		{
			//Set nest state
			USART_State = Receiving;

			//Reset the Rx count for a new incoming message.
			RxDataCount = 0;

			//Recurse to take next action now.
			USART_Process();
		}
		break;

	case Receiving:
		// If there's a new byte waiting, save it to our array
		if(USART1->SR & USART_SR_RXNE)
		{
			//Save the byte from the Data Register
			RxData[RxDataCount++] = USART1->DR;

			//Check to avoid over-running our array if message is too long.
			if(RxDataCount >= sizeof(RxData))
			{
				USART_State = RxOverflow;
			}

			//Note the time so we can tell when bytes stop coming in.
			ull_Timestamp = SysTick_Get_Timestamp();
		}
		else
		{
			//Has enough time elapsed to denote the end of message
			//This should really be calculated from baud rate
			if(SysTick_Elapsed_MicroSeconds(ull_Timestamp) > 500)//Each char is about 100us at 115200 baud rate.
			{
				//Message reception completed.
				USART_State = RxMsgCompleted;

				//Here again we can recurse to process this immediately.
				USART_Process();

			}
		}
		break;

	case RxOverflow:
		// Incoming message is too long.
		// Keep accepting but ignoring the bytes until finished.
		if(USART1->SR & USART_SR_RXNE)
		{
			//Access the register to 'ack' which clears theRXNE, but ignore the byte
			USART1->DR;

			// Note the time so we can tell when bytes stop coming in.
			ull_Timestamp = SysTick_Get_Timestamp();
		}
		else
		{
			//Has enough time elapsed to denote the end of message
			//This should really be calculated from baud rate
			if(SysTick_Elapsed_MicroSeconds(ull_Timestamp)>500)//Each char is about 100us at 115200 baud rate.
			{
				//Message reception completed.
				USART_State = RxMsgCompleted;
			}

		}
		break;

	case RxMsgCompleted:
		// Process the 'complete' message that has been received...

		//Echo complete received message back to sender for now
		p_TxData = RxData;
		TxDataCount = RxDataCount;
		USART_State = Transmitting;

		// Recurse to start transmission now
		USART_Process();
		break;

	case Transmitting:
		// Has the previous byte transmission completed?
		if(USART1->SR & USART_SR_TC)
		{
			//Transmit the next byte / char
			USART1->DR = *p_TxData;

			//Increment on to next character / byte
			p_TxData ++;

			//All bytes transmitted?
			if(--TxDataCount == 0)
			{
				//Transmission completed.
				USART_State = RxIdle;
			}

		}
		break;
	}

}


