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
	TransmitionStart,
	WaitingTransmissionEnd
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
uint8_t uc_RxProcessingCompleted = FALSE;

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

	//Set the flag to indicate any previous Rx processing has completed
	//This prompts re-setting to Rx mode.
	uc_RxProcessingCompleted = TRUE;

	//Enable interrupts for USART1...
	NVIC_EnableIRQ(USART1_IRQn);
	//Enable Rx Interrupt within the USART
	USART1->CR1 |= USART_CR1_RXNEIE;
	//Enable Tx Interrupt within the USART
	USART1->CR1 |= USART_CR1_TCIE;
}

/*********************************************
 * @brief USART1_Clear_Rx
 * Indicates to this module that any Rx data received
 * can now be discarded and preparations made for next.
 * @param None
 * @retval None
 */
void USART1_Clear_Rx(void)
{
	//Simply set the flag to be picked up by the state machine.
	uc_RxProcessingCompleted= TRUE;
}

/*********************************************
 * @brief USART1_IRQHandler
 * Handles all USART1 Interrupts
 * @param None
 * @retval None
 */
void USART1_IRQHandler(void)
{
	//Has the RxNotEmpty interrupt fired?
	if(USART1->SR & USART_SR_RXNE)
	{
		//Byte received.
		//Check we're not overflowing array
		if(RxDataCount < sizeof(RxData))
		{
			// No overflow - load into array
			RxData[RxDataCount++] = USART1->DR;
		}
		else
		{
			// Avoiding overflowing array , simply 'read' the DR to continue
			USART1->DR;
			RxDataCount++;
		}
	}

	// Has the TxCompleted interrupt fired?
	if(USART1->SR & USART_SR_TC)
	{
		// Clear the interrupt
		USART1->SR &= ~USART_SR_TC;

		//Byte sending has completed.

		//Are there more bytes to send?
		if(TxDataCount > 0)
		{
			// Transmit next byte
			USART1->DR = *p_TxData;

			// Increment on to next byte
			p_TxData ++;
			TxDataCount--;
		}
	}
}

/*********************************************
 * @brief USART_Process
 * Handles processing of USART communications data (Tx/Rx)
 * @param None
 * @retval None
 */
void USART_Process(void)
{
	// Receive whole messages via interrupt then echo back the whole message also via interrupts.

	static uint64_t ull_Timestamp; // Used for detecting end of message(i.e. No more Rx for a period).
	static uint32_t ul_RxCountLT; // Used for tracking how many nytes were received via IRQ last time we checked

	//Action here depends on current state of port
	switch(USART_State)
	{
	case RxIdle:
		// Has any previously received message been dealt with.
		// Is it time to re-initialise and wait for another Rx message?
		if ( uc_RxProcessingCompleted == TRUE)
		{

			// Clear the indicator
			uc_RxProcessingCompleted = FALSE;

			// Clear down the Rx counter, ready for a new message
			RxDataCount = 0;
		}
		else
		{
			// Has the interrupt started receiving characters / bytes?
			if(RxDataCount > 0)
			{
				// Message is incoming via interrupts

				// Note how many bytes received so far
				ul_RxCountLT = RxDataCount;

				// Move to next state to detect end of message.
				USART_State = Receiving ;

				// Note the time that we can tell when the bytes stop coming in.
				ull_Timestamp = SysTick_Get_Timestamp();
			}
		}
		break;

	case Receiving:
		// Have we received any more characters via IRQ?
		if ( RxDataCount > ul_RxCountLT)
		{
			// Update local copy of count
			ul_RxCountLT = RxDataCount ;

			// As more data coming in , note time to detect when its ended.
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

			}
		}
		break;

	case RxOverflow:
		// This state cannot be hit as the interrupt continues to count
		// bytes received but won't load into our array past the end.
		// Any overflow bytes are ignored at the interrupt level.
		break;

	case RxMsgCompleted:
		// Process the 'complete' message that has been received...

		//Echo complete received message back to sender for now
		p_TxData = RxData;
		TxDataCount = RxDataCount;
		if ( TxDataCount > sizeof(RxData))
		{
			TxDataCount = sizeof(RxData);
		}
		USART_State = TransmitionStart;

		// Recurse to start transmission now
		USART_Process();
		break;

	case TransmitionStart:
		// Here we load the first byte to be transmitted and let the transmission complete
		// interrupt handle sending all the required subsequent bytes.
		USART1->DR = *p_TxData;

		// Increment on to next character / byte for the interrupt to send once this first
		// one has 'left the building'.
		p_TxData ++;
		// And decrement the Tx count.
		TxDataCount --;

		// Now wait for the transmission to end.
		USART_State = WaitingTransmissionEnd;
		break;

	case  WaitingTransmissionEnd:
		// Has the transmission completed via interrupt?
		if (TxDataCount == 0)
		{
			// As there is no real processing, the processing of the incoming command is
			// therefore completed once the echo back transmission has finished.
			// we therefore call USART1_Clear_Rx(); when finished here for now.
			USART1_Clear_Rx();

			// And set the state back to RxIdle to go again.
			USART_State = RxIdle;
		}
		break;
	}

}



