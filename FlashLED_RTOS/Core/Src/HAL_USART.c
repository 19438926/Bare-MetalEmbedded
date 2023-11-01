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

#include "HAL_GlobalDef.h"
#include  "HAL_USART.h"
#include "stm32f4xx_hal.h"
#include "HAL_SysTick.h"


/****************************************************/
/*Local only definitions */
#define RX_BUFF_SIZE                      (MAX_CUSTOM_DATA_LENGTH * 6) + 13   // Up to 5 chars per number plus comma + length of the command.

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
uint8_t RxData[RX_BUFF_SIZE];
uint32_t RxDataCount = 0;
uint8_t *p_TxData;
uint32_t TxDataCount;
uint8_t uc_RxProcessingCompleted = TRUE;
uint8_t dmaValue;
uint64_t ull_Timestamp1;

uint8_t TransmissionRequired = FALSE;

_Rx_DATA Received_Data;


/*********************************************
 * @brief USART_Process
 * Handles processing of USART communications data (Tx/Rx)
 * @param None
 * @retval None
 */
void USART_Process(UART_HandleTypeDef *uart, DMA_HandleTypeDef *rdma,
		DMA_HandleTypeDef *tdma)
{
	// Receive whole messages via DMA then echo back the whole message also using DMA.

	static uint64_t ull_Timestamp; // Used for detecting end of message(i.e. No more Rx for a period).
	static uint32_t ul_RxCountLT; // Used for tracking how many nytes were received via DMA last time we checked

	// If there's a requirement to transmit a message, we can assume any received message is dealt with
	// and override with the transmission.
	if (TransmissionRequired)
	{
		USART_State = TransmitionStart;
	}

	//Action here depends on current state of port
	switch (USART_State) {
	case RxIdle:
		// Has any previously received message been dealt with.
		// Is it time to re-initialise and wait for another Rx message?
		if (uc_RxProcessingCompleted == TRUE)
		{

			// Clear the indicator
			uc_RxProcessingCompleted = FALSE;

			// Clear down the Rx counter, ready for a new message
			RxDataCount = 0;
			ul_RxCountLT = RX_BUFF_SIZE;
		}

		// Configure for receiving via DMA
		HAL_UART_Receive_DMA(uart,RxData, RX_BUFF_SIZE);
		USART_State = Receiving;
		break;

	case Receiving:
		// Have we received any more characters via DMA?
		if (rdma->Instance->NDTR < ul_RxCountLT)
		{
			// Update local copy of count
			ul_RxCountLT = rdma->Instance->NDTR;

			// As more data coming in, note time to detect when it's ended.
			ull_Timestamp = SysTick_Get_Timestamp();
		}
		// Check for overflow
		else if (rdma->Instance->NDTR == 0)
		{
			USART_State = RxIdle;

		}
		// Include check for 'some bytes'
		else if (rdma->Instance->NDTR < RX_BUFF_SIZE) {
			if (SysTick_Elapsed_MicroSeconds(ull_Timestamp) >1500)
			{
				 //ull_Timestamp1 = SysTick_Elapsed_MicroSeconds(ull_Timestamp);
				//HAL_UART_DMAStop(uart);
				// Message reception completed.
				USART_State = RxMsgCompleted;

				dmaValue = RX_BUFF_SIZE - rdma->Instance->NDTR;

				// Set the next character as a zero to properly null terminate
				RxData[RX_BUFF_SIZE - rdma->Instance->NDTR] = 0 ;
			}
		}
		else
		{
			// In case of the situation where the first character is received via DMA AFTER the condition on line 107 is tested and
			// BEFORE the condition on line 122 is tested, then we'll have a situation where the timer is 'old' and we'll assume
			// end of message prematurely (line 123), hence also seeing a large "ull_Timestamp1" when this happens.
			//
			// To avoid this, keep the time-stamp to current time while there are no characters coming in.
			ull_Timestamp = SysTick_Get_Timestamp();
		}


		break;

	case RxOverflow:
		// If DMA USART Rx receives enough bytes to fill up the buffer, this is an
		// overflow condition, simply reset to continue receiving bytes from the
		// start of the buffer again.
		// Incomplete / corrupt messages will be discarded later.
		USART_State = RxMsgCompleted;
		break;

	case RxMsgCompleted:
		// This is now a wait state - hold on to the data until another module has
		// processed it and called "USART_Clear_Rx(void)"


//		p_TxData = RxData;
//		TxDataCount =  RX_BUFF_SIZE - rdma->Instance->NDTR;
//		if ( TxDataCount > sizeof(RxData))
//		{
//			TxDataCount = sizeof(RxData);
//		}
//		USART_State = TransmitionStart;
//
//		// Recurse to start transmission now
//		USART_Process(uart,rdma,tdma);
		break;

	case TransmitionStart:
		// Trigger USART Tx via DMA...
		HAL_UART_Transmit_DMA(uart, p_TxData, TxDataCount);
		USART_State = WaitingTransmissionEnd;

		TransmissionRequired = FALSE;
		break;

	case WaitingTransmissionEnd:
		// Has the transmission completed via interrupt?
		if (TxDataCount == 0 )
		{
			HAL_UART_DMAStop(uart);
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


//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
////			HAL_UART_AbortTransmit_IT(huart);
//
//	      HAL_UART_DMAStop(huart);
//	     if(RxDataCount < sizeof(RxData))
//			{
//				// No overflow - load into array
//				HAL_UART_Receive_IT(huart, &RxData[++RxDataCount], 1);
//			}
//			else
//			{
//				// Avoiding overflowing array , simply 'read' the DR to continue
//				//USART1->DR;

//				RxDataCount++;
//			}
//
//}
/*********************************************
 * @brief USART1_IRQHandler
 * Handles all USART1 Interrupts
 * @param None
 * @retval None
 */
//void HAL_UART_TxCpltCallback (UART_HandleTypeDef *huart)
//{
//	        // Clear the interrupt
//			//USART1->SR &= ~USART_SR_TC;
//	//		HAL_UART_AbortTransmit_IT(huart);
//
//			HAL_UART_DMAStop(huart);
//			//Byte sending has completed.
//			TxDataCount = 0;
//			//Are there more bytes to send?
////			if(TxDataCount > 0)
////			{
////				// Transmit next byte
////				HAL_UART_Transmit_IT(huart,p_TxData, 1);
////
////				// Increment on to next byte
////				p_TxData ++;
////				TxDataCount--;
////			}
//
//}

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
 * @brief USART_Fetch_Rx
 * Returns Rx data - if any.
 * @param None
 * @retval _Rx_DATA - structure containing Rx details and data.
 */
_Rx_DATA USART_Fetch_Rx()
{
	// Is there a message to provide?
	if (USART_State == RxMsgCompleted)
	{
		// Message has been received.
		Received_Data.s_DataLen = RX_BUFF_SIZE - dmaValue;
		Received_Data.pData = (char *) RxData;
	}
	else
	{
		Received_Data.s_DataLen = 0 ;
	}
	return Received_Data;
}

/*********************************************
 * @brief USART_Request_Tx
 * Handles processing of USART communication data (Tx/Rx)
 * @param uint8_t*   pointer to data
 * 		uint32_t     Num bytes to transmit
 * @retval TRUE = Transmitting, FALSE = Can't transmit as already transmitting.
 */
uint8_t USART_Request_Tx(char *p_TxDataRequested , uint32_t TxDataCountRequested)
{
	uint8_t TransmissionAccepted = FALSE;
	switch( USART_State)
	{
	case RxIdle:
	case Receiving:
	case RxMsgCompleted:
		// Store the transmission details.
		p_TxData =(uint8_t *)p_TxDataRequested;
		TxDataCount = TxDataCountRequested;

		// Indicate to state machine below that a transmission is needed.
		TransmissionAccepted = TRUE;
		TransmissionRequired = TRUE;
		break;

	default:
		// Nothing to do here - can't transmit.
		break;
	}
	return TransmissionAccepted;

}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)

{
	GPIOG->ODR &= 0<<13;
	TxDataCount = 0;
	asm("nop");

}

