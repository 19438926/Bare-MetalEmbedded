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
char RxData[RX_BUFF_SIZE];
uint32_t RxDataCount = 0;
char *p_TxData;
uint32_t TxDataCount;
uint8_t uc_RxProcessingCompleted = FALSE;
uint8_t TransmissionRequired = FALSE;

_Rx_DATA Received_Data;


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
	float USARTDivider = (float)APB2_CLOCK_FRQ / (8 * 2 * ul_BaudRate);
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
	//USART1->CR1 |= USART_CR1_RXNEIE;
	//Enable Tx Interrupt within the USART
	//USART1->CR1 |= USART_CR1_TCIE;
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
 * @brief DMA2_Stream7_IRQHandler
 * Handles all DMA2_Stream7 Interrupts
 * Currently DMA transfer of last byte to USART Tx.
 * @param None
 * @retval None
 */
void DMA2_Stream7_IRQHandler (void)
{
	// DMA2 Tx Complete on USART1
	if(DMA2->HISR & DMA_HISR_TCIF7)
	{
		// Clear DMA interrupt flag
		DMA2->HIFCR |= DMA_HIFCR_CTCIF7;

		// Enable the USART Tx Complete interrupt to capture end of final byte transmission
		USART1->CR1 |= USART_CR1_TCIE;
	}
}

/*********************************************
 * @brief DMA2_Stream2_IRQHandler
 * Handles all DMA2_Stream7 Interrupts
 * Currently DMA USART Rx buffer filled.
 * @param None
 * @retval None
 */
void DMA2_Stream2_IRQHandler (void)
{
	// DMA Rx filled Rx buffer?
	if (DMA2->LISR & DMA_LISR_TCIF2)
	{
		// Simply ack the interrupt
		DMA2->LIFCR |= DMA_LIFCR_CTCIF2;
	}
}

/*********************************************
 * @brief USART1_IRQHandler
 * Handles all USART1 Interrupts
 * @param None
 * @retval None
 */
void USART1_IRQHandler(void)
{
	// Last byte sent after DMA transfer completed?
	if(USART1->SR & USART_SR_TC)
	{
		// Clear the interrupt
		USART1->SR &= ~USART_SR_TC;

		// Also disable the interrupt until needed again
		USART1->CR1 &= ~USART_CR1_TCIE;

		// Declare transmission completed by clearing num bytes to transmit
		TxDataCount = 0;
	}
}

/*********************************************
 * @brief Rx_Via_DMA
 * Configures and initiates Rx of data via DMA controller.
 * @param Dest/len : Source address / length of data transfer
 * @retval None
 */
void Rx_Via_DMA(uint32_t p_Dest , uint32_t len)
{
	// Disable DMA2 Stream 2 while we configure it.
	DMA2_Stream2->CR &= ~DMA_SxCR_EN;

	// Wait until DMA2 Stream2 is disabled.
	while(DMA2_Stream2->CR & DMA_SxCR_EN);

	// Clear all interrupt flags to ensure no miss-firing when enabling.
	// Stream transfer complete interrupt flag.
	DMA2->HIFCR |= DMA_HIFCR_CTCIF5;
	// Stream half transfer complete interrupt flag.
	DMA2->HIFCR |= DMA_HIFCR_CHTIF5;
	// Stream transfer error interrupt flag
	DMA2->HIFCR |= DMA_HIFCR_CTEIF5;
	// Direct mode error interrupt flag
	DMA2->HIFCR |=DMA_HIFCR_CDMEIF5;
	// FIFO Error interrupt flag.
	DMA2->HIFCR |= DMA_HIFCR_CFEIF5;

	// Set peripheral address - where to write the data received.
	DMA2_Stream2->PAR = (uint32_t)&USART1->DR;

	//Set destination address (for data coming from USART1 data register.)
	DMA2_Stream2->M0AR = p_Dest;

	// Set length - maximum length.
	DMA2_Stream2->NDTR = len;

	// Select stream/channel (Stream 2, channel 4 = USART1 Rx).
	DMA2_Stream2->CR = DMA_SxCR_CHSEL_2;

	// Configure for memory increment
	DMA2_Stream2->CR |= DMA_SxCR_MINC;

	// Configure transfer direction (clear bit 0 and 1).
	DMA2_Stream2->CR &= ~(DMA_SxCR_DIR_0 | DMA_SxCR_DIR_1);

	// Enable transfer complete interrupt - to trap overrun.
	DMA2_Stream2->CR |= DMA_SxCR_TCIE;

	// Enable direct mode and disable FIFO
	DMA2_Stream2->FCR = 0;

	// Enable DMA2 Stream
	DMA2_Stream2->CR |= DMA_SxCR_EN;

	// Configure USART to use DMA for Rx
	USART1->CR3 |= USART_CR3_DMAR;

	// Enable the DMA interrupt in NVIC
	NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/*********************************************
 * @brief Tx_Via_DMA
 * Configures and initiates Tx of data via DMA controller.
 * @param Src/len : Source address / length of data transfer
 * @retval None
 */
void Tx_Via_DMA(uint32_t p_Src,uint32_t len)
{
	// Disable DMA2 Stream 7 while we configure it.
	DMA2_Stream7->CR &= ~DMA_SxCR_EN;

	// Wait until DMA2 Stream7 is disabled.
	while(DMA2_Stream7->CR & DMA_SxCR_EN);

	// Clear all interrupt flags to ensure no miss-firing when enabling.
	// Stream transfer complete interrupt flag.
	DMA2->HIFCR |= DMA_HIFCR_CTCIF7;
	// Stream half transfer complete interrupt flag.
	DMA2->HIFCR |= DMA_HIFCR_CHTIF7;
	// Stream transfer error interrupt flag
	DMA2->HIFCR |= DMA_HIFCR_CTEIF7;
	// Direct mode error interrupt flag
	DMA2->HIFCR |=DMA_HIFCR_CDMEIF7;
	// FIFO Error interrupt flag.
	DMA2->HIFCR |= DMA_HIFCR_CFEIF7;

	// Set peripheral address - where to write the data received.
	DMA2_Stream7->PAR = (uint32_t)&USART1->DR;

	//Set destination address (for data coming from USART1 data register.)
	DMA2_Stream7->M0AR = p_Src;

	// Set length - maximum length.
	DMA2_Stream7->NDTR = len;

	// Select stream/channel (Stream 4, channel 4 = USART1 Rx).
	DMA2_Stream7->CR = DMA_SxCR_CHSEL_2;

	// Configure for memory increment
	DMA2_Stream7->CR |= DMA_SxCR_MINC;

	// Configure transfer direction (set bit 0 ).
	DMA2_Stream7->CR |= DMA_SxCR_DIR_0 ;

	// Enable transfer complete interrupt - to trap overrun.
	DMA2_Stream7->CR |= DMA_SxCR_TCIE;

	// Enable direct mode and disable FIFO
	DMA2_Stream7->FCR = 0;

	// Enable DMA2 Stream
	DMA2_Stream7->CR |= DMA_SxCR_EN;

	// Configure USART to use DMA for Tx
	USART1->CR3 |= USART_CR3_DMAT;

	// Enable the DMA interrupt in NVIC
	NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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
		Received_Data.s_DataLen = RX_BUFF_SIZE - DMA2_Stream2->NDTR;
		Received_Data.pData = RxData;
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
		p_TxData = p_TxDataRequested;
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

/*********************************************
 * @brief USART_Process
 * Handles processing of USART communications data (Tx/Rx)
 * @param None
 * @retval None
 */
void USART_Process(void)
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
			ul_RxCountLT = RX_BUFF_SIZE;
		}

		// Configure for receiving via DMA
		Rx_Via_DMA((uint32_t)RxData , RX_BUFF_SIZE);
		USART_State = Receiving;
		break;

	case Receiving:
		// Have we received any more characters via DMA?
		if(DMA2_Stream2->NDTR < ul_RxCountLT)
		{
			// Update local copy of count
			ul_RxCountLT = DMA2_Stream2->NDTR;

			// As more data coming in, note time to detect when it's ended.
			ull_Timestamp = SysTick_Get_Timestamp();
		}
		// Check for overflow
		else if (DMA2_Stream2->NDTR == 0)
		{
			USART_State = RxOverflow;
		}
		// Include check for 'some bytes'
		else if (DMA2_Stream2->NDTR < RX_BUFF_SIZE)
		{
			if(SysTick_Elapsed_MicroSeconds(ull_Timestamp) > 500)
			{
				// Message reception completed.
				USART_State = RxMsgCompleted;

				// Set the next character as a zero to properly null terminate
				RxData[RX_BUFF_SIZE - DMA2_Stream2->NDTR] = 0 ;
			}
		}

		break;

	case RxOverflow:
		// If DMA USART Rx receives enough bytes to fill up the buffer, this is an
		// overflow condition, simply reset to continue receiving bytes from the
		// start of the buffer again.
		// Incomplete / corrupt messages will be discarded later.
		USART_State = RxIdle;
		break;

	case RxMsgCompleted:
		// This is now a wait state - hold on to the data until another module has
		// processed it and called "USART_Clear_Rx(void)"
		break;

	case TransmitionStart:
		// Trigger USART Tx via DMA...
		Tx_Via_DMA((uint32_t)p_TxData,TxDataCount);
		USART_State = WaitingTransmissionEnd;
		TransmissionRequired = FALSE;
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



