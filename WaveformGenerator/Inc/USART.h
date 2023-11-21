/**************************************************************************************************************
 *                  File Name       :USART.h
 *                  Overview        :USART functionality.
 *                  Authors         :Wenze Pan
 **************************************************************************************************************
 *                  Copyright @ As Applicable
 **************************************************************************************************************/

#ifndef USART_H_
#define USART_H_
/***************************************************/
/* Definitions required by this module */

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

/***************************************************/
/* Types used by this module */
typedef struct
{
	uint16_t s_DataLen; // Number of bytes in data
	char *pData;        // Data pointer

}_Rx_DATA;

/**************************************************/
/* Externally available variable */

/*************************************************/
/*Externally available functions */
void USART_Init(uint32_t ul_BaudRate);
void USART_Process(void);
void USART1_IRQHandler(void);

_Rx_DATA USART_Fetch_Rx ();
void USART1_Clear_Rx(void);
uint8_t USART_Request_Tx(char *p_TxDataRequested , uint32_t TxDataCountRequested);
eUSART_STATE Get_USART_Status();


#endif /* USART_H_ */
