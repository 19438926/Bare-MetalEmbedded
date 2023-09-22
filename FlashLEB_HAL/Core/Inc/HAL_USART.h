/**************************************************************************************************************
 *                  File Name       :USART.h
 *                  Overview        :USART functionality.
 *                  Authors         :Wenze Pan
 **************************************************************************************************************
 *                  Copyright @ As Applicable
 **************************************************************************************************************/

#ifndef HAL_USART_H_
#define HAL_USART_H_

#include "stm32f4xx_hal.h"
/***************************************************/
/* Definitions required by this module */

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

void USART_Process(UART_HandleTypeDef *uart , DMA_HandleTypeDef *rdma ,  DMA_HandleTypeDef  *tdma );
void USART1_Clear_Rx(void);

_Rx_DATA USART_Fetch_Rx ();
void USART_Clear_Rx(void);
uint8_t USART_Request_Tx(char *p_TxDataRequested , uint32_t TxDataCountRequested);


#endif /* USART_H_ */
