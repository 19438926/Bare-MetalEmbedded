/**************************************************************************************************************
 *                  File Name       :UART.h
 *                  Overview        :UART functionality.
 *                  Authors         :Wenze Pan
 **************************************************************************************************************
 *                  Copyright @ As Applicable
 **************************************************************************************************************/

#ifndef UART_H_
#define UART_H_

/***************************************************/
/* Definitions required by this module */

/***************************/
/* Enumerations */

/***************************************************/
/* Types used by this module */

/**************************************************/
/* Externally available variable */

/*************************************************/
/*Externally available functions */
void UART_Init(uint32_t ul_BaudRate);
void Transmit(char* data,uint32_t length);

#endif /* UART_H_ */
