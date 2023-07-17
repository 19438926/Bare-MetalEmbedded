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

/***************************************************/
/* Types used by this module */

/**************************************************/
/* Externally available variable */

/*************************************************/
/*Externally available functions */
void USART_Init(uint32_t ul_BaudRate);
void USART_Process(void);


#endif /* USART_H_ */
