/**************************************************************************************************************
 *                  File Name       :flash.h
 *                  Overview        :flash read and write functionality.
 *                  Authors         :Wenze Pan
 **************************************************************************************************************
 *                  Copyright @ As Applicable
 **************************************************************************************************************/

#ifndef FLASH_H_
#define FLASH_H_

#include <stdio.h>


/***************************************************/
/* Definitions required by this module */

/***************************************************/
/* Types used by this module */

/**************************************************/
/* Externally available variable */

/*************************************************/
/*Externally available functions */
void read(uint32_t pageAddr , void* data,uint32_t length);
uint8_t write(uint32_t pageAddr , void* data,uint32_t length);

#endif /* INC_FLASH_H_ */
