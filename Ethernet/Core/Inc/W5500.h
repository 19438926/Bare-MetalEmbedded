/**************************************************************************************************************
 *                  File Name       :SPI.h
 *                  Overview        :SPI wirte and read functionality.
 *                  Authors         :Wenze Pan
 **************************************************************************************************************
 *                  Copyright @ As Applicable
 **************************************************************************************************************/

#ifndef INC_W5500_H_
#define INC_W5500_H_

#include "stm32f1xx_hal.h"
#include "main.h"

uint8_t W5500_Read(uint8_t number);
void W5500_Write();
void set_spi(SPI_HandleTypeDef* spi);


#endif /* INC_W5500_H_ */
