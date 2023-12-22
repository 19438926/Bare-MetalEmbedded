/**************************************************************************************************************
 *                  File Name       :I3g4250d_Gyro.h
 *                  Overview        :Gyro functionality.
 *                  Authors         :Wenze Pan
 **************************************************************************************************************
 *                  Copyright @ As Applicable
 **************************************************************************************************************/

#ifndef INC_I3G4250D_GYRO_H_
#define INC_I3G4250D_GYRO_H_

#include "stm32f4xx_hal.h"

void I3g4250d_Init(SPI_HandleTypeDef spi_handle);
void I3g4250d_Loop();

#endif /* INC_I3G4250D_GYRO_H_ */
