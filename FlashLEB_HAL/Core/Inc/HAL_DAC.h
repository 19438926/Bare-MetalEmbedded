/**************************************************************************************************************
 *                  File Name       :HAL_DAC.h
 *                  Overview        :DAC functionality.
 *                  Authors         :Wenze Pan
 **************************************************************************************************************
 *                  Copyright @ As Applicable
 **************************************************************************************************************/

#ifndef DAC_H_
#define DAC_H_
#include "stm32f4xx_hal.h"
/***************************************************/
/* Definitions required by this module */

/***************************************************/
/* Types used by this module */

/**************************************************/
/* Externally available variable */

/*************************************************/
/*Externally available functions */
void DAC_Init(void);
void DAC_Set_Output_x100(uint32_t us_New_Demand_x100,DAC_HandleTypeDef *dac);
//uint16_t DAC_Get_Output_For_Demand(uint16_t us_Required_Demand);
//uint8_t DAC_Init_DMA_Transfer(uint16_t *p_DataPoints, uint16_t ul_Num_Points, uint64_t ull_Period_us);


#endif /* DAC_H_ */

