/**************************************************************************************************************
 *                  File Name       :HAL_DAC.c
 *                  Overview        :DAC functionality.
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
#include "HAL_DAC.h"
#include"HAL_GlobalDef.h"
#include "stm32f4xx_hal.h"

/****************************************************/
/*Local only definitions */
#define END_BAND_SIZE     250

/***************************/
/* Enumerations */

/**************************/
/*Structure types */

/*********************************************/
/* Global variable references */

/*********************************************/
/* Local only variable declaration */
uint64_t ul_Set_Output_Scalar;


/**************************/
/* Local only function prototypes */

/*********************************************
 * @brief DAC_Set_Output_x100
 * Sets the 12 bit DAC output according to new demand(demand X 100 supplier).
 * @param uint32_t us_New_Demand_x100
 * @retval None
 */
void DAC_Set_Output_x100(uint32_t us_New_Demand_x100, DAC_HandleTypeDef *dac)
{
	// Set the range of value voltage
	ul_Set_Output_Scalar = 0xFFF - (END_BAND_SIZE * 2);

	// Set the DAC outpiut value.
	HAL_DAC_SetValue(dac, DAC1_CHANNEL_1, DAC_ALIGN_12B_R, END_BAND_SIZE + ((ul_Set_Output_Scalar * us_New_Demand_x100)/10000));

	// Trigger the DAC output
	HAL_DAC_Start(dac, DAC1_CHANNEL_1);

}
