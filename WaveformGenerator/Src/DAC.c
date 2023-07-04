/**************************************************************************************************************
 *                  File Name       :DAC.c
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
#include "stm32f429xx.h"
#include "DAC.h"



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
 * @brief DAC_Init
 * Initialise DAC1 output on GPIOA pin 4 (PA4).
 * @param None
 * @retval None
 */
void DAC_Init()
{
	// Enable DAC clock, bit 29 on APB1ENR
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;

	//Enable DAC channel
	DAC->CR |= DAC_CR_EN1;
	//Enable trigger
	DAC->CR |= DAC_CR_TEN1;
	// Select software trigger as source.
	DAC->CR |= DAC_CR_TSEL1;

	// Set output value to 'off'
	DAC->DHR12R1 = 0;
	// Trigger SW Update of output.
	DAC->SWTRIGR |= DAC_SWTRIGR_SWTRIG1;

	// As we're using 12bit ,the scalar for DAC_Set_Output_x100 needs to be
	// configured accordingly.
	ul_Set_Output_Scalar = 0xFFF - (END_BAND_SIZE * 2);

}


/*********************************************
 * @brief DAC_Set_Output_x100
 * Sets the 12 bit DAC output according to new demand(demand X 100 supplier).
 * @param uint32_t us_New_Demand_x100
 * @retval None
 */
void DAC_Set_Output_x100(uint32_t us_New_Demand_x100)
{
	// Set new output demand.
	DAC->DHR12R1 = END_BAND_SIZE + ((ul_Set_Output_Scalar * us_New_Demand_x100)/10000);
	// Trigger update of output
	DAC->SWTRIGR |= DAC_SWTRIGR_SWTRIG1;

}




