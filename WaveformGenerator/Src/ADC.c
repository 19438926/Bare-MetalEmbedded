/**************************************************************************************************************
 *                  File Name       :ADC.c
 *                  Overview        :ADC functionality.
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
#include "ADC.h"
#include"GlobalDefs.h"

/****************************************************/
/*Local only definitions */

/***************************/
/* Enumerations */

/**************************/
/*Structure types */

/*********************************************/
/* Global variable references */

/*********************************************/
/* Local only variable declaration */
float FilteredTemerature = 25.0;
float RawTemperature;
uint32_t ADC_Temperature_Output;
uint8_t uc_New_Temp_Data_Ready= FALSE;

float fADC;
uint32_t ADC_Analogue_Value;
uint32_t ul_ADC_Averaging = 10;

/**************************/
/* Local only function prototypes */



/*********************************************
 * @brief ADC_Init
 * Performs all ADC Initialisation.
 * @param None
 * @retval None
 */
void ADC_Init(void)
{
	///////////////////////////////////
	// Enable temperature reading
	// Enable ADC1 clock
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

	// ADC Prescalar / 4
	ADC->CCR |= ADC_CCR_ADCPRE_0 | ADC_CCR_ADCPRE_1;

	// Enable the Temperature Sensor Channel
	ADC->CCR |= ADC_CCR_TSVREFE;

	// Channel selection (In18 for Temperature) for sequence 0.
	ADC1->SQR3 |= ADC_SQR3_SQ1_1 | ADC_SQR3_SQ1_4;

	// Set sample time - as slow as possible (480 cycles).
	ADC1->SMPR1 |= ADC_SMPR1_SMP18_0 | ADC_SMPR1_SMP18_1 | ADC_SMPR1_SMP18_2;

	// Enable ADC Interrupt upon conversion completed.
	ADC1->CR1 |= ADC_CR1_EOCIE;

	// Enable Interrupt
	NVIC_EnableIRQ(ADC_IRQn);

	// Enable the ADC
	ADC1->CR2 |= ADC_CR2_ADON;

	// Trigger conversion start
	ADC1->CR2 |= ADC_CR2_SWSTART;

	/////////////////////////////////////
	// Initialise the ADC Input reading
	// GPIO Config
	// Enable the GPIOF clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;

	// Configure PF6 as an analogue input.
	GPIOF->MODER |= (GPIO_MODER_MODER6_0 | GPIO_MODER_MODER6_1);

	// ADC Config
	// Enable the ADC3 clock
	RCC->APB2ENR |= RCC_APB2ENR_ADC3EN;

	// Configure the ADC3 number of channels to convert
	ADC3->SQR1 &= ~(ADC_SQR1_L_0 | ADC_SQR1_L_1 | ADC_SQR1_L_2 | ADC_SQR1_L_3);

	// Configure what channel's to be read: IN4
	ADC3->SQR3 |= ADC_SQR3_SQ1_2;

	// Set sample time - as slow as possible(480 cycles).
	ADC3->SMPR2 |= ADC_SMPR2_SMP4_0 | ADC_SMPR2_SMP4_1 | ADC_SMPR2_SMP4_2;

	// Set for continuous readings
	ADC3->CR2 |= ADC_CR2_CONT;

	// Start the ADC
	ADC3->CR2 |= ADC_CR2_ADON;

	// Trigger initial conversion
	ADC3->CR2 |= ADC_CR2_SWSTART;
}

/*********************************************
 * @brief ADC_IRQHandler
 * Handles ADC interrupts
 * @param None
 * @retval None
 */
void ADC_IRQHandler(void)
{
	// Record the new data.
	ADC_Temperature_Output = ADC1->DR;
	// Indicate update needed.
	uc_New_Temp_Data_Ready = TRUE;
}



/*********************************************
 * @brief ADC_Run
 * Performs all ADC functionality.
 * @param None
 * @retval None
 */
void ADC_Run(void)
{
	static uint8_t uc_InitDone;

	if(!uc_InitDone)
	{
		// Initialise
		uc_InitDone = TRUE;
		ADC_Init();
	}

	// Update temperature from ADC1?
	if(uc_New_Temp_Data_Ready)
	{
		uc_New_Temp_Data_Ready = FALSE;
		// Trigger next conversion
		ADC1->CR2 |= ADC_CR2_SWSTART;

		// Calculate new Temperature from data of last conversion.
		RawTemperature = (((float)((3*(float)ADC_Temperature_Output)/(float)4096) - 0.76) / 0.0025) + 25;
		// Also calculate a 'Rolling Average' (filtered value).
		FilteredTemerature += ((RawTemperature - FilteredTemerature)*0.0001); // Average over 10000 readings.
	}

	// Has the Analogue input conversion completed?
	if (ADC3->SR & ADC_SR_EOC)
	{
		// Simple Rolling Average of input value using floating point.
		fADC = (((fADC * (ul_ADC_Averaging - 1)) + ADC3->DR) / ul_ADC_Averaging);
		// Update our 32 bit value from floating point number.
		ADC_Analogue_Value = (uint32_t)fADC;
	}

}
