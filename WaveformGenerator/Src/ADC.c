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
#include "SysTick.h"

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
float FilteredTemperature = 25.0;
float RawTemperature;
uint32_t ADC_Temperature_Output;
uint8_t uc_New_Temp_Data_Ready= FALSE;

float fADC;
uint32_t ADC_Analogue_Value;
uint32_t ul_ADC_Averaging = 10;

uint16_t ADC_Number_Filter;
uint16_t ADC_Sample_Time;
uint8_t  ADC_Sampling_Status = FALSE;
static uint16_t counter;
uint32_t uS_Record[1000];
uint32_t Data_Record[1000];

_ADC_DATA Data;

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
	static uint64_t TimeStamp ;
	static uint64_t TimeStampRecord;

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
		FilteredTemperature += ((RawTemperature - FilteredTemperature)*0.0001); // Average over 10000 readings.
	}

	// Has the Analogue input conversion completed?
	if (ADC3->SR & ADC_SR_EOC)
	{
		// Simple Rolling Average of input value using floating point.
		fADC = (((fADC * (ul_ADC_Averaging - 1)) + ADC3->DR) / ul_ADC_Averaging);
		// Update our 32 bit value from floating point number.
		ADC_Analogue_Value = (uint32_t)fADC;

	}

	//Sample mode for application
	if(ADC_Sampling_Status)
	{

		while(SysTick_Elapsed_MicroSeconds(TimeStamp)>ADC_Sample_Time) // Check if it exceeded the setting time.
		{
			TimeStamp = SysTick_Get_Timestamp();  // Update TimeStamp

			uS_Record[counter] = SysTick_Elapsed_MicroSeconds(TimeStampRecord); // Record number of elapsed microseconds.

			Data_Record[counter] = ADC_Analogue_Value; // Record the ADC value according to the elapsed microsecond.

			counter++; // Increase the counter for next data

			if(counter == ADC_Number_Filter ) // Check if it has reached the number of samples
			{
				ADC_Sampling_Status = FALSE; // Close the loop meaning sampling finished

				counter = 0; // Reset the counter for next getdata command
			}
		}

	}
	else
	{
		TimeStamp = SysTick_Get_Timestamp(); // Update TimeStamp

		TimeStampRecord = SysTick_Get_Timestamp(); // Update TimeStampRecord
	}

}


/*********************************************
 * @brief ADC_Set_Filter
 * Set the number of samples to 'average'.
 * @param SampleNumber
 * @retval None
 */
void ADC_Set_Filter(uint16_t SampleNumber)
{
	ADC_Number_Filter = SampleNumber;
}

/*********************************************
 * @brief ADC_Set_Time
 * Set the number of uS between samples.
 * @param SampleTime
 * @retval None
 */
void ADC_Set_Time(uint16_t SampleTime)
{
	ADC_Sample_Time = SampleTime;
}

/*********************************************
 * @brief ADC_Start
 * Start ADC sampling
 * @param None
 * @retval None
 */
void ADC_Start()
{
	ADC_Sampling_Status = TRUE ;
}

/*********************************************
 * @brief ADC_Get_Temperature
 * get temperature
 * @param None
 * @retval None
 */
uint32_t ADC_Get_Temperature()
{
	return ADC_Temperature_Output;
}

/*********************************************
 * @brief ADC_Get_Status
 * get status
 * @param None
 * @retval None
 */
uint8_t ADC_Get_Status()
{
	return ADC_Sampling_Status;
}

/*********************************************
 * @brief ADC_Get_Reading
 * get reading
 * @param None
 * @retval None
 */
uint32_t ADC_Get_Reading()
{
	return ADC_Analogue_Value;
}

/*********************************************
 * @brief ADC_Get_Sample_Time
 * get the uS
 * @param None
 * @retval None
 */
uint16_t ADC_Get_Sample_Time()
{
	return ADC_Sample_Time;
}

/*********************************************
 * @brief ADC_Fetch_Data
 * Returns ADC data - if any.
 * @param None
 * @retval _ADC_DATA - structure containing ADC details and data.
 */
_ADC_DATA ADC_Fetch_Data()
{

	Data.s_DataLen = ADC_Number_Filter ; // Set the number of sample

	Data.uS_Data = uS_Record; // Set the start address of the data for Elapsed microseconds

	Data.pData = Data_Record; // Set the start address of the data for ADC value


	return Data;
}



