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
#define  NUM_OF_DATA_COLLECTING               1000

#define TIMER_MIN         0x0080
#define TIMER_MAX         0xFFFF

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
uint64_t time_measure;
uint64_t timestamp1;

_ADC_DATA Data;

uint8_t  uc_ADC_DMA_Transfer_Completed = TRUE;

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

//	// Set sample time - as slow as possible(480 cycles).
//	ADC3->SMPR2 |= ADC_SMPR2_SMP4_0 | ADC_SMPR2_SMP4_1 | ADC_SMPR2_SMP4_2;

//	// Start the ADC
//	ADC3->CR2 |= ADC_CR2_ADON;
//
//	// Trigger initial conversion
//	ADC3->CR2 |= ADC_CR2_SWSTART;
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

	if(uc_ADC_DMA_Transfer_Completed)//for beginning or resetting back to normal sampling.
	{
		// Clear the past relevant DMA setting.
		ADC3->CR2 = 0;

		// Set sample time - as slow as possible(480 cycles).
		ADC3->SMPR2 |= ADC_SMPR2_SMP4_0 | ADC_SMPR2_SMP4_1 | ADC_SMPR2_SMP4_2;

		// Set ADC to be continuous reading mode.
		ADC3->CR2 |= ADC_CR2_CONT;

		// Start the ADC
		ADC3->CR2 |= ADC_CR2_ADON;

		// Trigger initial conversion
		ADC3->CR2 |= ADC_CR2_SWSTART;

		uc_ADC_DMA_Transfer_Completed = FALSE; // finish setting and will not come here before another DMA have happened
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
		if(ADC_Sample_Time >= 30)
		{

			while(SysTick_Elapsed_MicroSeconds(TimeStamp)>ADC_Sample_Time) // Check if it exceeded the setting time.
			{
				TimeStamp = SysTick_Get_Timestamp();  // Update TimeStamp

				uS_Record[counter] = SysTick_Elapsed_MicroSeconds(TimeStampRecord); // Record number of elapsed microseconds.

				Data_Record[counter] = ADC_Analogue_Value; // Record the ADC value according to the elapsed microsecond.

				counter++; // Increase the counter for next data

				if(counter == NUM_OF_DATA_COLLECTING ) // Check if it has reached the number of samples
				{
					ADC_Sampling_Status = FALSE; // Close the loop meaning sampling finished

					counter = 0; // Reset the counter for next getdata command
				}
			}
		}
		else // Use DMA for faster collecting
		{
			ADC_Init_DMA_Transfer(Data_Record, NUM_OF_DATA_COLLECTING, ADC_Sample_Time);// Configure DMA ,TIMER,ADC and restart

			ADC_Sampling_Status = FALSE; // Set the status to avoid run DMA again
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
void ADC_Set_Filter(uint32_t SampleNumber)
{
	ul_ADC_Averaging = SampleNumber;
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

	Data.s_DataLen = NUM_OF_DATA_COLLECTING ; // Set the number of sample

	Data.uS_Data = uS_Record; // Set the start address of the data for Elapsed microseconds

	Data.pData = Data_Record; // Set the start address of the data for ADC value


	return Data;
}

/*********************************************
 * @brief ADC_Init_DMA_Transfer
 * Initiates DMA transfer (in normal mode) of data from
 * the ADC at the required  rate to RAM
 * Note: The ADC triggers the DMA transfer when the Timer expires.
 * @param uint16_t *p_DataPoints - pointer to the 32 bit data points
 * @param uint16_t uint16_t ul_Num_Points - Number of data points
 * @param uint64_t ull_Required_uS
 * @retval uint8_t Ok/Not ok.
 */
uint8_t ADC_Init_DMA_Transfer(uint32_t *p_DataPoints, uint16_t ul_Num_Points, uint32_t ull_Required_us)
{
	uint8_t Success = FALSE;

	// With our System Clock we need to divide down to the required update rate to load into timer counter
	int32_t sl_Clock_Count = (APB1_TIMER_CLOCK_FRQ / 1000000)  * ull_Required_us ;

	// Check within bounds of the 16-bit timer
	if((sl_Clock_Count > TIMER_MIN) && (sl_Clock_Count < TIMER_MAX))
	{
		// Ok to continue+
		Success = TRUE;

		// Clear relevant ADC setting before configuring DMA
		ADC3->CR2 = 0; ADC3->SMPR2 = 0;

		// Configure DMA channel / stream required.
		// Disable DMA2 Stream 1 while we configure it.
		DMA2_Stream1->CR &= ~DMA_SxCR_EN;
		// Wait until DMA Stream is disabled.
		while(DMA2_Stream1->CR & DMA_SxCR_EN);
		// Clear all interrupt flags to ensure no miss-firing when enabling.
		// Stream transfer complete interrupt flag.
		DMA2->LIFCR |= DMA_LIFCR_CTCIF1;
		// Stream half transfer complete interrupt flag.
		DMA2->LIFCR |= DMA_LIFCR_CHTIF1;
		// Stream transfer error interrupt flag
		DMA2->LIFCR |= DMA_LIFCR_CTEIF1;
		// Direct mode error interrupt flag
		DMA2->LIFCR |=DMA_LIFCR_CDMEIF1;
		// FIFO Error interrupt flag.
		DMA2->LIFCR |= DMA_LIFCR_CFEIF1;
		// Set data-width (MSize/PSize) to  be word (32 bit for Record_Data).
		DMA2_Stream1->CR |= DMA_SxCR_MSIZE_1 |DMA_SxCR_PSIZE_1;
		// Configure for memory increment
		DMA2_Stream1->CR |= DMA_SxCR_MINC;
		// Select Channel 2
		DMA2_Stream1->CR |= DMA_SxCR_CHSEL_1 ;
		DMA2_Stream1->CR &= ~(DMA_SxCR_CHSEL_0 | DMA_SxCR_CHSEL_2);
		// Set memory address (for  sending data - the waveform data).
		DMA2_Stream1->M0AR = (uint32_t)p_DataPoints;
		// Set the peripheral address (where to get the data)
		DMA2_Stream1->PAR = (uint32_t)&ADC3->DR; // DAC holding register.
		// Set the transfer direction (clearing unwanted bit)
		DMA2_Stream1->CR &= ~(DMA_SxCR_DIR_0 | DMA_SxCR_DIR_1);
		// Set the number of data transfer
		DMA2_Stream1->NDTR = ul_Num_Points;
		// Enable transfer complete interrupt - to trap overrun.
		DMA2_Stream1->CR |= DMA_SxCR_TCIE;
		// Enable the DMA interrupt in NVIC
		NVIC_EnableIRQ(DMA2_Stream1_IRQn);
		// Enable the channel - note it won't transfer until triggered by the ADC
		DMA2_Stream1->CR |= DMA_SxCR_EN;


		// Configure the timer(TIM3) and set the required update rate.
		// Note: This will prompt the ADC reading from data register  AND
		// initiate another DMA transfer into the RAM.
		// Enable the timer clock.
		RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
		// Set the auto-reload as calculated above
		TIM3->ARR = (uint16_t)sl_Clock_Count;
		// Enable TRGO event generation via master mode selection
		TIM3->CR2 |= TIM_CR2_MMS_1;
		// Enable the timer
		TIM3->CR1 |= TIM_CR1_CEN;


		// ADC Config
		//  Configuration for Timer / DMA usage.
		// Set DMA enable
		ADC3->CR2 |= ADC_CR2_DMA;
		// Set  ADC  Trigger Selection (Timer 3 TRGO)
		ADC3->CR2 |= ADC_CR2_EXTSEL_3;
		ADC3->CR2 &= ~(ADC_CR2_EXTSEL_0 | ADC_CR2_EXTSEL_1 | ADC_CR2_EXTSEL_2);
		// Enable the ADC
		ADC3->CR2 |= ADC_CR2_ADON;
		// Reinitialise DMA mode in ADC(for another DMA)
		ADC3->CR2 |= ADC_CR2_DDS;
		// Reset status ADC3(for another DMA)
		ADC3->SR = 0;
		// Enable the channel trigger - this kicks things off.
		ADC3->CR2 |= ADC_CR2_EXTEN_0;
		// Reference for time measuring
		timestamp1 = SysTick_Get_Timestamp();


	}
	return Success;
}

void DMA2_Stream1_IRQHandler(void)
{
	// Check how many microseconds has the whole process taken
	time_measure = SysTick_Elapsed_MicroSeconds(timestamp1);

	// Simply ack the interrupt
	DMA2->LIFCR |= DMA_LIFCR_CTCIF1;

	//Set the under DMA flag TRUE for setting to be normal continuous reading
	uc_ADC_DMA_Transfer_Completed = TRUE;
}


