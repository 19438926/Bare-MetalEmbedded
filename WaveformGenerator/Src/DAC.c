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
#include"GlobalDefs.h"


/****************************************************/
/*Local only definitions */
#define END_BAND_SIZE     250

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
uint64_t ul_Set_Output_Scalar;
uint8_t  uc_DAC_Under_DMA_Control = TRUE;

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

//	//Enable DAC channel
//	DAC->CR |= DAC_CR_EN1;
//	//Enable trigger
//	DAC->CR |= DAC_CR_TEN1;
//	// Select software trigger as source.
//	DAC->CR |= DAC_CR_TSEL1;
//
//	// Set output value to 'off'
//	DAC->DHR12R1 = 0;
//	// Trigger SW Update of output.
//	DAC->SWTRIGR |= DAC_SWTRIGR_SWTRIG1;

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
	// Determine if init for SW control is needed...
	if(uc_DAC_Under_DMA_Control ==TRUE)
	{
		// As output was being controlled by DMA, disabled DMA transfers.
		// Also disable related timer and DAC setting.
		DAC1->CR &= ~DAC_CR_DMAEN1;				// DAC not using DMA
		TIM4->CR1 &= ~TIM_CR1_CEN;				// Timer disabled
		DMA1_Stream5->CR &= ~DMA_SxCR_EN;		// DMA Disabled

		// Configure DAC for SW control
		//Enable DAC channel
		DAC->CR |= DAC_CR_EN1;
		//Enable trigger
		DAC->CR |= DAC_CR_TEN1;
		// Select software trigger as source.
		DAC->CR |= DAC_CR_TSEL1;

		// Only do the above once, each time switching over from DMA
		uc_DAC_Under_DMA_Control = FALSE;
	}

	// Set new output demand.
	DAC->DHR12R1 = END_BAND_SIZE + ((ul_Set_Output_Scalar * us_New_Demand_x100)/10000);
	// Trigger update of output
	DAC->SWTRIGR |= DAC_SWTRIGR_SWTRIG1;
}

/*********************************************
 * @brief DAC_Get_Output_For_Demand
 * Returns the required DAC low level value for a given demand
 * @param uint16_t us_Required_Demand
 * @retval uint16_t DAC register Value required
 */
uint16_t DAC_Get_Output_For_Demand(uint16_t us_Required_Demand)
{
	return (uint16_t)(END_BAND_SIZE + ((ul_Set_Output_Scalar * us_Required_Demand) / 10000));
}

/*********************************************
 * @brief DAC_Init_DMA_Transfer
 * Initiates DMA transfer (in circular mode, so continual) of data to
 * the DAC at the required frequency update rate to achieve a total
 * period as required.
 * Note: The DAC triggers the DMA transfer when the Timer expires.
 * @param uint16_t *p_DataPoints - pointer to the 16 bit data points
 * @param uint16_t uint16_t ul_Num_Points - Number of data points
 * @param uint64_t ull_Period_uS - Total period for waveform (all data points)
 * @retval uiny8_t Ok/Not ok.
 */
uint8_t DAC_Init_DMA_Transfer(uint16_t *p_DataPoints, uint16_t ul_Num_Points, uint64_t ull_Period_us)
{
	uint8_t Success = FALSE;
	// First work out what frequency we need to run the timer at (time per individual sample update)
	float f_Timer_Freq = 1.0/(((float)ull_Period_us / 1000000.0) / ul_Num_Points);
	// With our System Clock we need to divide down to the required update rate to load into timer counter
	int32_t sl_Clock_Count = SYS_CLOCK_FRQ / f_Timer_Freq;

	// Check within bounds of the 16-bit timer
	if((sl_Clock_Count > TIMER_MIN) && (sl_Clock_Count < TIMER_MAX))
	{
		// Ok to continue+
		Success = TRUE;
		uc_DAC_Under_DMA_Control = TRUE;


		// Configure DMA channel / stream required.
		// Enable clock for DMA1
		RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
		// Disable DMA1 Stream 5 while we configure it.
		DMA1_Stream5->CR &= ~DMA_SxCR_EN;
		// Wait until DMA Stream is disabled.
		while(DMA1_Stream5->CR & DMA_SxCR_EN);
		// Clear all interrupt flags to ensure no miss-firing when enabling.
		// Stream transfer complete interrupt flag.
		DMA1->HIFCR |= DMA_HIFCR_CTCIF5;
		// Stream half transfer complete interrupt flag.
		DMA1->HIFCR |= DMA_HIFCR_CHTIF5;
		// Stream transfer error interrupt flag
		DMA1->HIFCR |= DMA_HIFCR_CTEIF5;
		// Direct mode error interrupt flag
		DMA1->HIFCR |=DMA_HIFCR_CDMEIF5;
		// FIFO Error interrupt flag.
		DMA1->HIFCR |= DMA_HIFCR_CFEIF5;
		// Set data-width (MSize/PSize) to  be half-word (16 bit as 12 bit needed for DAC Values).
		DMA1_Stream5->CR |= DMA_SxCR_MSIZE_0 |DMA_SxCR_PSIZE_0;
		// Make sure other bits are cleared out for PSize and MSize.
		DMA1_Stream5->CR &= ~(DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1 );
		// Configure for memory increment
		DMA1_Stream5->CR |= DMA_SxCR_MINC;
		// Make the DMA work in a circular fashion
		DMA1_Stream5->CR |= DMA_SxCR_CIRC;
		// Select Channel 7
		DMA1_Stream5->CR |= DMA_SxCR_CHSEL_0 | DMA_SxCR_CHSEL_1 | DMA_SxCR_CHSEL_2;
		// Set source address (for data to be sent to DAC - the waveform data).
		DMA1_Stream5->M0AR = (uint32_t)p_DataPoints;
		// Set the destination address (where to send the data)
		DMA1_Stream5->PAR = (uint32_t)&DAC->DHR12R1; // DAC holding register.
		// Set the transfer direction (clearing unwanted bit)
		DMA1_Stream5->CR |= DMA_SxCR_DIR_0 ;
		DMA1_Stream5->CR &=  ~DMA_SxCR_DIR_1 ;
		// Set the number of data transfer per cycle(circular transfer)
		DMA1_Stream5->NDTR = ul_Num_Points;
		// Enable the channel - note it won't transfer until triggered by the DAC
		DMA1_Stream5->CR |= DMA_SxCR_EN;


		// Configure the timer(TIM4) and set the required update rate.
		// Note: This will prompt the DAC transfer from holding register to output register AND
		// initiate another DMA transfer into the holding register.
		// Enable the timer clock.
		RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
		// Set the auto-reload as calculated above
		TIM4->ARR = (uint16_t)sl_Clock_Count;
		// Enable TRGO event generation via master mode selection
		TIM4->CR2 |= TIM_CR2_MMS_1;
		// Enable the timer
		TIM4->CR1 |= TIM_CR1_CEN;



		// DAC Configuration for Timer / DMA usage.
		// Enable clock
		RCC->APB1ENR |= RCC_APB1ENR_DACEN;
		// Set DMA enable
		DAC1->CR |= DAC_CR_DMAEN1;
		// Set DAC Ch1 Trigger Selection (Timer 4 TRGO)
		DAC1->CR |= DAC_CR_TSEL1_0 | DAC_CR_TSEL1_2;
		DAC1->CR &= ~DAC_CR_TSEL1_1;
		// Enable the DAC
		DAC1->CR |= DAC_CR_EN1;
		// Enable the channel trigger - this kicks things off.
		DAC1->CR |= DAC_CR_TEN1;



	}
	return Success;
}



