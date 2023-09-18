/**************************************************************************************************************
 *                  File Name       :HAL_WaveformGenerator.c
 *                  Overview        :WaveformGenerator functionality.
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
#include "math.h"

#include "HAL_GlobalDef.h"
#include"HAL_SysTick.h"
#include"HAL_WaveformGenerator.h"
#include <string.h>
#include "HAL_DAC.h"
#include "HAL_PWM.h"
#include "stm32f4xx_hal.h"

/****************************************************/
/*Local only definitions */
#define Pi       3.1415926

/***************************/
/* Enumerations */

/**************************/
/*Structure types */
typedef struct
{
	uint16_t us_NumPoints;
	uint16_t us_DataPoints[MAX_CUSTOM_DATA_LENGTH];// 0-10000 = 0 - 100%
} _CUSTOM_WAVEFORM_DATA;

/*********************************************/
/* Global variable references */

/*********************************************/
/* Local only variable declaration */
_WAVEFORM_DESCRIPTOR  CurrentWaveform;
_CUSTOM_WAVEFORM_DATA CustomWaveform;

/**************************/
/* Local only function prototypes */
float f_Interpolate_Over_Time( uint64_t  ull_Previous_uS, float f_Previous_Reading,
		                       uint64_t  ull_Next_uS, float f_Next_Reading,
							   uint64_t  ull_Return_uS);


/*********************************************
 * @brief WaveformGenerator_UpdateOuputs
 * Called from main loop to update outputs
 * Note: Only functions if frequency < 200Hz, otherwise it's all handled by DMA
 * @param None
 * @retval None
 */
void WaveformGenerator_UpdateOutputs(DAC_HandleTypeDef *hdac, TIM_HandleTypeDef *htim)
{
	static uint64_t ull_TimeStamp;
	static uint64_t ull_WaveStamp;
	float f_WaveSignal;

	// Check frequency requires manual outputting.
	if(CurrentWaveform.ull_Period_uS >= MAX_FREQ_MANUALLY_OUTPUTTED_uS)
	{
		//Update our timestamp
		ull_TimeStamp = SysTick_Get_Timestamp();
		while(ull_TimeStamp > (ull_WaveStamp + SysTick_MicroSeconds_to_Counts(CurrentWaveform.ull_Period_uS)))
		{
			ull_WaveStamp += SysTick_MicroSeconds_to_Counts(CurrentWaveform.ull_Period_uS);
		}
		// Get the signal level required for 'now'.
		f_WaveSignal = WaveformGenerator_ComputeSignal(&CurrentWaveform, ull_TimeStamp - ull_WaveStamp);


		//Set PWM Output
		PWM_Set_Duty_x10((uint16_t)(f_WaveSignal*10), htim);


		//Set DAC Output
		DAC_Set_Output_x100((uint32_t)(f_WaveSignal * 100),hdac);


	}
	else
	{
		// PWM Must be switched off as frequency is too high and DAC is running on DMA
		PWM_Set_Duty_x10(0,htim);
	}

}

/*********************************************
 * @brief WaveformGenerator_ComputeSignal
 * Computes an output value according to contents of pWave
 * @param _WAVEFORM_DESCRIPTOR pWave - Waveform Description
 * @retval float - signal level for given timestamp in waveform
 */
float WaveformGenerator_ComputeSignal(_WAVEFORM_DESCRIPTOR *pWave,uint64_t ull_Timestamp)
{
	float f_Result = 0.0;
	float f_Calc;
	uint64_t ull_Calc;

	//Generate the relevant point on the relevant wave type
	switch (pWave->e_WaveType)
	{
	case eWT_Sine:
		//Calculate the 'x' value in the sine function by using linear interpolation
		f_Calc = f_Interpolate_Over_Time(0,0,SysTick_MicroSeconds_to_Counts(pWave->ull_Period_uS),2*Pi, ull_Timestamp);
		//Calculate the amplitude by sine function and added offset and constant(avoid to negative value)
		f_Result = ((pWave->f_Amplitude)/2)*sin(f_Calc) + (pWave->f_Amplitude)/2 + pWave->f_Offset ;

		break;
	case eWT_SawTooth:
		//Calculate the Amplitude at certain point(timestamp)
		f_Result = f_Interpolate_Over_Time(0,0,SysTick_MicroSeconds_to_Counts(pWave->ull_Period_uS) ,pWave->f_Amplitude , ull_Timestamp);

        break;
	case eWT_Triangular:
		// Triangular wave form has two different wave ,each  going up/ going down takes half period.
		if((ull_Timestamp) < SysTick_MicroSeconds_to_Counts(pWave->ull_Period_uS)/2)
		{
			f_Result = f_Interpolate_Over_Time(0,0,SysTick_MicroSeconds_to_Counts(pWave->ull_Period_uS)/2 ,pWave->f_Amplitude,ull_Timestamp);

		}else
		{
			f_Result = f_Interpolate_Over_Time(SysTick_MicroSeconds_to_Counts((pWave->ull_Period_uS)/2),pWave->f_Amplitude,SysTick_MicroSeconds_to_Counts(pWave->ull_Period_uS) ,0, ull_Timestamp);

		}
		break;
	case eWT_Square:
		// Square wave form has two different wave ,each   up/ down takes half period.
		if((ull_Timestamp) < SysTick_MicroSeconds_to_Counts(pWave->ull_Period_uS)/2)
		{

			f_Result = pWave->f_Amplitude;

		}
		break;
	case eWT_Custom:
		// Work out how far through the waveform as a fraction
		f_Calc = (float)ull_Timestamp / (float)SysTick_MicroSeconds_to_Counts(pWave->ull_Period_uS);

		// Get closest value before timestamp
		uint16_t ul_NearestElementBefore = f_Calc * ( CustomWaveform.us_NumPoints - 1);

		// Compute time per element
		float f_Time_Per_Element_uS = (float)pWave->ull_Period_uS / (CustomWaveform.us_NumPoints - 1);

		// Calculate new timestamp ticks 'between' elements.
		ull_Calc = ull_Timestamp - SysTick_MicroSeconds_to_Counts(ul_NearestElementBefore * f_Time_Per_Element_uS);

		// Now compute that as a fraction
		f_Calc = (float)ull_Calc / (float)SysTick_MicroSeconds_to_Counts(f_Time_Per_Element_uS);

		// Trap 'At end' of data
		if (ul_NearestElementBefore < (CustomWaveform.us_NumPoints - 1))
		{
			// Interpolate to derive value between relevant elements.
			f_Result = f_Interpolate_Over_Time(0,(float)CustomWaveform.us_DataPoints[ul_NearestElementBefore], 1000, (float)CustomWaveform.us_DataPoints[ul_NearestElementBefore + 1], f_Calc * 1000);
		}
		else
		{
			// Last element value
			f_Result = CustomWaveform.us_DataPoints[ul_NearestElementBefore];

		}

			f_Result = (f_Result / 10000.0) * pWave ->f_Amplitude;
		break;

	}
	return f_Result;
}



/*********************************************
 * @brief f_Interpolate_Over_Time
 * Interpolates between two points
 * @param Previous / next point in time and levels.
 * @retval float - interpolated value at ull_Return_uS - the interpolation point.
 */
float f_Interpolate_Over_Time(uint64_t ull_Previous_uS, float f_Previous_Reading,uint64_t ull_Next_uS, float f_Next_Reading,
							   uint64_t  ull_Return_uS)
{
	return f_Previous_Reading + ((f_Next_Reading - f_Previous_Reading) / (ull_Next_uS - ull_Previous_uS))*
			                  (ull_Return_uS-ull_Previous_uS);
}

/*********************************************
 * @brief WaveformGenerator_Get_Waveform
 * Retruns a copy of the current waveform.
 * @param  None
 * @retval _Waveform_DESCRIPTOR - waveform to output.
 */
_WAVEFORM_DESCRIPTOR WavefromGenerator_Get_Waveform()
{
	return CurrentWaveform;
}

/*********************************************
 * @brief WaveformGenerator_Get_Waveform
 * Configures the current output to the waveform provided.
 * @param _WAVEFORM_DESCRIPTOR - waveform to output
 * @retval None
 */
void WaveformGenerator_Set_Waveform(_WAVEFORM_DESCRIPTOR NewWave)
{
	// Copy the new waveform into our structure.
	memcpy ((void *)&CurrentWaveform, (void*)&NewWave, sizeof(_WAVEFORM_DESCRIPTOR));

	// Do any further processing depending on things like frequency etc.
	if (CurrentWaveform.ull_Period_uS < MAX_FREQ_MANUALLY_OUTPUTTED_uS)
	{
		 // Data has to be processed for use with DMA
		//CreateDMAPattern();
	}
}

/*********************************************
 * @brief WaveformGenerator_Clear_Custom_Data
 * Clears custom waveform data ready for re-loading.
 * @param  None
 * @retval None
 */
void WaveformGenerator_Clear_Custom_Data()
{
	// Set custom waveform data to zero
	CustomWaveform.us_NumPoints = 0;

	// Clear out DMA data if currently in use
	if((CurrentWaveform.e_WaveType == eWT_Custom) && (CurrentWaveform.ull_Period_uS > MAX_FREQ_MANUALLY_OUTPUTTED_uS))
	{
		// Data has  to be processed for use with DMA
		//CreateDMAPattern(); // All zero's in this instance.
	}
}

/*********************************************
 * @brief WaveformGenerator_Add_Custom_Data
 * Adds custom waveform data to memory for outputting.
 * @param  uint8_t* Pointer to data
 * 			uint16_t Number of data pointd.
 * @retval uint8_t TRUE = Success ,otherwise failure.
 */
uint8_t WaveformGenerator_Add_Custom_Data(uint16_t *p_Data, uint16_t us_NumData)
{
	uint8_t c_DataUpdated = FALSE;

	// Safety check - within data bounds?
	if (us_NumData <= (MAX_CUSTOM_DATA_LENGTH - CustomWaveform.us_NumPoints))
	{
		// Copy the data across...
		memcpy((void*)&CustomWaveform.us_DataPoints[CustomWaveform.us_NumPoints],(void*)p_Data,us_NumData * 2);
		CustomWaveform.us_NumPoints += us_NumData;
		c_DataUpdated = TRUE;

		// Do any further processing depending on frequency.
		if((CurrentWaveform.e_WaveType == eWT_Custom)&& (CurrentWaveform.ull_Period_uS > MAX_FREQ_MANUALLY_OUTPUTTED_uS))
		{
			// Data has to be processed for use with DMA
			// CreateDMAPattern();
		}
	}
	// else will naturally return false, indicating unable to add data.

	return c_DataUpdated;
}

/*********************************************
 * @brief WaveformGenerator_Get_Waveform
 * Retruns a copy of the current waveform.
 * @param  None
 * @retval _Waveform_DESCRIPTOR - waveform to output.
 */
_WAVEFORM_DESCRIPTOR WaveformGenerator_Get_Waveform()
{
	return CurrentWaveform;
}
