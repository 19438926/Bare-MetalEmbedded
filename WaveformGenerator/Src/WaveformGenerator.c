/**************************************************************************************************************
 *                  File Name       :WaveformGenerator.c
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

#include"SysTick.h"
#include"WaveformGenerator.h"



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


/**************************/
/* Local only function prototypes */
float f_Interpolate_Over_Time( uint64_t  ull_Previous_uS, float f_Previous_Reading,
		                       uint64_t  ull_Next_uS, float f_Next_Reading,
							   uint64_t  ull_Return_uS);

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


	//Generate the relevant point on the relevant wave type
	switch (pWave->e_WaveType)
	{
	case eWT_Sine:
		//Calculate the 'x' value in the sine function by using linear interpolation
		f_Calc = f_Interpolate_Over_Time(0,0,SysTick_MicroSeconds_to_Counts(pWave->ull_Period_uS),2*M_PI, ull_Timestamp);
		//Calculate the amplitude by sine function and added offset and constant(avoid to negative value)
		f_Result = ((pWave->f_Amplitude)/2)*sin(f_Calc) + (pWave->f_Amplitude)/2 ;

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
			f_Result=pWave->f_Amplitude;
		}
		break;
	case eWT_Custom:
		return f_Result;
		break;

	}
	return f_Result + pWave->f_Offset ;
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


