/**************************************************************************************************************
 *                  File Name       :HAL_WaveformGenerator.h
 *                  Overview        :WaveformGenerator functionality.
 *                  Authors         :Wenze Pan
 **************************************************************************************************************
 *                  Copyright @ As Applicable
 **************************************************************************************************************/


#include "stm32f4xx_hal.h"
/***************************************************/
/* Definitions required by this module */

/***************************************************/
/*Enumerations used by this module */
typedef enum
{
	eWT_Sine,
	eWT_SawTooth,
	eWT_Triangular,
	eWT_Square,
	eWT_Custom,//Not implemented yet
}eWaveformType;
/***************************************************/
/* Types used by this module */
typedef struct
{
	eWaveformType  e_WaveType;
	uint64_t       ull_Period_uS;
	float          f_Amplitude;
	float          f_Offset;

}_WAVEFORM_DESCRIPTOR;

/**************************************************/
/* Externally available variable */

/*************************************************/
/*Externally available functions */
float WaveformGenerator_ComputeSignal(_WAVEFORM_DESCRIPTOR*pWave,uint64_t ull_Timestamp);
void WaveformGenerator_Set_Waveform(_WAVEFORM_DESCRIPTOR NewWave);
_WAVEFORM_DESCRIPTOR WaveformGenerator_Get_Waveform();
void WaveformGenerator_UpdateOutputs(DAC_HandleTypeDef *hdac,TIM_HandleTypeDef *htim);

uint8_t WaveformGenerator_Add_Custom_Data(uint16_t *p_Data,uint16_t NumData);
void WaveformGenerator_Clear_Custom_Data();
