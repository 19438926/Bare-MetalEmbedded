/**************************************************************************************************************
 *                  File Name       :WaveformGenerator.h
 *                  Overview        :WaveformGenerator functionality.
 *                  Authors         :Wenze Pan
 **************************************************************************************************************
 *                  Copyright @ As Applicable
 **************************************************************************************************************/

#ifndef WAVEFORMGENERATOR_H_
#define WAVEFORMGENERATOR_H_

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
uint8_t WaveformGenerator_Set_Waveform(_WAVEFORM_DESCRIPTOR NewWave);
_WAVEFORM_DESCRIPTOR WaveformGenerator_Get_Waveform();
void WaveformGenerator_UpdateOutputs();

uint8_t WaveformGenerator_Add_Custom_Data(uint16_t *p_Data,uint16_t NumData);
void WaveformGenerator_Clear_Custom_Data();
uint16_t WaveformGenerator_Get_CallRate(eWaveformType Type);
uint16_t WaveformGenerator_Get_OptimumPeriod(eWaveformType Type);
void WaveformGenerator_Set_NUM_DAC_Sample(uint16_t NumSample);
uint16_t WaveformGenerator_Get_NUM_DAC_Sample();

#endif
