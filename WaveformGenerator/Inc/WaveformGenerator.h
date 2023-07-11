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

#endif
