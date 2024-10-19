/**************************************************************************************************************
 *                  File Name       :ParametersOperation.c
 *                  Overview        :Provide a path for fetching the Holding Registers.
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
#include <stdint.h>
#include"GlobalDefs.h"
#include"WaveformGenerator.h"

/****************************************************/
/*Local only definitions */
#define NUM_HOLDING_REGISTERS sizeof (HOLDING_REGISTER) / sizeof(uint16_t *)

/***************************/
/* Enumerations */

/**************************/
/*Structure types */

/*********************************************/
/* Global variable references */

/*********************************************/
/* Local only variable declaration */

_WAVEFORM_DESCRIPTOR WaveformShadow  ;

uint16_t *HOLDING_REGISTER[] =
{
		(uint16_t *) &WaveformShadow.e_WaveType,
		(uint16_t *) &WaveformShadow.ull_Period_uS,
		(uint16_t *) (((uint8_t *)&WaveformShadow.ull_Period_uS)+2),
		(uint16_t *) &WaveformShadow.f_Offset,
		(uint16_t *) (((uint8_t *)&WaveformShadow.f_Offset)+2),
		(uint16_t *) &WaveformShadow.f_Amplitude,
		(uint16_t *) (((uint8_t *)&WaveformShadow.f_Amplitude)+2),

};
// These 5 holding registers(16bits) are holding the value of the waveform(simplified)
// And its number represent its address in the holding register region
// {	1 (0x0001)      ,		WaveformType }
// {	2 (0x0002)     ,		Period_LOW(32bits) }
// {	3 (0x0003)     ,		Period_HIGH(32bits) }
// {	4 (0x0004)     ,		Offset_LOW(float) }
// {	5 (0x0005)     ,		Offset_HIGH(float) }
// {	6 (0x0006)     ,		Amplitude_LOW(float) }
// {	7 (0x0007)     ,		Amplitude_HIGH(float) }



/*********************************************/
/* Local only function prototype */





/*********************************************
 * @brief ValUpdate
 * Get THe Current Waveform Value(only first time,rest of time will be updated by write message.
 * @param None
 * @retval None
 */
void ValUpdate()
{

		// Assign relevant value of the waveform to Current Waveform
		WaveformShadow = WaveformGenerator_Get_Waveform();
}

/*********************************************
 * @brief GetVal
 * Get value for respond message when reading registers.
 * @param  uint16_t
 * @retval uint16_t
 */
uint16_t GetVal(uint16_t number)
{
	// Get the value from holding registers.
	uint16_t *Val = HOLDING_REGISTER[number-1];

	return *Val;
}

/*********************************************
 * @brief SetVal
 * Set value for the waveform by modbus command
 * @param  uint16_t number
 * @param  uint16_t data
 * @retval none
 */
void SetVal(uint16_t number,uint16_t data)
{
	// Set the data to the local waveform
	*HOLDING_REGISTER[number-1] = data;

	// Set the data to the to the real current waveform
	WaveformGenerator_Set_Waveform(WaveformShadow);
}
