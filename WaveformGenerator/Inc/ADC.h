/**************************************************************************************************************
 *                  File Name       :ADC.h
 *                  Overview        :ADC functionality.
 *                  Authors         :Wenze Pan
 **************************************************************************************************************
 *                  Copyright @ As Applicable
 **************************************************************************************************************/

#ifndef ADC_H_
#define ADC_H_

/***************************************************/
/* Definitions required by this module */

/***************************************************/
/* Types used by this module */
typedef struct
{
	uint16_t s_DataLen; // Number of bytes in data
	uint32_t *uS_Data;  // Data pointer for Elapsed Microseconds
	uint32_t *pData;    // Data pointer for ADC Data.

}_ADC_DATA;

/**************************************************/
/* Externally available variable */

/*************************************************/
/*Externally available functions */
void ADC_Run();
void ADC_Set_Filter(uint16_t SampleNumber);
void ADC_Set_Time(uint16_t SampleTime);
_ADC_DATA ADC_Fetch_Data ();
void ADC_Start();
uint32_t ADC_Get_Temperature();
uint8_t ADC_Get_Status();
uint32_t ADC_Get_Reading();
uint16_t ADC_Get_Sample_Time();



#endif /* ADC_H_ */
