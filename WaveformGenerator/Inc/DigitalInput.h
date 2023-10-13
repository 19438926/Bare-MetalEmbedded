/**************************************************************************************************************
 *                  File Name       :DigitalInput.h
 *                  Overview        :DigitalInput function.
 *                  Authors         :Wenze Pan
 **************************************************************************************************************
 *                  Copyright @ As Applicable
 **************************************************************************************************************/


#ifndef DIGITALINPUT_H_
#define DIGITALINPUT_H_

/***************************************************/
/* Definitions required by this module */

/***************************************************/
/*Enumerations used by this module */
// Enumeration for available digital inputs (1 real and two 'pretend' for demo purpose/ future expansion.
typedef enum
{
	DI_UserButton_1,
	DI_ExampleOtherInput,
	DI_ExampleAnotherInput
}eDIGITAL_INPUT;

/***************************************************/
/* Types used by this module */


/**************************************************/
/* Externally available variable */

/*************************************************/
/*Externally available functions */
void DI_Process(void);
uint8_t DI_Get_Input(eDIGITAL_INPUT e_InputRequired);

#endif /* DIGITALINPUT_H_ */
