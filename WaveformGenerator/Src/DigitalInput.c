/**************************************************************************************************************
 *                  File Name       :DigitalInput.c
 *                  Overview        :DigitalInput functionality.
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

#include "GlobalDefs.h"
#include "SysTick.h"
#include "DigitalInput.h"

/****************************************************/
/*Local only definitions */


/***************************/
/* Enumerations */

/**************************/
/*Structure types */
// Structure to contain all the info we need for a particular digital input
typedef struct
{
	eDIGITAL_INPUT  e_DigitalInputIDentifier;      // The digital input enumeration for clarity.
	uint8_t         uc_Live_State;                 // The live input state (not de-bounced) of specific input pin.
	uint8_t         uc_Steady_State;               // The de-bounced (steady) state of the input
	uint16_t        us_Debounce_Period_mS;         // A value of 0 = no de-bounce
	uint64_t        ull_BounceTimestamp;           // Records a time-stamp for the last change of live state
	uint8_t         uc_Active;                     // TRUE / FALSE, indicating this input is currently de-bouncing / changing state.
	GPIO_TypeDef    *Port;                         // GPIO Port for input
	uint8_t         uc_Pin;                        // GPIO Port number for input
}_DIGITAL_INPUT_STRUCT;

/*********************************************/
/* Global variable references */

/*********************************************/
/* Local only variable declaration */

// Declare our digital input array and populate with defaults.
// Note all are defaulted to 'Active' to ensure initial states are determined.
_DIGITAL_INPUT_STRUCT DigitalInput[] =
{		// Enumeration/ID,			Live State,		Steady State,		De-bounce Period (mS),  Bounce Time-stamp,  Active flag,	Port register,	Pin
		{DI_UserButton_1,           FALSE,          FALSE,              10,                     0,                  TRUE,           GPIOA,          0},
		{DI_ExampleOtherInput,      FALSE,          FALSE,              1000,                   0,                  TRUE,           GPIOA,          0},
		{DI_ExampleAnotherInput,    FALSE,          FALSE,              0,                      0,                  TRUE,           GPIOA,          0}
};

/*********************************************/
/* Local only macro's (same as definitions really, but are dependent on the structures above being defined).  */
#define  NUM_DIGITAL_INPUTS                           sizeof(DigitalInput) / sizeof(_DIGITAL_INPUT_STRUCT)

/**************************/
/* Local only function prototypes */

/*********************************************
 * @brief DI_Process
 * Performs Digital Input processing.
 * @param None
 * @retval None
 */
void DI_Process(void)
{
	// Use a pointer to save repeatedly indexing into structure array....
	_DIGITAL_INPUT_STRUCT  *p_DigitalInput;
//	uint8_t  uc_InputPortPinState;
//
//	// Loop through all digital inputs checking for debounce period expired and saving
//	// the steady state once stabilised.
	for (int i =0; i<NUM_DIGITAL_INPUTS; i++)
	{
		p_DigitalInput = &DigitalInput[i];
//
//		// Check for a change of live state and reset the time-stamp if found (note abbreviated if).
//		uc_InputPortPinState = (p_DigitalInput->Port->IDR & (0x01 << p_DigitalInput->uc_Pin))?  TRUE:FALSE;
//		if(uc_InputPortPinState != p_DigitalInput->uc_Live_State)
//		{
//			// Input value is different to recorded live value, update and store new time stamp.
//			p_DigitalInput->uc_Live_State = uc_InputPortPinState;
//			p_DigitalInput->ull_BounceTimestamp = SysTick_Get_Timestamp();
//			// Also set the 'active' flag
//			p_DigitalInput->uc_Active = TRUE;
//		}
//		else
//		{
			// Live state not changing, was this input changing previously?
			if (p_DigitalInput->uc_Active == TRUE)
			{
				// Was changing, has it stablilsed for long enough.
				if(SysTick_Elapsed_MicroSeconds(p_DigitalInput->ull_BounceTimestamp) / 1000 > p_DigitalInput->us_Debounce_Period_mS)
				{
					// De-bouce period has elapsed, we can now copy the live value to our steady value
					p_DigitalInput->uc_Steady_State = p_DigitalInput->uc_Live_State;
					p_DigitalInput->uc_Active = FALSE;
				}
			}
	}
}

/*********************************************
 * @brief DI_Get_Input
 * Returns the steady state for the enumerated input.
 * @param enum Input required
 * @retval uc_InputSteadyState
 */
uint8_t DI_Get_Input(eDIGITAL_INPUT e_InputRequired)
{
	uint8_t uc_InputSteadyState = FALSE;

	// Safety - check within bounds of our digital inputs array pr return false if not
	if(e_InputRequired < NUM_DIGITAL_INPUTS)
	{
		uc_InputSteadyState = DigitalInput[e_InputRequired].uc_Steady_State;
	}

	return uc_InputSteadyState;
}

/*********************************************
 * @brief EXTI0_IRQHandler
 * Interrupt handler for EXTI0 which we've configured for PA0 both edges.
 * @param None
 * @retval None
 */
uint32_t Debug_NumTimesIRQCalled = 0;
void EXTI0_IRQHandler(void)
{
	// Clear the interrupt pending bit
	// Here, we're only doing this for the one specific input we've enabled, but if there were several
	// interrupts active on this port we'd use the EXTI->PR (Pending Register) to work out
	// which particular input caused this interrupt to fire and act accordingly.
	// So in this instance, clear the pending bit for the only interrupt we've enabled on EXTI0.
	EXTI->PR |= (0x01 << DigitalInput[0].uc_Pin);

	// Store the state, set as active and save a time-stamp for the change in state.
	// For this demo, we're going to write the new state to all our digital inputs.
	// Collect values.
	_DIGITAL_INPUT_STRUCT *p_DigitalInput = &DigitalInput[0];
	uint8_t uc_NewLiveState = (p_DigitalInput->Port->IDR & (0x01 << p_DigitalInput->uc_Pin))?  TRUE:FALSE;
	uint64_t ull_Timestamp = SysTick_Get_Timestamp();

	// Popu;ate digital input array
	for (int i=0; i < NUM_DIGITAL_INPUTS; i++)
	{
		p_DigitalInput = &DigitalInput[i];
		p_DigitalInput->uc_Live_State = uc_NewLiveState;
		p_DigitalInput->uc_Active = TRUE;
		p_DigitalInput->ull_BounceTimestamp = ull_Timestamp;
	}

	// Debug
	Debug_NumTimesIRQCalled++;

}
