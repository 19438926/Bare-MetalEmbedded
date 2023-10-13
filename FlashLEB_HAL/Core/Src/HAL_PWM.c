/**************************************************************************************************************
 *                  File Name       :HAL_PWM.c
 *                  Overview        :PWM functionality.
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
#include "HAL_GlobalDef.h"
#include "HAL_PWM.h"
#include "stm32f4xx_hal.h"




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
uint8_t InitDone = FALSE;
TIM_HandleTypeDef  *Local_pwm;


/**************************/
/* Local only function prototypes */

/*********************************************
 * @brief PWM_Set_Duty_x10
 * Sets the PWM Duty Cycle to the value given (i.e. Variable = 123 => 12.3%).
 * @param uint32_t ull_PWM_Frequency
 * @retval None
 */
void PWM_Set_Duty_x10(uint16_t us_New_Demand_x10, TIM_HandleTypeDef  *pwm )
{
	// Set the timer address once
	if(!InitDone)
	{
		InitDone = TRUE;
		Local_pwm = pwm;
	}
	//Set the required duty in clock cycles.
	__HAL_TIM_SET_COMPARE(pwm,TIM_CHANNEL_1,(pwm->Init.Period * us_New_Demand_x10) / 1000);

}

/*********************************************
 * @brief PWM_Get_Base_Frequency
 * Gets the current PWM base frequency.
 * @param uint16_t us_New_Demand_x10
 * @retval None
 */
uint32_t PWM_GET_Base_Frequency()
{
	return (APB1_TIMER_CLOCK_FRQ / (Local_pwm->Init.Period+1)) ;
}

/*********************************************
 * @brief PWM_Set_Base_Frequency
 * Gets the current PWM base frequency.
 * @param None
 * @retval None
 */
void PWM_SET_Base_Frequency(uint32_t Base_Freq)
{
	Local_pwm->Instance->ARR=(APB1_TIMER_CLOCK_FRQ/Base_Freq)-1;
}



