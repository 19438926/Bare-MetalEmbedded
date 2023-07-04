/**************************************************************************************************************
 *                  File Name       :SysTick.c
 *                  Overview        :SysTick functionality.
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
#include "PWM.h"



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
uint64_t ull_Configured_PWM_Frequency;

/**************************/
/* Local only function prototypes */
/*********************************************
 * @brief PWM_Init
 * Initialise PWM output to the given frequency..
 * @param uint32_t ull_PWM_Frequency
 * @retval None
 */
void PWM_Init(uint32_t ull_PWM_Frequency)
{
	// Note frequency configured.
	ull_Configured_PWM_Frequency = ull_PWM_Frequency;

	//////////////////////////
	//Timer configuration (TIM2.CH1)
	//Enable the timer clock.
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	//Set the auto-reload value - this is the count value where it will reset to satrt of cycle.
	//72MHz clock and we want a 100KHz PWMfrequency for instance ,so 72M / 100k =720. Note -1 as zero is an index
	TIM2->ARR = (SYS_CLOCK_FRQ / ull_Configured_PWM_Frequency) - 1;

	//Set PWM Mode
	TIM2->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;

	//Initialise the counter to zero
	TIM2->CNT = 0;

	//Enable PWM Channel 1
	TIM2->CCER = TIM_CCER_CC1E;

	//Set duty cycle period to zero until set otherwise.
	TIM2->CCR1 =200 ;

	//Start timer with auto-preload register enabled.
	TIM2->CR1 = TIM_CR1_CEN;

}


/*********************************************
 * @brief PWM_Set_Duty_x10
 * Sets the PWM Duty Cycle to the value given (i.e. Variable = 123 => 12.3%).
 * @param uint32_t ull_PWM_Frequency
 * @retval None
 */
void PWM_Set_Duty_x10(uint16_t us_New_Demand_x10)
{
	//Calculate number of clock cycles perPWM cycle.
	uint32_t ull_cycle_clocks = (SYS_CLOCK_FRQ / ull_Configured_PWM_Frequency) - 1;

	//Set the required duty in clock cycles.
	TIM2->CCR1 = (ull_cycle_clocks * us_New_Demand_x10) / 1000;

}




