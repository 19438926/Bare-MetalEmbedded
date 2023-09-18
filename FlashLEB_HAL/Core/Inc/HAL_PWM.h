/**************************************************************************************************************
 *                  File Name       :HAL_PWM.h
 *                  Overview        :PWM functionality.
 *                  Authors         :Wenze Pan
 **************************************************************************************************************
 *                  Copyright @ As Applicable
 **************************************************************************************************************/

#ifndef PWM_H_
#define PWM_H_

#include "stm32f4xx_hal.h"

/***************************************************/
/* Definitions required by this module */

/***************************************************/
/* Types used by this module */

/**************************************************/
/* Externally available variable */

/*************************************************/
/*Externally available functions */
void PWM_Set_Duty_x10(uint16_t us_New_Demand_x10 , TIM_HandleTypeDef *pwm );
uint32_t PWM_GET_Base_Frequency(TIM_HandleTypeDef  *pwm);


#endif /* PWM_H_ */
