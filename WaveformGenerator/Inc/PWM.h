/**************************************************************************************************************
 *                  File Name       :PWM.h
 *                  Overview        :PWM functionality.
 *                  Authors         :Wenze Pan
 **************************************************************************************************************
 *                  Copyright @ As Applicable
 **************************************************************************************************************/

#ifndef PWM_H_
#define PWM_H_
/***************************************************/
/* Definitions required by this module */

/***************************************************/
/* Types used by this module */

/**************************************************/
/* Externally available variable */

/*************************************************/
/*Externally available functions */
void PWM_Init(uint32_t ull_PWM_Frequency);
void PWM_Set_Duty_x10(uint16_t us_New_Demand_x10);


#endif /* PWM_H_ */
