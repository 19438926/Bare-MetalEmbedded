/**************************************************************************************************************
 *                  File Name       :HAL_SysTick.h
 *                  Overview        :SysTick functionality.
 *                  Authors         :Wenze Pan
 **************************************************************************************************************
 *                  Copyright @ As Applicable
 **************************************************************************************************************/

#ifndef SYSTICK_H
#define SYSTICK_H

/***************************************************/
/* Definitions required by this module */

/***************************************************/
/* Types used by this module */

/**************************************************/
/* Externally available variable */

/*************************************************/
/*Externally available functions */
uint64_t SysTick_Get_Timestamp( void );
uint64_t SysTick_Elapsed_MicroSeconds( uint64_t ull_Start_Timestamp );
uint64_t SysTick_MicroSeconds_to_Counts( uint64_t ull_MicroSeconds );
uint64_t SysTick_Period_MicroSeconds( uint64_t ull_Start_Count, uint64_t ull_Finish_Count );

#endif
