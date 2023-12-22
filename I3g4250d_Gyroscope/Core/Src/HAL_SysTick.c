/**************************************************************************************************************
 *                  File Name       :HAL_SysTick.c
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
#include "stm32f4xx_hal.h"

/****************************************************/
/*Local only definitions */


/***************************/
/* Enumerations */

/**************************/
/*Structure types */

/*********************************************/
/* Global variable references */
#define   SYS_CLOCK_FRQ                  16000000

/*********************************************/
/* Local only variable declaration */

/**************************/
/* Local only function prototypes */

/*********************************************
 * @brief SysTick_Get_TimeStamp
 * Supply a high resolution timestamp from SysTick.
 * @param None
 * @retval None
 */
uint64_t SysTick_Get_Timestamp( void )
{
	uint64_t ull_New_Count;
	uint32_t ul_Interrupt_Flag = 0xFFFFFFFF;
	uint64_t count;

	//Load the SysTick interrupt fired flag.
	ul_Interrupt_Flag = (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk);

	//Monitor the calculation and go again if it's interrupted by the SysTick Interrupt.
	do{

	// Get the number of ticks in milliseconds(32 bit)
		count = HAL_GetTick();

	//Calculate the number of counts from imterrupts already
	ull_New_Count = (uint64_t)(count*SysTick->LOAD);

	//Factor in the current count-down timer register value.
	ull_New_Count = (uint64_t)(ull_New_Count + SysTick->LOAD - SysTick->VAL);

	//Reload the SysTick interrupt fired flag
	ul_Interrupt_Flag = (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk);

	} while(ul_Interrupt_Flag != 0);

	return ull_New_Count;
}
/*********************************************
 * @brief SysTick_Elapsed_MicroSeconds
 * Return elapsed time in microseconds sincestart_stamp.
 * @param None
 * @retval None
 */
uint64_t SysTick_Elapsed_MicroSeconds(uint64_t ull_Start_Timestamp)
{
   //Fetch the lastest count.
	uint64_t ull_Elapsed_Count = SysTick_Get_Timestamp();

	//Subtract the start count from the latest value
	ull_Elapsed_Count -= ull_Start_Timestamp;

	//Convert counts to microseconds
	ull_Elapsed_Count = (uint64_t)(ull_Elapsed_Count / (SYS_CLOCK_FRQ/1000000));

	return ull_Elapsed_Count;
}
/*********************************************
 * @brief SysTick_MicroSeconds_to_Counts
 * Return count equivalent of MicroSeconds value supplied.
 * @param None
 * @retval None
 */
uint64_t SysTick_MicroSeconds_to_Counts(uint64_t ull_MicroSeconds)
{
     //Return the count
	 return (uint64_t)(ull_MicroSeconds*(SYS_CLOCK_FRQ/1000000));
}
/*********************************************
 * @brief SysTick_Period_MicroSeconds
 * Return number of microseconds between Start_Count and Finish_Count.
 * @param None
 * @retval None
 */
uint64_t SysTick_Period_MicroSeconds(uint64_t ull_Start_Count, uint64_t ull_Finish_Count)
{
    //Fetch the latest count
	uint64_t ull_Elapsed_Count;

	//Subtract the finish count from the start count
	ull_Elapsed_Count = (ull_Finish_Count - ull_Start_Count);

	//counts to microseconds using SysTick config.
	ull_Elapsed_Count = (uint64_t)(ull_Elapsed_Count / (SYS_CLOCK_FRQ/1000000));

	return ull_Elapsed_Count;


}

/*****************/
/* End of files */

