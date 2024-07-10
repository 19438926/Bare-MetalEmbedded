/**************************************************************************************************************
 *                  File Name       :main.c
 *                  Overview        :Micro and System initialisation  as well as eternal loop.
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



/****************************************************/
/*Local only definitions */
#define FPU_CP10_FULL (0x00200000 + 0x00100000)
#define FPU_CP11_FULL (0x00800000 + 0x00400000)

/***************************/
/* Enumerations */

/**************************/
/*Structure types */

/*********************************************/
/* Global variable references */

/*********************************************/
/* Local only variable declaration */

/**************************/
/* Local function prototypes */
void Micro_Initialisation(void);
void Initialise_GPIO(void);
void Initialise_External_Clock(void);


/*********************************************
 * @brief main
 * Main entry point, carries out initialisation and calling all sub-systems within eternal loop.
 * @param None
 * @retval None
 */
int main(void)
{
    /* Loop forever */
	for(;;);
}
