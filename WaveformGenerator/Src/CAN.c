/**************************************************************************************************************
 *                  File Name       :CAN.c
 *                  Overview        :CAN bus functionality.
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
#include "CAN.h"



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

/*********************************************/
/* Local only function prototype */



/*********************************************
 * @brief CAN_Init
 * Initialise the CAN.
 * @param None
 * @retval None
 */
void CAN_Init()
{
	// Enable the clock bus for CAN1.
	RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;

	// Enter initialisation mode for configuring the bit timing and operation mode.
	// Send initialisation mode request
	CAN1->MCR |= CAN_MCR_INRQ;
	// Wait until hardware confirms
	while(!(CAN1->MSR & CAN_MSR_INAK)){};

	// Set Baud rate to be 1M bits/s
	// Prescaler: 5
	// Segment1 : 4
	// Segment2 : 4
	CAN1->BTR |= 0b101;
	CAN1->BTR |= 0b100<<19;
	CAN1->BTR |= 0b100<<15;


}























