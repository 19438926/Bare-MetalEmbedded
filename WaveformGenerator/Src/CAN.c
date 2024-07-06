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
	// Send initialisation mode request.
	CAN1->MCR |= CAN_MCR_INRQ;
	// Wait until hardware confirms.
	while(!(CAN1->MSR & CAN_MSR_INAK)){};

	// Set Baud rate to be 1M bits/s.
	// Prescaler: 5  BRP: 0b100
	// Segment1 : 4	 TS1: 0b11
	// Segment2 : 4	 TS2: 0b11
	CAN1->BTR |= 0b100;
	CAN1->BTR |= 0b11<<20;
	CAN1->BTR |= 0b11<<16;

	// Initialisation the CAN filter banks.
	// Set initialisation mode for the filters.
	CAN1->FMR |= CAN_FMR_FINIT;
	// Set Filter 0 to be single 32-bit scale.
	CAN1->FS1R |= CAN_FS1R_FSC0;
	// Set Filter 0 to be mask mode.
	CAN1->FM1R &= ~(CAN_FM1R_FBM0);
	// Filter assigned to the FIFO0.
	CAN1->FFA1R &= ~(CAN_FFA1R_FFA0);
	// Set Identifier to be 0 (mask mode so any ID will be accepted
	CAN1->sFilterRegister[0].FR1 = 0;
	CAN1->sFilterRegister[0].FR2 = 0;
	// Deactivate all filters.
	CAN1->FA1R = 0;
	// Activate the filter bank 0.
	CAN1->FA1R |= CAN_FA1R_FACT0;

	// Enter the normal mode by clear INRQ bit.
	CAN1->MCR &= ~(CAN_MCR_INRQ);
	CAN1->MCR &= ~(CAN_MCR_SLEEP);


}























