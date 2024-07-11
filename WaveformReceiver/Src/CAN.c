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
#include "SysTick.h"



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
static uint64_t CAN_TimeStamp;
uint8_t F0Data[8];
char Response[8]="ThankYou";

/*********************************************/
/* Local only function prototype */
void CAN_Response();



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
	// Prescaler: 2  BRP: 0b01
	// Segment1 : 4	 TS1: 0b11
	// Segment2 : 3	 TS2: 0b10
	CAN1->BTR |= 0b1;
	CAN1->BTR |= 0b11<<16;
	CAN1->BTR |= 0b10<<20;

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
	CAN1->sFilterRegister[0].FR1 = 123<<21;
	CAN1->sFilterRegister[0].FR2 = 123<<21;
	// Deactivate all filters.
	CAN1->FA1R = 0;
	// Activate the filter bank 0.
	CAN1->FA1R |= CAN_FA1R_FACT0;

	// No automatically retransmit otherwise it will keep firing messages.
	CAN1->MCR |= CAN_MCR_NART;

	// Enable the FIFO0 pending interrupt for receiving data
	CAN1->IER |= CAN_IER_FMPIE0;
	NVIC_EnableIRQ(CAN1_RX0_IRQn);

	// Enter the normal mode by clear INRQ,SLEEP,FINT bit.
	CAN1->MCR &= ~(CAN_MCR_INRQ);
	CAN1->MCR &= ~(CAN_MCR_SLEEP);
	CAN1->FMR &= ~(CAN_FMR_FINIT);

	// Set the identifier to be 446
	CAN1->sTxMailBox[0].TIR = 446 << 21;

	// Set the Data Length to be 8
	CAN1->sTxMailBox[0].TDTR = 8;

	// Set the DATA1,2,3,4
	CAN1->sTxMailBox[0].TDLR = Response[3]<<24 | Response[2]<<16 | Response[1]<<8 | Response[0];

	// Set the DATA  5,6,7,8
	CAN1->sTxMailBox[0].TDHR = Response[7]<<24 | Response[6]<<16 | Response[5]<<8 | Response[4];


}

/*********************************************
 * @brief CAN_Transmit_WData
 * Transmit response information by CAN bus
 * @param None
 * @retval None
 */
void CAN_Response()
{

	// Request transmit the message
	CAN1->sTxMailBox[0].TIR |= CAN_TI0R_TXRQ;
}

/*********************************************
 * @brief CAN_Process
 * Flash the LED by different waveform type..
 * @param None
 * @retval None
 */
void CAN_Process()
{
	// Check if one seconds has passed
	if(SysTick_Elapsed_MicroSeconds(CAN_TimeStamp)>(100000*(uint8_t)(F0Data[0])+100000))
	{
		// Update time stamp
		CAN_TimeStamp = SysTick_Get_Timestamp();

		// Toggle the LED.
		GPIOG->ODR ^= GPIO_ODR_ODR_13;

	}
}

void CAN1_RX0_IRQHandler()
{
	// Get first 4 data
	for(int i=0;i<4;i++)
	{
	F0Data[i] = (CAN1->sFIFOMailBox[0].RDLR>>8*i);
	}

	// Get next 4 data
	for(int i=4;i<8;i++)
	{
		F0Data[i] = (CAN1->sFIFOMailBox[0].RDHR>>8*(i-4));
	}

	// Release the FIFO.
	CAN1->RF0R |= CAN_RF0R_RFOM0;

	// Say ThankYou
	CAN_Response();
}



















