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

#include "stm32f1xx.h"
#include  "GlobalDefs.h"
#include "Systick.h"
#include  "Flash.h"
#include "UART.h"
#include <stdlib.h>
#include <string.h>


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
char data[500];
char* lyrics = "Heartbeat,is coming in so strong Oh,if you don't stop,I'm gonna need a second one Oh,there's something,I've been meaning to Say to you,baby (Hold that thought) Yeah,there's something,I've been meaning to Say to you,baby,but I just can't do it What a call,moving in I feel like I can loosen my lips (Come on so strong),I can summarise it for you,It's simple and it goes like this,I'm in love with you,I'm in love with you,I'm in love with you,I'm in love with you,";
char* trans_data;
uint32_t index1=0;
uint32_t index2 = 0;

/**************************/
/* Local function prototypes */
void Micro_Initialisation(void);
void Initialise_Internal_Clock(void);
void Initialise_GPIO(void);
char* Message_Handle();

int main(void)
{
	// Initialise MCU
	Micro_Initialisation();

	// Write lyrics to the flash
	write(0x0800F800, lyrics, strlen(lyrics));
	// Read lyrics from the flash
	read(0x0800F800,data, sizeof(data));
	// Initialise a timestamp for tracking time
	uint64_t timestamp = SysTick_Get_Timestamp();
	// set the address of trans_data to data[500]
	trans_data = data;
	while(1)
	{
		// delay 1s
		while(SysTick_Elapsed_MicroSeconds(timestamp)<1000000)
		{

		}
		// transmit lyrics
		Transmit(Message_Handle(),index2);
		// record timestamp
		timestamp = SysTick_Get_Timestamp();
		// delay 1s again
		while(SysTick_Elapsed_MicroSeconds(timestamp)<1000000)
		{

		}
		// transmit line field and carriage return
		Transmit("\r\n",strlen("\r\n"));
		// record timestamp
		timestamp = SysTick_Get_Timestamp();
	}
}

/***********************************************
 * @brief   Micro_Initialisation
 * This function configures the micro clocks ,gpio and peripherals used by the system
 * @param None
 * @retval None
 */
void Micro_Initialisation(void)
{
	//configure chip to use internal clock source.
	Initialise_Internal_Clock();

	//Initialise any GPIO required.
	Initialise_GPIO();

	//Initialise SysTick Interrupt.
	SysTick_Init();

	//Initialise UART
	UART_Init(115200);
}

/***********************************************
 * @brief Initialise Internal Clock
 * This function configures the mirco to use Internal 16MHz clock source(HSI)
 * @param None
 * @retval None
 */
void Initialise_Internal_Clock(void)
{
	//Set the power enable clock and voltage regulator
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;

	// Enable the prefetch buffer
	FLASH->ACR |= FLASH_ACR_PRFTBE ;

	// Set the PLL multiplication factor x6
	RCC->CFGR |= RCC_CFGR_PLLMULL6;

	// Enable the PLL and wait until its ready
	RCC->CR |= RCC_CR_PLLON;
	while(!(RCC->CR & RCC_CR_PLLRDY));

	// Select the clock source and wait for it to be set
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}

/********************************************
 * @Brief Initialise_GPIO
 * This function initialises any GPIO used by the system.
 *
 * @param None
 * @retval None
 */
void Initialise_GPIO(void)
{
	//Enable clock access to GPIOA and AFIO
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;
	// Configure PA9 as Alternate Function Push-Pull, max speed 50MHz
	GPIOA->CRH |= (GPIO_CRH_MODE9_1 | GPIO_CRH_MODE9_0);  // Output mode, max speed
	GPIOA->CRH &= ~GPIO_CRH_CNF9 ;
	GPIOA->CRH |= GPIO_CRH_CNF9_1;
}


/********************************************
 * @Brief Message_Handle
 * get seperate line of lyrics
 * @param None
 * @retval None
 */
char* Message_Handle()
{
	// clear last index
	index2=0;
	// check if encounter ',' character
	while(trans_data[index1] != ',')
	{
		//increase address index and character index
		index1++;
		index2++;
		// start over when read the end of the array
		if(index1 == 499)
		{
			index1 = 0;
			index2=0;
			return FALSE;
		}
	}
	// increase a step to skip current ','character
	index1++;
	// return pointer of start of the current line
	return (&trans_data[index1]-index2-1);
}








