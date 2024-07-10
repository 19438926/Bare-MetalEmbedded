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

#include "GlobalDefs.h"
#include "SysTick.h"
#include "CAN.h"



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
void Initialise_Internal_Clock(void);


/*********************************************
 * @brief main
 * Main entry point, carries out initialisation and calling all sub-systems within eternal loop.
 * @param None
 * @retval None
 */
int main(void)
{
	//Initialise to use external clock , enable relevant gpio and peripheral etc.
	Micro_Initialisation();

	//static uint64_t timestamp;

	/******************/
	/* Loop forever */
	while (1)
	{
		CAN_Process();
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

	//Initialise SysTick Interrupt.
	SysTick_Init();

	//Initialise any GPIO required.
	Initialise_GPIO();

	//Enable the FPU (floating point co-processor,access = full access)
	SCB->CPACR |= FPU_CP10_FULL | FPU_CP11_FULL ;

	//Initialise the CAN bus.
	CAN_Init();
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
	PWR->CR |= PWR_CR_VOS;

	//Configure the FLASH Pre-fetch and Latency related settings
	FLASH->ACR |= FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN
			| FLASH_ACR_LATENCY_0WS;
}

/********************************************
 * @Brief Initialise_GPIO
 * This function initialises any GPIO used by the system.
 *
 * Currently this includes;
 *
 * PG13: Green LED.
 *
 * @param None
 * @retval None
 */
void Initialise_GPIO(void)
{
	//Enable clock access to GPIOG and GPIOA.
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN | RCC_AHB1ENR_GPIOAEN;

	//Set PG13 as output
	GPIOG->MODER |= (1 << 26);
	GPIOG->MODER &= ~(1 << 27);

	//CAN : Configure the pin we need(PA11(CAN1_RX)/PA12(CAN1_TX)) for relevant alternate function.
	GPIOA->MODER |= GPIO_MODER_MODE11_1;
	GPIOA->MODER |= GPIO_MODER_MODE12_1;
	GPIOA->AFR[1] |= GPIO_AFRH_AFRH3_0 | GPIO_AFRH_AFRH3_3;
	GPIOA->AFR[1] |= GPIO_AFRH_AFRH4_0 | GPIO_AFRH_AFRH4_3;
}
