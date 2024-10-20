/**************************************************************************************************************
 *                  File Name       :main.c
 *                  Overview        :Micro and System initialisation  as well as external loop.
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

//Pin definitions
#define PIN13                      (1<<13)


//PLL reconfiguration definitions
#define PLL_M     4    //bits 0-5
#define PLL_N     72   //bits 6-14
#define PLL_P     0    //bits 16-17
#define PLL_Q     3    //bits 24-27
#define PLL_SRC   1    //bits 22

//GPIO Pins in use
#define LED_PIN                     PIN13

/***************************/
/* Enumerations */

/**************************/
/*Structure types */

/*********************************************/
/* Global variable references */

/**************************/
/* Local function prototypes */
void Micro_Initialisation (void);
void Initialise_GPIO (void );
void Initialise_External_Clock (void);


/*********************************************
 * @brief main
 * Main entry point, carries out initialisation and calling all sub-systems within eternal loop.
 * @param None
 * @retval None
 */
int main(void){
	//Initialise to use external clock , enable relevant gpio and peripheral etc.
	Micro_Initialisation();

    /******************/
    /* Loop forever */
	while(1){
	GPIOG->ODR ^=LED_PIN;//inverts the specific bit
    }
}

/***********************************************
 * @brief   Micro_Initialisation
 * This function configures the micro clocks ,gpio and peripherals used by the system
 * @param None
 * @retval None
 */
void Micro_Initialisation(void){
	//configure chip to use external clock source.
	Initialise_External_Clock();

	//Initialise any GPIO required.
	Initialise_GPIO();
}

/***********************************************
 * @brief Initialise External Clock
 * This function configures the mirco to use external 8MHz clock source.
 * @param None
 * @retval None
 */
void Initialise_External_Clock (void){


    //Enable HSE and wait for the HSE to become ready
	RCC->CR |= RCC_CR_HSEON;
	while (!(RCC->CR & RCC_CR_HSERDY));


	//Set the power enable clock and voltage regulator
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	PWR->CR |= PWR_CR_VOS;

	//Configure the FLASH Pre-fetch and Latency related settings
	FLASH->ACR |= FLASH_ACR_DCEN | FLASH_ACR_ICEN | FLASH_ACR_PRFTEN
			| FLASH_ACR_LATENCY_2WS;

    //Configure the Prescalers HCLK,PCLK1 AND PCLK2
    //AHB Prescaler
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
    //APB1 Prescaler
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;
    //APB2 Prescaler
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;

    //Configure the Main PLL
	RCC->PLLCFGR = (PLL_M << 0) | (PLL_N << 6) | (PLL_P << 16) | (PLL_Q << 24)
			| (PLL_SRC << 22);

    //Enable the PLL and wait ready
	RCC->CR |= RCC_CR_PLLON;
	while (!(RCC->CR & RCC_CR_PLLRDY));


    //Select the clock source and wait for it to be set
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

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
void Initialise_GPIO (void){
    //Enable clock access to GPIOG
     RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;
     //Set PG13 as output
	 GPIOG->MODER |= (1 << 26);
	 GPIOG->MODER &= ~(1 << 27);
  }

/*****************/
/* End of files */

