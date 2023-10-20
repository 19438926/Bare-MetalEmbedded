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

#include "SysTick.h"
#include "PWM.h"
#include "DAC.h"
#include"WaveformGenerator.h"
#include"USART.h"
#include"CommandHandler.h"
#include"DigitalInput.h"


/****************************************************/
/*Local only definitions */
#define FPU_CP10_FULL (0x00200000 + 0x00100000)
#define FPU_CP11_FULL (0x00800000 + 0x00400000)

//Pin definitions
#define PIN13                      (1<<13)

//PLL reconfiguration definitions
#define PLL_M     4    //bits 0-5
#define PLL_N     180   //bits 6-14
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

/*********************************************/
/* Local only variable declaration */
_WAVEFORM_DESCRIPTOR  Waveform;
static uint32_t test = 0;
static volatile uint32_t test1 = 0;



//uint64_t rinige;
//uint64_t ull_TimeStamp;
//uint64_t rinidi;
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
int main(void) {

	_WAVEFORM_DESCRIPTOR Waveform;

	//Initialise to use external clock , enable relevant gpio and peripheral etc.
	Micro_Initialisation();

	//configure our waveform descriptor structure.
	Waveform.e_WaveType = eWT_Triangular;
	Waveform.f_Amplitude = 100;
	Waveform.f_Offset = 0.0;
	Waveform.ull_Period_uS=10000;

	// Initialise the starting waveform as above.
	WaveformGenerator_Set_Waveform(Waveform);

	/******************/
	/* Loop forever */
	while (1) {

		// Update outputs in case manual updates are needed.
		WaveformGenerator_UpdateOutputs();


		//Process USART COMS
		USART_Process();



		CommandHandler_Run();

		DI_Process();


		uint8_t i = DI_Get_Input(2);

		if ( i)
		{
			GPIOG->ODR |= 1<<13;
		}
		else
		{
			GPIOG->ODR &= 0<<13;
		}


		test1 = 0;
		uint8_t *pByte = (uint8_t *)&test;
		for (uint8_t i = 1; i < 7; i++)
		{
		*pByte = i;
		pByte++;
		}



//		uint64_t count = 0;0
//		do {
//			if (SysTick_Elapsed_MicroSeconds(ull_TimeStamp) > 2000) {
//
//				PWM_Set_Duty_x10(count);
//				count++;
//				//Update our timestamp for the next iteration
//				ull_TimeStamp += SysTick_MicroSeconds_to_Counts(2000);
////
////			GPIOG->ODR ^= LED_PIN; //inverts the specific bit
//			}
//		} while (count < 1000);
//		do {
//			if (SysTick_Elapsed_MicroSeconds(ull_TimeStamp) > 2000) {
//
//				PWM_Set_Duty_x10(count);
//				count--;
//				//Update our timestamp for the next iteration
//				ull_TimeStamp += SysTick_MicroSeconds_to_Counts(2000);
//
//			}
//		} while (count > 0);



	}
}

/***********************************************
 * @brief   Micro_Initialisation
 * This function configures the micro clocks ,gpio and peripherals used by the system
 * @param None
 * @retval None
 */
void Micro_Initialisation(void) {
	//configure chip to use external clock source.
	Initialise_External_Clock();

	//Initialise SysTick Interrupt.
	SysTick_Init();

	//Initialise any GPIO required.
	Initialise_GPIO();

	//Initialise PWM Output
	PWM_Init(100000);

	//Initialise DAC
	DAC_Init();

	//Enable the FPU (floating point co-processor,access = full access)
	SCB->CPACR |= FPU_CP10_FULL | FPU_CP11_FULL ;

	//Initialise USART
	USART_Init(2000000);
}

/***********************************************
 * @brief Initialise External Clock
 * This function configures the mirco to use external 8MHz clock source.
 * @param None
 * @retval None
 */
void Initialise_External_Clock(void) {

	//Enable HSE and wait for the HSE to become ready
	RCC->CR |= RCC_CR_HSEON;
	while (!(RCC->CR & RCC_CR_HSERDY))
		;

	//Set the power enable clock and voltage regulator
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	PWR->CR |= PWR_CR_VOS;

	//Configure the FLASH Pre-fetch and Latency related settings
	FLASH->ACR |= FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN
			| FLASH_ACR_LATENCY_4WS;

	//Configure the Prescalers HCLK,PCLK1 AND PCLK2
	//AHB Prescaler
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
	//APB1 Prescaler
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;
	//APB2 Prescaler
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV2 ;

	//Configure the Main PLL
	RCC->PLLCFGR = (PLL_M << 0) | (PLL_N << 6) | (PLL_P << 16) | (PLL_Q << 24)
			| (PLL_SRC << 22);

	//Enable the PLL and wait ready
	RCC->CR |= RCC_CR_PLLON;
	while (!(RCC->CR & RCC_CR_PLLRDY))
		;

	//Select the clock source and wait for it to be set
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
		;

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
void Initialise_GPIO(void) {


	//Enable clock access to GPIOG and GPIOA
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN | RCC_AHB1ENR_GPIOAEN;

	// Enable clock access for DMA2
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

	//Set PG13 as output
	GPIOG->MODER |= (1 << 26);
	GPIOG->MODER &= ~(1 << 27);

	//PWM Output - PA.5
	//Configure GPIOA.5 to be Alternate Function.
	GPIOA->MODER |= GPIO_MODER_MODE5_1;

	//Configure the Alternate function index for pinA.5 (to map onto TIM2.CH1).
	GPIOA->AFR[0] |= GPIO_AFRL_AFRL5_0;

	//USART : Configure the pins we need (PA9(tx)/10(rx)) for relevant alternate function
	GPIOA->MODER |= GPIO_MODER_MODE9_1 | GPIO_MODER_MODE10_1;
	GPIOA->AFR[1] |= GPIO_AFRH_AFRH1_0 | GPIO_AFRH_AFRH1_1 | GPIO_AFRH_AFRH1_2;
	GPIOA->AFR[1] |= GPIO_AFRH_AFRH2_0 | GPIO_AFRH_AFRH2_1 | GPIO_AFRH_AFRH2_2;

	// Enable interrupt for pin A0 on EXTI0
	EXTI->IMR |= EXTI_IMR_IM0;
	// Trigger on rising edge
	EXTI->RTSR = EXTI_RTSR_TR0;
	// Trigger on falling edge
	EXTI->FTSR = EXTI_FTSR_TR0;
	// Finally enable the interrupt
	NVIC_EnableIRQ(EXTI0_IRQn);
}


/*****************/
/* End of files */

