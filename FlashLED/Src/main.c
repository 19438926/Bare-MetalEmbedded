
#include "stm32f429xx.h"



#define PIN13                      (1<<13)
#define LED_PIN                     PIN13



int main(void)
{
   RCC_TypeDef*pRCC=(RCC_TypeDef*)RCC_BASE;
   pRCC->AHB1ENR |=RCC_AHB1ENR_GPIOGEN;


	GPIO_TypeDef*pGPIOG=(GPIO_TypeDef*)GPIOG_BASE;
	pGPIOG->MODER |= (1<<26);
	pGPIOG->MODER &= ~(1<<27);


    /* Loop forever */
	for(;;){
	pGPIOG->ODR ^=LED_PIN;

	}
}
