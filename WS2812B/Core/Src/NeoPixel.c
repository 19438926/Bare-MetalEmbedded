/**************************************************************************************************************
 *                  File Name       :NeoPixel.c
 *                  Overview        :NeoPixel driver
 *                  Authors         :Wenze Pan
 **************************************************************************************************************
 *                  Copyright @ As Applicable
 **************************************************************************************************************/

/****************************************************/
/*Required Header Files */
#include "NeoPixel.h"

/****************************************************/
/*Local only definitions */
// set many LEDs we want in our RGB LED array
// IMPORTANT: each LED can use up to 60mA, so 100 LEDS for example worst case can draw 100*60mA = 6000mA = 6 Amps!!!
#define ONE_HIGH      __asm__ volatile("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n");
#define ONE_LOW		  __asm__ volatile("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n");

/***************************/
/* Enumerations */

/**************************/
/*Structure types */

/*********************************************/
/* Global variable references */

// used to manipulate text

/*********************************************/
/* Local only variable declaration */

/**************************/
/* Local only function prototypes */

/*********************************************
 * @brief WS2812_Reset
 * Reset WS2812
 * @param None
 * @retval None
 */
void WS2812_Reset( void )
{
/*
Description:    This function resets and latches the RGB LEDs by writing a logic LOW for an amount of time .
                Depending on manufacturer 50-200uS is usually required.

Parms:          None.

Returns:        Nothing.
*/

uint16_t counter;

// Set pin to LOW
GPIOA->ODR &= ~GPIO_ODR_7;


//// Hold serial bus LOW for 50-200+ us, TODO confirm timing of this again
for( counter = 0; counter < 275; counter++)
    {
    asm volatile("nop\n");
    asm volatile("nop\n");
    asm volatile("nop\n");
    asm volatile("nop\n");
    asm volatile("nop\n");
    asm volatile("nop\n");
    asm volatile("nop\n");
    asm volatile("nop\n");
    asm volatile("nop\n");
    asm volatile("nop\n");
    asm volatile("nop\n");
    asm("nop\n");
    asm("nop\n");
    asm("nop\n");
    } // end for

} // end WS2812_Reset

/*********************************************
 * @brief WS2812_WriteLeds
 * Write leds
 * @param None
 * @retval None
 */
void WS2812_WriteLeds( WS2812_LED *leds, uint8_t numLeds )
{
/*
Description:  This function writes all the LEDs in a single serial transmission.

Parms:        leds - Pointer to array of LED structs.
              numLeds - Number of LEDs in array.

Returns:      Nothing.
*/

uint8_t index;

// timing is critical, disable interrupts for a moment here...
//asm("cli");
__disable_irq();

for( index = 0; index < numLeds; index++)
    {
    // WS2812 requires data in GRB format, MSB to LSB: G7..G0 | R7..R0| B7..B0
    WS2812_SendByte(leds[ index ].green);
    WS2812_SendByte(leds[ index ].red);
    WS2812_SendByte(leds[ index ].blue);
    } // end for

// re-enable interrupts
//asm("sei");
__enable_irq();

} // end WS2812_WriteLeds

/*********************************************
 * @brief WS2812_SendByte
 * This function sends a single byte to the WS2812x RGB serial LED (or clone). The protocol is based on a
temporal NRZ protocol, where a "1" and "0" is determined by a bit sent as a HIGH/LOW ratio. The length of
the HIGH pulse determines if the bit is "1" or "0", and after each HIGH, a LOW can be up to 20 uS, but most
drivers shoot for a 1.25uS total bit time. Based on this timing spec to send a "1", you must send a pulse
of HIGH 0.8uS followed by 0.45 us LOW (up to 20 us actually for the LOW), and for a "0", you must send a pulse
of 0.4uS, followed by 0.85uS (up to 20uS). The IMPORTANT thing is the HIGH portion only. Finally, to "reset"
the RGB and latch the sent data, you must hold the signal line LOW for 50-80uS (depending on vendor of clone)
so 100-200uS is a good safe number.

The function requires accurate timing, so assumes a 16mhz clock, and interrupts MUST be off. This function was
tunned with an oscope. The use of NOPs are not necessary, C code code be used, but this is cleaner to generate
1 cycle timing delays, and we don't have to guess.
 * @param data8 - 8-bit data to send.
 * @retval None
 */
void WS2812_SendByte( uint8_t data8 )
{
// Bit position counter
int8_t counter = 7;

// Use while loop, so we are sure of where code execution occurs, "for" can be optimized more, don't want that.
while( counter >= 0)
    {
    if ( data8 & (1 << counter ) )
        {
        // send 1: 0.8 us high, 0.45 us low, note: digitalWrite() is too slow, so we use direct port manipulation
        //digitalWrite( NEOPIXEL_DATA_PIN, 1 );
    	GPIOA->ODR |= GPIO_ODR_7;

        // use inline asm for fun to create proper time delay
    	ONE_HIGH;
        }
        else
        {
        //send 0: 0.4 us high, 0.85 us low, note: digitalWrite() is too slow, so we use direct port manipulation
        //digitalWrite( NEOPIXEL_DATA_PIN, 1 );
		GPIOA->ODR |= GPIO_ODR_7;

        // use inline asm for fun to create proper time delay
		ONE_LOW;
        } // end else

    // no need to add nops here, sw will take time to load next bit and keep our timing protocol close enough
    //digitalWrite( NEOPIXEL_DATA_PIN, 0 );
    GPIOA->ODR &= ~GPIO_ODR_7;
    // asm("nop");

    counter--;
    } // end while

} // end WS2812_SendByte

