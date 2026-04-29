/**************************************************************************************************************
 *                  File Name       :NeoPixel.h
 *                  Overview        :NeoPixel driver
 *                  Authors         :Wenze Pan
 **************************************************************************************************************
 *                  Copyright @ As Applicable
 **************************************************************************************************************/

#ifndef INC_NEOPIXEL_H_
#define INC_NEOPIXEL_H_

#include "stm32f0xx_hal.h"
/***************************************************/
/* Definitions required by this module */
#define NUM_LEDS    6

// Our color palette
#define RGB_COLOR_BLACK       { 0x00, 0x00, 0x00 }
#define RGB_COLOR_GRAY        { 0x80, 0x80, 0x80 }
#define RGB_COLOR_WHITE       { 0xFF, 0xFF, 0xFF }
#define RGB_COLOR_RED         { 0xFF, 0x00, 0x00 }
#define RGB_COLOR_GREEN       { 0x00, 0xFF, 0x00 }
#define RGB_COLOR_BLUE        { 0x00, 0x00, 0xFF }
#define RGB_COLOR_ORANGE      { 0xFF, 0xA5, 0x00 }
#define RGB_COLOR_YELLOW      { 0xFF, 0xFF, 0x00 }
#define RGB_COLOR_VIOLET      { 0xEE, 0x82, 0xEE }
#define RGB_COLOR_INDIGO      { 0x4B, 0x00, 0x82 }
#define RGB_COLOR_CYAN        { 0x00, 0xFF, 0xFF }
#define RGB_COLOR_MAGENTA     { 0xFF, 0x00, 0xFF }
#define RGB_COLOR_AQUA        { 0x00, 0xCE, 0xD1 }



/***************************************************/
/* Types used by this module */

// Type holds RGB value, WS2812 sends data in GRB order, so keep that in mind
typedef struct WS2812_LED_TYP
{
	uint8_t red;      // 8-bit red
	uint8_t green;    // 8-bit green
	uint8_t blue;     // 8-bit blue
} WS2812_LED;

/**************************************************/
/* Externally available variable */

/*************************************************/
/*Externally available functions */
void WS2812_Reset( void );
void WS2812_WriteLeds(WS2812_LED *leds, uint8_t numLeds );
void WS2812_SendByte( uint8_t data8 );


#endif /* INC_NEOPIXEL_H_ */
