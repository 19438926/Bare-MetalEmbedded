/**************************************************************************************************************
 *                  File Name       :SPI.c
 *                  Overview        :SPI wirte and read functionality.
 *                  Authors         :Wenze Pan
 **************************************************************************************************************
 *                  Copyright @ As Applicable
 **************************************************************************************************************/

/****************************************************/
/*Required Header Files */
#include <stdio.h>
#include <W5500.h>
#include "main.h"

/****************************************************/
/*Local only definitions */
#define W5500_CS_PIN_ENABLE			GPIO_PIN_RESET   // Note: Active Low.
#define W5500_CS_PIN_DISENABLE		GPIO_PIN_SET   //   Note: Off Active High.

// Simply Setting / c;earing CS Pin
#define W5500_CS_DISABLE                HAL_GPIO_WritePin(CS_GPIO_Port,CS_Pin,W5500_CS_PIN_DISENABLE)
#define W5500_CS_ENABLE                 HAL_GPIO_WritePin(CS_GPIO_Port,CS_Pin,W5500_CS_PIN_ENABLE)

//W5500 Chip block&register addresses
#define COMMON_REGISTER                 0X0
#define OFFSET_CHIP_VERSION				0x0039

/***************************/
/* Enumerations */

/**************************/
/*Structure types */

/*********************************************/
/* Global variable references */

/*********************************************/
/* Local only variable declaration */
uint8_t address_offset[2] = {0,0X39};
uint8_t block_phase = 0B00000000;
SPI_HandleTypeDef* hspi;
uint8_t ReData[50];


/**************************/
/* Local only function prototypes */


/*********************************************
 * @brief set_spi
 * set spi instance
 * @param SPI_HandleTypeDef* spi
 * @retval uint8_t
 */
void set_spi(SPI_HandleTypeDef* spi)
{
	hspi = spi;
}

/*********************************************
 * @brief SPI_Read
 * read data
 * @param uint8_t number
 * @retval uint8_t
 */
uint8_t W5500_Read(uint8_t number)
{

	W5500_CS_ENABLE;
	HAL_SPI_Transmit(hspi,address_offset, 2, HAL_MAX_DELAY);
	HAL_SPI_Transmit(hspi,(uint8_t*)&block_phase, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(hspi,  ReData, 1, HAL_MAX_DELAY);
	W5500_CS_DISABLE;
	return 1;
}

///*********************************************
// * @brief SPI_Write
// * read data
// * @param uint8_t number
// * @retval uint8_t
// */
//uint8_t SPI_Write(uint8_t number)
//{
//	HAL_SPI_Transmit(hspi, &address_offset, 2, 10);
//}
