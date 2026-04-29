/**************************************************************************************************************
 *                  File Name       :flash.c
 *                  Overview        :flash read and write functionality.
 *                  Authors         :Wenze Pan
 **************************************************************************************************************
 *                  Copyright @ As Applicable
 **************************************************************************************************************/

/****************************************************/
/*Required Header Files */

//#include <stdio.h>
//#include <stdlib.h>
#include "GlobalDefs.h"
#include "stm32f103xb.h"
#include <string.h>

/****************************************************/
/*Local only definitions */
#define FLASH_PAGE_SIZE  				0x400U
#define KEY1							0x45670123U
#define KEY2							0xCDEF89ABU

/***************************/
/* Enumerations */

/**************************/
/*Structure types */

/*********************************************/
/* Global variable references */

/*********************************************/
/* Local only variable declaration */


/**************************/
/* Local only function prototypes */
uint32_t cal_len(uint32_t len,uint32_t div);
uint32_t read_addr(uint32_t address);
uint8_t validate(uint32_t pageAddr , uint32_t* data,uint32_t length);
uint8_t unlock();
void lock();
uint8_t page_erase(uint32_t pageAddr);
void page_program(uint32_t pageAddr,void* data);


/*********************************************
 * @brief cal_len
 * calculate the length of the words to be read
 * @param uint32_t len , uint32_t div
 * @retval uint32_t
 */
uint32_t cal_len(uint32_t len,uint32_t div)
{
	uint32_t data_length = len / div;
	if(data_length == 0)
	{
		++data_length;
		return data_length;
	}
	if(len % div != 0)
	{
		++data_length;
	}
	return data_length;
}

/*********************************************
 * @brief read_addr
 * read the value from a specific address
 * @param uint32_t address
 * @retval uint32_t
 */
uint32_t read_addr(uint32_t address)
{
	// return the dereferenced value from a specific address
	return  *(volatile  uint32_t*)address ;
}

/*********************************************
 * @brief read
 * read the flash
 * @param uint32_t pageAddr , void* data,uint32_t length
 * @retval none
 */
void read(uint32_t pageAddr , void* data,uint32_t length)
{
	uint32_t i =0;
	// calculate how many addresses(32 bit) we need read
	uint32_t data_length = cal_len(length, sizeof(uint32_t));
	// set a pointer that pointer to the same address but 32 bit to the pointer in the parameters
	uint32_t* flash_data = (uint32_t*)data;
	// record the data(32 bit) from specific address inclined every 4 bytes(32 bit)
	for(i = 0 ; i< data_length ;++i)
	{
		flash_data[i] = read_addr(pageAddr+ i*sizeof(uint32_t));
	}
}

/*********************************************
 * @brief write
 * write the flash
 * @param uint32_t pageAddr , void* data,uint32_t length
 * @retval uint8_t
 */
uint8_t write(uint32_t pageAddr , void* data,uint32_t length)
{
	uint8_t return_flag = TRUE;
	uint32_t i = 0;
	uint32_t j = 0;
	// calculate the number of addresses need to be written
	uint32_t data_len = cal_len(length, sizeof(uint32_t));
	// set a pointer that pointer to the same address but 32 bit to the pointer in the parameters
	uint32_t *flash_data = (uint32_t *)data;
	// check if the data to be written in the specific address is identical to its value already
	if(!validate(pageAddr, flash_data,data_len))
	{
		return TRUE;
	}

	// get a variable for number of pages need to be written( write pages by pages)
	uint32_t number_of_pages = cal_len(length, FLASH_PAGE_SIZE);
	// get a variable of how many 32bit addresses in one page
	const uint32_t page_size_u32 = FLASH_PAGE_SIZE /sizeof(uint32_t);
	// set the start address offset
	uint32_t page_offset = pageAddr;
	// set a 32bit buffer for storing value of address
	uint32_t page_buffer[page_size_u32] ;
	// remain number of 32 bit addresses
	uint32_t page_rem = data_len;

	// For writing data to flash , need to unlock ,erase ,write few steps
	// unlock the flash and check if successful or not
	if(!unlock())
	{
		// lock the flash if fails to unlock it
		lock();
		return FALSE;
	}

	// start writting data to flash page by page by sequence
	for(i = 0 ; i<number_of_pages ;i++)
	{
		// set the buffer value to be this page's value by address for keep unwritten data complete later
		for(j = 0 ;j<page_size_u32;j++)
		{
			page_buffer[j]=read_addr(page_offset+i*4);
		}
		// check if the remain data length exceed a page
		if(page_rem / page_size_u32 == 0)
		{
			// no remain data (0 byte0
			if(page_rem == 0)
			{
				return TRUE;
			}
			//record the wanted data to the buffer when remain data length is less than one page
			for(j = 0 ;j<page_rem;j++)
			{
				page_buffer[j]=flash_data[j+(i*page_size_u32)];
			}
		}
		else
		{
			for(j = 0 ;j<page_size_u32;j++)
			{
				//record the wanted data to the buffer
				page_buffer[j]=flash_data[j+(i*page_size_u32)];
			}
		}
		// erase the flash by page
		return_flag = page_erase(page_offset);
		if(return_flag)
		{
			lock();
			return FALSE;
		}

		// write data to the flash page by page
		page_program(page_offset, page_buffer);
		// decrease the data length
		page_rem -= page_size_u32;
		// increase the address to the next page
		page_offset += FLASH_PAGE_SIZE;
	}
	// lock the flash
	lock();
	// validate data again
	if(validate(pageAddr, flash_data,data_len))
	{
		return FALSE;
	}
	return TRUE;

}

/*********************************************
 * @brief validate
 * validate the data by read the flash checking if these are identical
 * @param uint32_t pageAddr , uint32_t* data,uint32_t length
 * @retval uint8_t
 */
uint8_t validate(uint32_t pageAddr , uint32_t* data,uint32_t length)
{
	// set the write flag
	uint8_t should_write = FALSE;
	for( uint32_t i =0 ;i<length;i++)
	{
		// keep checking if the data is the same of the value in address
		if(data[i]!=read_addr(pageAddr+i*sizeof(uint32_t)))
		{
			should_write = TRUE;
			break;
		}
	}
	return should_write;
}

/*********************************************
 * @brief unlock
 * unlock the flash for reading and programming
 * @param none
 * @retval uint8_t
 */
uint8_t unlock()
{
	// enter the key value for unlocking flash
	FLASH->KEYR = KEY1;
	FLASH->KEYR = KEY2;

	//check if status has been unlocked
	if(FLASH_CR_LOCK & FLASH->CR)
	{
		return FALSE;
	}
	else
	{
		return TRUE;
	}
}

/*********************************************
 * @brief lock
 * lock the flash for not reading and programming
 * @param none
 * @retval none
 */
void lock()
{
	//Lock the flash
	FLASH->CR |= FLASH_CR_LOCK;
}

/*********************************************
 * @brief page_erase
 * erase flash by page
 * @param uint32_t pageAddr
 * @retval uint8_t
 */
uint8_t page_erase(uint32_t pageAddr)
{
	// wait until no flash operation
	while((FLASH->SR & FLASH_SR_BSY));

	// set to erase page
	FLASH->CR |= FLASH_CR_PER;
	// set the page address
	FLASH->AR = pageAddr;
	// set to start erasing
	FLASH->CR |= FLASH_CR_STRT;
	// wait the erase process finishing
	while((FLASH->SR & FLASH_SR_BSY));

	// validate if the this page's flash has been erased
	uint8_t buffer[FLASH_PAGE_SIZE];
	memset(buffer,0XFF,FLASH_PAGE_SIZE);

	// disable to erase page
	FLASH->CR &= ~FLASH_CR_PER;

	return validate(pageAddr, (uint32_t*)buffer, FLASH_PAGE_SIZE/4);
}

/*********************************************
 * @brief page_program
 * program flash by page
 * @param uint32_t pageAddr,void* data
 * @retval none
 */
void page_program(uint32_t pageAddr,void* data)
{
	// make data to be 16bit format
	uint16_t* flash_data = (uint16_t*)data;
	// wait until no flash operation
	while((FLASH->SR & FLASH_SR_BSY));


	uint16_t* addr_track = (uint16_t*)pageAddr;
	for(int i=0 ; i<FLASH_PAGE_SIZE/2;i++)
	{
		// set to program page
		FLASH->CR |= FLASH_CR_PG;
		// write data according to address
		addr_track[i]=flash_data[i];
		// wait the program process finishing
		while((FLASH->SR & FLASH_SR_BSY));
	}
	// disable to program page
	FLASH->CR &= ~FLASH_CR_PG;
}


















