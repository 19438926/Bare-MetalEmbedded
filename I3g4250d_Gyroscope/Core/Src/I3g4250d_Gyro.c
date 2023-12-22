/**************************************************************************************************************
 *                  File Name       :I3g4250d_Gyro.h
 *                  Overview        :Gyro functionality.
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
#include "I3g4250d_Gyro.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include "HAL_SysTick.h"
#include "main.h"  // For pin configurations


/****************************************************/
/*Local only definitions */
#define GYRO_CS_PIN_ENABLE             GPIO_PIN_RESET   // Note: Active Low.
#define GYRO_CS_PIN_DISABLE 		   GPIO_PIN_SET     // Note: Active Low.

#define GYRO_CMD_READ_or               0x80                // MSB set to indicate a read command.
#define GYRO_CMD_WRITE_and             ~GYRO_CMD_READ_or   // MSB clear to indicate a write command.


// Simply Setting / c;earing CS Pin
#define GYRO_CS_DISABLE                HAL_GPIO_WritePin(GYRO_nCS_GPIO_Port,GYRO_nCS_Pin,GYRO_CS_PIN_DISABLE)
#define GYRO_CS_ENABLE                 HAL_GPIO_WritePin(GYRO_nCS_GPIO_Port,GYRO_nCS_Pin,GYRO_CS_PIN_ENABLE)

// Gyro chip register addresses
#define GYRO_REG_WHOAMI                0x0F
#define GYRO_REG_CTRL1                 0x20
#define GYRO_REG_CTRL2                 0x21
#define GYRO_REG_CTRL3                 0x22
#define GYRO_REG_CTRL4                 0x23
#define GYRO_REG_CTRL5                 0x24
#define GYRO_REG_STATUS                0x27
#define GYRO_REG_OUT_X_L               0x28
#define GYRO_REG_OUT_X_H               0x29
#define GYRO_REG_OUT_Y_L               0x2A
#define GYRO_REG_OUT_Y_H               0x2B
#define GYRO_REG_OUT_Z_L               0x2C
#define GYRO_REG_OUT_Z_H               0x2D

// Values to configure into the control registers
// See documentation for bit relevance.
#define GYRO_REG_CTRL1_SETTINGS     0xFF
#define GYRO_REG_CTRL2_SETTINGS     0x00
#define GYRO_REG_CTRL3_SETTINGS     0x00
#define GYRO_REG_CTRL4_SETTINGS     0x20
#define GYRO_REG_CTRL5_SETTINGS     0x10

// What to expect back from a working WHOAMI command response
#define WHOAMI_RESPONSE                0xD3

// Status register bit for ZYX Data available ZYXDA
#define GYRO_STATUS_ZYX_DATA_AVAILABLE_BIT        0x08

// Used to convert from Gyro values to degrees.
#define GYRO_SENSITIVITY                   ((float)0.036)

// Anything below 1 degree per second is considered 'noise'.
#define GYRO_NOISE                          ((float)1.0)

#if 1 // Change to #if 0 to allow -ve degrees rather than wrapping
#define WRAP_ANGLE_AT_360(a){      \
while (a >= 360.0)				   \
	a -= 360.0;					   \
while (a < 0.0)					   \
	a += 360.0;					   \
}
#else
#define WRAP_ANGLE_AT_360(a)
#endif

/***************************/
/* Enumerations */

/**************************/
/*Structure types */

/*********************************************/
/* Global variable references */

/*********************************************/
/* Local only variable declaration */

//The handle to the SPI interface to use
SPI_HandleTypeDef hspi;

// Angular momentum raw values from device - file scope for debug / diagnosis
int16_t s_Raw_Ang_Mom_X_ADC;
int16_t s_Raw_Ang_Mom_Y_ADC;
int16_t s_Raw_Ang_Mom_Z_ADC;


// Actual angles
float Angles_X;
float Angles_Y;
float Angles_Z;


/*********************************************/
/* Local only function prototype */

/*********************************************
 * @brief I3g4250d_Read
 * Inline function to simplify reading register data from Gyro chip.
 * @param uint8_t : Register address to read from
 * @retval uint8_t : Value read from register.
 */
static inline uint8_t Gyro_SPI_Read(uint8_t uc_RegisterAddress)
{
	// Create the Tx message needed
	uint8_t uc_Data;
	uc_Data = uc_RegisterAddress | GYRO_CMD_READ_or;

	// Transmit the read request
	GYRO_CS_ENABLE;
	HAL_SPI_Transmit(&hspi, &uc_Data, 1, 50);
	HAL_SPI_Receive(&hspi, &uc_Data, 1, 50);
	GYRO_CS_DISABLE;
	return uc_Data;
}

/*********************************************
 * @brief I3g4250d_Write
 * Inline function to simplify writing register data to the Gyro chip.
 * @param uint8_t : Register address to write to
 * @param uint8_t : Data to write to register.
 * @retval None
 */
static inline void Gyro_SPI_Write(uint8_t uc_RegisterAddress,uint8_t uc_DataToWrite)
{
	// Create the Tx Message needed.
	uint8_t Message[2];
	Message[0] = uc_RegisterAddress & GYRO_CMD_WRITE_and;
	Message[1] = uc_DataToWrite;

	// Enable CS, transfer the data and disable CS.
	GYRO_CS_ENABLE;
	HAL_SPI_Transmit(&hspi, Message, 2, 50); //Write 2 bytes - the command and the value. 50mS timeout.
	GYRO_CS_DISABLE;
}

/*********************************************
 * @brief I3g4250d_Init
 * Innitialise the Gyro on the given SPI port.
 * @param SPI_HandleTypeDef : SPI Port to use
 * @retval None
 */
void I3g4250d_Init(SPI_HandleTypeDef spi_handle)
{
	// Save the SPI interface to use
	hspi = spi_handle;

	// Cycle the Gyro CS to indicate start of communications
	GYRO_CS_DISABLE  ;

	// Write required setting to CTRL1
	Gyro_SPI_Write(GYRO_REG_CTRL1, GYRO_REG_CTRL1_SETTINGS);

	// Write required setting to CTRL2
	Gyro_SPI_Write(GYRO_REG_CTRL2, GYRO_REG_CTRL2_SETTINGS);

	// Write required setting to CTRL3
	Gyro_SPI_Write(GYRO_REG_CTRL3, GYRO_REG_CTRL3_SETTINGS);

	// Write required setting to CTRL4
	Gyro_SPI_Write(GYRO_REG_CTRL4, GYRO_REG_CTRL4_SETTINGS);

	// Write required setting to CTRL5
	Gyro_SPI_Write(GYRO_REG_CTRL5, GYRO_REG_CTRL5_SETTINGS);

	// Always make sure CS is disabled at the end of init!
	GYRO_CS_PIN_DISABLE;

	// Initialise GPIOA clock for our user button(zero angles) input
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
}

/*********************************************
 * @brief I3g4250d_Loop
 * Perform all gyro related tasks.
 * @param None
 * @retval None
 */
void I3g4250d_Loop(void)
{
	// Timestamps for when previous and new data arrived
	uint64_t ull_DataNowTimestamp;
	static uint64_t ull_DataPreviousTimestamp; // Static as retained from one call to another.

	// Difference in time between changes in angular momentum data
	float difftime;

	// Retained LastTime values.
	static float LastAngleRate_X = 0;
	static float LastAngleRate_Y = 0;
	static float LastAngleRate_Z = 0;

	// Here we're going to read the status register and also read new data when available.
	// Is new data ready? Only fetch new XYZ data if it's available!
	if (Gyro_SPI_Read(GYRO_REG_STATUS) & GYRO_STATUS_ZYX_DATA_AVAILABLE_BIT)
	{
		// Note time (and time difference since last change).
		// We do this before comms as we need this to be as accurate as possible.
		ull_DataNowTimestamp = SysTick_Get_Timestamp();

		// Fetch new XYZ Data...
		s_Raw_Ang_Mom_X_ADC = (Gyro_SPI_Read(GYRO_REG_OUT_X_L) | (Gyro_SPI_Read(GYRO_REG_OUT_X_H) << 8));
		s_Raw_Ang_Mom_Y_ADC = (Gyro_SPI_Read(GYRO_REG_OUT_Y_L) | (Gyro_SPI_Read(GYRO_REG_OUT_Y_H) << 8));
		s_Raw_Ang_Mom_Z_ADC = (Gyro_SPI_Read(GYRO_REG_OUT_Z_L) | (Gyro_SPI_Read(GYRO_REG_OUT_Z_H) << 8));

		//Now calculate actual angles by integrating over time.
		difftime = (float)SysTick_Period_MicroSeconds(ull_DataPreviousTimestamp, ull_DataNowTimestamp) / 1000000.0;
		ull_DataPreviousTimestamp = ull_DataNowTimestamp;

		float angleRate_x = (float)(s_Raw_Ang_Mom_X_ADC * GYRO_SENSITIVITY);
		float angleRate_y = (float)(s_Raw_Ang_Mom_Y_ADC * GYRO_SENSITIVITY);
		float angleRate_z = (float)(s_Raw_Ang_Mom_Z_ADC * GYRO_SENSITIVITY);

		// Compute changes in angle
		if((angleRate_x > GYRO_NOISE) || (angleRate_x < -GYRO_NOISE))
		{
		Angles_X += ((angleRate_x + LastAngleRate_X) * difftime);
		LastAngleRate_X = angleRate_x;
		}

		if((angleRate_y > GYRO_NOISE) || (angleRate_y < -GYRO_NOISE))
		{
		Angles_Y += ((angleRate_y + LastAngleRate_Y) * difftime);
		LastAngleRate_Y = angleRate_y;
		}

		if((angleRate_z > GYRO_NOISE) || (angleRate_z < -GYRO_NOISE))
		{
		Angles_Z += ((angleRate_z + LastAngleRate_Z) * difftime);
		LastAngleRate_Z = angleRate_z;
		}




		// Lastly check for a requirement to zero the angles if user button pressed.
		// Lazy implementation as no debouncing etc.
		if(GPIOA->IDR & 0x01)
		{
			Angles_X = 0.0;
			Angles_Y = 0.0;
			Angles_Z = 0.0;
		}

		// Finally, wrap at 0/359 degrees.
		WRAP_ANGLE_AT_360(Angles_X);
		WRAP_ANGLE_AT_360(Angles_Y);
		WRAP_ANGLE_AT_360(Angles_Z);
	}
}
