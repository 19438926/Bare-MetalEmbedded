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
#include "stm32f429xx.h"
#include "I3g4250d_Gyro.h"
#include <stdint.h>
#include "SysTick.h"



/****************************************************/
/*Local only definitions */
//#define GYRO_CS_PIN_ENABLE             GPIO_PIN_RESET   // Note: Active Low.
//#define GYRO_CS_PIN_DISABLE 		   GPIO_PIN_SET     // Note: Active Low.

#define GYRO_CMD_READ_or               0x80                // MSB set to indicate a read command.
#define GYRO_CMD_WRITE_and             ~GYRO_CMD_READ_or   // MSB clear to indicate a write command.


// Simply Setting / c;earing CS Pin
#define GYRO_CS_DISABLE                GPIOC->ODR |= GPIO_ODR_OD1
#define GYRO_CS_ENABLE                 GPIOC->ODR &= ~GPIO_ODR_OD1

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
#define GYRO_SENSITIVITY                   ((float)0.070)

// Anything below 1 degree per second is considered 'noise'.
#define GYRO_NOISE                          ((float)2.0)

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
 * @brief SPI_Init
 * Initialise the SPI.
 * @param None
 * @retval None
 */
void SPI_Init()
{
	//************************//
	// Configure the STM32F429 in master mode SPI.

	// Enable the bus for SPI5
	RCC->APB2ENR |= RCC_APB2ENR_SPI5EN;

	// Set the Baud rate to be APB2 /32
	SPI5->CR1 |= SPI_CR1_BR_2;
	SPI5->CR1 &= ~(SPI_CR1_BR_0|SPI_CR1_BR_1);

	// Set CPOL to be LOW , Set CPHA to be 1 edge.(CPOL = 0,CPHA = 0)
	SPI5->CR1 &= ~SPI_CR1_CPOL;
	SPI5->CR1 &= ~SPI_CR1_CPHA;

	// Set the Data frame format (8 bits)
	SPI5->CR1 &= ~SPI_CR1_DFF;

	// Set the frame format(MSB)
	SPI5->CR1 &= ~SPI_CR1_LSBFIRST;

	// Set the NSS to be software management mode
	SPI5->CR1 |= SPI_CR1_SSM;
	SPI5->CR1 |= SPI_CR1_SSI;

	// Set the STM32F429 to be master
	SPI5->CR1 |= SPI_CR1_MSTR;

	// Enable the SPI
	SPI5->CR1 |= SPI_CR1_SPE;

}


/*********************************************
 * @brief I3g4250d_Check
 * Check if gyroscope works normally.
 * @param None
 * @retval None
 */
void I3g4250d_Check()
{
	//*************************//
	// Initialise the gyroscope.
	// A 2 byte buffer for transmitting commands
	uint8_t spiTxRxBuff[2];

	// Cycle the Gyro CS to indicate start of communications.
	 GYRO_CS_DISABLE;
	 GYRO_CS_ENABLE;

	// Create the required message
	spiTxRxBuff[0] = (GYRO_REG_WHOAMI | GYRO_CMD_READ_or);

	// Transmit read WHOAMI message
	SPI_Transmit(&spiTxRxBuff[0], 1);

	// Receive the reply from gyroscope
	SPI_Receive(&spiTxRxBuff[1], 1);

	// Cancel the slave
	GYRO_CS_DISABLE;

	// Check if it get the correct respond
	if(spiTxRxBuff[1] != WHOAMI_RESPONSE)
	{
		asm("nop");

	}else
	{
		asm("nop");
	}
}

/*********************************************
 * @brief SPI_Transmit
 * SPI transmit function.
 * @param uint8_t :*data , int: size
 * @retval None
 */
void SPI_Transmit (uint8_t *data, int size)
{
	int i=0;
	while (i<size)
	{
		while (!((SPI5->SR)&SPI_SR_TXE)) {};  // wait for TXE bit to set -> This will indicate that the buffer is empty
		SPI5->DR = data[i];  // load the data into the Data Register
		i++;
	}
	while (!((SPI5->SR)&SPI_SR_TXE)) {};  // wait for TXE bit to set -> This will indicate that the buffer is empty
	while (((SPI5->SR)&SPI_SR_BSY)) {};  // wait for BSY bit to Reset -> This will indicate that SPI is not busy in communication

	//  Clear the Overrun flag by reading DR and SR
	SPI5->DR;
	SPI5->SR;
}

/*********************************************
 * @brief SPI_Receive
 * SPI receive function.
 * @param uint8_t :*data , int: size
 * @retval None
 */
void SPI_Receive (uint8_t *data, int size)
{
	while (size)
	{
		while (((SPI5->SR)&SPI_SR_BSY)) {};  // wait for BSY bit to Reset -> This will indicate that SPI is not busy in communication
		SPI5->DR = 0;  // send dummy data for triggering receiving the message
		while (!((SPI5->SR) & SPI_SR_RXNE)){};  // Wait for RXNE to set -> This will indicate that the Rx buffer is not empty
		*data++ = (SPI5->DR);
		size--;
	}
}

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
	SPI_Transmit(&uc_Data, 1);
	SPI_Receive(&uc_Data, 1);
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
	SPI_Transmit( Message, 2); //Write 2 bytes - the command and the value. 50mS timeout.
	GYRO_CS_DISABLE;
}

/*********************************************
 * @brief I3g4250d_Init
 * Innitialise the Gyro on the given SPI port.
 * @param None
 * @retval None
 */
void I3g4250d_Init()
{
	// Cycle the Gyro CS to indicate start of communications
	GYRO_CS_ENABLE ;

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
	GYRO_CS_DISABLE;
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
			Angles_X += (LastAngleRate_X  *difftime);
			LastAngleRate_X = angleRate_x;
		}

			if((angleRate_y > GYRO_NOISE) || (angleRate_y < -GYRO_NOISE))
		{
			Angles_Y += (LastAngleRate_Y  * difftime);
			LastAngleRate_Y = angleRate_y;
		}

		if((angleRate_z > GYRO_NOISE) || (angleRate_z < -GYRO_NOISE))
		{
			Angles_Z += (LastAngleRate_Z  * difftime);
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

/*********************************************
 * @brief Get_Y_Data
 * Get Y angle data divided by 360
 * @param None
 * @retval float: Y angle percentage
 */
float Get_Y_Data()
{
	float WaveformY;
	WaveformY = Angles_Y / 360.0;
	return	WaveformY;
}
