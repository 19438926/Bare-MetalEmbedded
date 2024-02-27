/**************************************************************************************************************
 *                  File Name       :Touch.c
 *                  Overview        :TouchScreen functionality.
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

#include "stm32f4xx_hal.h"

#include "touch.h"


/****************************************************/
/*Local only definitions */
#define STMPE811_DEVICE_ADDRESS           0x82

//	Register addresses
#define STMPE811_REG_CHP_ID_MSB           0x00 //ID address first 8 bit
#define STMPE811_REG_CHP_ID_LSB           0x01 //ID address second 8 bit
#define STMPE811_REG_SYS_CTRL1			  0x03
#define STMPE811_REG_SYS_CTRL2            0x04
#define STMPE811_REG_INT_STA			  0x0B
#define STMPE811_REG_IO_AF				  0x17
#define STMPE811_REG_ADC_CTRL1            0x20
#define STMPE811_REG_ADC_CTRL2			  0x21
#define STMPE811_REG_TSC_CTRL			  0x40
#define STMPE811_REG_TSC_CFG			  0x41
#define STMPE811_REG_FIFO_TH			  0x4A
#define STMPE811_REG_FIFO_STA			  0x4B
#define STMPE811_REG_FIFO_SIZE			  0x4C
#define STMPE811_REG_TSC_FRACT_XYZ		  0x56
#define STMPE811_REG_TSC_I_DRIVE		  0x58
#define STMPE811_REG_TSC_DATA_NON_INC	  0xD7

// Register values
#define REG_SYS_CTRL2_ADC_FCT			  0x01
#define REG_SYS_CTRL2_TS_FCT			  0x08
#define REG_SYS_CTRL2_TSC_FCT			  0x02
#define REG_SYS_CTRL2_IO_FCT			  0x04
#define REG_SYS_CTRL1_SOFT_RESET_ON   	  0x02
#define REG_SYS_CTRL1_SOFT_RESET_OFF	  0x00
#define REG_ADC_CTRL1_12_BIT_ADC		  (0x01 << 3)
#define REG_ADC_CTRL1_SAMPLE_TIME_80CLK   (0x04 << 4)
#define REG_ADC_CTRL2_3_25MHz			  0x01
#define REG_TSC_CFG_4_SAMPLES			  (0x02 << 6)
#define REG_TSC_CFG_TOUCH_DELAY_500uS	  (0x03 << 3)
#define REG_TSC_CFG_SETTLING_TIME_500uS   0x02

#define REG_TEX__TS_CTRL_STATUS			  0x80
#define REG_IO_AF_TOUCH_IO_ALL			  0xF0


#define I2CxTIMEOUT                       10 //ms

/***************************/
/* Enumerations */

/**************************/
/*Structure types */

/*********************************************/
/* Global variable references */

/*********************************************/
/* Local only variable declaration */

// I2C interface to use
I2C_HandleTypeDef hi2cX;

// ID returned from the device
uint16_t STMPE811_ID;

// Touch info
uint16_t us_TouchPointX, us_TouchPointY;
uint32_t ul_TouchCount;
uint8_t debug;

/*********************************************/
/* Local only function prototype */
// I2C Helpers
void I2Cx_WriteData(uint8_t Addr, uint8_t Reg, uint8_t Value);
uint8_t I2Cx_ReadData(uint8_t Addr, uint8_t Reg);
uint8_t I2Cx_ReadBuffer(uint8_t Addr, uint8_t Reg, uint8_t *pBuffer, uint16_t Length);
void I2Cx_Error( void );

// STMPE811 functionality
void stmpe811_TS_GetXY(uint16_t *X, uint16_t *Y);
uint8_t stmpe811_TS_DetectTouch(uint8_t DeviceAddr);
void stmpe811_TS_Start(uint8_t DeviceAddr);

/****************************/
/* HELPER FUNCTIONS */
/***************************/

/*********************************************
 * @brief I2Cx_WriteData
 * Write data to TSC register
 * @param uint8_t Addr : slave address, uint8_t Reg : register address in slave device, uint8_t Value: Register value
 * @retval None
 */
void I2Cx_WriteData(uint8_t Addr, uint8_t Reg, uint8_t Value)
{
	HAL_StatusTypeDef status = HAL_OK;

	// Write the required register
	status = HAL_I2C_Mem_Write(&hi2cX, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, &Value, 1, I2CxTIMEOUT);

	/* Check the communication status */
	if (status != HAL_OK)
	{
		// Indicate error
		I2Cx_Error();
	}
}

/*********************************************
 * @brief I2Cx_ReadData
 * Reads a single register address
 * @param uint8_t: Device Address
 * @param uint8_t: Register Address
 * @retval: Value of register read.
 */
uint8_t I2Cx_ReadData(uint8_t Addr, uint8_t Reg)
{
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t value = 0;

	// Read the register
	status = HAL_I2C_Mem_Read(&hi2cX, Addr, Reg, I2C_MEMADD_SIZE_8BIT, &value, 1,I2CxTIMEOUT);

	/* Check the communication status */
	if (status != HAL_OK)
	{
		// Indicate error
		I2Cx_Error();
	}
	return value;
}

/*********************************************
 * @brief I2Cx_ReadBuffer
 * Reads a number of registers from the device
 * @param uint8_t: Device Address
 * @param uint8_t: Register Address to read from beginning
 * @param uint8_t*: Buffer to load data read into
 * @param uint16_t: Number of registers to read.
 * @retval: 0 / 1 indicating error(0=Ok)
 */
uint8_t I2Cx_ReadBuffer(uint8_t Addr, uint8_t Reg, uint8_t *pBuffer, uint16_t Length)
{
	uint8_t ErrorState = 0;
	HAL_StatusTypeDef status = HAL_OK;

	// Perform the required read / length
	status = HAL_I2C_Mem_Read(&hi2cX, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, pBuffer, Length, I2CxTIMEOUT);

	/* Check the communication status */
	if (status != HAL_OK)
	{
		// Indicate error
		I2Cx_Error();
		ErrorState = 1;
	}

	return ErrorState;
}

/*********************************************
 * @brief I2Cx_Error
 * Catch the error, stop
 * @param None
 * @retval: None
 */
void I2Cx_Error(void)
{
	// Debugging break point trap
	asm("nop");
}

/*********************************************
 * @brief stmpe811_IO_EnableAF
 * Enable GPIO Alternate function (for use as 4 wire Touch Interface).
 * @param uint8_t: Device Address
 * @param uint8_t: Pins to set to AF
 * @retval: None
 */
void stmpe811_IO_EnableAF(uint8_t DeviceAddr, uint8_t IO_Pin)
{
	uint8_t tmp = 0;

	// Get the current register value
	tmp = I2Cx_ReadData(DeviceAddr, STMPE811_REG_IO_AF);

	// Enable the selected pins alternate function
	tmp &= ~(uint8_t)IO_Pin;

	// Write back the new register value
	I2Cx_WriteData(DeviceAddr,STMPE811_REG_IO_AF,tmp);
}

/*********************************************
 * @brief stmpe811_TS_GetXY
 * Get the XY touch coordinates
 * @param uint16_t*: Pointer to X
 * @param uint16_t*: Pointer to Y
 * @retval: None
 */
void stmpe811_TS_GetXY(uint16_t *X,uint16_t *Y)
{
	uint8_t dataXYZ[4];
	uint32_t uldataXYZ;

	// rRead the required registers
	I2Cx_ReadBuffer(STMPE811_DEVICE_ADDRESS,STMPE811_REG_TSC_DATA_NON_INC,dataXYZ,sizeof(dataXYZ));

	// Calculate positions values
	uldataXYZ = (dataXYZ[0]<<24)|(dataXYZ[1]<<16)|(dataXYZ[2]<<8)|(dataXYZ[3]<<0);
	*X = (uldataXYZ >> 20) & 0x00000FFF;
	*Y = (uldataXYZ >> 8)  & 0x00000FFF;

	// Reset FIFO
	I2Cx_WriteData(STMPE811_DEVICE_ADDRESS, STMPE811_REG_FIFO_STA, 0x01);
	// Enable the FIFO again
	I2Cx_WriteData(STMPE811_DEVICE_ADDRESS, STMPE811_REG_FIFO_STA, 0x00);
}

/*********************************************
 * @brief stmpe811_TS_DetectTouch
 * Return if there is touch detected or not
 * @param uint8_t DeviceAddr: Device address on communication Bus.
 * @retval: Touch detected state(0/1 = TouchDetected).
 */
uint8_t stmpe811_TS_DetectTouch(uint8_t DeviceAddr)
{
	uint8_t uc_State;
	uint8_t uc_Touched = 0;

	// Read the touch status register and check for the status of the Z bit.
	uc_State = ((I2Cx_ReadData(DeviceAddr, STMPE811_REG_TSC_CTRL) & REG_TEX__TS_CTRL_STATUS) == REG_TEX__TS_CTRL_STATUS);

	// Bit set?
	if(uc_State >0)
	{
		if(I2Cx_ReadData(DeviceAddr, STMPE811_REG_FIFO_SIZE)>0)
		{
			// Touch detected
			uc_Touched = 1;
		}
	}
	else
	{
		// No touch detected
		// Reset FIFO
		I2Cx_WriteData(STMPE811_DEVICE_ADDRESS, STMPE811_REG_FIFO_STA, 0x01);
		// Enable the FIFO again
		I2Cx_WriteData(STMPE811_DEVICE_ADDRESS, STMPE811_REG_FIFO_STA, 0x00);
	}
	return uc_Touched;
}

/*********************************************
 * @brief stmpe811_TS_Start
 * Start Touch Screen operations - initialise as needed
 * @param uint16_t: DeviceAddress
 * @retval: None
 */
void stmpe811_TS_Start(uint8_t DeviceAddr)
{
	uint8_t uc_Mode;

	// Get the current register value for CTRL2
	uc_Mode = I2Cx_ReadData(DeviceAddr,STMPE811_REG_SYS_CTRL2);

	// Set the Functionalities to be Enabled
	uc_Mode &= ~(REG_SYS_CTRL2_IO_FCT);

	// Write the new register value
	I2Cx_WriteData(DeviceAddr,STMPE811_REG_SYS_CTRL2,uc_Mode);

	// Select TSC pins in TSC alternate mode
	stmpe811_IO_EnableAF(DeviceAddr, REG_IO_AF_TOUCH_IO_ALL);

	// Set the Functionalities to be Enabled
	uc_Mode &= ~(REG_SYS_CTRL2_TS_FCT | REG_SYS_CTRL2_ADC_FCT | REG_SYS_CTRL2_TSC_FCT);
	// Set the new register value
	// Note: Temperature sensor needed to compensate for touch-screen parameters.
	// ADC is used for 4-wire touch-screen operation.
	I2Cx_WriteData(DeviceAddr, STMPE811_REG_SYS_CTRL2,uc_Mode);

	// Select Sample Time, bit number and ADC Reference
	I2Cx_WriteData(DeviceAddr, STMPE811_REG_ADC_CTRL1, REG_ADC_CTRL1_12_BIT_ADC | REG_ADC_CTRL1_SAMPLE_TIME_80CLK);

	//HAL_Delay(5);

	// Select the ADC clock speed: 3.25 MHz
	I2Cx_WriteData(DeviceAddr, STMPE811_REG_ADC_CTRL2, REG_ADC_CTRL2_3_25MHz);

	//debug = I2Cx_ReadData(DeviceAddr, STMPE811_REG_ADC_CTRL2);

	// Select 2 nF filter capacitor
	/* Configuration:
	 * Touch average control	: 4 samples(0x02 << 6)
	 * Touch delay time			: 500uS(0x03 << 3)
	 * Panel driver setting time: 500uS(0x02)
	 */
	I2Cx_WriteData(DeviceAddr, STMPE811_REG_TSC_CFG, REG_TSC_CFG_4_SAMPLES | REG_TSC_CFG_SETTLING_TIME_500uS | REG_TSC_CFG_TOUCH_DELAY_500uS);

	// Configure the Touch FIFO threshold: single point reading
	I2Cx_WriteData(DeviceAddr, STMPE811_REG_FIFO_TH, 0x01);

	// Clear the FIFO memory content
	I2Cx_WriteData(DeviceAddr, STMPE811_REG_FIFO_STA, 0x01);

	// Put the FIFO back into operation mode
	I2Cx_WriteData(DeviceAddr, STMPE811_REG_FIFO_STA, 0x00);

	/* Set the range and accuracy pf the pressure measurement (Z):
	 * Fractional part  :7
	 * Whole part       :1
	 */
	I2Cx_WriteData(DeviceAddr, STMPE811_REG_TSC_FRACT_XYZ ,0x01);

	// Set the driving capability (limit) of the device for TSC pins: 50mA
	I2Cx_WriteData(DeviceAddr, STMPE811_REG_TSC_I_DRIVE, 0x01);

	/* Touch screen control configuration (enable TSC):
	 * No window tracking index
	 * XYZ acquisition mode
	 */
	I2Cx_WriteData(DeviceAddr, STMPE811_REG_TSC_CTRL, 0x01);

	// Clear all the status pending bits if any
	I2Cx_WriteData(DeviceAddr, STMPE811_REG_INT_STA, 0xFF);

}

/*********************************************
 * @brief Touch_Init
 * Initialise Touch interface
 * @param I2C_HandleTypeDef : Handle to I2C interface to use
 * @retval None
 */
void Touch_Init (I2C_HandleTypeDef New_hi2cX)
{
	// Store the handle to the relevant I2C interface to use
	hi2cX = New_hi2cX;

	// Shut down / restart the device
	I2Cx_WriteData(STMPE811_DEVICE_ADDRESS, STMPE811_REG_SYS_CTRL1, REG_SYS_CTRL1_SOFT_RESET_ON);
	HAL_Delay(10);
	I2Cx_WriteData(STMPE811_DEVICE_ADDRESS, STMPE811_REG_SYS_CTRL1, REG_SYS_CTRL1_SOFT_RESET_OFF);
	HAL_Delay(2);

	// Start the touch screen operations.
	stmpe811_TS_Start(STMPE811_DEVICE_ADDRESS);

}

/*********************************************
 * @brief Touch_Process
 * Run the touch interface
 * @param None
 * @retval None
 */
void Touch_Process(void)
{
	// Has touch been indicated?
	uint8_t uc_TouchDetected = stmpe811_TS_DetectTouch(STMPE811_DEVICE_ADDRESS);
	if(uc_TouchDetected)
	{
		// Increment touch count and fetch X / Y Values for the touch point.
		ul_TouchCount++;
		stmpe811_TS_GetXY(&us_TouchPointX, &us_TouchPointY);
	}
}
