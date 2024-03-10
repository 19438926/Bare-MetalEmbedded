/**************************************************************************************************************
 *                  File Name       :STMPE811.h
 *                  Overview        :touch screen controller functionality.
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
#include "STMPE811.h"
#include "GlobalDefs.h"


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

// ID returned from the device
uint16_t STMPE811_ID;
uint8_t Value[2];
uint8_t dataXYZ[4];

// Touch info
uint16_t us_TouchPointX, us_TouchPointY;
uint32_t ul_TouchCount;
uint8_t data;

/*********************************************/
/* Local only function prototype */
// I2C communication operation
void I2C_Start();
void I2C_Call_Address(uint8_t Address);
void I2C_Transmit(uint8_t *Data,uint8_t Number);
void I2C_Receive(uint8_t *Data,uint8_t Number,uint8_t Address);
void I2C_Stop();

uint8_t I2CStartNonBlocking();
uint8_t I2C_Call_AddressNonBlocking(uint8_t Address);
uint8_t I2C_TransmitNonBlocking(uint8_t *Data);
uint8_t I2C_ReceiveNonBlocking(uint8_t *Data,uint8_t Address,uint8_t Number);

// I2C Helpers for touch screen controller
void I2Cx_WriteData(uint8_t Addr, uint8_t Reg, uint8_t Value);
uint8_t I2Cx_ReadData(uint8_t Addr, uint8_t Reg);
void I2Cx_ReadBuffer(uint8_t Addr, uint8_t Reg, uint8_t *pBuffer, uint16_t Length);
void I2Cx_Error( void );

uint8_t I2Cx_ReadDataNonBlocking(uint8_t Addr, uint8_t Reg);
uint8_t I2Cx_ReadBufferNonBlocking(uint8_t Addr, uint8_t Reg, uint8_t *pBuffer, uint16_t Length);

// STMPE811 functionality
void stmpe811_TS_GetXY(uint16_t *X, uint16_t *Y);
uint8_t stmpe811_TS_DetectTouch(uint8_t DeviceAddr);
void stmpe811_TS_Start(uint8_t DeviceAddr);
uint8_t detectTouchNonBlocking(uint8_t Address);
uint8_t GetXYNonBLocking(uint16_t *X,uint16_t *Y);



/*********************************************
 * @brief I2C_Init
 * Initialise the I2C.
 * @param None
 * @retval None
 */
void I2C_Init()
{
	//************************//
	// Configure the STM32F429 in master mode I2C.

	// Enable the bus for I2C3.
	RCC->APB1ENR |= RCC_APB1ENR_I2C3EN;

	// Program the correct clock value in regard APB1 45MHz(b101101)
	I2C3->CR2 |= I2C_CR2_FREQ_5 | I2C_CR2_FREQ_3 |I2C_CR2_FREQ_2 | I2C_CR2_FREQ_0;

	// Configure the clock control register
	//100K baud rate:  Th=5000ns
	//45Mhz means the Tpclk1 = 22.2ns
	//CCR = Th/Tpclk1=225(b1110 0001)
	I2C3->CCR |= 0b11100001;

	// Configure the rise time register(1000ns)
	// Value: 1000/22.2 +1 =46(b101110)
	I2C3->TRISE |= 0b101110 ;

	// Enable the I2C
	I2C3->CR1 |= I2C_CR1_PE;
}

/*********************************************
 * @brief I2C_Start
 * Start or Restart I2C communication.
 * @param None
 * @retval None
 */
void I2C_Start()
{

	//Enable the acknowledge
	I2C3->CR1 |= I2C_CR1_ACK;

	//Start the I2C communication
	I2C3->CR1 |= I2C_CR1_START;

	//Wait the SB bit set
	while(!(I2C3->SR1 & I2C_SR1_SB));
}

/*********************************************
 * @brief I2C_Call_Address
 * Transmit slave address byte
 * @param uint8_t Address
 * @retval None
 */
void I2C_Call_Address(uint8_t Address)
{
	// Send the address
	I2C3->DR = Address;

	while(!(I2C3->SR1 & I2C_SR1_ADDR));// Wait the ADDR to be set and read SR1 register

	// Read SR1/SR2 register by rule
	I2C3->SR1; I2C3->SR2;
}

/*********************************************
 * @brief I2C_Transmit
 * Transmit data to slave
 * @param uint8_t *Data, uint8_t Number
 * @retval None
 */
void I2C_Transmit(uint8_t *Data,uint8_t Number)
{
	for(int i=0;i<Number;i++)
	{
		while(!(I2C3->SR1 & I2C_SR1_TXE)){}; // Wait Tx Buffer empty
		I2C3->DR = Data[i]; // Continue transmitting data
	}
	while(!(I2C3->SR1 & I2C_SR1_BTF)){}; // Wait byte transmit finishing
}

/*********************************************
 * @brief I2C_Receive
 * Receive data from slave
 * @param uint8_t *Data, uint8_t Number
 * @retval None
 */
void I2C_Receive(uint8_t *Data,uint8_t Number,uint8_t Address)
{

	if(Number == 1)
	{
		// Send the address
		I2C3->DR = Address;

		while(!(I2C3->SR1 & I2C_SR1_ADDR));// Wait the ADDR to be set and read SR1 register

		I2C3->CR1 &= ~(I2C_CR1_ACK); //disable acknowledge bit

		// Read SR1/SR2 register by rule
		I2C3->SR1; I2C3->SR2;

		I2C_Stop();	//Stop the communication
		while(!(I2C3->SR1 & I2C_SR1_RXNE)){};//Wait the receive buffer not empty
		Data[0] = I2C3->DR; // Collect the data
	}
	else
	{
		I2C_Call_Address(Address); // Call slave
		for(int i = 0;i<Number-2;i++)//Receive data until last two bytes
		{
			while(!(I2C3->SR1 & I2C_SR1_RXNE)){};//Wait the receive buffer not empty
			Data[i] = I2C3->DR; // Collect the data
		}
		while(!(I2C3->SR1 & I2C_SR1_RXNE)){};//Wait the receive buffer not empty
		Data[Number-2] = I2C3->DR; // Collect the data
		I2C3->CR1 &= ~(I2C_CR1_ACK); //disable acknowledge bit
		I2C_Stop();	//Stop the communication
		while(!(I2C3->SR1 & I2C_SR1_RXNE)){};//Wait the receive buffer not empty
		Data[Number-1] = I2C3->DR; // Collect the data
	}
}

/*********************************************
 * @brief I2C_Stop
 * Stop the I2C communication
 * @param None
 * @retval None
 */
void I2C_Stop()
{
	// Stop the I2C
	I2C3->CR1 |= I2C_CR1_STOP;
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
	uint8_t value;//Store the received data

	uint8_t AddressForRead = (Addr | 1<<0);

	// Read the register
	I2C_Start();//Start I2C
	I2C_Call_Address(Addr | 0);//Send device address with write
	I2C_Transmit(&Reg,1); // Send register address
	I2C_Start();//Restart I2C
	I2C_Receive(&value, 1,AddressForRead);//Receive the data

	return value;
}

/*********************************************
 * @brief I2Cx_WriteData
 * Write data to TSC register
 * @param uint8_t Addr : slave address, uint8_t Reg : register address in slave device, uint8_t Value: Register value
 * @retval None
 */
void I2Cx_WriteData(uint8_t Addr, uint8_t Reg, uint8_t Value)
{
	uint8_t info = Value;//Store the value

	// Write the register
	I2C_Start();//Start I2C
	I2C_Call_Address(Addr | 0);//Send device address with write
	I2C_Transmit(&Reg,1); // Send register address
	I2C_Transmit(&info,1); // Send register value
	I2C_Stop();// Stop the communication
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
void I2Cx_ReadBuffer(uint8_t Addr, uint8_t Reg, uint8_t *pBuffer, uint16_t Length)
{
	uint8_t AddressForRead = (Addr | 1<<0);

	// Read the register
	I2C_Start();//Start I2C
	I2C_Call_Address(Addr | 0);//Send device address with write
	I2C_Transmit(&Reg,1); // Send register address
	I2C_Start();//Restart I2C
	I2C_Receive(pBuffer, Length,AddressForRead);//Receive the data
}

/*********************************************
 * @brief I2C_Check
 * Check communication.
 * @param None
 * @retval None
 */
void I2C_Check()
{
	// Get LSB ID
	Value[0]=I2Cx_ReadData(STMPE811_DEVICE_ADDRESS, STMPE811_REG_CHP_ID_LSB);

	// Get MSB ID
	Value[1]=I2Cx_ReadData(STMPE811_DEVICE_ADDRESS, STMPE811_REG_CHP_ID_MSB);

	// Check if we receive correct ID
	if((Value[1]<<8 | Value[0]) == 0x0811)
	{
		asm("nop");
	}
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
void Touch_Init ()
{
	// Shut down / restart the device
	I2Cx_WriteData(STMPE811_DEVICE_ADDRESS, STMPE811_REG_SYS_CTRL1, REG_SYS_CTRL1_SOFT_RESET_ON);

	I2Cx_WriteData(STMPE811_DEVICE_ADDRESS, STMPE811_REG_SYS_CTRL1, REG_SYS_CTRL1_SOFT_RESET_OFF);

	// Start the touch screen operations.
	stmpe811_TS_Start(STMPE811_DEVICE_ADDRESS);
}

/*********************************************
 * @brief I2CStartNonBlocking
 * Start or Restart I2C communication(performing non-blocking)
 * @param None
 * @retval :uint8_t
 */
uint8_t I2CStartNonBlocking()
{
	static uint8_t state = 0;
	uint8_t finished = FALSE;

	switch (state)
	{
	case 0:
		// Perform a start non-blocking
		//Enable the acknowledge
		I2C3->CR1 |= I2C_CR1_ACK;

		//Start the I2C communication
		I2C3->CR1 |= I2C_CR1_START;
		state ++;
		break;

	case 1:
		// Started Ok?
		if ((I2C3->SR1 & I2C_SR1_SB))
		{
			finished = TRUE;
			state = 0;
		}
		break;
	}

	return finished;
}

/*********************************************
 * @brief I2C_Call_AddressNonBlocking
 * Transmit slave address byte (perform  non-blocking)
 * @param uint8_t Address
 * @retval uint8_t
 */
uint8_t I2C_Call_AddressNonBlocking(uint8_t Address)
{
	static uint8_t state = 0;
	uint8_t finished = FALSE;

	switch (state)
	{
	case 0:
		// Send the address
		I2C3->DR = Address;
		state ++;
		break;

	case 1:
		// Address Ok?
		if ((I2C3->SR1 & I2C_SR1_ADDR))
		{
			// Read SR1/SR2 register by rule
			I2C3->SR1; I2C3->SR2;
			finished = TRUE;
			state = 0;
		}
		break;
	}

	return finished;

}

/*********************************************
 * @brief I2C_TransmitNonBlocking
 * Transmit data to slave(performing non-blocking)
 * @param uint8_t *Data
 * @retval uint8_t
 */
uint8_t I2C_TransmitNonBlocking(uint8_t *Data)
{
	static uint8_t state = 0;
	uint8_t finished = FALSE;

	switch (state)
	{
	case 0:
		// If the transmit flag empty
		if((I2C3->SR1 & I2C_SR1_TXE))
		{
		state ++;
		}
		break;

	case 1:
		I2C3->DR = *Data; // transmitting data
		state ++;
		break;

	case 2:
		if((I2C3->SR1 & I2C_SR1_BTF))
		{
			finished = TRUE;
			state = 0;
		}
		break;
	}
	return finished;
}

/*********************************************
 * @brief I2C_ReceiveNonBlocking
 * Receive data from slave for 4 BYTES or 1 BYTE(performing non-Blocking)
 * @param uint8_t *Data, uint8_t Address,uint8_t Number
 * @retval uint8_t
 */
uint8_t I2C_ReceiveNonBlocking(uint8_t *Data,uint8_t Address,uint8_t Number)
{
	static uint8_t state = 0;
	uint8_t finished = FALSE;

	if(Number == 1)
	{
		switch(state)
		{
		case 0:
			// Send the address
			I2C3->DR = Address;
			state++;
			break;

		case 1:
			if((I2C3->SR1 & I2C_SR1_ADDR))
			{
				I2C3->CR1 &= ~(I2C_CR1_ACK); //disable acknowledge bit

				// Read SR1/SR2 register by rule
				I2C3->SR1; I2C3->SR2;

				I2C_Stop();	//Stop the communication

				state++;
			}
			break;

		case 2:
			if((I2C3->SR1 & I2C_SR1_RXNE))
			{
				Data[0] = I2C3->DR; // Collect the data
				finished = TRUE;
				state = 0;
			}
			break;
		}
	}
	else
	{
		switch(state)
		{
		case 0:
			if(I2C_Call_AddressNonBlocking(Address))
			{
				state++;
			}
			break;

		case 1:
			if((I2C3->SR1 & I2C_SR1_RXNE))
			{
				Data[0] = I2C3->DR; // Collect the data
				state++;
			}
			break;

		case 2:
			if((I2C3->SR1 & I2C_SR1_RXNE))
			{
				Data[1] = I2C3->DR; // Collect the data
				state++;
			}
			break;

		case 3:
			if((I2C3->SR1 & I2C_SR1_RXNE))
			{
				Data[2] = I2C3->DR; // Collect the data
				I2C3->CR1 &= ~(I2C_CR1_ACK); //disable acknowledge bit
				I2C_Stop();	//Stop the communication
				state++;
			}
			break;

		case 4:
			if((I2C3->SR1 & I2C_SR1_RXNE))
			{
				Data[3] = I2C3->DR; // Collect the data
				finished = TRUE;
				state = 0;
			}
			break;
		}
	}
	return finished;
}

/*********************************************
 * @brief I2Cx_ReadDataNonBlocking
 * Reads a single register address(performing non-blocking)
 * @param uint8_t: Device Address
 * @param uint8_t: Register Address
 * @retval: Value of register read.
 */
uint8_t I2Cx_ReadDataNonBlocking(uint8_t Addr, uint8_t Reg)
{
	static uint8_t state = 0;
	uint8_t finished = FALSE;
	//uint8_t value;//Store the received data
	uint8_t AddressForRead =  (Addr |0x01);

	// Read the register
	switch(state)
	{
	case 0 :
		//Start I2C
		if(I2CStartNonBlocking())
		{
			state++;
		}
		break;
	case 1:
		//Send device address with write
		if(I2C_Call_AddressNonBlocking(Addr))
		{
			state++;
		}
		break;
	case 2:
		// Send register address
		if(I2C_TransmitNonBlocking(&Reg))
		{
			state++;
		}
		break;
	case 3 :
		//Start I2C
		if(I2CStartNonBlocking())
		{
			state++;
		}
		break;
	case 4:
		//Receive the data
		if(I2C_ReceiveNonBlocking(&data, AddressForRead,1))
		{
			finished = TRUE;
			state = 0;
		}
		break;
	}
	return finished;
}

/*********************************************
 * @brief I2Cx_ReadBufferNonBlocking
 * Reads a number of registers from the device(perform non-blocking)
 * @param uint8_t: Device Address
 * @param uint8_t: Register Address to read from beginning
 * @param uint8_t*: Buffer to load data read into
 * @param uint16_t: Number of registers to read.
 * @retval: 0 / 1 indicating error(0=Ok)
 */
uint8_t I2Cx_ReadBufferNonBlocking(uint8_t Addr, uint8_t Reg, uint8_t *pBuffer, uint16_t Length)
{
	static uint8_t state = 0;
	uint8_t finished = FALSE;
	uint8_t AddressForRead =  (Addr |0x01);

	// Read the register
	switch(state)
	{
	case 0 :
		//Start I2C
		if(I2CStartNonBlocking())
		{
			state++;
		}
		break;
	case 1:
		//Send device address with write
		if(I2C_Call_AddressNonBlocking(Addr))
		{
			state++;
		}
		break;
	case 2:
		// Send register address
		if(I2C_TransmitNonBlocking(&Reg))
		{
			state++;
		}
		break;
	case 3 :
		//Start I2C
		if(I2CStartNonBlocking())
		{
			state++;
		}
		break;
	case 4:
		//Receive the data
		if(I2C_ReceiveNonBlocking(pBuffer, AddressForRead,Length))
		{
			finished = TRUE;
			state = 0;
		}
		break;
	}
	return finished;
}

/*********************************************
 * @brief I2Cx_WriteData
 * Write data to TSC register(performing non-blocking)
 * @param uint8_t Addr : slave address, uint8_t Reg : register address in slave device, uint8_t Value: Register value
 * @retval uint8_t
 */
uint8_t I2Cx_WriteDataNonBlocking(uint8_t Addr, uint8_t Reg, uint8_t Value)
{
	static uint8_t state = 0;
	uint8_t finished = FALSE;

	uint8_t info = Value;//Store the value

	switch (state)
	{
	case 0:
		if (I2CStartNonBlocking())
		{
			state ++;
		}
		break;

	case 1:
		//Send device address with write
		if(I2C_Call_AddressNonBlocking(Addr))
		{
			state++;
		}
		break;
	case 2:
		// Send register address
		if(I2C_TransmitNonBlocking(&Reg))
		{
			state++;
		}
		break;
	case 3 :
		//Start I2C
		if(I2C_TransmitNonBlocking(&info))
		{
			state++;
		}
		break;
	case 4:
		//Receive the data
		I2C_Stop();
		finished = TRUE;
		state = 0;
		break;

	}

	return finished;

}

/*********************************************
 * @brief stmpe811_TS_DetectTouch
 * Return if there is touch detected or not(performing non-blocking)
 * @param uint8_t DeviceAddr: Device address on communication Bus.
 * @retval: Touch detected state(0/1 = TouchDetected).
 */
uint8_t detectTouchNonBlocking(uint8_t Address)
{

	static uint8_t state = 0;
	uint8_t uc_Touched = 0;

	switch (state)
	{
	case 0:
		// Read the touch status register and check for the status of the Z bit.
		if(I2Cx_ReadDataNonBlocking(Address, STMPE811_REG_TSC_CTRL))
		{
			// if Z bit set
			if(data == 129)
			{
				state=1;
				data=0;
			}
			// no bit set so reset the FIFO
			else
			{
				state = 2;
			}
		}
		break;
	case 1:
		// Check the FIFO size
		if(I2Cx_ReadDataNonBlocking(Address, STMPE811_REG_FIFO_SIZE))
		{
			if(data>0)
			{
				// Touch detected
				uc_Touched = 1;

			}
			state = 0;

		}
		break;
	case 2:
		// No touch detected
		// Reset FIFO
		if(I2Cx_WriteDataNonBlocking(STMPE811_DEVICE_ADDRESS, STMPE811_REG_FIFO_STA, 0x01))
		{
			state++;
		}
		break;

	case 3:
		// Enable FIFO again
		if(I2Cx_WriteDataNonBlocking(STMPE811_DEVICE_ADDRESS, STMPE811_REG_FIFO_STA, 0x00))
		{
			state=0;
		}
		break;
	}

	return uc_Touched;
}

/*********************************************
 * @brief stmpe811_TS_GetXY
 * Get the XY touch coordinates(performing non-blocking)
 * @param uint16_t*: Pointer to X
 * @param uint16_t*: Pointer to Y
 * @retval: uint8_t
 */
uint8_t GetXYNonBLocking(uint16_t *X,uint16_t *Y)
{
	static uint8_t state = 0;
	uint8_t finished = FALSE;

	uint32_t uldataXYZ;
	switch (state)
	{
	case 0:
		// rRead the required registers
		if(I2Cx_ReadBufferNonBlocking(STMPE811_DEVICE_ADDRESS,STMPE811_REG_TSC_DATA_NON_INC,dataXYZ,sizeof(dataXYZ)))
		{
			state++;

			// Calculate positions values
			uldataXYZ = (dataXYZ[0]<<24)|(dataXYZ[1]<<16)|(dataXYZ[2]<<8)|(dataXYZ[3]<<0);
			*X = (uldataXYZ >> 20) & 0x00000FFF;
			*Y = (uldataXYZ >> 8)  & 0x00000FFF;
		}
		break;
	case 1:
		// Reset FIFO
		if(I2Cx_WriteDataNonBlocking(STMPE811_DEVICE_ADDRESS, STMPE811_REG_FIFO_STA, 0x01))
		{
			state++;
		}
		break;
	case 2:
		// Enable the FIFO
		if(I2Cx_WriteDataNonBlocking(STMPE811_DEVICE_ADDRESS, STMPE811_REG_FIFO_STA, 0x00))
		{
			state=0;
			finished = TRUE;
		}
		break;
	}

	return finished;
}

/*********************************************
 * @brief Touch_Process
 * Run the touch interface
 * @param None
 * @retval None
 */
void Touch_Process(void)
{
	static int state = 0;

	switch (state)
	{
	case 0: // Detect Touch
		if (detectTouchNonBlocking(STMPE811_DEVICE_ADDRESS))
		{
			state ++;
		}
		break;

	case 1: // Get XY
		if (GetXYNonBLocking(&us_TouchPointX, &us_TouchPointY))
		{
			ul_TouchCount++;
			state = 0;
		}
	}
}


//	// Has touch been indicated?
//	uint8_t uc_TouchDetected = stmpe811_TS_DetectTouch(STMPE811_DEVICE_ADDRESS);
//	if(uc_TouchDetected)
//	{
//		// Increment touch count and fetch X / Y Values for the touch point.
//		ul_TouchCount++;
//		stmpe811_TS_GetXY(&us_TouchPointX, &us_TouchPointY);
//	}

