/**************************************************************************************************************
 *                  File Name       :ModbusSlave.c
 *                  Overview        :This module handled all ModBus slave type messages that are received.
 *                  It generates the responses as required and triggers the transmission.
 *                  Authors         :Wenze Pan
 **************************************************************************************************************
 *                  Copyright @ As Applicable
 **************************************************************************************************************/


/****************************************************/

/*Required Header Files */
#include <stdint.h>
#include "USART.h"
#include "ParametersOperation.h"
#include "GlobalDefs.h"
#include "ModbusSlave.h"



/****************************************************/
/*Local only definitions */
#define ResponseMAX          50

/***************************/
/* Enumerations */

/**************************/
/*Structure types */

/*********************************************/
/* Global variable references */

/*********************************************/
/* Local only variable declaration */
_Rx_DATA CommandBuff;
uint16_t ResponseLength;
char ResponseBuff[60];

/*********************************************/
/* Local only function prototype */
uint8_t CheckCRC( char *Array, uint16_t Length);
uint16_t CalcChecksum(char *Array, uint16_t Size);
uint16_t ActionRxMsg (char *RXmsg, uint16_t RXmsgByteCount, char *replyMsg, uint16_t replyMAX);
int ActionReadRegisterRequest( char *RXmsg, uint16_t RXmsgByteCount, char *replyMsg, uint16_t replyMAX);
void AddCRC ( char *Array, uint16_t Length);
uint32_t GenerateErrorResponse ( char *RXmsg,  uint16_t RXmsgByteCount,  char *replyMsg, uint16_t replyMAX, uint16_t errorCode);

/*********************************************
 * @brief ModbusSlave_Run
 * Process the message from master and generate response
 * @param None
 * @retval None
 */
void ModbusSlave_Run()
{
	// Fetch the receive elements from USART.
	CommandBuff =  USART_Fetch_Rx();

	// If got any message and message are valid(>=4).
	if(CommandBuff.s_DataLen >0)
	{
		// Filter the invalid command
		if(CommandBuff.s_DataLen > 4)
		{
			// Check ID
			if(CommandBuff.pData[0]==8)
			{
				// CRC check
				if(CheckCRC(CommandBuff.pData, CommandBuff.s_DataLen))
				{
					// Checksum ok...

					// Message is for us and checksum is valid...
					// Following function decodes the reply, takes appropriate action and loads replyMsg with required response as well as
					// returning the number of bytes in reply message
					ResponseLength = ActionRxMsg (CommandBuff.pData, CommandBuff.s_DataLen, ResponseBuff, ResponseMAX);

					// Transmit ModBus Slave response.
					USART_Request_Tx(ResponseBuff, ResponseLength);

					// Restart counting CommandBuff
					USART1_Clear_Rx();
				}
				else
				{
					// Transmit ModBus Slave response.
					USART_Request_Tx(ResponseBuff, 0);

					// Restart counting CommandBuff
					USART1_Clear_Rx();
				}
			}
			else
			{
				// Transmit ModBus Slave response.
				USART_Request_Tx(ResponseBuff, 0);

				// Restart counting CommandBuff
				USART1_Clear_Rx();
			}
		}
		else
		{
			// Transmit ModBus Slave response.
			USART_Request_Tx(ResponseBuff, 0);

			// Restart counting CommandBuff
			USART1_Clear_Rx();
		}
	}
}

/*********************************************
 * @brief CheckCRC
 * Checks the CRC on the end of an array
 * @param  char*  Array - Pointer to received data buffer
 * 		   uint16_t Length-buffer length
 * @retval uint8_t(TURE OR FALSE)
 */
uint8_t CheckCRC( char *Array, uint16_t Length)
{
	uint16_t CRCtarget;
	uint16_t CRCactual;

	// Calculate what the CRC should be...
	CRCtarget = CalcChecksum(Array, Length);

	// Extract the CRC already within the last two bytes of the array
	CRCactual = (Array[Length-2] << 8) | Array[Length-1];

	// And check...
	if (CRCtarget == CRCactual)
		return TRUE;
	else
		return FALSE;
}


/*********************************************
 * @brief CalcChecksum
 * Calculate a checksum given an array of bytes.
 * @param  char*  Array - Pointer to received data buffer
 * 		   uint16_t Length-buffer length
 * @retval uint16_t (2 BYTE CHECKSUM VALUE).
 */
uint16_t CalcChecksum( char *Array, uint16_t Size)
{
	 uint32_t Index = 0;
	 uint32_t CRC = 0xffff;
	 uint32_t next = 0;
	 uint32_t carry = 0;
	 uint32_t n = 0;

		// Make sure the calculation doesn't include any existing checksum...
		Size -= 2;

		// Loop through the array calculating the checksum...
		while (Size-- > 0){
			next = Array[Index];
			CRC ^= next;
			for (n=0; n<8; n++){
				carry = CRC & 1;
				CRC >>= 1;
				if (carry != 0)
					CRC ^= 0xA001;
			}
			Index ++;
		}

		// Endian swap the result
		CRC &= 0xFFFF;
		uint16_t temp = (CRC & 0xFF) << 8;
		temp |= (CRC & 0xFF00) >> 8;
		return temp;
}

/*********************************************
 * @brief AddCRC
 * Add checksum to last two bytes of an array...
 * @param  char*  Array - Pointer to received data buffer
 * 		   uint16_t Length-buffer length
 * @retval
 */
void AddCRC ( char *Array, uint16_t Length)
{
	int CRC = CalcChecksum (Array, Length + 2);
	Array[Length] = ( char)((CRC & 0xFF00) >> 8);// High byte of checksum
	Array[Length + 1] = (char)(CRC & 0x00FF);   // Low byte of checksum
}

/*********************************************
 * @brief ActionRxMsg
 * Function actions the received message - it generates any reply required and returns number of bytes ]
 * in replyMsg to transmit as a response.
 * @param char *RXmsg - Pointer to received data buffer
 * 		  uint16_t RXmsgByteCount- number of bytes in message array
 * 		  char *replyMsg - Pointer to response buffer
 * 		  uint16_t replyMAX - Maximum size that can be used in response buffer.
 * @retval uint16_t (number of bytes to transmit)
 */
uint16_t ActionRxMsg ( char *RXmsg, uint16_t RXmsgByteCount, char *replyMsg, uint16_t replyMAX)
{
	uint16_t replyByteCount = 0;

	// Action here depends on function code embedded within message
	switch (RXmsg[1])
	{
	////////////////////////
	case MODBUS_READ_REGISTER_FC:
		replyByteCount = ActionReadRegisterRequest (RXmsg, RXmsgByteCount, replyMsg, replyMAX);
	break;

	////////////////////////
	case MODBUS_WRITE_REGISTER_FC:
		//replyByteCount = ActionWriteRegisterRequest (RXmsg, RXmsgByteCount, replyMsg, replyMAX);
	break;

	////////////////////////
	case MODBUS_WRITE_MULTIPLE_REGISTERS_FC:
		//replyByteCount = ActionWriteMultipleRegisterRequest (RXmsg, RXmsgByteCount, replyMsg, replyMAX);
	break;

	////////////////////////
	// Unsupported function codes
	case MODBUS_READ_BIT_FC:
	case MODBUS_WRITE_BIT_FC:
	case MODBUS_LOOPBACK_COMMAND_FC:
	// Intentional fall-through to default
	default:
		// Unknown command
		// Generate "ILLEGAL_FUNCTION_CODE" response
		//replyByteCount = GenerateErrorResponse (RXmsg, RXmsgByteCount, replyMsg, replyMAX, ILLEGAL_FUNCTION_CODE);
	break;
	}

	return replyByteCount;
}

/*********************************************
 * @brief ActionReadRegisterRequest
 * Decode and respond to a Holding Register(s) command Returns number of bytes in replyMsg to send out
 * @param char *RXmsg - Pointer to received data buffer
 * 		  uint16_t RXmsgByteCount- number of bytes in message array
 * 		  char *replyMsg - Pointer to response buffer
 * 		  uint16_t replyMAX - Maximum size that can be used in response buffer.
 * @retval int (+ve = number of bytes to transmit, -ve = exception)
 */
int ActionReadRegisterRequest( char *RXmsg, uint16_t RXmsgByteCount, char *replyMsg, uint16_t replyMAX)
{
	uint32_t replyByteCount = 0;
	uint16_t startingAddress;
	uint32_t wordsRequested;
	uint16_t wordsLeftToRead;

	// Make sure the word count is within limits
	wordsRequested = ((int)RXmsg[4] << 8) | (int)RXmsg[5];
	if (wordsRequested > MAX_READ_LENGTH)
	{
		return GenerateErrorResponse (RXmsg, RXmsgByteCount, replyMsg, replyMAX, ILLEGAL_DATA_ADDRESS_CODE);
	}
	wordsLeftToRead = wordsRequested;

	// What's the address of the first register (word) we need?
	startingAddress = (((int)RXmsg[2] << 8) | RXmsg[3]);

	// Make sure the data address is within the range.
	if(startingAddress+wordsRequested>(0x0007+1))
	{
		return GenerateErrorResponse (RXmsg, RXmsgByteCount, replyMsg, replyMAX, ILLEGAL_DATA_ADDRESS_CODE);
	}

	ValUpdate();

	// Collect the data
	for(int i=0;i<wordsLeftToRead;i++)
	{
		// Start from the first address in the command.
		replyMsg[REPLY_DATA_OFFSET+2*i]=(GetVal(startingAddress+i)>>8);
		replyMsg[REPLY_DATA_OFFSET+2*i+1]=GetVal(startingAddress+i);
	}

	// Finally, sort the rest of the message (header)
	// Copy ID across (rather than writing our ID we copy is across to maintain validity if ALWAYS_RESPOND address used
	replyMsg[0] = RXmsg[0];
	// Copy function code across
	replyMsg[1] = RXmsg[1];
	// Now set the byte count
	replyMsg[2] = wordsRequested << 1; // *2 as the message shows byte count and includes a number of 2-byte words

	// Add checksum
	AddCRC(replyMsg, 3 + (2*wordsRequested));

	// And set reply byte count
	replyByteCount = 5 + (2*wordsRequested);

	return replyByteCount;
}

/*********************************************
 * @brief GenerateErrorResponse
 * Function generates a specified ModBus error response
 * @param char *RXmsg - Pointer to received data buffer
 * 		  uint16_t RXmsgByteCount- number of bytes in message array
 * 		  char *replyMsg - Pointer to response buffer
 * 		  uint16_t replyMAX - Maximum size that can be used in response buffer.
 * 		  uint16_t errorCode - Error code
 * @retval int (+ve = number of bytes to transmit, -ve = exception)
 */
uint32_t GenerateErrorResponse ( char *RXmsg,  uint16_t RXmsgByteCount,  char *replyMsg, uint16_t replyMAX, uint16_t errorCode)
{
	int replyByteCount = 0;

	// Make sure we've got enough room in our replyMsg
	if (replyMAX < 6){
		// Can't do anything really, so don't reply...
		return 0;
	}
	// Copy across ID
	replyMsg[0] = RXmsg[0];
	// Copy across function code
	replyMsg[1] = RXmsg[1];
	// Set the error bit in the function code
	replyMsg[1] |= 0x80;
	// Set the error code
	replyMsg[2] = ( char)(errorCode & 0xff);
	// Add the checksum
	AddCRC(replyMsg, 3);
	// Set reply byte count
	replyByteCount = 5;

	return replyByteCount;
}
