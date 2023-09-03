/**************************************************************************************************************
 *                  File Name       :CommandHandler.c
 *                  Overview        :External Command handling functionality.
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
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>

#include "GlobalDefs.h"
#include "CommandHandler.h"
#include "USART.h"
#include "PWM.h"
#include "WaveformGenerator.h"


/****************************************************/
/*Local only definitions */
// Standard string responses
#define UNKNOWN_COMMAND_RESPONSE            "Unknown Command?\r\n"
#define COMMAND_ACTIONED_RESPONSE           "ACK\r\n"
#define MALFORMED_COMMAND_RESPONSE          "CMD_Error\r\n"
#define COMMAND_NOT_IMPLEMENTED_YET         "NACK\r\n"
#define NOT_POSSIBLE_RESPONSE               "Out Of Bounds! Change It Back\r\n"

#define COMMAND_RESPONSE_MAX_LENGTH         128

/***************************/
/* Enumerations */

/**************************/
/*Structure types */
typedef struct
{
	const char *str_UniqueIdentifier;   // How to identify this command uniquely.
	const char *str_HelpText;           // What to output as help info for this command.
	uint8_t c_ShowInHelp;               // Flag for whether this command should be included in the help output.
	void (*func_ptr)(char *);           // Function pointer to command 'handling' function.

}_COMMAND_FORMAT;

/*********************************************/
/* Global variable references */

/**************************/
/* Local only function prototypes */
void RunCommand(_Rx_DATA Rx_Data);

void cmd_SetType(char *p_Data);
void cmd_SetFreq(char *p_Data);
void cmd_SetPeriod(char *p_Data);
void cmd_SetAmp(char *p_Data);
void cmd_SetOffs(char *p_Data);
void cmd_SetPWM(char *p_Data);
void cmd_SetCust_Clear(char *p_Data);
void cmd_SetCust_Add(char *p_Data);
void cmd_SaveCust(char *p_Data);
void cmd_SetSamples(char *p_Data);
void cmd_GetType(char *p_Data);
void cmd_GetFreq(char *p_Data);
void cmd_GetPeriod(char *p_Data);
void cmd_GetAmp(char *p_Data);
void cmd_GetOffs(char *p_Data);
void cmd_GetPWM(char *p_Data);
void cmd_GetSamples(char *p_Data);
void cmd_Help(char *p_Data);
void cmd_HandlerUnknownCommands(char *p_Data);

/*********************************************/
/* Local only variable declaration */
_COMMAND_FORMAT CommandTable [] =
{
		// Specific 'Set' Command handlers. Help String can't exceed COMMAND_RESPONSE_MAX_LENGTH in length or buffer overrun will occur with unpredictable results.
		{ "Set Type",		"Set Type X: Set Waveform Type(X=> Sine, Sawtooth, Triangle, Squre, Custom" ,                                           TRUE,	  cmd_SetType },
		{ "Set Freq",       "Set Freq XYZ: Set output Frequency in Hz (floating point number)\r\n",                                                 TRUE,     cmd_SetFreq },
		{ "Set Period",     "Set Period XYZ: Set output Period in Seconds (floating point number)\r\n",                                             TRUE,     cmd_SetPeriod},
		{ "Set Amp",        "Set Amp XYZ: Set output Offser (float 0->100%)\r\n",                                                                   TRUE,     cmd_SetAmp},
		{ "Set Offs",       "Set Offs XYZ: Set output Offset(float 0->100%)\r\n",                                                                   TRUE,     cmd_SetOffs},
		{ "Set PWM",        "Set PWM XYZ: Set the PWM output frequency (not duty cycle)\r\n",                                                       TRUE,     cmd_SetPWM},
		{ "Set Cust Clear", "Set Cust Clear: Clear custom data ready for adding new.\r\n",                                                          TRUE,     cmd_SetCust_Clear},
		{ "Set Cust Add",   "Set Cust Add d1...dn: Add to Custom Waveform Data, max 1000 points (Data1...DataN Where Data 0->10000 = 0-100%)\r\n",  TRUE,     cmd_SetCust_Add },
		{ "Save Cust",      "Save Cust: Save the current custom waveform to FLASH - to be auto restored at power on/restart up\r\n",                TRUE,     cmd_SaveCust},
		{ "Set Samples",    "Set Samples: Set the number of DAC samples when using DMA mode\r\n",                                                   TRUE,     cmd_SetSamples},

		// Specific 'Get' Command handlers
		{ "Get Type",       "Get Type: Get Waveform Type (0=Sine, 1=Sawtooth, 2=Triangle, 3=Squre, 4=Custom)\r\n",                                  TRUE,     cmd_GetType},
		{ "Get Freq",       "Get Freq: Get output Frequency inHz (floating point number)\r\n",                                                      TRUE,     cmd_GetFreq},
		{ "Get Period",     "Get Period: Get output Period in Seconds (floating point number)\r\n",                                                 TRUE,     cmd_GetPeriod},
		{ "Get Amp",        "Get Amp: Get output Amplitude (float from 0.0->1.0 = 0->100%)\r\n",                                                    TRUE,     cmd_GetAmp},
		{ "Get Offs",       "Get Offs: Get output Offset (floatfrom 0.0->1.0 = 0->100%)\r\n",                                                       TRUE,     cmd_GetOffs},
		{ "Get PWM",        "Get PWM: Get the PWM output frequency (not duty cycle)\r\n",                                                           TRUE,     cmd_GetPWM},
		{ "Get Samples",     "Get Samples: Get the number of DAC samples when using DMA mode\r\n",                                                  TRUE,     cmd_GetSamples},


		// General commands
		{ "Help",           "Help Provide help for all messages\r\n",                                                                               TRUE,     cmd_Help},

		// Catch unknown commands handler - this has to always be last in the list/table.
		{ "NotKnownCommand","",                                                                                                                     FALSE,    cmd_HandlerUnknownCommands}


};

// Pointer to current command or NULL if no command currently running iteratively.
void (*fp_CurrentCommand)(char *) = NULL;
char *p_CommandData = NULL;

// Buffer for creating and transmitting on-the-fly responses.
char CommandResponseBuff[COMMAND_RESPONSE_MAX_LENGTH];


/*********************************************
 * @brief CommandHandler_Run
 * Main entry point for handling all external commands.
 * @param None
 * @retval None
 */
void CommandHandler_Run( void )
{
	// First of all, is there a command already running?
	if( fp_CurrentCommand != NULL)
	{
		// There's already an active command, keep calling it until finished.
		fp_CurrentCommand(p_CommandData);
	}
	else
	{
		// As no command is currently running, check for a new command...
		_Rx_DATA Rx_Data = USART_Fetch_Rx();
		if(Rx_Data.s_DataLen > 0)
		{
			// Command received, time to process it...
			RunCommand(Rx_Data);
		}
	}
}

/*********************************************
 * @brief RunCommand
 * Finds and runs the required command
 * @param _Rx_DATA - Structure containing received command details.
 * @retval None
 */
void RunCommand(_Rx_DATA Rx_Data)
{
	_COMMAND_FORMAT *pCommand;
	uint16_t s_StrLen;
	uint8_t c_CommandCalled = FALSE;

	// Loop through all commands looking for a match
	for (int i=0; i < (sizeof(CommandTable) / sizeof(_COMMAND_FORMAT)); i++)
	{
		// Get a pointer to the specific command - quicker then indexing into table for each time.
		pCommand = &CommandTable[i];

		// Get the length of the unique identifier
		s_StrLen = strlen(pCommand ->str_UniqueIdentifier);

		// Check for a match
		if (strncmp (pCommand->str_UniqueIdentifier, Rx_Data.pData,s_StrLen) == 0)
		{
			// Command found.
			// Save it to our file scope pointer in case it needs to run repeatedl
			fp_CurrentCommand = pCommand->func_ptr;

			// Save the data pointer
			p_CommandData = Rx_Data.pData;

			// Finally, we can call the command handler
			fp_CurrentCommand(p_CommandData);
			c_CommandCalled = TRUE;
		}
	}

	// If no command called above, we need to run the catch-all (unknown command function).
	if(c_CommandCalled == FALSE)
	{
		fp_CurrentCommand = cmd_HandlerUnknownCommands;
		fp_CurrentCommand(p_CommandData);
	}
}

/*********************************************
 * @brief CommandFinished
 * Helper to wrap up once a command has been dealt with.
 * Note "static inline" - the lines of code within this function will
 * be compiled 'Inline' wherever this function is used. Means
 * larger compiled code but no overhead of calling a function.
 * @param  None
 * @retval None
 */
static inline void CommandFinished()
{
	// declare the received command as delat with to the USART handler.
	USART1_Clear_Rx();
	// And set the local function pointer to null so the command handler function doesn't get called again.
	fp_CurrentCommand = NULL;
}

/*********************************************
 * @brief FindNumberInString
 * Helper that returns a pointer to the first number within a string, or NULL if none found.
 * @param  Str - String to search (MUST be NULL terminated).
 * @retval None
 */
char *FindNumberInString(char *Str)
{
	char *p_Num = NULL;
	uint16_t StrLen = strlen(Str);
	for(int i = 0; i< StrLen; i++)
	{
		if(isdigit((int)Str[i]))
		{
			p_Num = &Str[i];
			break;
		}
	}
	return p_Num;
}

/*********************************************
 * @brief FindCommaInString
 * Helper that returns a pointer to the first comma within a string, or NULL if none found.
 * @param  Str - String to search (MUST be NULL terminated).
 * @retval None
 */
char *FindCommaInString(char *Str)
{
	char *p_Comma = NULL;
	uint16_t StrLen = strlen(Str);
	for(int i =0; i<StrLen; i++)
	{
		if(Str[i] ==',')
		{
			p_Comma = &Str[i];
			break;
		}
	}
	return p_Comma;
}

/*********************************************
 * @brief cmd_SetType
 * Specific command handler
 * @param  char *p_Data - pointer to command message after unique identifier.
 * @retval None
 */
void cmd_SetType( char *p_Data)
{
	uint8_t c_CommandActioned = FALSE;
	_WAVEFORM_DESCRIPTOR Waveform = WaveformGenerator_Get_Waveform();

	// Extract type required from string (Note command is "Set Type XYZ" so XYZ starts at index 9)
	if (strncmp(&p_Data[9],"Sine",4) == 0)
	{
		// Sine wave requested.
		Waveform.e_WaveType = eWT_Sine;
		c_CommandActioned = TRUE;
	}
	else if (strncmp(&p_Data[9], "Triangle", 8) == 0)
	{
		// Triangle wave requested.
		Waveform.e_WaveType = eWT_Triangular;
		c_CommandActioned = TRUE;
	}
	else if(strncmp(&p_Data[9],"Sawtooth", 8) == 0)
	{
		// Sawtooth wave requested.
		Waveform.e_WaveType = eWT_SawTooth;
		c_CommandActioned = TRUE;
	}
	else if(strncmp(&p_Data[9],"Square", 6) == 0)
		{
			// Sawtooth wave requested.
			Waveform.e_WaveType = eWT_Square;
			c_CommandActioned = TRUE;
		}
	else if(strncmp(&p_Data[9],"Custom", 6) == 0)
		{
			// Sawtooth wave requested.
			Waveform.e_WaveType = eWT_Custom;
			c_CommandActioned = TRUE;
		}

	if (c_CommandActioned)
	{
		// Command processed ok.
		if (WaveformGenerator_Set_Waveform(Waveform))
		{
			// Provide ack
			USART_Request_Tx((char*) COMMAND_ACTIONED_RESPONSE,
					strlen(COMMAND_ACTIONED_RESPONSE));
		}
		else
		{
			// Provide Our Of Bounds
			USART_Request_Tx((char*) NOT_POSSIBLE_RESPONSE,
					strlen(NOT_POSSIBLE_RESPONSE));
		}
	}
	else
	{
		// Badly formatted command
		USART_Request_Tx((char *)MALFORMED_COMMAND_RESPONSE, strlen(MALFORMED_COMMAND_RESPONSE));
	}

	CommandFinished();
}

/*********************************************
 * @brief cmd_SetFreq
 * Specific command handler
 * @param  char *p_Data - pointer to command message after unique identifier.
 * @retval None
 */
void cmd_SetFreq( char *p_Data)
{
	// Get the start of the number provided
	char *p_Num = FindNumberInString(p_Data);

	if (p_Num != NULL)
	{
		// Process number
		_WAVEFORM_DESCRIPTOR Waveform = WaveformGenerator_Get_Waveform();
		Waveform.ull_Period_uS = (uint64_t) (1.0 / atof(p_Num) * 1000000);// 1/f=period
		if (WaveformGenerator_Set_Waveform(Waveform))
		{
			// Provide ack
			USART_Request_Tx((char*) COMMAND_ACTIONED_RESPONSE,
					strlen(COMMAND_ACTIONED_RESPONSE));
		} else
		{
			// Provide Our Of Bounds
			USART_Request_Tx((char*) NOT_POSSIBLE_RESPONSE,
					strlen(NOT_POSSIBLE_RESPONSE));
		}
	}

	else
	{
		// Badly constructed command.
		USART_Request_Tx((char *)MALFORMED_COMMAND_RESPONSE,strlen( MALFORMED_COMMAND_RESPONSE));
	}

	// Declare done with this message.
	CommandFinished();
}

/*********************************************
 * @brief cmd_SetPeriod
 * Specific command handler
 * @param  char *p_Data - pointer to command message after unique identifier.
 * @retval None
 */
void cmd_SetPeriod(char *p_Data)
{
	// Get the start of the number provided
	char *p_Num = FindNumberInString(p_Data);

	if(p_Num != NULL)
	{
		// Process number
		_WAVEFORM_DESCRIPTOR Waveform = WaveformGenerator_Get_Waveform();
		Waveform.ull_Period_uS = (uint64_t)(atof(p_Num)*1000000); //*1000000 to convert S to uS
		if(WaveformGenerator_Set_Waveform(Waveform))
		{
			// Provide ack
			USART_Request_Tx((char *)COMMAND_ACTIONED_RESPONSE, strlen(COMMAND_ACTIONED_RESPONSE));
		}
		else
		{
			// Provide Our Of Bounds
			USART_Request_Tx((char*)NOT_POSSIBLE_RESPONSE,strlen(NOT_POSSIBLE_RESPONSE));
		}



	}
	else
	{
			// Badly formatted command
			USART_Request_Tx((char *)MALFORMED_COMMAND_RESPONSE, strlen(MALFORMED_COMMAND_RESPONSE));
	}

	// Declare done with this message.
	CommandFinished();
}

/*********************************************
 * @brief cmd_SetAmp
 * Specific command handler
 * @param  char *p_Data - pointer to command message after unique identifier.
 * @retval None
 */
void cmd_SetAmp(char *p_Data)
{
	// Get the start of the number provided
	char *p_Num = FindNumberInString(p_Data);

	if(p_Num != NULL)
	{
		// Process number
		_WAVEFORM_DESCRIPTOR Waveform = WaveformGenerator_Get_Waveform();
		Waveform.f_Amplitude = atof(p_Num);
		WaveformGenerator_Set_Waveform(Waveform);

		// Provide ack
		USART_Request_Tx((char *)COMMAND_ACTIONED_RESPONSE, strlen(COMMAND_ACTIONED_RESPONSE));
	}
	else
	{
			// Badly formatted command
			USART_Request_Tx((char *)MALFORMED_COMMAND_RESPONSE, strlen(MALFORMED_COMMAND_RESPONSE));
	}

	// Declare done with this message.
	CommandFinished();
}

/*********************************************
 * @brief cmd_SetOffs
 * Specific command handler
 * @param  char *p_Data - pointer to command message after unique identifier.
 * @retval None
 */
void cmd_SetOffs(char *p_Data)
{
	// Get the start of the number provided
	char *p_Num = FindNumberInString(p_Data);

	if(p_Num != NULL)
	{
		// Process number
		_WAVEFORM_DESCRIPTOR Waveform = WaveformGenerator_Get_Waveform();
		Waveform.f_Offset = atof(p_Num);
		WaveformGenerator_Set_Waveform(Waveform);

		// Provide ack
		USART_Request_Tx((char *)COMMAND_ACTIONED_RESPONSE, strlen(COMMAND_ACTIONED_RESPONSE));
	}
	else
	{
			// Badly formatted command
			USART_Request_Tx((char *)MALFORMED_COMMAND_RESPONSE, strlen(MALFORMED_COMMAND_RESPONSE));
	}

	// Declare done with this message.
	CommandFinished();
}

/*********************************************
 * @brief cmd_SetPWM
 * Specific command handler
 * @param  char *p_Data - pointer to command message after unique identifier.
 * @retval None
 */
void cmd_SetPWM(char *p_Data)
{
	// Get the start of the number provided
	char *p_Num = FindNumberInString(p_Data);

	if(p_Num != NULL)
	{
		// Process number and assign PWM base frequency
		PWM_Init(atoi(p_Num));

		// Provide ack
		USART_Request_Tx((char *)COMMAND_ACTIONED_RESPONSE, strlen(COMMAND_ACTIONED_RESPONSE));
	}
	else
	{
			// Badly formatted command
			USART_Request_Tx((char *)MALFORMED_COMMAND_RESPONSE, strlen(MALFORMED_COMMAND_RESPONSE));
	}

	// Declare done with this message.
	CommandFinished();
}

/*********************************************
 * @brief cmd_SetCust_Clear
 * Specific command handler - clear currentcustom waveform.
 * @param  char *p_Data - pointer to command message after unique identifier.
 * @retval None
 */
void cmd_SetCust_Clear(char *p_Data)
{
	// Indicate to waveform generator module to clear out data
	WaveformGenerator_Clear_Custom_Data();

	// Respond with ACK...
	USART_Request_Tx((char *)COMMAND_ACTIONED_RESPONSE, strlen(COMMAND_ACTIONED_RESPONSE));
	CommandFinished();
}

/*********************************************
 * @brief cmd_SetCust_Add
 * Specific command handler - Add data points to current custom waveform.
 * Note: This function is iterative - will need to be continually called
 * until completed (CommandFinished() called).
 * @param  char *p_Data - pointer to command message after unique identifier.
 * @retval None
 */
/* Definitions used in this function for clarity...*/
#define CUST_ADD__INDEX_OF_START_OF_DATA          12    // "Set Cust Add" - leave space to find index of first number below
#define CUST_ADD__POINTS_TO_PROCESS_PER_PASS      10    // Number of data points to parse per pass - to avoid hogging on the main loop
//States...
#define CUST_ADD__START                           0
#define CUST_ADD__PROCESSING                      1
void cmd_SetCust_Add(char *p_Data)
{
	static uint8_t uc_ComplexCommandState; // State machine state
	static char *p_RxDataPtr;              // Static pointer to current index in data
	static uint16_t us_NumPointAdded;      // Counter for number of points added
	uint16_t DataPoints[CUST_ADD__POINTS_TO_PROCESS_PER_PASS]; // Data points array (per pass)

	// This command takes repeated processing to avoid hogging the main loop....
	switch(uc_ComplexCommandState)
	{
	case CUST_ADD__START:
		// Initialise to process the command....
		p_RxDataPtr = FindNumberInString(&p_Data[CUST_ADD__INDEX_OF_START_OF_DATA]);

		// At least one data point found?
		if(p_RxDataPtr != NULL)
		{
			// Initialise to process the data
			us_NumPointAdded = 0;

			// Set processing state
			uc_ComplexCommandState = CUST_ADD__PROCESSING;

			// Recurse to start straight away
			cmd_SetCust_Add(p_Data);
		}
		else
		{
			// No data points provided.
			// Indicate ERROR
			USART_Request_Tx((char *)MALFORMED_COMMAND_RESPONSE, strlen(MALFORMED_COMMAND_RESPONSE));

			// Reset this state machine for another go of this command.
			CommandFinished();
		}
		break;
	case CUST_ADD__PROCESSING:
	{
		// Process up to 10 data points...
		uint8_t uc_PointIndex = 0;

		// Loop processing data-points until limit per pass exceed or
		// end of command data found
		while((uc_PointIndex < CUST_ADD__POINTS_TO_PROCESS_PER_PASS)&&(p_RxDataPtr != NULL))
		{
			// Make sure we find a number character...
			p_RxDataPtr = FindNumberInString(p_RxDataPtr);

			// Check for end of string
			if(p_RxDataPtr != NULL)
			{
				// Process the next data-point
				DataPoints[uc_PointIndex++] = atoi(p_RxDataPtr);
				us_NumPointAdded++;

				// Find next data point or end of command string
				// Start be searching for the next comma.
				p_RxDataPtr = FindCommaInString(p_RxDataPtr);

				// If not NULL there should be more data.
				// so we can continue with the while loop by simply
				// incrementing on to the next character, the start of the
				// next number
				if(p_RxDataPtr != NULL)
				{
					// Increment to next data-point.
					p_RxDataPtr++;
				}
			}
		}

		// Any data processed above to pass to waveform generator?
		if (uc_PointIndex >0)
		{
			// Add data received to data set - and check data saved Ok
			if (WaveformGenerator_Add_Custom_Data(DataPoints, uc_PointIndex) == FALSE)
			{
				// Unable to add data
				// Setting p_RxDataPtr = NULL and us_NumPointsAdded = 0 will cause the malformed response below
				p_RxDataPtr = NULL;
				us_NumPointAdded = 0;
			}
		}

		// Check for end of command string
		if(p_RxDataPtr == NULL)
		{
			if(us_NumPointAdded >0)
			{
				// Indicate ACK
				// Response with an ACK
				USART_Request_Tx((char *)COMMAND_ACTIONED_RESPONSE, strlen(COMMAND_ACTIONED_RESPONSE));
			}
			else
			{
				// Indicate ERROR
				USART_Request_Tx((char *)MALFORMED_COMMAND_RESPONSE, strlen(MALFORMED_COMMAND_RESPONSE));
			}

			// Reset this state machine for another go of this command.
			CommandFinished();
			uc_ComplexCommandState = CUST_ADD__START;
		}
		// else come back here to process more data
	}
	break;
	}
}

/*********************************************
 * @brief cmd_SaveCust
 * Specific command handler
 * @param  char *p_Data - pointer to command message after unique identifier.
 * @retval None
 */
void cmd_SaveCust(char *p_Data)
{
	// To be done later, simple NACK for now.
	USART_Request_Tx((char *)COMMAND_NOT_IMPLEMENTED_YET, strlen(COMMAND_NOT_IMPLEMENTED_YET));
	CommandFinished();
}

/*********************************************
 * @brief cmd_SetSamples
 * Specific command handler
 * @param  char *p_Data - pointer to command message after unique identifier.
 * @retval None
 */
void cmd_SetSamples(char *p_Data)
{
	// Get the start of the number provided
	char *p_Num = FindNumberInString(p_Data);

	if(p_Num != NULL && atoi(p_Num)<MAX_DAC_SAMPLE)
	{
		// Process number and assign PWM base frequency
		WaveformGenerator_Set_NUM_DAC_Sample(atoi(p_Num));

		// Provide ack
		USART_Request_Tx((char *)COMMAND_ACTIONED_RESPONSE, strlen(COMMAND_ACTIONED_RESPONSE));
	}
	else
	{
			// Badly formatted command
			USART_Request_Tx((char *)MALFORMED_COMMAND_RESPONSE, strlen(MALFORMED_COMMAND_RESPONSE));
	}

	// Declare done with this message.
	CommandFinished();
}

/*********************************************
 * @brief cmd_GetType
 * Specific command handler
 * @param  char *p_Data - pointer to command message after unique identifier.
 * @retval None
 */
void cmd_GetType(char *p_Data)
{
	// Respond with current wave type...
	_WAVEFORM_DESCRIPTOR Waveform = WaveformGenerator_Get_Waveform();
	switch(Waveform.e_WaveType)
	{
	case eWT_Sine:
		sprintf(CommandResponseBuff, "Sine\r\n");
		break;

	case eWT_SawTooth:
		sprintf(CommandResponseBuff, "Sawtooth\r\n");
		break;

	case eWT_Triangular:
		sprintf(CommandResponseBuff, "Triangular\r\n");
		break;

	case eWT_Square:
		sprintf(CommandResponseBuff, "Square\r\n");
		break;

	case eWT_Custom:
		sprintf(CommandResponseBuff, "Custom\r\n");
		break;
	}
	USART_Request_Tx(CommandResponseBuff, strlen(CommandResponseBuff));
	CommandFinished();
}

/*********************************************
 * @brief cmd_GetFreq
 * Specific command handler
 * @param  char *p_Data - pointer to command message after unique identifier.
 * @retval None
 */
void cmd_GetFreq(char *p_Data)
{
	// Respond with current frequency (from period).
	_WAVEFORM_DESCRIPTOR Waveform = WaveformGenerator_Get_Waveform();

	// Use floating point in sprintf you need to configure project settings.
	sprintf(CommandResponseBuff, "%lf\r\n",( 1.0 / ((float)Waveform.ull_Period_uS / 1000000.0)));

	USART_Request_Tx(CommandResponseBuff, strlen(CommandResponseBuff));
	CommandFinished();
}

/*********************************************
 * @brief cmd_GetPeriod
 * Specific command handler
 * @param  char *p_Data - pointer to command message after unique identifier.
 * @retval None
 */
void cmd_GetPeriod(char *p_Data)
{
	// Respond with current period.
	_WAVEFORM_DESCRIPTOR Waveform = WaveformGenerator_Get_Waveform();

	// Use floating point in sprintf you need to configure project settings.
	sprintf(CommandResponseBuff, "%lf\r\n", (float)Waveform.ull_Period_uS / 1000000.0);

	USART_Request_Tx(CommandResponseBuff, strlen(CommandResponseBuff));
	CommandFinished();
}

/*********************************************
 * @brief cmd_GetAmp
 * Specific command handler
 * @param  char *p_Data - pointer to command message after unique identifier.
 * @retval None
 */
void cmd_GetAmp(char *p_Data)
{
	// Respond with current amplitude.
	_WAVEFORM_DESCRIPTOR Waveform = WaveformGenerator_Get_Waveform();

	// Use floating point in sprintf you need to configure project settings.
	sprintf(CommandResponseBuff, "%lf\r\n", Waveform.f_Amplitude);

	USART_Request_Tx(CommandResponseBuff, strlen(CommandResponseBuff));
	CommandFinished();
}

/*********************************************
 * @brief cmd_GetOffs
 * Specific command handler
 * @param  char *p_Data - pointer to command message after unique identifier.
 * @retval None
 */
void cmd_GetOffs(char *p_Data)
{
	// Respond with current offsets.
	_WAVEFORM_DESCRIPTOR Waveform = WaveformGenerator_Get_Waveform();

	// Use floating point in sprintf you need to configure project settings.
	sprintf(CommandResponseBuff, "%lf\r\n", Waveform.f_Offset);

	USART_Request_Tx(CommandResponseBuff, strlen(CommandResponseBuff));
	CommandFinished();
}

/*********************************************
 * @brief cmd_GetPWM
 * Specific command handler
 * @param  char *p_Data - pointer to command message after unique identifier.
 * @retval None
 */
void cmd_GetPWM(char *p_Data)
{
	// Respond with current PWM Base frequency.
	sprintf(CommandResponseBuff, "%lu\r\n", PWM_GET_Base_Frequency());

	USART_Request_Tx(CommandResponseBuff, strlen(CommandResponseBuff));
	CommandFinished();
}

/*********************************************
 * @brief cmd_GetSamples
 * Specific command handler
 * @param  char *p_Data - pointer to command message after unique identifier.
 * @retval None
 */
void cmd_GetSamples(char *p_Data)
{
	// Respond with current PWM Base frequency.
	sprintf(CommandResponseBuff, "%u\r\n", WaveformGenerator_Get_NUM_DAC_Sample());

	USART_Request_Tx(CommandResponseBuff, strlen(CommandResponseBuff));
	CommandFinished();
}

/*********************************************
 * @brief cmd_Help
 * Specific command handler
 * @param  char *p_Data - pointer to command message after unique identifier.
 * @retval None
 */
void cmd_Help(char *p_Data)
{
	static uint16_t CommandIndex;

	// Transmit another command detail?
	if(CommandIndex < sizeof(CommandTable) / sizeof(_COMMAND_FORMAT))
	{
		// Can we transmit next command detail?
		const char *p_Str = CommandTable[CommandIndex].str_HelpText;
		if(USART_Request_Tx((char *)p_Str, strlen(p_Str)))
		{
			// Transmission underway , set up for next
			do
			{
				CommandIndex++;
				if(CommandIndex >= sizeof(CommandTable) / sizeof(_COMMAND_FORMAT))
				{
					// All commands exhausted.
					CommandIndex = 0;
					CommandFinished();
				}
			}while(CommandTable[CommandIndex].c_ShowInHelp == FALSE);
		}
		// else - come back here and try again.
	}
	else
	{
		// Finished
		CommandIndex = 0;
		CommandFinished();
	}
}

/*********************************************
 * @brief cmd_HandleUnknownCommands
 * Specific command handler
 * @param  char *p_Data - pointer to command message after unique identifier.
 * @retval None
 */
void cmd_HandlerUnknownCommands(char * p_Data)
{
	USART_Request_Tx((char *)UNKNOWN_COMMAND_RESPONSE, strlen(UNKNOWN_COMMAND_RESPONSE));
	CommandFinished();
}



