/**************************************************************************************************************
 *                  File Name       :GlobalDefs.h
 *                  Overview        :Central place to keep all global definitions and types etc.
 *                  Authors         :Wenze Pan
 **************************************************************************************************************
 *                  Copyright @ As Applicable
 **************************************************************************************************************/

#ifndef GLOBALDEFS_H_
#define GLOBALDEFS_H_
/***************************************************/
/* Global Definitions  */
#define          SYS_CLOCK_FRQ       72000000

#ifndef  FALSE
#define  FALSE   0
#endif
#ifndef  TRUE
#define  TRUE   !FALSE
#endif

// Custom Waveform definitions
#define MAX_CUSTOM_DATA_LENGTH           1000  // Maximum number of 16 bit values in custom waveform data.
#define MAX_FREQ_MANUALLY_OUTPUTTED_uS    5000  // 5mS / 200Hz max for manually outputting data via main loop.

/***************************************************/
/* Global Types */

/**************************************************/
/* Globally available variable */

/*************************************************/
/*Globally available functions */


#endif /* GLOBALDEFS_H_ */
