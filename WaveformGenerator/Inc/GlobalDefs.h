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
#define          SYS_CLOCK_FRQ              180000000
#define          APB1_TIMER_CLOCK_FRQ       90000000
#define          APB2_CLOCK_FRQ             90000000

#ifndef  FALSE
#define  FALSE   0
#endif
#ifndef  TRUE
#define  TRUE   !FALSE
#endif

// Custom Waveform definitions
#define MAX_CUSTOM_DATA_LENGTH           1000  // Maximum number of 16 bit values in custom waveform data.
#define MAX_FREQ_MANUALLY_OUTPUTTED_uS    5000  // 5mS / 200Hz max for manually outputting data via main loop.
#define MAX_DAC_SAMPLE                   1000 // Maximum number of DAC samples to be outputed by DMA.

/***************************************************/
/* Global Types */

/**************************************************/
/* Globally available variable */

/*************************************************/
/*Globally available functions */


#endif /* GLOBALDEFS_H_ */
