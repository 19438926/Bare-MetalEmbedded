/**************************************************************************************************************
 *                  File Name       :STMPE811.h
 *                  Overview        :touch screen controller functionality.
 *                  Authors         :Wenze Pan
 **************************************************************************************************************
 *                  Copyright @ As Applicable
 **************************************************************************************************************/

#ifndef STMPE811_H_
#define STMPE811_H_

void Touch_Init();
void Touch_Process(void);
void I2C_Init();
void I2C_Check();
void I2C_DMA_En();



#endif /* STMPE811_H_ */
