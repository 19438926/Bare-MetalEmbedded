/*
 * Touch.h
 *
 *  Created on: Jan 31, 2024
 *      Author: Wenze Pan
 */

#ifndef INC_TOUCH_H_
#define INC_TOUCH_H_

void Touch_Init(I2C_HandleTypeDef New_hi2cX);
void Touch_Process(void);

#endif /* INC_TOUCH_H_ */
