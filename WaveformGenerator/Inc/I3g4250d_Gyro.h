/**************************************************************************************************************
 *                  File Name       :I3g4250d_Gyro.h
 *                  Overview        :Gyro functionality.
 *                  Authors         :Wenze Pan
 **************************************************************************************************************
 *                  Copyright @ As Applicable
 **************************************************************************************************************/

#ifndef INC_I3G4250D_GYRO_H_
#define INC_I3G4250D_GYRO_H_


void SPI_Init();
void I3g4250d_Check();
void SPI_Receive();
void SPI_Transmit();
void I3g4250d_Init();
void I3g4250d_Loop();
float Get_Y_Data();

#endif /* INC_I3G4250D_GYRO_H_ */
