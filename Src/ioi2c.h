#ifndef IOI2C_H
#define IOI2C_H
#include "main.h"
#include <stdint.h>

void IOI2C_INIT(void);
void I2C_SDA_OUT(void);
void I2C_SDA_IN(void);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_Ack(void);
void I2C_NAck(void);
uint8_t I2C_Wait_Ack(void);
void I2C_Send_Byte(uint8_t txd);
uint8_t I2C_Read_Byte(uint8_t ack);


#endif
