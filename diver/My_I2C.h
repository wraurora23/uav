#ifndef __MY_I2C_H
#define __MY_I2C_H

#include "stdint.h"
//硬件iic
void My_I2C_Init(void);
void I2C_WriteData(uint8_t Data, uint8_t Addr1, uint8_t Addr2);
uint8_t I2C_ReadData(uint8_t Addr1, uint8_t Addr2);
void I2C_ReadBuf(uint8_t Addr1, uint8_t Addr2, uint8_t *buf, uint8_t len);

//软件iic
void I2CSoft_Init(void);
void MyI2C_Start(void);
void MyI2C_Stop(void);
void MyI2C_SendByte(uint8_t Byte);
uint8_t MyI2C_ReceiveByte(void);
void MyI2C_SendAck(uint8_t Ack);
uint8_t MyI2C_ReceiveAck(void);

#endif
