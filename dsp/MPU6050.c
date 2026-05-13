#include "stm32f10x.h"
#include "My_I2C.h"

void MPU6050_WriteReg(uint8_t Reg, uint8_t Data)
{
    I2C_WriteData(Data, 0x68, Reg);
}

uint8_t MPU6050_ReadReg(uint8_t Reg)
{
    uint8_t Data;
    Data = I2C_ReadData(0x68, Reg);

    return Data;
}

void MPU6050_ReadBuf(uint8_t Reg, uint8_t *buf, uint8_t Len)
{
    I2C_ReadBuf(0x68, Reg, buf, Len);
}

void MPU6050_GetData(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[14];

    MPU6050_ReadBuf(0x3B, buf, 14);

    *ax = (int16_t)(buf[0] << 8 | buf[1]);
    *ay = (int16_t)(buf[2] << 8 | buf[3]);
    *az = (int16_t)(buf[4] << 8 | buf[5]);
    *gx = (int16_t)(buf[8] << 8 | buf[9]);
    *gy = (int16_t)(buf[10] << 8 | buf[11]);
    *gz = (int16_t)(buf[12] << 8 | buf[13]);
}

void MPU6050_GetAccel(int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t buf[6];

    MPU6050_ReadBuf(0x3B, buf, 6);

    *ax = (int16_t)(buf[0] << 8 | buf[1]);
    *ay = (int16_t)(buf[2] << 8 | buf[3]);
    *az = (int16_t)(buf[4] << 8 | buf[5]);
}

void MPU6050_GetGroy(int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[6];

    MPU6050_ReadBuf(0x43, buf, 6);

    *gx = (int16_t)(buf[0] << 8 | buf[1]);
    *gy = (int16_t)(buf[2] << 8 | buf[3]);
    *gz = (int16_t)(buf[4] << 8 | buf[5]);
}

void MPU6050_Init(void)
{
    My_I2C_Init();
    MPU6050_WriteReg(0x6B, 0x01);				//电源管理寄存器1，取消睡眠模式，选择时钟源为X轴陀螺仪
	MPU6050_WriteReg(0x6C, 0x00);				//电源管理寄存器2，保持默认值0，所有轴均不待机
	MPU6050_WriteReg(0x19, 0x09);				//采样率分频寄存器，配置采样率
	MPU6050_WriteReg(0x1A, 0x06);					//配置寄存器，配置DLPF
	MPU6050_WriteReg(0x1B, 0x18);			//陀螺仪配置寄存器，选择满量程为±2000°/s
	MPU6050_WriteReg(0x1C, 0x18);			//加速度计配置寄存器，选择满量程为±16g
}

void MPU_WriteReg(uint8_t Reg, uint8_t Data)
{
    MyI2C_Start();
    MyI2C_SendByte(0x68 << 1);
    MyI2C_ReceiveAck();

    MyI2C_SendByte(Reg);
    MyI2C_ReceiveAck();

    MyI2C_SendByte(Data);
    MyI2C_ReceiveAck();
    MyI2C_Stop();
}

uint8_t MPU_ReadReg(uint8_t Reg)
{
    uint8_t Data;

    MyI2C_Start();
    MyI2C_SendByte(0x68 << 1);
    MyI2C_ReceiveAck();

    MyI2C_SendByte(Reg);
    MyI2C_ReceiveAck();

    MyI2C_Start();
    MyI2C_SendByte((0x68 << 1) | 1);
    MyI2C_ReceiveAck();

    Data = MyI2C_ReceiveByte();
    MyI2C_SendAck(1);
    MyI2C_Stop();

    return Data;
}

void MPU_ReadBuf(uint8_t Reg, uint8_t *buf, uint8_t Len)
{
    MyI2C_Start();
    MyI2C_SendByte(0x68 << 1);
    MyI2C_ReceiveAck();

    MyI2C_SendByte(Reg);
    MyI2C_ReceiveAck();

    MyI2C_Start();
    MyI2C_SendByte((0x68 << 1) | 1);
    MyI2C_ReceiveAck();

    for (uint8_t i = 0; i < Len - 1; i++)
    {
        buf[i] = MyI2C_ReceiveByte();
        MyI2C_SendAck(0);
    }
    buf[Len - 1] = MyI2C_ReceiveByte();
    MyI2C_SendAck(1);

    MyI2C_Stop();
}


void MPU_GetData(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[14];

    MPU_ReadBuf(0x3B, buf, 14);

    *ax = (int16_t)(buf[0] << 8 | buf[1]);
    *ay = (int16_t)(buf[2] << 8 | buf[3]);
    *az = (int16_t)(buf[4] << 8 | buf[5]);
    *gx = (int16_t)(buf[8] << 8 | buf[9]);
    *gy = (int16_t)(buf[10] << 8 | buf[11]);
    *gz = (int16_t)(buf[12] << 8 | buf[13]);
}


void MPU_Init(void)
{
    I2CSoft_Init();
    MPU_WriteReg(0x6B, 0x01);				//电源管理寄存器1，取消睡眠模式，选择时钟源为X轴陀螺仪
	MPU_WriteReg(0x6C, 0x00);				//电源管理寄存器2，保持默认值0，所有轴均不待机
	MPU_WriteReg(0x19, 0x09);				//采样率分频寄存器，配置采样率
	MPU_WriteReg(0x1A, 0x06);					//配置寄存器，配置DLPF
	MPU_WriteReg(0x1B, 0x18);			//陀螺仪配置寄存器，选择满量程为±2000°/s
	MPU_WriteReg(0x1C, 0x18);			//加速度计配置寄存器，选择满量程为±16g
}

/*
void MPU_GetData(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz)
{
    *ax = (MPU_ReadReg(0x3B) << 8) | MPU_ReadReg(0x3C);
    *ay = (MPU_ReadReg(0x3D) << 8) | MPU_ReadReg(0x3E);
    *az = (MPU_ReadReg(0x3F) << 8) | MPU_ReadReg(0x40);
    *gx = (MPU_ReadReg(0x43) << 8) | MPU_ReadReg(0x44);
    *gy = (MPU_ReadReg(0x45) << 8) | MPU_ReadReg(0x46);
    *gz = (MPU_ReadReg(0x47) << 8) | MPU_ReadReg(0x48);
}
*/
