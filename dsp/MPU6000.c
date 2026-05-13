#include "stm32f10x.h"
#include "My_SPI.h"

void MPU_NSS_Set(uint8_t bit)
{
    GPIO_WriteBit(GPIOA, GPIO_Pin_4, (BitAction)bit);
}

void MPU6000_WriteReg(uint8_t Reg, uint8_t Data)
{
    MPU_NSS_Set(0);
    
    SPI_SwapData(Reg & 0x7F);

    SPI_SwapData(Data);

    MPU_NSS_Set(1);
}

uint8_t MPU6000_ReadReg(uint8_t Reg)
{
    uint8_t Data;
    MPU_NSS_Set(0);
    SPI_SwapData(Reg | 0x80);

    Data = SPI_SwapData(0x00);

    MPU_NSS_Set(1);

    return Data;
}

void MPU6000_ReadBuf(uint8_t Reg, uint8_t *buf, uint8_t Len)
{
    MPU_NSS_Set(0);

    SPI_SwapData(Reg | 0x80);
    
    for (uint8_t i = 0; i < Len; i++)
    {
        buf[i] = SPI_SwapData(0x00);
    }
    
    MPU_NSS_Set(1);
}

void MPU6000_Init(void)
{
    My_SPI_Init();

    MPU6000_WriteReg(0x6A, 0x10);                   //禁用I2C
    MPU6000_WriteReg(0x6B, 0x01);				//电源管理寄存器1，取消睡眠模式，选择时钟源为X轴陀螺仪
	MPU6000_WriteReg(0x6C, 0x00);				//电源管理寄存器2，保持默认值0，所有轴均不待机
	MPU6000_WriteReg(0x19, 0x09);				//采样率分频寄存器，配置采样率
	MPU6000_WriteReg(0x1A, 0x06);					//配置寄存器，配置DLPF
	MPU6000_WriteReg(0x1B, 0x18);			//陀螺仪配置寄存器，选择满量程为±2000°/s
	MPU6000_WriteReg(0x1C, 0x18);			//加速度计配置寄存器，选择满量程为±16g
}

void MPU6000_GetAccel(int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t buf[6];

    MPU6000_ReadBuf(0x3B, buf, 6);

    *ax = (int16_t)(buf[0] << 8 | buf[1]);
    *ay = (int16_t)(buf[2] << 8 | buf[3]);
    *az = (int16_t)(buf[4] << 8 | buf[5]);
}

void MPU6000_GetGyro(int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[6];

    MPU6000_ReadBuf(0x43, buf, 6);

    *gx = (int16_t)(buf[0] << 8 | buf[1]);
    *gy = (int16_t)(buf[2] << 8 | buf[3]);
    *gz = (int16_t)(buf[4] << 8 | buf[5]);
}

void MPU6000_GetData(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[14];

    MPU6000_ReadBuf(0x3B, buf, 14);

    *ax = (int16_t)(buf[0] << 8 | buf[1]);
    *ay = (int16_t)(buf[2] << 8 | buf[3]);
    *az = (int16_t)(buf[4] << 8 | buf[5]);
    *gx = (int16_t)(buf[8] << 8 | buf[9]);
    *gy = (int16_t)(buf[10] << 8 | buf[11]);
    *gz = (int16_t)(buf[12] << 8 | buf[13]);
}
