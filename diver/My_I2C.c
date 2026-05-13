#include "stm32f10x.h"
#include "Delay.h"

void My_I2C_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    I2C_InitTypeDef I2C_InitStruct;
    I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStruct.I2C_ClockSpeed = 400000;
    I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStruct.I2C_OwnAddress1 = 0x00;
    I2C_Init(I2C1, &I2C_InitStruct);

    I2C_Cmd(I2C1, ENABLE);
}

void I2C_WriteData(uint8_t Data, uint8_t Addr1, uint8_t Addr2)
{
    //生成起始信号，等待ev5事件
    I2C_GenerateSTART(I2C1, ENABLE);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
    
    //发送从机地址，等待ev6事件
    I2C_Send7bitAddress(I2C1, Addr1 << 1, I2C_Direction_Transmitter);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    //发送寄存器地址，等待ev8事件
    I2C_SendData(I2C1, Addr2);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING));

    //发送数据，等待ev8_2事件
    I2C_SendData(I2C1, Data);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    //生成停止信号
    I2C_GenerateSTOP(I2C1, ENABLE);
}

uint8_t I2C_ReadData(uint8_t Addr1, uint8_t Addr2)
{
    uint8_t Data;

    //生成起始信号，等待ev5事件
    I2C_GenerateSTART(I2C1, ENABLE);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

    //发送从机地址，方向为写，等待ev6
    I2C_Send7bitAddress(I2C1, Addr1 << 1, I2C_Direction_Transmitter);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    //发送寄存器地址，等待ev8_2信号
    I2C_SendData(I2C1, Addr2);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    //生成起始信号，等待ev5事件
    I2C_GenerateSTART(I2C1, ENABLE);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

    //发送从机地址，方向为读，等待ev6信号
    I2C_Send7bitAddress(I2C1, Addr1 << 1, I2C_Direction_Receiver);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

    //在要发送的最后一个字节的数据前停止发送ack
    I2C_AcknowledgeConfig(I2C1, DISABLE);

    //等待ev7事件，读取数据
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
    Data = I2C_ReceiveData(I2C1);

    I2C_GenerateSTOP(I2C1, ENABLE);

    //发送ack信号，不影响后续可能产生的读取数据
    I2C_AcknowledgeConfig(I2C1, ENABLE);

    return Data;
}

void I2C_ReadBuf(uint8_t Addr1, uint8_t Addr2, uint8_t *buf, uint8_t len)
{
    I2C_GenerateSTART(I2C1, ENABLE);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

    //发送从机地址，方向为写，等待ev6信号
    I2C_Send7bitAddress(I2C1, Addr1 << 1, I2C_Direction_Transmitter);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    //发送寄存器地址，等待ev8_2信号
    I2C_SendData(I2C1, Addr2);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    //生成起始信号，等待ev5事件
    I2C_GenerateSTART(I2C1, ENABLE);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

    //发送从机地址，方向为读，等待ev6信号
    I2C_Send7bitAddress(I2C1, Addr1 << 1, I2C_Direction_Receiver);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

    for (uint8_t i = 0; i < len; i++)
    {
        if (i == len - 1)
        {
            I2C_AcknowledgeConfig(I2C1, DISABLE);
        }
        
        //等待ev7事件，读取数据
        while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
        buf[i] = I2C_ReceiveData(I2C1);
    }
    I2C_GenerateSTOP(I2C1, ENABLE);

    //发送ack信号，不影响后续可能产生的读取数据
    I2C_AcknowledgeConfig(I2C1, ENABLE);
}

void I2CSoft_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_SetBits(GPIOB, GPIO_Pin_6 | GPIO_Pin_7);
}

void SCL_Write(uint8_t Bit)
{
    GPIO_WriteBit(GPIOB, GPIO_Pin_6, (BitAction)Bit);
    Delay_us(5);
}

void SDA_Write(uint8_t Bit)
{
    GPIO_WriteBit(GPIOB, GPIO_Pin_7, (BitAction)Bit);
    Delay_us(5);
}

uint8_t SDA_Read(void)
{
    uint8_t Bit;
    Bit = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7);
    Delay_us(5);

    return Bit;
}

void MyI2C_Start(void)
{
    SDA_Write(1);
    SCL_Write(1);
    SDA_Write(0);
    SCL_Write(0);
}

void MyI2C_Stop(void)
{
    SDA_Write(0);
    SCL_Write(1);
    SDA_Write(1);
}

void MyI2C_SendByte(uint8_t Byte)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        SDA_Write(!!(Byte & (0x80 >> i)));
        SCL_Write(1);
        SCL_Write(0);
    }
    
}

uint8_t MyI2C_ReceiveByte(void)
{
    uint8_t Byte = 0x00;
    SDA_Write(1);
    for (uint8_t i = 0; i < 8; i++)
    {
        Byte <<= 1;
        SCL_Write(1);
        if (SDA_Read())
        {
            Byte |= 0x01;
        }
        SCL_Write(0);
    }

    return Byte;
}

void MyI2C_SendAck(uint8_t Ack)
{
    SDA_Write(Ack);
    SCL_Write(1);
    SCL_Write(0);
}

uint8_t MyI2C_ReceiveAck(void)
{
    uint8_t Ack;
    SDA_Write(1);
    SCL_Write(1);

    Ack = SDA_Read();
    SCL_Write(0);

    return Ack;
}
