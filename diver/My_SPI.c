#include "stm32f10x.h"

void My_SPI_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1 | RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    GPIO_SetBits(GPIOA, GPIO_Pin_6);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    GPIO_SetBits(GPIOA, GPIO_Pin_4);

    SPI_InitTypeDef SPI_InitStruct;
    //全双工模式
    SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    //作为主机
    SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
    //数据大小
    SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
    //空闲时钟信号为高电平
    SPI_InitStruct.SPI_CPOL = SPI_CPOL_High;
    //在第二个变化沿进行数据采集
    SPI_InitStruct.SPI_CPHA = SPI_CPHA_2Edge;
    //片选信号由软件触发
    SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
    //对时钟信号进行分频
    SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;
    //数据高位先行
    SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
    //CRC校验，给一个初始值
    SPI_InitStruct.SPI_CRCPolynomial = 7;
    SPI_Init(SPI1, &SPI_InitStruct);

    SPI_Cmd(SPI1, ENABLE);
}

uint8_t SPI_SwapData(uint8_t Data)
{
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
    
    SPI_I2S_SendData(SPI1, Data);

    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
    
    return SPI_I2S_ReceiveData(SPI1);
}
