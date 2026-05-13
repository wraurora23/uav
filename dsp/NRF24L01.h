#ifndef __NRF24L01_H
#define __NRF24L01_H

#include <stdint.h>
#include "stm32f10x.h"

/*
 * Default CC3D receiver-input mapping from schematic:
 * S1IN = PB6
 * S2IN = PB5
 * S3IN = PB0
 * S4IN = PB1
 * S5IN = PA0
 * S6IN = PA1
 *
 * Current soft-SPI assignment:
 * S1IN -> SCK
 * S2IN -> MOSI
 * S3IN -> MISO
 * S4IN -> CSN
 * S5IN -> CE
 *
 * If your fly-wire order is different, change only these 5 macros.
 */
#define NRF24L01_CE_PORT                    GPIOA
#define NRF24L01_CE_PIN                     GPIO_Pin_0

#define NRF24L01_CSN_PORT                   GPIOB
#define NRF24L01_CSN_PIN                    GPIO_Pin_1

#define NRF24L01_SCK_PORT                   GPIOB
#define NRF24L01_SCK_PIN                    GPIO_Pin_6

#define NRF24L01_MOSI_PORT                  GPIOB
#define NRF24L01_MOSI_PIN                   GPIO_Pin_5

#define NRF24L01_MISO_PORT                  GPIOB
#define NRF24L01_MISO_PIN                   GPIO_Pin_0

#define NRF24L01_PAYLOAD_WIDTH              12u
#define NRF24L01_FLAG_ARMED                 0x01u

/*
 * 默认接收帧格式:
 * byte0  = 0xA5
 * byte1  = 0x5A
 * byte2  = throttle low
 * byte3  = throttle high
 * byte4  = yaw low
 * byte5  = yaw high
 * byte6  = pitch low
 * byte7  = pitch high
 * byte8  = roll low
 * byte9  = roll high
 * byte10 = flags
 * byte11 = checksum(sum of byte0~byte10)
 */
typedef struct
{
    uint16_t throttle;
    uint16_t yaw;
    uint16_t pitch;
    uint16_t roll;
    uint8_t flags;
} NRF24L01_RemoteFrame_t;

void NRF24L01_Init(void);
uint8_t NRF24L01_Check(void);
uint8_t NRF24L01_ReadRemoteFrame(NRF24L01_RemoteFrame_t *frame);
uint8_t NRF24L01_IsConfigured(void);

#endif
