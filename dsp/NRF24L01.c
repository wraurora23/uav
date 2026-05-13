#include "NRF24L01.h"
#include "Delay.h"

#define NRF24L01_CMD_R_REGISTER             0x00u
#define NRF24L01_CMD_W_REGISTER             0x20u
#define NRF24L01_CMD_R_RX_PAYLOAD           0x61u
#define NRF24L01_CMD_FLUSH_RX               0xE2u
#define NRF24L01_CMD_NOP                    0xFFu

#define NRF24L01_REG_CONFIG                 0x00u
#define NRF24L01_REG_EN_AA                  0x01u
#define NRF24L01_REG_EN_RXADDR              0x02u
#define NRF24L01_REG_SETUP_AW               0x03u
#define NRF24L01_REG_SETUP_RETR             0x04u
#define NRF24L01_REG_RF_CH                  0x05u
#define NRF24L01_REG_RF_SETUP               0x06u
#define NRF24L01_REG_STATUS                 0x07u
#define NRF24L01_REG_RX_ADDR_P0             0x0Au
#define NRF24L01_REG_TX_ADDR                0x10u
#define NRF24L01_REG_RX_PW_P0               0x11u
#define NRF24L01_REG_FIFO_STATUS            0x17u

#define NRF24L01_STATUS_RX_DR               0x40u
#define NRF24L01_STATUS_TX_DS               0x20u
#define NRF24L01_STATUS_MAX_RT              0x10u
#define NRF24L01_FIFO_STATUS_RX_EMPTY       0x01u

static const uint8_t nrf24l01_addr[5] = {0x34u, 0x43u, 0x10u, 0x10u, 0x01u};

typedef struct
{
    uint8_t config;
    uint8_t en_aa;
    uint8_t en_rxaddr;
    uint8_t setup_aw;
    uint8_t setup_retr;
    uint8_t rf_ch;
    uint8_t rf_setup;
    uint8_t rx_addr_p0[5];
    uint8_t tx_addr[5];
} NRF24L01_ConfigSnapshot_t;

static void NRF24L01_EnableGPIOClock(GPIO_TypeDef *port)
{
    if (port == GPIOA)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    }
    else if (port == GPIOB)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    }
    else if (port == GPIOC)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    }
}

static void NRF24L01_InitPin(GPIO_TypeDef *port, uint16_t pin, GPIOMode_TypeDef mode)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    NRF24L01_EnableGPIOClock(port);

    GPIO_InitStruct.GPIO_Pin = pin;
    GPIO_InitStruct.GPIO_Mode = mode;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(port, &GPIO_InitStruct);
}

static void NRF24L01_DelayShort(void)
{
    __NOP();
    __NOP();
    __NOP();
    __NOP();
}

static void NRF24L01_CE(uint8_t bit)
{
    GPIO_WriteBit(NRF24L01_CE_PORT, NRF24L01_CE_PIN, (BitAction)bit);
}

static void NRF24L01_CSN(uint8_t bit)
{
    GPIO_WriteBit(NRF24L01_CSN_PORT, NRF24L01_CSN_PIN, (BitAction)bit);
}

static void NRF24L01_SCK(uint8_t bit)
{
    GPIO_WriteBit(NRF24L01_SCK_PORT, NRF24L01_SCK_PIN, (BitAction)bit);
}

static void NRF24L01_MOSI(uint8_t bit)
{
    GPIO_WriteBit(NRF24L01_MOSI_PORT, NRF24L01_MOSI_PIN, (BitAction)bit);
}

static uint8_t NRF24L01_MISO(void)
{
    return (uint8_t)GPIO_ReadInputDataBit(NRF24L01_MISO_PORT, NRF24L01_MISO_PIN);
}

static void NRF24L01_GPIO_Init(void)
{
    NRF24L01_InitPin(NRF24L01_CE_PORT, NRF24L01_CE_PIN, GPIO_Mode_Out_PP);
    NRF24L01_InitPin(NRF24L01_CSN_PORT, NRF24L01_CSN_PIN, GPIO_Mode_Out_PP);
    NRF24L01_InitPin(NRF24L01_SCK_PORT, NRF24L01_SCK_PIN, GPIO_Mode_Out_PP);
    NRF24L01_InitPin(NRF24L01_MOSI_PORT, NRF24L01_MOSI_PIN, GPIO_Mode_Out_PP);
    NRF24L01_InitPin(NRF24L01_MISO_PORT, NRF24L01_MISO_PIN, GPIO_Mode_IPU);

    NRF24L01_CE(0u);
    NRF24L01_CSN(1u);
    NRF24L01_SCK(0u);
    NRF24L01_MOSI(0u);
}

static uint8_t NRF24L01_SoftSPI_TransferByte(uint8_t data)
{
    uint8_t i;
    uint8_t receive = 0u;

    for (i = 0u; i < 8u; i++)
    {
        NRF24L01_SCK(0u);
        NRF24L01_MOSI((uint8_t)((data & 0x80u) != 0u));
        NRF24L01_DelayShort();

        NRF24L01_SCK(1u);
        receive <<= 1;
        if (NRF24L01_MISO() != 0u)
        {
            receive |= 0x01u;
        }
        NRF24L01_DelayShort();

        data <<= 1;
    }

    NRF24L01_SCK(0u);
    return receive;
}

static uint8_t NRF24L01_ReadReg(uint8_t reg)
{
    uint8_t value;

    NRF24L01_CSN(0u);
    NRF24L01_SoftSPI_TransferByte(NRF24L01_CMD_R_REGISTER | reg);
    value = NRF24L01_SoftSPI_TransferByte(NRF24L01_CMD_NOP);
    NRF24L01_CSN(1u);

    return value;
}

static void NRF24L01_ReadBuf(uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t i;

    NRF24L01_CSN(0u);
    NRF24L01_SoftSPI_TransferByte(NRF24L01_CMD_R_REGISTER | reg);
    for (i = 0u; i < len; i++)
    {
        buf[i] = NRF24L01_SoftSPI_TransferByte(NRF24L01_CMD_NOP);
    }
    NRF24L01_CSN(1u);
}

static void NRF24L01_WriteReg(uint8_t reg, uint8_t value)
{
    NRF24L01_CSN(0u);
    NRF24L01_SoftSPI_TransferByte(NRF24L01_CMD_W_REGISTER | reg);
    NRF24L01_SoftSPI_TransferByte(value);
    NRF24L01_CSN(1u);
}

static void NRF24L01_WriteBuf(uint8_t reg, const uint8_t *buf, uint8_t len)
{
    uint8_t i;

    NRF24L01_CSN(0u);
    NRF24L01_SoftSPI_TransferByte(NRF24L01_CMD_W_REGISTER | reg);
    for (i = 0u; i < len; i++)
    {
        NRF24L01_SoftSPI_TransferByte(buf[i]);
    }
    NRF24L01_CSN(1u);
}

static void NRF24L01_ReadPayload(uint8_t *buf)
{
    uint8_t i;

    NRF24L01_CSN(0u);
    NRF24L01_SoftSPI_TransferByte(NRF24L01_CMD_R_RX_PAYLOAD);
    for (i = 0u; i < NRF24L01_PAYLOAD_WIDTH; i++)
    {
        buf[i] = NRF24L01_SoftSPI_TransferByte(NRF24L01_CMD_NOP);
    }
    NRF24L01_CSN(1u);
}

static void NRF24L01_FlushRx(void)
{
    NRF24L01_CSN(0u);
    NRF24L01_SoftSPI_TransferByte(NRF24L01_CMD_FLUSH_RX);
    NRF24L01_CSN(1u);
}

static uint8_t NRF24L01_ParseFrame(const uint8_t *raw, NRF24L01_RemoteFrame_t *frame)
{
    uint8_t checksum = 0u;
    uint8_t i;

    if ((raw[0] != 0xA5u) || (raw[1] != 0x5Au))
    {
        return 0u;
    }

    for (i = 0u; i < 11u; i++)
    {
        checksum = (uint8_t)(checksum + raw[i]);
    }

    if (checksum != raw[11])
    {
        return 0u;
    }

    frame->throttle = (uint16_t)(raw[2] | ((uint16_t)raw[3] << 8));
    frame->yaw = (uint16_t)(raw[4] | ((uint16_t)raw[5] << 8));
    frame->pitch = (uint16_t)(raw[6] | ((uint16_t)raw[7] << 8));
    frame->roll = (uint16_t)(raw[8] | ((uint16_t)raw[9] << 8));
    frame->flags = raw[10];

    return 1u;
}

void NRF24L01_Init(void)
{
    NRF24L01_GPIO_Init();

    Delay_ms(5);
    NRF24L01_CE(0u);

    NRF24L01_WriteReg(NRF24L01_REG_CONFIG, 0x0Cu);
    NRF24L01_WriteReg(NRF24L01_REG_EN_AA, 0x01u);
    NRF24L01_WriteReg(NRF24L01_REG_EN_RXADDR, 0x01u);
    NRF24L01_WriteReg(NRF24L01_REG_SETUP_AW, 0x03u);
    NRF24L01_WriteReg(NRF24L01_REG_SETUP_RETR, 0x5Fu);
    NRF24L01_WriteReg(NRF24L01_REG_RF_CH, 40u);
    NRF24L01_WriteReg(NRF24L01_REG_RF_SETUP, 0x27u);
    NRF24L01_WriteBuf(NRF24L01_REG_RX_ADDR_P0, nrf24l01_addr, 5u);
    NRF24L01_WriteBuf(NRF24L01_REG_TX_ADDR, nrf24l01_addr, 5u);
    NRF24L01_WriteReg(NRF24L01_REG_RX_PW_P0, NRF24L01_PAYLOAD_WIDTH);
    NRF24L01_WriteReg(NRF24L01_REG_STATUS,
                      NRF24L01_STATUS_RX_DR | NRF24L01_STATUS_TX_DS | NRF24L01_STATUS_MAX_RT);
    NRF24L01_FlushRx();

    NRF24L01_WriteReg(NRF24L01_REG_CONFIG, 0x0Fu);
    Delay_ms(2);
    NRF24L01_CE(1u);
}

uint8_t NRF24L01_Check(void)
{
    uint8_t read_back[5];

    NRF24L01_WriteBuf(NRF24L01_REG_TX_ADDR, nrf24l01_addr, 5u);
    NRF24L01_ReadBuf(NRF24L01_REG_TX_ADDR, read_back, 5u);

    if ((read_back[0] == nrf24l01_addr[0]) &&
        (read_back[1] == nrf24l01_addr[1]) &&
        (read_back[2] == nrf24l01_addr[2]) &&
        (read_back[3] == nrf24l01_addr[3]) &&
        (read_back[4] == nrf24l01_addr[4]))
    {
        return 1u;
    }

    return 0u;
}

uint8_t NRF24L01_ReadRemoteFrame(NRF24L01_RemoteFrame_t *frame)
{
    uint8_t status;
    uint8_t fifo_status;
    uint8_t raw[NRF24L01_PAYLOAD_WIDTH];

    if (frame == 0)
    {
        return 0u;
    }

    status = NRF24L01_ReadReg(NRF24L01_REG_STATUS);
    fifo_status = NRF24L01_ReadReg(NRF24L01_REG_FIFO_STATUS);
    if (((status & NRF24L01_STATUS_RX_DR) == 0u) &&
        ((fifo_status & NRF24L01_FIFO_STATUS_RX_EMPTY) != 0u))
    {
        return 0u;
    }

    NRF24L01_ReadPayload(raw);
    NRF24L01_WriteReg(NRF24L01_REG_STATUS,
                      NRF24L01_STATUS_RX_DR | NRF24L01_STATUS_TX_DS | NRF24L01_STATUS_MAX_RT);

    return NRF24L01_ParseFrame(raw, frame);
}

static void NRF24L01_GetConfigSnapshot(NRF24L01_ConfigSnapshot_t *snapshot)
{
    if (snapshot == 0)
    {
        return;
    }

    snapshot->config = NRF24L01_ReadReg(NRF24L01_REG_CONFIG);
    snapshot->en_aa = NRF24L01_ReadReg(NRF24L01_REG_EN_AA);
    snapshot->en_rxaddr = NRF24L01_ReadReg(NRF24L01_REG_EN_RXADDR);
    snapshot->setup_aw = NRF24L01_ReadReg(NRF24L01_REG_SETUP_AW);
    snapshot->setup_retr = NRF24L01_ReadReg(NRF24L01_REG_SETUP_RETR);
    snapshot->rf_ch = NRF24L01_ReadReg(NRF24L01_REG_RF_CH);
    snapshot->rf_setup = NRF24L01_ReadReg(NRF24L01_REG_RF_SETUP);
    NRF24L01_ReadBuf(NRF24L01_REG_RX_ADDR_P0, snapshot->rx_addr_p0, 5u);
    NRF24L01_ReadBuf(NRF24L01_REG_TX_ADDR, snapshot->tx_addr, 5u);
}

uint8_t NRF24L01_IsConfigured(void)
{
    NRF24L01_ConfigSnapshot_t snapshot;
    uint8_t i;

    NRF24L01_GetConfigSnapshot(&snapshot);

    if ((snapshot.config != 0x0Fu) ||
        (snapshot.en_aa != 0x01u) ||
        (snapshot.en_rxaddr != 0x01u) ||
        (snapshot.setup_aw != 0x03u) ||
        (snapshot.setup_retr != 0x5Fu) ||
        (snapshot.rf_ch != 40u) ||
        (snapshot.rf_setup != 0x27u))
    {
        return 0u;
    }

    for (i = 0u; i < 5u; i++)
    {
        if ((snapshot.rx_addr_p0[i] != nrf24l01_addr[i]) ||
            (snapshot.tx_addr[i] != nrf24l01_addr[i]))
        {
            return 0u;
        }
    }

    return 1u;
}
