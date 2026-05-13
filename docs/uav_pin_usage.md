# 无人机飞控端引脚使用说明

本文档只整理 `uav` 工程中无人机飞控端当前主流程实际使用的引脚，不包含遥控器端。

当前飞控板：`CC3D / CopterControl 3D`

主控芯片：`STM32F103`

代码入口：`app/main.c -> System_Init() -> Task_Run()`

注意：当前 `app/Task.c` 中 `TASK_ESC_CALIBRATION_MODE = 1u`，表示飞控固件处于电调校准模式。正常飞控运行前需要改回 `0u`。

## 1. 主控芯片引脚总表

| MCU 引脚 | 当前用途 | 外设/模式 | 连接对象 | 代码位置 |
|---|---|---|---|---|
| `PA0` | nRF24L01 `CE` | GPIO 输出 | 接收机口 `S5IN` | `dsp/NRF24L01.h` |
| `PA4` | MPU6000 `CS/NSS` | GPIO 输出 | MPU6000 片选 | `diver/My_SPI.c`, `dsp/MPU6000.c` |
| `PA5` | MPU6000 `SCK` | `SPI1_SCK` | MPU6000 时钟 | `diver/My_SPI.c` |
| `PA6` | MPU6000 `MISO` | `SPI1_MISO` | MPU6000 数据输出 | `diver/My_SPI.c` |
| `PA7` | MPU6000 `MOSI` | `SPI1_MOSI` | MPU6000 数据输入 | `diver/My_SPI.c` |
| `PA8` | 舵机 2 | `TIM1_CH1 / PWM` | 飞控 `Out4` | `diver/PWM.c`, `dsp/Servo.c` |
| `PA9` | 串口 TX | `USART1_TX` | 串口模块/蓝牙 RX | `diver/USART.c` |
| `PA10` | 串口 RX | `USART1_RX` | 串口模块/蓝牙 TX | `diver/USART.c` |
| `PB0` | nRF24L01 `MISO` | GPIO 输入上拉 | 接收机口 `S3IN` | `dsp/NRF24L01.h` |
| `PB1` | nRF24L01 `CSN` | GPIO 输出 | 接收机口 `S4IN` | `dsp/NRF24L01.h` |
| `PB5` | nRF24L01 `MOSI` | GPIO 输出 | 接收机口 `S2IN` | `dsp/NRF24L01.h` |
| `PB6` | nRF24L01 `SCK` | GPIO 输出 | 接收机口 `S1IN` | `dsp/NRF24L01.h` |
| `PB7` | 舵机 1 | `TIM4_CH2 / PWM` | 飞控 `Out3` | `diver/PWM.c`, `dsp/Servo.c` |
| `PB8` | 电机 2 | `TIM4_CH3 / PWM` | 飞控 `Out2` / 电调 2 | `diver/PWM.c`, `dsp/Motor.c` |
| `PB9` | 电机 1 | `TIM4_CH4 / PWM` | 飞控 `Out1` / 电调 1 | `diver/PWM.c`, `dsp/Motor.c` |

## 2. nRF24L01 接收模块接线

飞控没有直接引出 SPI，因此当前使用 CC3D 接收机输入口模拟 SPI。

| nRF24L01 引脚 | CC3D 接收机口 | MCU 引脚 | 方向 |
|---|---|---|---|
| `CE` | `S5IN` | `PA0` | 飞控输出 |
| `CSN` | `S4IN` | `PB1` | 飞控输出 |
| `SCK` | `S1IN` | `PB6` | 飞控输出 |
| `MOSI` | `S2IN` | `PB5` | 飞控输出 |
| `MISO` | `S3IN` | `PB0` | 飞控输入 |
| `IRQ` | 不接 | 未使用 | 无 |
| `VCC` | 独立 `3.3V` | 不接 5V | 供电 |
| `GND` | 飞控 GND | GND | 共地 |

CC3D 接收机口与 MCU 引脚对应关系：

| 接收机口 | MCU 引脚 | 当前用途 |
|---|---|---|
| `S1IN` | `PB6` | nRF24L01 `SCK` |
| `S2IN` | `PB5` | nRF24L01 `MOSI` |
| `S3IN` | `PB0` | nRF24L01 `MISO` |
| `S4IN` | `PB1` | nRF24L01 `CSN` |
| `S5IN` | `PA0` | nRF24L01 `CE` |
| `S6IN` | `PA1` | 当前未使用 |

注意：

- nRF24L01 只能接 `3.3V`，不能接接收机口的 `5V`。
- 建议在 nRF24L01 的 `VCC-GND` 旁边并联 `0.1uF + 47uF~100uF` 电容。
- `IRQ` 当前代码没有使用，可以悬空不接。

## 3. MPU6000 姿态传感器

当前主流程使用 `MPU6000`，通过 `SPI1` 通信。

| MPU6000 引脚 | MCU 引脚 | 外设/模式 |
|---|---|---|
| `CS/NSS` | `PA4` | GPIO 输出 |
| `SCLK` | `PA5` | `SPI1_SCK` |
| `SDO/MISO` | `PA6` | `SPI1_MISO` |
| `SDI/MOSI` | `PA7` | `SPI1_MOSI` |

当前代码中没有使用 MPU6000 的外部中断引脚，姿态读取由 `TIM3` 定时任务触发。

## 4. 电机和舵机输出

PWM 周期配置为 `20000us`，即常见舵机/电调 `50Hz` PWM。

| 输出名称 | MCU 引脚 | 定时器通道 | 当前连接 |
|---|---|---|---|
| `Out1` | `PB9` | `TIM4_CH4` | 电机/电调 1 |
| `Out2` | `PB8` | `TIM4_CH3` | 电机/电调 2 |
| `Out3` | `PB7` | `TIM4_CH2` | 舵机 1 |
| `Out4` | `PA8` | `TIM1_CH1` | 舵机 2 |

代码映射：

| 控制函数 | 实际输出口 | MCU 引脚 |
|---|---|---|
| `SetMotor1()` | `Out1` | `PB9` |
| `SetMotor2()` | `Out2` | `PB8` |
| `Servo1_SetAngle()` | `Out3` | `PB7` |
| `Servo2_SetAngle()` | `Out4` | `PA8` |

## 5. 串口调试接口

| 串口功能 | MCU 引脚 | 配置 |
|---|---|---|
| `USART1_TX` | `PA9` | `115200, 8N1` |
| `USART1_RX` | `PA10` | `115200, 8N1` |

接线：

| 飞控 | 串口模块/蓝牙 |
|---|---|
| `PA9 / TX` | `RX` |
| `PA10 / RX` | `TX` |
| `GND` | `GND` |

如果只看飞控打印，可以只接 `PA9 -> RX` 和 `GND -> GND`。

## 6. 定时器资源

| 定时器 | 用途 | 是否占用外部引脚 |
|---|---|---|
| `TIM1` | 舵机 2 PWM，`CH1` | 使用 `PA8` |
| `TIM3` | 1ms 系统调度中断 | 不占用外部引脚 |
| `TIM4` | 电机 1、电机 2、舵机 1 PWM | 使用 `PB9/PB8/PB7` |

`TIM3` 分频任务：

| 任务 | 周期 |
|---|---|
| 姿态更新 | `5ms / 200Hz` |
| 控制更新 | `10ms / 100Hz` |
| 串口打印 | `100ms / 10Hz` |

## 7. 当前未使用或需要注意的引脚

| 引脚 | 状态 | 说明 |
|---|---|---|
| `PA1` | 当前未使用 | CC3D 接收机口 `S6IN`，可作为备用 GPIO |
| `PB6/PB7` I2C 旧驱动 | 当前主流程未使用 | 工程里存在 `My_I2C.c / MPU6050.c` 旧代码，但当前主流程使用 `MPU6000`，不要同时启用，否则会和 nRF/舵机引脚冲突 |
| `PA4~PA7` 普通 SPI 口 | 已被 MPU6000 占用 | 不建议再接其它 SPI 设备 |
| `PB0/PB1/PB5/PB6/PA0` | 已被 nRF24L01 软 SPI 占用 | 不建议再接接收机 PWM 或其它外设 |

## 8. 简化接线表

飞控到外设的实际接线可以按下面看：

| 外设 | 信号 | 飞控 MCU 引脚 |
|---|---|---|
| nRF24L01 | `CE` | `PA0` |
| nRF24L01 | `CSN` | `PB1` |
| nRF24L01 | `SCK` | `PB6` |
| nRF24L01 | `MOSI` | `PB5` |
| nRF24L01 | `MISO` | `PB0` |
| MPU6000 | `CS` | `PA4` |
| MPU6000 | `SCK` | `PA5` |
| MPU6000 | `MISO` | `PA6` |
| MPU6000 | `MOSI` | `PA7` |
| 电调 1 | `PWM` | `PB9 / Out1` |
| 电调 2 | `PWM` | `PB8 / Out2` |
| 舵机 1 | `PWM` | `PB7 / Out3` |
| 舵机 2 | `PWM` | `PA8 / Out4` |
| 串口/蓝牙 | `TX` | `PA9` |
| 串口/蓝牙 | `RX` | `PA10` |

