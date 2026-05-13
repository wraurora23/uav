#include "stm32f10x.h"
#include "Event.h"

static uint16_t imu_tick = 0;
static uint16_t control_tick = 0;
static uint16_t usart_tick = 0;

void Timer_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStruct.TIM_Period = 1000 - 1;
    TIM_TimeBaseInitStruct.TIM_Prescaler = 72 - 1;
    TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);

    TIM_InternalClockConfig(TIM3);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
}

void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

        imu_tick++;
        control_tick++;
        usart_tick++;

        if (imu_tick >= 5u)
        {
            MPU_Flag = 1u;
            imu_tick = 0u;
        }

        if (control_tick >= 10u)
        {
            Control_Flag = 1u;
            control_tick = 0u;
        }

        if (usart_tick >= 100u)
        {
            Usart_Flag = 1u;
            usart_tick = 0u;
        }
    }
}
