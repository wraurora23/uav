#include "stm32f10x.h"

void MyPWM_Init(void)
{
    static uint8_t pwm_initialized = 0u;

    if (pwm_initialized != 0u)
    {
        return;
    }

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    TIM_DeInit(TIM1);
    TIM_DeInit(TIM4);

    TIM_InternalClockConfig(TIM1);
    TIM_InternalClockConfig(TIM4);

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_TimeBaseInitStruct.TIM_Prescaler = 72 - 1;
    TIM_TimeBaseInitStruct.TIM_Period = 20000 - 1;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;

    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStruct);
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStruct);

    TIM_OCInitTypeDef TIM_OCInitStruct;
    TIM_OCStructInit(&TIM_OCInitStruct);
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_Pulse = 0;

    TIM_OC1Init(TIM1, &TIM_OCInitStruct);
    TIM_OC2Init(TIM4, &TIM_OCInitStruct);
    TIM_OC3Init(TIM4, &TIM_OCInitStruct);
    TIM_OC4Init(TIM4, &TIM_OCInitStruct);

    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_ARRPreloadConfig(TIM4, ENABLE);

    {
        TIM_BDTRInitTypeDef TIM_BDTRInitStruct;

        TIM_BDTRStructInit(&TIM_BDTRInitStruct);
        TIM_BDTRInitStruct.TIM_Break = TIM_Break_Disable;
        TIM_BDTRInitStruct.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
        TIM_BDTRConfig(TIM1, &TIM_BDTRInitStruct);
    }

    TIM_CtrlPWMOutputs(TIM1, ENABLE);

    TIM_GenerateEvent(TIM1, TIM_EventSource_Update);
    TIM_GenerateEvent(TIM4, TIM_EventSource_Update);

    TIM_Cmd(TIM1, ENABLE);
    TIM_Cmd(TIM4, ENABLE);

    pwm_initialized = 1u;
}

void Out1Set(uint16_t Compare)
{
    TIM_SetCompare4(TIM4, Compare);
}

void Out2Set(uint16_t Compare)
{
    TIM_SetCompare3(TIM4, Compare);
}

void Out3Set(uint16_t Compare)
{
    TIM_SetCompare2(TIM4, Compare);
}

void Out4Set(uint16_t Compare)
{
    TIM_SetCompare1(TIM1, Compare);
}
