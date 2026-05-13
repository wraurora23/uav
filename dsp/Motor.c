#include "stm32f10x.h"
#include "PWM.h"
#include"Delay.h"

void Motor_Init(void)
{
    MyPWM_Init();
}

void SetMotor1(uint16_t Motor)
{
    Out1Set(Motor);
}

void SetMotor2(uint16_t Motor)
{
    Out2Set(Motor);
}

