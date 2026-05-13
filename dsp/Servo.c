#include "stm32f10x.h"
#include "PWM.h"

void Servo_Init(void)
{
    MyPWM_Init();
}

void Servo1_SetAngle(uint16_t Angle)
{
    Out3Set(2000 * Angle / 180  + 500);
}

void Servo2_SetAngle(uint16_t Angle)
{
    Out4Set(2000 * Angle / 180  + 500);
}

void Servo1_SetPulse(uint16_t pulse_us)
{
    Out3Set(pulse_us);
}

void Servo2_SetPulse(uint16_t pulse_us)
{
    Out4Set(pulse_us);
}
