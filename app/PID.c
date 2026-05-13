#include "stm32f10x.h"
#include "PID.h"
#include <math.h>

static float PID_Clamp(float value, float min_value, float max_value)
{
    if (value > max_value)
    {
        return max_value;
    }
    if (value < min_value)
    {
        return min_value;
    }
    return value;
}

void PID_Reset(PID_t *pid)
{
    pid->Target = 0.0f;
    pid->Measure = 0.0f;
    pid->Error = 0.0f;
    pid->Last_Error = 0.0f;
    pid->Integral = 0.0f;
    pid->Derivative = 0.0f;
    pid->Output = 0.0f;
}

float PID_Calc(PID_t *pid, float target, float current, float dt)
{
    float error;
    float derivative_raw;
    uint8_t allow_integral = 1u;

    if (dt <= 0.0f)
    {
        dt = 0.001f;
    }

    pid->Target = target;
    pid->Measure = current;

    error = target - current;

    if (fabsf(error) < pid->DeadBand)
    {
        error = 0.0f;
    }

    pid->Error = error;

    if (pid->Enable_Integral_Separation != 0u)
    {
        if (fabsf(error) > pid->Integral_Separation_Threshold)
        {
            allow_integral = 0u;
        }
    }

    if (allow_integral != 0u)
    {
        pid->Integral += error * dt;
        pid->Integral = PID_Clamp(pid->Integral, -pid->Integral_Max, pid->Integral_Max);
    }

    derivative_raw = (error - pid->Last_Error) / dt;
    pid->Derivative = pid->Derivative * pid->Derivative_LPF_Alpha +
                      derivative_raw * (1.0f - pid->Derivative_LPF_Alpha);

    pid->Output = pid->Kp * error +
                  pid->Ki * pid->Integral +
                  pid->Kd * pid->Derivative;
    pid->Output = PID_Clamp(pid->Output, -pid->Output_Max, pid->Output_Max);

    pid->Last_Error = error;

    return pid->Output;
}
