#ifndef __PID_H
#define __PID_H

#include <stdint.h>

typedef struct
{
    float Kp;
    float Ki;
    float Kd;

    float Target;
    float Measure;
    float Error;
    float Last_Error;

    float Integral;
    float Integral_Max;

    float Derivative;
    float Derivative_LPF_Alpha;

    float DeadBand;
    float Output;
    float Output_Max;

    uint8_t Enable_Integral_Separation;
    float Integral_Separation_Threshold;
} PID_t;

void PID_Reset(PID_t *pid);
float PID_Calc(PID_t *pid, float target, float current, float dt);

#endif
