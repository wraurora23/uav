#include <stdio.h>
#include <math.h>

float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float exInt = 0, eyInt = 0, ezInt = 0;

float Kp = 2.0f;
float Ki = 0.005f;

#define I_LIMIT 0.1f

void Mahony_Update(float gx, float gy, float gz,
                   float ax, float ay, float az, float dt)
{
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;

    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0) return;
    ax /= norm;
    ay /= norm;
    az /= norm;

    vx = 2 * (q1 * q3 - q0 * q2);
    vy = 2 * (q0 * q1 + q2 * q3);
    vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    if (norm > 0.8f && norm < 1.2f)
    {
        ex = (ay * vz - az * vy);
        ey = (az * vx - ax * vz);
        ez = (ax * vy - ay * vx);

        exInt += ex * Ki;
        eyInt += ey * Ki;
        ezInt += ez * Ki;

        if (exInt > I_LIMIT) exInt = I_LIMIT;
        if (exInt < -I_LIMIT) exInt = -I_LIMIT;

        if (eyInt > I_LIMIT) eyInt = I_LIMIT;
        if (eyInt < -I_LIMIT) eyInt = -I_LIMIT;

        if (ezInt > I_LIMIT) ezInt = I_LIMIT;
        if (ezInt < -I_LIMIT) ezInt = -I_LIMIT;

        gx += Kp * ex + exInt;
        gy += Kp * ey + eyInt;
        gz += Kp * ez + ezInt;
    }

    q0 += (-q1*gx - q2*gy - q3*gz) * dt * 0.5f;
    q1 += ( q0*gx + q2*gz - q3*gy) * dt * 0.5f;
    q2 += ( q0*gy - q1*gz + q3*gx) * dt * 0.5f;
    q3 += ( q0*gz + q1*gy - q2*gx) * dt * 0.5f;

    norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 /= norm;
    q1 /= norm;
    q2 /= norm;
    q3 /= norm;
}

void Mahony_GetAngle(float *pitch, float *roll)
{
    *roll  = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2)) * 57.3f;
    *pitch = asin(2*(q0*q2 - q3*q1)) * 57.3f;
}

void Mahony_GetEuler(float *pitch, float *roll, float *yaw)
{
    if (roll != 0)
    {
        *roll = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2)) * 57.3f;
    }

    if (pitch != 0)
    {
        *pitch = asin(2*(q0*q2 - q3*q1)) * 57.3f;
    }

    if (yaw != 0)
    {
        *yaw = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3)) * 57.3f;
    }
}

void Mahony_Reset(void)
{
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;

    exInt = 0.0f;
    eyInt = 0.0f;
    ezInt = 0.0f;
}
