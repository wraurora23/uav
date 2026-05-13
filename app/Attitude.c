#include <math.h>
#include <stdio.h>
#include "MPU6000.h"
#include "Delay.h"
#include "Mahony.h"

static float mahony_pitch_offset = 0.0f;
static float mahony_roll_offset = 0.0f;
static float mahony_yaw_offset = 0.0f;
static float pitch_rate_dps = 0.0f;
static float roll_rate_dps = 0.0f;
static float yaw_rate_dps = 0.0f;

float gyro_x_offset = 0;
float gyro_y_offset = 0;
float gyro_z_offset = 0;
float pitch_offset = 0;
float roll_offset = 0;

#define ATTITUDE_SAMPLE_DT_S 0.005f
#define ATTITUDE_MOUNT_CUP_VERTICAL 1u
#define ATTITUDE_MAHONY_WARMUP_SAMPLES 800
#define ATTITUDE_MAHONY_CALIB_SAMPLES  300
#define ATTITUDE_MAHONY_CALIB_PASSES   2

uint8_t Still_Flag;
uint16_t Still_Count;

#define max_g           5
#define Still_Time      200

static float Attitude_NormalizeAngle(float angle)
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }

    while (angle < -180.0f)
    {
        angle += 360.0f;
    }

    return angle;
}

static void Attitude_MapAccelToBody(int16_t raw_ax, int16_t raw_ay, int16_t raw_az,
                                    float *body_x_g, float *body_y_g, float *body_z_g)
{
#if ATTITUDE_MOUNT_CUP_VERTICAL
    /*
     * CC3D is mounted vertically on the cup wall:
     * board face points outward, board top points up.
     * Body X: roll axis, Body Y: pitch axis, Body Z: main shaft/up axis.
     */
    *body_x_g = (float)raw_az / 2048.0f;
    *body_y_g = (float)raw_ay / 2048.0f;
    *body_z_g = (float)raw_ax / 2048.0f;
#else
    *body_x_g = (float)raw_ay / 2048.0f;
    *body_y_g = (float)raw_ax / 2048.0f;
    *body_z_g = (float)raw_az / 2048.0f;
#endif
}

static void Attitude_MapGyroToBody(int16_t raw_gx, int16_t raw_gy, int16_t raw_gz,
                                   float *body_roll_raw, float *body_pitch_raw, float *body_yaw_raw)
{
#if ATTITUDE_MOUNT_CUP_VERTICAL
    *body_roll_raw = (float)raw_gz;
    *body_pitch_raw = (float)raw_gy;
    *body_yaw_raw = (float)raw_gx;
#else
    *body_roll_raw = (float)raw_gy;
    *body_pitch_raw = (float)raw_gx;
    *body_yaw_raw = (float)(-raw_gz);
#endif
}

void MPU6000_GetAngle(float *pitch, float *roll)
{
    int16_t ax, ay, az, gx, gy, gz;
    float body_x_g;
    float body_y_g;
    float body_z_g;

    MPU6000_GetData(&ax, &ay, &az, &gx, &gy, &gz);

    Attitude_MapAccelToBody(ax, ay, az, &body_x_g, &body_y_g, &body_z_g);

    *roll = atan2(body_x_g, body_z_g) * 57.3f;
    *pitch = atan2(-body_y_g, sqrt((body_x_g * body_x_g) + (body_z_g * body_z_g))) * 57.3f;
}

void MPU6000_GetAngle_Raw(float *pitch, float *roll)
{
    int16_t ax, ay, az, gx, gy, gz;
    static float pitch_f = 0;
    static float roll_f = 0;
    float accel_pitch;
    float accel_roll;
    float dt = ATTITUDE_SAMPLE_DT_S;
    float body_x_g;
    float body_y_g;
    float body_z_g;
    float roll_raw;
    float pitch_raw;
    float yaw_raw;
    float gyro_roll;
    float gyro_pitch;

    MPU6000_GetData(&ax, &ay, &az, &gx, &gy, &gz);

    Attitude_MapAccelToBody(ax, ay, az, &body_x_g, &body_y_g, &body_z_g);
    Attitude_MapGyroToBody(gx, gy, gz, &roll_raw, &pitch_raw, &yaw_raw);
    (void)yaw_raw;

    accel_roll = atan2(body_x_g, body_z_g) * 57.3f;
    accel_pitch = atan2(-body_y_g, sqrt((body_x_g * body_x_g) + (body_z_g * body_z_g))) * 57.3f;

    gyro_roll = (roll_raw - gyro_x_offset) / 16.4f;
    gyro_pitch = (pitch_raw - gyro_y_offset) / 16.4f;

    roll_f = 0.98f * (roll_f + gyro_roll * dt) + 0.02f * accel_roll;
    pitch_f = 0.98f * (pitch_f + gyro_pitch * dt) + 0.02f * accel_pitch;

    *roll = roll_f;
    *pitch = pitch_f;
}

void MPU6000_GetAngle_Filter(float *pitch, float *roll)
{
    int16_t ax, ay, az, gx, gy, gz;
    static float pitch_f = 0;
    static float roll_f = 0;
    float accel_pitch;
    float accel_roll;
    float dt = ATTITUDE_SAMPLE_DT_S;
    float body_x_g;
    float body_y_g;
    float body_z_g;
    float roll_raw;
    float pitch_raw;
    float yaw_raw;
    float gyro_roll;
    float gyro_pitch;

    MPU6000_GetData(&ax, &ay, &az, &gx, &gy, &gz);

    Attitude_MapAccelToBody(ax, ay, az, &body_x_g, &body_y_g, &body_z_g);
    Attitude_MapGyroToBody(gx, gy, gz, &roll_raw, &pitch_raw, &yaw_raw);
    (void)yaw_raw;

    accel_roll = atan2(body_x_g, body_z_g) * 57.3f;
    accel_pitch = atan2(-body_y_g, sqrt((body_x_g * body_x_g) + (body_z_g * body_z_g))) * 57.3f;

    gyro_roll = (roll_raw - gyro_x_offset) / 16.4f;
    gyro_pitch = (pitch_raw - gyro_y_offset) / 16.4f;

    roll_f = 0.98f * (roll_f + gyro_roll * dt) + 0.02f * accel_roll;
    pitch_f = 0.98f * (pitch_f + gyro_pitch * dt) + 0.02f * accel_pitch;

    *roll = roll_f - roll_offset;
    *pitch = pitch_f - pitch_offset;
}

void MPU6000_GetBodyRate(float *pitch_rate, float *roll_rate, float *yaw_rate)
{
    if (pitch_rate != 0)
    {
        *pitch_rate = pitch_rate_dps;
    }
    if (roll_rate != 0)
    {
        *roll_rate = roll_rate_dps;
    }
    if (yaw_rate != 0)
    {
        *yaw_rate = yaw_rate_dps;
    }
}

void Gyro_Calibrate(void)
{
    int16_t gx, gy, gz;
    float sum_x = 0;
    float sum_y = 0;
    float sum_z = 0;
    float roll_raw;
    float pitch_raw;
    float yaw_raw;

    for (uint16_t i = 0; i < 500; i++)
    {
        MPU6000_GetGyro(&gx, &gy, &gz);
        Attitude_MapGyroToBody(gx, gy, gz, &roll_raw, &pitch_raw, &yaw_raw);

        sum_x += roll_raw;
        sum_y += pitch_raw;
        sum_z += yaw_raw;

        Delay_ms(5);
    }

    gyro_x_offset = sum_x / 500.0f;
    gyro_y_offset = sum_y / 500.0f;
    gyro_z_offset = sum_z / 500.0f;
}

void Angle_Calibrate(void)
{
    float p;
    float r;
    float sum_p = 0;
    float sum_r = 0;

    for (uint8_t i = 0; i < 200; i++)
    {
        MPU6000_GetAngle_Raw(&p, &r);

        sum_p += p;
        sum_r += r;

        Delay_ms(5);
    }

    pitch_offset = sum_p / 200.0f;
    roll_offset = sum_r / 200.0f;
}

void MPU6000_DataChange(void)
{
    int16_t ax, ay, az, gx, gy, gz;
    float gx_f;
    float gy_f;
    float gz_f;
    float ax_f;
    float ay_f;
    float az_f;
    float roll_raw;
    float pitch_raw;
    float yaw_raw;

    MPU6000_GetData(&ax, &ay, &az, &gx, &gy, &gz);

    Attitude_MapAccelToBody(ax, ay, az, &ax_f, &ay_f, &az_f);
    Attitude_MapGyroToBody(gx, gy, gz, &roll_raw, &pitch_raw, &yaw_raw);

    roll_rate_dps = (roll_raw - gyro_x_offset) / 16.4f;
    pitch_rate_dps = (pitch_raw - gyro_y_offset) / 16.4f;
    yaw_rate_dps = (yaw_raw - gyro_z_offset) / 16.4f;

    gx_f = roll_rate_dps * 0.0174533f;
    gy_f = pitch_rate_dps * 0.0174533f;
    gz_f = yaw_rate_dps * 0.0174533f;

    Mahony_Update(gx_f, gy_f, gz_f, ax_f, ay_f, az_f, ATTITUDE_SAMPLE_DT_S);
}

void MPU6000_GetAngle_Mahony(float *pitch, float *roll)
{
    Mahony_GetAngle(pitch, roll);
    *pitch = Attitude_NormalizeAngle(*pitch - mahony_pitch_offset);
    *roll = Attitude_NormalizeAngle(*roll - mahony_roll_offset);
}

void MPU6000_GetEuler_Mahony(float *pitch, float *roll, float *yaw)
{
    float raw_pitch;
    float raw_roll;
    float raw_yaw;

    Mahony_GetEuler(&raw_pitch, &raw_roll, &raw_yaw);

    if (pitch != 0)
    {
        *pitch = Attitude_NormalizeAngle(raw_pitch - mahony_pitch_offset);
    }

    if (roll != 0)
    {
        *roll = Attitude_NormalizeAngle(raw_roll - mahony_roll_offset);
    }

    if (yaw != 0)
    {
        *yaw = Attitude_NormalizeAngle(raw_yaw - mahony_yaw_offset);
    }
}

void Mahony_AngleCalibrate(void)
{
    float p;
    float r;
    float y;
    float sp = 0;
    float sr = 0;
    float sy = 0;
    int pass;

    mahony_pitch_offset = 0.0f;
    mahony_roll_offset = 0.0f;
    mahony_yaw_offset = 0.0f;
    Mahony_Reset();

    for (int i = 0; i < ATTITUDE_MAHONY_WARMUP_SAMPLES; i++)
    {
        MPU6000_DataChange();
        Delay_ms(5);
    }

    for (pass = 0; pass < ATTITUDE_MAHONY_CALIB_PASSES; pass++)
    {
        sp = 0.0f;
        sr = 0.0f;
        sy = 0.0f;

        for (int i = 0; i < ATTITUDE_MAHONY_CALIB_SAMPLES; i++)
        {
            MPU6000_DataChange();
            MPU6000_GetEuler_Mahony(&p, &r, &y);
            sp += p;
            sr += r;
            sy += y;
            Delay_ms(5);
        }

        mahony_pitch_offset += sp / (float)ATTITUDE_MAHONY_CALIB_SAMPLES;
        mahony_roll_offset += sr / (float)ATTITUDE_MAHONY_CALIB_SAMPLES;
        mahony_yaw_offset += sy / (float)ATTITUDE_MAHONY_CALIB_SAMPLES;
    }
}
