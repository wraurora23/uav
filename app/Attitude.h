#ifndef __ATTITUDE_H
#define __ATTITUDE_H

void MPU6000_GetAngle(float *pitch, float *roll);
void MPU6000_GetAngle_Filter(float *pitch, float *roll);
void MPU6000_GetBodyRate(float *pitch_rate_dps, float *roll_rate_dps, float *yaw_rate_dps);
void Gyro_Calibrate(void);
void Angle_Calibrate(void);
void MPU6000_DataChange(void);
void MPU6000_GetAngle_Mahony(float *pitch, float *roll);
void MPU6000_GetEuler_Mahony(float *pitch, float *roll, float *yaw);
void Mahony_AngleCalibrate(void);

#endif
