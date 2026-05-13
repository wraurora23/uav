#ifndef __MAHONY_H
#define __MAHONY_H

void Mahony_Update(float gx, float gy, float gz,
                   float ax, float ay, float az, float dt);
void Mahony_GetAngle(float *pitch, float *roll);
void Mahony_GetEuler(float *pitch, float *roll, float *yaw);
void Mahony_Reset(void);

#endif
