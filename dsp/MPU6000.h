#ifndef __MPU6000_H
#define __MPU6000_H

#include "stdint.h"
void MPU6000_Init(void);
void MPU6000_GetAccel(int16_t *ax, int16_t *ay, int16_t *az);
void MPU6000_GetGyro(int16_t *gx, int16_t *gy, int16_t *gz);
void MPU6000_GetData(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz);

#endif
