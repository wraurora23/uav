#ifndef __MPU6050_H
#define __MPU6050_H

#include <stdint.h>
//使用硬件iic
void MPU6050_Init(void);
//使用硬件iic
void MPU6050_GetData(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz);
void MPU6050_GetAccel(int16_t *ax, int16_t *ay, int16_t *az);
void MPU6050_GetGroy(int16_t *gx, int16_t *gy, int16_t *gz);




//使用软件iic
void MPU_Init(void);
//使用软件iic
void MPU_GetData(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz);

#endif
