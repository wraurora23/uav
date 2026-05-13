#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>
void Motor_Init(void);
void SetMotor1(uint16_t Motor);
void SetMotor2(uint16_t Motor);
void Motor_Calibrate(void);

#endif
