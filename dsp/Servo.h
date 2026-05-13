#ifndef __SERVO_H
#define __SERVO_H

#include <stdint.h>
void Servo_Init(void);
void Servo1_SetAngle(uint16_t Angle); /* Out3 */
void Servo2_SetAngle(uint16_t Angle); /* Out4 */
void Servo1_SetPulse(uint16_t pulse_us);
void Servo2_SetPulse(uint16_t pulse_us);

#endif
