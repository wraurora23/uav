#ifndef __PWM_H
#define __PWM_H

#include <stdint.h>
void MyPWM_Init(void);
void Out1Set(uint16_t Compare);
void Out2Set(uint16_t Compare);
void Out3Set(uint16_t Compare);
void Out4Set(uint16_t Compare);

#endif
