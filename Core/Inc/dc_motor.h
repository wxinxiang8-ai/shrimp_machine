#ifndef __DC_MOTOR_H
#define __DC_MOTOR_H

#include "main.h"
#include <stdint.h>

void DC_Motor_Init(void);
void DC_Motor_SetDirection(uint8_t direction);
void DC_Motor_SetSpeed(int16_t speed_rpm);

#endif /* __DC_MOTOR_H */
