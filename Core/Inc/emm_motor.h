#ifndef __EMM_MOTOR_H
#define __EMM_MOTOR_H

#include <stdint.h>
#include <stdbool.h>

void EMM_MOTOR_Init(void);
void EMM_Vel_control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF);

#endif