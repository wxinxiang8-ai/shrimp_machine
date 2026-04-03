#ifndef __EMM_MOTOR_H
#define __EMM_MOTOR_H

#include <stdint.h>
#include <stdbool.h>
#include "Emm_V5.h"

void EMM_MOTOR_Init(void);
void EMM_Vel_control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF);

#endif