#ifndef __BOOMERANG_TASK_H
#define __BOOMERANG_TASK_H

#include "main.h"
#include "DJI_Motor.h"
#include "DM_Motor.h"
#include "Motors.h"
#include "ALL_Init.h"
#include "control.h"

void MOTOR_PID_Boomerang_INIT(MOTOR_Typedef *motor);
void Boomerang_task();

#endif