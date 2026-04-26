#ifndef CONTROL__H
#define CONTROL__H

#include "RUI_DBUS.h"
#include "pid_temp.h"
#include "DJI_Motor.h"
// #include "tim.h"
#include "gpio.h"

extern void ALL_Init();
// 将角度（0~180°）转换为 CCR 值
extern int pin_switch;
extern int  Angle_To_CCR(float angle); 
void ServoMove(uint8_t id, uint16_t angle, uint16_t time_ms);
void BuildServoCommand(uint8_t id, uint8_t cmd, const uint8_t *params, uint8_t params_len, uint8_t *buffer);
void Control(uint8_t mod);
void Controlservo(uint8_t mod);
void ServoMotorModeWrite(uint8_t id, uint8_t mode, uint8_t turn_mode, int16_t speed) ;
void ServoPosRead(uint8_t id);
void Servo_SetAngle(TIM_HandleTypeDef *htim, uint32_t Channel, float angle);
#endif
