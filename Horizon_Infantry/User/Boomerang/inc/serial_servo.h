//
// Created by lucas on 2022-08-29.
//
#ifndef __SERIAL_SERVO_H
#define __SERIAL_SERVO_H
//#include "control.h"
#include "RUI_DBUS.h"
#include "main.h"
//#include "pid.h"
//#include "DJI_Motor.h"
//#include "tim.h"

// 控制板协议定义
#define FRAME_HEADER_1      0x55
#define FRAME_HEADER_2      0x55
#define CMD_SERVO_MOVE      0x03
#define CMD_GET_BATTERY     0x0F
#define RX_BUF_SIZE 32
	void ServoMove(uint8_t id, uint16_t angle, uint16_t time_ms);
	void ServoMoveMulti(uint8_t num_servos, uint8_t *ids, uint16_t *angles, uint16_t time_ms);
#endif
