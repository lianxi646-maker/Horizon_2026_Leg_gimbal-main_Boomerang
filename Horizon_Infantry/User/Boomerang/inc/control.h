#ifndef CONTROL__H
#define CONTROL__H

#include "RUI_DBUS.h"
#include "pid_temp.h"
#include "DJI_Motor.h"
// #include "tim.h"
#include "gpio.h"
#include "Referee.h"
#include "Boomerang_task.h"

#define DJI_6020_turn_test 1
#define DJI_2006_trigger_test 2
#define Vision_Test 3
#define DJI_3508_Pull_test 4
#define trigger_test 5
#define turn_test2 6
#define turn_test3 7
#define turn_test4 8

extern uint8_t k;
extern uint8_t pin_switch_down;
extern uint8_t pin_switch_up;
extern uint8_t pin_switch_power;
uint32_t  Angle_To_CCR(float angle); 
//void ServoMove(uint8_t id, uint16_t angle, uint16_t time_ms);
//void BuildServoCommand(uint8_t id, uint8_t cmd, const uint8_t *params, uint8_t params_len, uint8_t *buffer);
void Control(uint8_t mod);
void windmill(uint8_t mod, User_Data_T usr_data);
//void ServoMotorModeWrite(uint8_t id, uint8_t mode, uint8_t turn_mode, int16_t speed) ;
//void ServoPosRead(uint8_t id);
void Servo_SetAngle(TIM_HandleTypeDef *htim, uint32_t Channel, float angle);
void Control_Referee( User_Data_T *User_data);
void ControlServo(uint8_t mod, User_Data_T *User_data);
void turn_target(uint8_t mod,User_Data_T User_data,float Aim_base,float Aim_outpost);
void Control_test(uint8_t mod);

#endif
