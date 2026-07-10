#include "control.h"
#include "usart.h"
#include "serial_servo.h"
#include "cmsis_os.h"
#include "pid_temp.h"
#include "All_Init.h"
#include "mymath.h"
#include "arm.h"

uint8_t pin_switch_down=0, pin_switch_up=0,pin_switch_power=0;

	uint8_t ids[] = {1, 2};          // 舵机ID列表
	uint16_t angles[] = {140, 700}; // 放置参数2
	uint16_t angles2[] = {400, 567};//放置参数1
	uint16_t angles3[] = {905, 460};	// 等待位置
	uint16_t angles4[] = {120, 700};//第四发镖吸附参数	
	uint16_t angles5[] = {400, 550};//第四发镖取下参数
	uint16_t angles6[] = {100,705};//第三发镖吸附参数
	uint16_t angles7[] = {400,550};//第三发镖取下参数
	uint16_t time_ms = 1000; 

	
	uint8_t  first = 1;
	uint8_t  second = 0;
	uint8_t  third = 0;
	uint8_t  forth = 0;
	uint8_t  fifth = 0;
	uint8_t state_launch = 0;
	uint8_t state_power = 0;
	uint8_t state_process = 0;
	uint32_t state_cnt = 0;
	uint8_t state_power_mid_turn = 0;
	uint8_t Servo_996R_angle_close = 60;
	uint8_t Servo_996R_angle_open = 180;


	


float qqq = 0.5f;
int temp1;
uint8_t shoot=0;
void Control(uint8_t mod)
{
	switch(mod)
	{
		case 1://基于裁判系统信息的全自动模式
			Control_Referee( &User_data);
			break;
		case 3://初始（遥控器拨盘位于中间）测试模式
			//Control_test(turn_test2);
			break;
		case 2://手动模式射击		
			//ALL_MOTOR.DJI_2006_Trigger.DATA.Aim -= WHW_V_DBUS.Remote.CH3_int16 * 0.4f;
			//ALL_MOTOR.DJI_2006_Yaw.DATA.Aim -= WHW_V_DBUS.Remote.CH2_int16 * 0.5;
			ALL_MOTOR.DJI_3508_Pull.DATA.Aim = WHW_V_DBUS.Remote.CH1_int16 * 4.0f;
			break;
		default://关闭遥控器时保持原样不动
			ALL_MOTOR.DJI_2006_Trigger.DATA.Aim = ALL_MOTOR.DJI_2006_Trigger.DATA.Angle_Infinite;
			VisionRxData.Data.isOnline = 0;
			ALL_MOTOR.DJI_3508_Pull.DATA.Aim = ALL_MOTOR.DJI_3508_Pull.DATA.Angle_Infinite;
			break;	
	}
}
	// 将角度（0~180°）转换为 CCR 值
uint32_t  Angle_To_CCR(float angle) 
{
	// 高电平时间范围：0.5ms  2.5ms
	float pulse_min = 0.5;  // 0° 对应 0.5ms
	float pulse_max = 2.5;  // 180° 对应 2.5ms
	float pulse_width = pulse_min + (angle / 180.0) * (pulse_max - pulse_min);
				
	// 计算 CCR 值：CCR = (pulse_width / 20ms) * (ARR + 1)
	return (uint32_t)((pulse_width / 20.0) * 20000); // ARR+1 = 19999+1=20000
}		

uint8_t open_cnt = 0;
void Control_Referee( User_Data_T* User_data)
{	
	static uint16_t dart_launch_opening_status_last = 0;
	if (User_data->dart_client_cmd.dart_launch_opening_status == 0 && dart_launch_opening_status_last == 2) {
		// 打开1次
		open_cnt++;
	}
	dart_launch_opening_status_last = User_data->dart_client_cmd.dart_launch_opening_status;
	//检测飞镖闸门状态 
	switch(open_cnt)
	{
		case 1:
			state_launch = 1;
			ControlServo(state_launch, User_data);
			state_launch = 2;
			break;
		case 2:
			ControlServo(state_launch, User_data);
			break;
		case 0:
				break;
		default:
			break;	
	}
	
}
			
	
//适用于原始换弹的飞镖全自动流程（舵机角度与原始换弹共用，试用前需查看更改）
void ControlServo(uint8_t mod, User_Data_T* User_data)
	{
		switch(mod)
		{
			case 1://第一发和第二发
				if(first==1){
					//电磁铁上磁
					HAL_GPIO_WritePin(GPIOC ,GPIO_PIN_6 ,GPIO_PIN_RESET);
					//初始化换弹yaw轴位置
					ALL_MOTOR.DJI_6020_turn.DATA.Aim = 3435;
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Angle_To_CCR(Servo_996R_angle_close));
					switch(pin_switch_down)
					{
						case 1:
							ALL_MOTOR.DJI_3508_Pull.DATA.Aim =-2500.0;
							break;
						case 0:
							switch(pin_switch_up)
							{
								case 1:
									ALL_MOTOR.DJI_3508_Pull.DATA.Aim =7000.0;
									break;
								case 0:
									ALL_MOTOR.DJI_3508_Pull.DATA.Aim =0;
									osDelay(300);
									Dart_Trigger_Fire();//发射
									second = 1;
									first=0;
									break;
								default:
									break;
								}
							break;
						default:
							break;
					}
				}
				if(second ==1){
					switch(pin_switch_down)
					{
						case 1:
							ALL_MOTOR.DJI_3508_Pull.DATA.Aim =-2500.0;
							break;
						case 0:
							osDelay(25);
							switch(pin_switch_up)
							{
								case 1:
									ALL_MOTOR.DJI_3508_Pull.DATA.Aim =7000.0;
									break;
								case 0:
									ALL_MOTOR.DJI_3508_Pull.DATA.Aim =0;
										Dart_put();//放下第二发镖
										osDelay(300);
										Dart_Trigger_Fire();//发射
										Arm_Action_Sequence(2829.0f);//装填第三发
										second =0;
										third = 1;
									break;
								default:
									break;
								}
							break;
						default:
							break;
					}
				}
					if(third == 1){
					switch(pin_switch_down)
					{
						case 1:
							ALL_MOTOR.DJI_3508_Pull.DATA.Aim =-2500.0;
							break;
						case 0:
							osDelay(25);
							switch(pin_switch_up)
							{
								case 1:
									ALL_MOTOR.DJI_3508_Pull.DATA.Aim =7000.0;
									break;
								case 0:
									ALL_MOTOR.DJI_3508_Pull.DATA.Aim =0;
									Dart_put();//放下第三发镖
									osDelay(300);
									Arm_Action_Sequence(4092.0f);//装填第四发
									third = 0;
									forth = 1;
									break;
								default:
									break;
								}
							break;
						default:
							break;
					}
				}
				break;
			case 2://第三发和第四发
				if(forth == 1){
					osDelay(4000);//给视觉标定留出足够时间
					Dart_Trigger_Fire();//发射
					forth = 0;
					fifth = 1;
					}
				if(fifth ==1){
					switch(pin_switch_down){
						case 1:
							ALL_MOTOR.DJI_3508_Pull.DATA.Aim =-2500.0;
							break;
						case 0:
							osDelay(25);
							switch(pin_switch_up){
								case 1:
									ALL_MOTOR.DJI_3508_Pull.DATA.Aim =7000.0;
									break;
								case 0:
									ALL_MOTOR.DJI_3508_Pull.DATA.Aim =0;
									Dart_put();//放下第四发镖
									osDelay(300);
									Dart_Trigger_Fire();//发射
									open_cnt = 0;
									break;
								default:
									break;
							}
							break;
						default:
							break;
					}
				}
				break;
			default:
				break;
		}
	}


void Control_test(uint8_t mod)
{
	switch(mod)
	{
		case 1://测试，确定机械臂转动到镖的等待位置的编码器值
			break;
		case 2://测试，确定发射力度标到基地与前哨的编码器值
			switch(pin_switch_power)
			{
				case  1:
					ALL_MOTOR.DJI_2006_Trigger.DATA.Aim += 0.0;
					break;
				case 0:
					ALL_MOTOR.DJI_2006_Trigger.DATA.Aim +=0.0;
					break;
			}
			break;
		case 3://测试，视觉标定的双环速控参数
			// ALL_MOTOR.DJI_2006_Yaw.DATA.Aim -= a;
			ALL_MOTOR.DJI_2006_Yaw.DATA.Aim = 0;
			break;
		case 4://测试，确定同步带下降与上升速度
			switch(pin_switch_down)
			{
				case  1:
					ALL_MOTOR.DJI_3508_Pull.DATA.Aim = -2500.0;
					break;
				case 0:
					ALL_MOTOR.DJI_3508_Pull.DATA.Aim =0.0;
					switch(pin_switch_up)
					{
						case  1:
							ALL_MOTOR.DJI_3508_Pull.DATA.Aim = 7000.0;
							break;
						case 0:
							ALL_MOTOR.DJI_3508_Pull.DATA.Aim =0.0;
							break;
						default:
							break;
					}
					break;
			}
			break;
		case 5://测试扳机扣下角度
			Dart_Trigger_Fire();
			// __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Angle_To_CCR(Servo_996R_angle_close));
			break;

		case 6://测试第二发换弹流程与时间
			Arm_Action_Sequence(2829.0f);//装填第三发
			Dart_put();
			break;
		case 7://测试第三发换弹流程与时间
			 //Dart_put();
			 Arm_Action_Sequence(4092.0f);//装填第三发
			 Dart_put();
			break;
		case 8://测试第四发换弹流程与时间
			//将机械臂移到第四发镖等待位置
			ALL_MOTOR.DJI_6020_turn.DATA.Aim = 5000;
			osDelay(500);
			//吸取第四发镖
			ServoMoveMulti(2, ids, angles7, time_ms);
			osDelay(1000);
			ServoMoveMulti(2, ids, angles6, time_ms);
			osDelay(1000);
			ServoMoveMulti(2, ids, angles3, time_ms);
			osDelay(1000);
			//机械臂回到待装填位置
			ALL_MOTOR.DJI_6020_turn.DATA.Aim = 3500;
			osDelay(500);
			ServoMoveMulti(2, ids, angles4, time_ms);
			break;
		default:
			break;
	}
}



void Servo_SetAngle(TIM_HandleTypeDef *htim, uint32_t Channel, float angle) {
    // 约束角度范围
    if (angle < 0) angle = 0;
    else if (angle > 270) angle = 270;
    
    // 计算CCR值（500~2500μs）
    uint32_t ccr = (uint32_t)((angle / 270.0f) * 2000.0f + 500.0f);
    
    // 设置PWM占空比
    __HAL_TIM_SET_COMPARE(htim, Channel, ccr);
}

void Dart_Trigger_Fire(void)
{
    // 打开扳机
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Angle_To_CCR(Servo_996R_angle_open));
    osDelay(500);

    // 关闭扳机（完成一次发射）
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Angle_To_CCR(Servo_996R_angle_close));
	osDelay(500);
}


