#include "control.h"
#include "usart.h"
#include "serial_servo.h"
#include "vition.h"
//#include "root.h"
#include "cmsis_os.h"
#include "pid_temp.h"
#include "All_Init.h"


uint8_t pin_switch_down=0, pin_switch_up=0;
	uint8_t ids[] = {1, 2};          // 舵机ID列表
	uint16_t angles[] = {450, 700}; // 初始
	uint16_t angles2[] = {500, 630};
	uint16_t angles3[] = {2000, 600};	// 目标
	uint16_t angles4[] = {730, 600};	// 等待位置
	uint16_t angles5[] = {500, 650};
	uint16_t angles6[] = {500,630};
	uint16_t angles7[] = {450,713};
	uint16_t time_ms = 1000; 
	// uint8_t  b = 1;
	// uint8_t  a = 1;
	// uint8_t  c = 1;
	// uint8_t  d = 1;
	// uint8_t  e = 1;
	// uint8_t  f = 1;
	// uint8_t  g = 1;
	// uint8_t  h = 1;
	// uint8_t  j = 1;
	// uint8_t  k = 1;
	// uint8_t  l=1;

	
	uint8_t  first = 1;
	uint8_t  second = 0;
	uint8_t  third = 0;
	uint8_t  forth = 0;
	//uint8_t  fifth = 0;
	uint8_t state = 0;
	//uint8_t  load = 1;
	//uint8_t state_now = 5;


float qqq = 0.5f;
int temp1;
uint8_t shoot=0;
void Control(uint8_t mod)
{
	switch(mod)
	{
		case 1://基于裁判系统信息的全自动模式
			Control_Referee( User_data);
			break;
		case 3://初始（遥控器拨盘位于中间）无模式
			break;
		case 2://手动模式射击		
			ALL_MOTOR.DJI_2006_Trigger.DATA.Aim -= WHW_V_DBUS.Remote.CH3_int16 * 3.0f;
			ALL_MOTOR.DJI_2006_Yaw.DATA.Aim -= WHW_V_DBUS.Remote.CH2_int16 * 3.0;
			ALL_MOTOR.DJI_3508_Pull.DATA.Aim += WHW_V_DBUS.Remote.CH1_int16 * 4.0f;
			break;
		default://清空
			ALL_MOTOR.DJI_2006_Trigger.DATA.Aim = ALL_MOTOR.DJI_2006_Trigger.DATA.Angle_Infinite;
			ALL_MOTOR.DJI_2006_Yaw.DATA.Aim = ALL_MOTOR.DJI_2006_Yaw.DATA.Angle_Infinite;
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

void Control_Referee( User_Data_T User_data)
{
	//检测飞镖闸门状态
	switch(User_data.dart_client_cmd.dart_launch_opening_status)
	{
		case 1:
			break;
		case 2:
			break;
		case 0:
			switch(state)
			{
				case 0:
					osDelay(7000);
					state = 1;
					break;
				case 1:
					//windmill(state, User_data);
					ControlServo(state, User_data);
					if(second == 1){
						state = 1;
					}else if(second == 0){
						state = 2;
					}
					break;
				case 2:
					//windmill(state, User_data);
					ControlServo(state, User_data);
					break;
				default:
					break;
			}
		default:
			break;	
	}
	
}
			
//适用于风车换弹的飞镖全自动流程（舵机角度与原始换弹共用，试用前需查看更改）
void windmill(uint8_t mod, User_Data_T User_data)
{
	
	
//	if(root_t.motorRoot.feedAmmoD == df_offLine)
//	{
//		ALL_MOTOR.DJI_3508_Pull.DATA.Aim = ALL_MOTOR.DJI_3508_Pull.DATA.Angle_Infinite;
//		return ;
//	}
	
		if(mod == 1){//第一发和第二发

			
			if(first==1){
				// //切换目标
				// if(User_data.dart_info.dart_selected_target == 1){
				// 	ALL_MOTOR.DJI_2006_Yaw.DATA.Aim = -20000;
				// 	state_now=1;
				// }
				//视觉标定
				if(VisionRxData.Data.x0>50){
					ALL_MOTOR.DJI_2006_Yaw.DATA.Aim -= 2.0;
				}else if(VisionRxData.Data.x0<-50){
					ALL_MOTOR.DJI_2006_Yaw.DATA.Aim += 2.0;
				}
				//初始化6020位置
				ALL_MOTOR.DJI_6020_turn.DATA.Aim = 0;
				//初始化舵机位置
				ServoMoveMulti(3, ids, angles, time_ms);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Angle_To_CCR(90));
				if(pin_switch_down == 1){
					ALL_MOTOR.DJI_3508_Pull.DATA.Aim -=0.5;
				}else if(pin_switch_down == 0){
					if(pin_switch_up == 1){
						osDelay(300);
						ALL_MOTOR.DJI_3508_Pull.DATA.Aim +=2.0;
					}else if(pin_switch_up == 0){
							osDelay(500);
							__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Angle_To_CCR(150));
							osDelay(50);
							__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Angle_To_CCR(90));//第一发打出
							//转动六十度使第二发镖与发射导轨垂直
							ALL_MOTOR.DJI_6020_turn.DATA.Aim = 1365;
							

							ServoMoveMulti(3, ids, angles, time_ms);
							second = 1;
							first=0;
						// for (int i = 0; i < 5; i++) {
						// 	first = 0;
						// 	a = 0;
						// 	ALL_MOTOR.DJI_3508_Pull.DATA.Aim +=445000;
						// 	}
						
							// osDelay(500);
							// ServoMoveMulti(2, ids, angles2, time_ms);
							// osDelay(1000);
							// ServoMoveMulti(2, ids, angles3, time_ms);
							// osDelay(1000);
							// //再次转动六十度防止换弹装置影响发射装置
							// ALL_MOTOR.DJI_6020.DATA.Aim = 2730;
							// osDelay(500);
							// ServoMoveMulti(2, ids, angles4, time_ms);
						}
				}
			}
				
		
			if(second ==1){
				if(pin_switch_down == 1){
					ALL_MOTOR.DJI_3508_Pull.DATA.Aim -=0.5;
				}else if(pin_switch_down == 0){
					osDelay(25);
					//第二发装填完成
					HAL_GPIO_WritePin(GPIOC ,GPIO_PIN_6 ,GPIO_PIN_RESET);
					osDelay(300);
					if(pin_switch_up == 1){
					ALL_MOTOR.DJI_3508_Pull.DATA.Aim +=2.0;
					}else if(User_data.dart_info.dart_remaining_time <= 4){

					}else if(pin_switch_up == 0){
						osDelay(25);
						//再次转动六十度防止换弹装置影响发射装置
						ALL_MOTOR.DJI_6020_turn.DATA.Aim = 2730;
						//osDelay(500);
						//ServoMoveMulti(3, ids, angles2, time_ms);
						osDelay(500);
						__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Angle_To_CCR(150));
						osDelay(50);
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Angle_To_CCR(90));//第二发打出
					//使第三发镖与发射导轨垂直
						ALL_MOTOR.DJI_6020_turn.DATA.Aim = 4096;
						ServoMoveMulti(3, ids, angles2, time_ms);
						second =0;
						third = 1;
					}
				}
			}
		}else if(mod == 2){//第三发和第四发
			if(third == 1){
				// //目标切换
				// if((User_data.dart_info.dart_selected_target == 2||User_data.dart_info.dart_selected_target == 3||User_data.dart_info.dart_selected_target == 4)&&state_now==1){
				// 	ALL_MOTOR.DJI_2006_Yaw.DATA.Aim = 7000;
				// 	state_now=5;
				// }
				//视觉标定
				if(VisionRxData.Data.x0>50){
					ALL_MOTOR.DJI_2006_Yaw.DATA.Aim -= 2.0;
				}else if(VisionRxData.Data.x0<-50){
					ALL_MOTOR.DJI_2006_Yaw.DATA.Aim += 2.0;
				}
				if(pin_switch_down == 1){
					ALL_MOTOR.DJI_3508_Pull.DATA.Aim -=0.5;
				}else if(pin_switch_down == 0){
					osDelay(25);
					//第三发装填完成
					HAL_GPIO_WritePin(GPIOI ,GPIO_PIN_6 ,GPIO_PIN_RESET);
					osDelay(300);
					if(pin_switch_up == 1){
					ALL_MOTOR.DJI_3508_Pull.DATA.Aim +=2.0;
					}else if(User_data.dart_info.dart_remaining_time <= 4){

					}else if(pin_switch_up == 0){
						osDelay(25);
						//再次转动六十度防止换弹装置影响发射装置
						ALL_MOTOR.DJI_6020_turn.DATA.Aim = 5461;
						//osDelay(500);
						//ServoMoveMulti(3, ids, angles3, time_ms);
						osDelay(500);
						__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Angle_To_CCR(150));
						osDelay(50);
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Angle_To_CCR(90));//第三发打出
					//使第四发镖与发射导轨垂直
					ALL_MOTOR.DJI_6020_turn.DATA.Aim = 6826;
					ServoMoveMulti(3, ids, angles3, time_ms);
						third =0;
						forth = 1;
					}
				}
				
			}
			if(forth ==1){
				if(pin_switch_down == 1){
					ALL_MOTOR.DJI_3508_Pull.DATA.Aim -=0.5;
				}else if(pin_switch_down == 0){
					osDelay(25);
					//第四发装填完成
					HAL_GPIO_WritePin(GPIOI ,GPIO_PIN_7 ,GPIO_PIN_RESET);
					osDelay(300);
					if(pin_switch_up == 1){
					ALL_MOTOR.DJI_3508_Pull.DATA.Aim +=2.0;
					}else if(User_data.dart_info.dart_remaining_time <= 4){

					}else if(pin_switch_up == 0){
						osDelay(25);
						//再次转动六十度防止换弹装置影响发射装置
						ALL_MOTOR.DJI_6020_turn.DATA.Aim = 0;
						//osDelay(500);
						//ServoMoveMulti(3, ids, angles5, time_ms);
						osDelay(500);
						__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Angle_To_CCR(150));
						osDelay(50);
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Angle_To_CCR(90));//第四发打出
					
					}
				}
			}
		}
		//else if(mod == 3){//换弹
}
	
//适用于原始换弹的飞镖全自动流程（舵机角度与原始换弹共用，试用前需查看更改）
void ControlServo(uint8_t mod, User_Data_T User_data)
	{
		if(mod == 1){//第一发和第二发

			
			if(first==1){
				// //切换目标
				// if(User_data.dart_info.dart_selected_target == 1){
				// 	ALL_MOTOR.DJI_2006_Yaw.DATA.Aim = -20000;
				// 	state_now=1;
				// }
				//视觉标定
				if(VisionRxData.Data.x0>50){
					ALL_MOTOR.DJI_2006_Yaw.DATA.Aim -= 2.0;
				}else if(VisionRxData.Data.x0<-50){
					ALL_MOTOR.DJI_2006_Yaw.DATA.Aim += 2.0;
				}
				//初始化舵机位置
				//Servo_SetAngle(&htim12, TIM_CHANNEL_2, 102.0f);
				//ServoMoveMulti(3, ids, angles, time_ms);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Angle_To_CCR(90));
				if(pin_switch_down == 1){
					ALL_MOTOR.DJI_3508_Pull.DATA.Aim -=0.5;
				}else if(pin_switch_down == 0){
					if(pin_switch_up == 1){
						osDelay(300);
						ALL_MOTOR.DJI_3508_Pull.DATA.Aim +=2.0;
					}else if(pin_switch_up == 0){
							osDelay(500);
							__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Angle_To_CCR(150));
							osDelay(50);
							__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Angle_To_CCR(90));//第一发打出
							//将机械臂移动到第二发镖的等待位置
							ALL_MOTOR.DJI_6020_turn.DATA.Aim = 3000;
							osDelay(500);
							//吸取飞镖
							ServoMoveMulti(2, ids, angles, time_ms);
							osDelay(1000);
							ServoMoveMulti(2, ids, angles2, time_ms);
							osDelay(1000);
							ServoMoveMulti(2, ids, angles3, time_ms);
							osDelay(1000);
							//机械臂回到待装填位置
							ALL_MOTOR.DJI_6020_turn.DATA.Aim = 4000;
							osDelay(500);
							ServoMoveMulti(2, ids, angles4, time_ms);
							second = 1;
							first=0;
						// for (int i = 0; i < 5; i++) {
						// 	first = 0;
						// 	a = 0;
						// 	ALL_MOTOR.DJI_3508_Pull.DATA.Aim +=445000;
						// 	}
						
							// osDelay(500);
							// ServoMoveMulti(2, ids, angles2, time_ms);
							// osDelay(1000);
							// ServoMoveMulti(2, ids, angles3, time_ms);
							// osDelay(1000);
							// //再次转动六十度防止换弹装置影响发射装置
							// ALL_MOTOR.DJI_6020.DATA.Aim = 2730;
							// osDelay(500);
							// ServoMoveMulti(2, ids, angles4, time_ms);
						}
				}
			}
				
		
			if(second ==1){
				if(pin_switch_down == 1){
					ALL_MOTOR.DJI_3508_Pull.DATA.Aim -=0.5;
				}else if(pin_switch_down == 0){
					osDelay(25);
					//第二发装填完成
					HAL_GPIO_WritePin(GPIOC ,GPIO_PIN_6 ,GPIO_PIN_RESET);
					if(pin_switch_up == 1){
					ALL_MOTOR.DJI_3508_Pull.DATA.Aim +=2.0;
					}else if(User_data.dart_info.dart_remaining_time <= 2){

					}else if(pin_switch_up == 0){
						osDelay(300);
						__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Angle_To_CCR(150));
						osDelay(50);
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Angle_To_CCR(90));//第二发打出
					//将机械臂移到第三发镖等待位置
					ALL_MOTOR.DJI_6020_turn.DATA.Aim = 2000;
					osDelay(500);
					//吸取第三发镖
					ServoMoveMulti(2, ids, angles, time_ms);
					osDelay(1000);
					ServoMoveMulti(2, ids, angles2, time_ms);
					osDelay(1000);
					ServoMoveMulti(2, ids, angles3, time_ms);
					osDelay(1000);
					//机械臂回到待装填位置
					ALL_MOTOR.DJI_6020_turn.DATA.Aim = 4000;
					osDelay(500);
					ServoMoveMulti(2, ids, angles4, time_ms);
						second =0;
						third = 1;
					}
				}
			}
		}else if(mod == 2){//第三发和第四发
			if(third == 1){
				// //目标切换
				// if((User_data.dart_info.dart_selected_target == 2||User_data.dart_info.dart_selected_target == 3||User_data.dart_info.dart_selected_target == 4)&&state_now==1){
				// 	ALL_MOTOR.DJI_2006_Yaw.DATA.Aim = 7000;
				// 	state_now=5;
				// }
				//视觉标定
				if(VisionRxData.Data.x0>50){
					ALL_MOTOR.DJI_2006_Yaw.DATA.Aim -= 2.0;
				}else if(VisionRxData.Data.x0<-50){
					ALL_MOTOR.DJI_2006_Yaw.DATA.Aim += 2.0;
				}
				if(pin_switch_down == 1){
					ALL_MOTOR.DJI_3508_Pull.DATA.Aim -=0.5;
				}else if(pin_switch_down == 0){
					osDelay(25);
					//第三发装填完成
					HAL_GPIO_WritePin(GPIOC ,GPIO_PIN_6 ,GPIO_PIN_RESET);
					// osDelay(300);
					if(pin_switch_up == 1){
					ALL_MOTOR.DJI_3508_Pull.DATA.Aim +=2.0;
					}else if(User_data.dart_info.dart_remaining_time <= 2){

					}else if(pin_switch_up == 0){
						osDelay(300);
						__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Angle_To_CCR(150));
						osDelay(50);
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Angle_To_CCR(90));//第三发打出
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
					ALL_MOTOR.DJI_6020_turn.DATA.Aim = 4000;
					osDelay(500);
					ServoMoveMulti(2, ids, angles4, time_ms);
						third =0;
						forth = 1;
					}
				}
				
			}
			if(forth ==1){
				if(pin_switch_down == 1){
					ALL_MOTOR.DJI_3508_Pull.DATA.Aim -=0.5;
				}else if(pin_switch_down == 0){
					osDelay(25);
					//第四发装填完成
					HAL_GPIO_WritePin(GPIOC ,GPIO_PIN_6 ,GPIO_PIN_RESET);
					if(pin_switch_up == 1){
					ALL_MOTOR.DJI_3508_Pull.DATA.Aim +=2.0;
					}else if(User_data.dart_info.dart_remaining_time <= 2){

					}else if(pin_switch_up == 0){
						osDelay(300);
						__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Angle_To_CCR(150));
						osDelay(50);
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Angle_To_CCR(90));//第四发打出
					}
				}
			}
		}
		//else if(mod == 3){//换弹
}
	// }
//					Servo_SetAngle(&htim12, TIM_CHANNEL_2, 102.0f);
//					osDelay(500);
//					ServoMoveMulti(2, ids, angles, time_ms);
//					osDelay(1000);
//					ServoMoveMulti(2, ids, angles2, time_ms);
//					osDelay(1000);
//					ServoMoveMulti(2, ids, angles3, time_ms);
//					osDelay(1000);
//					Servo_SetAngle(&htim12, TIM_CHANNEL_2, 130.0f);
//					osDelay(500);
//					ServoMoveMulti(2, ids, angles4, time_ms);
//	Servo_SetAngle(&htim12, TIM_CHANNEL_2, 81.0f);
//					osDelay(500);
//					ServoMoveMulti(2, ids, angles, time_ms);
//					osDelay(1000);
//					ServoMoveMulti(2, ids, angles2, time_ms);
//					osDelay(1000);
//					ServoMoveMulti(2, ids, angles3, time_ms);
//					osDelay(1000);
//					Servo_SetAngle(&htim12, TIM_CHANNEL_2, 126.0f);
//					osDelay(500);
//					ServoMoveMulti(2, ids, angles4, time_ms);
//				Servo_SetAngle(&htim12, TIM_CHANNEL_2, 154.0f);
//				osDelay(500);
//				ServoMoveMulti(2, ids, angles7, time_ms);
//				osDelay(1000);
//				ServoMoveMulti(2, ids, angles6, time_ms);
//				osDelay(1000);
//				ServoMoveMulti(2, ids, angles3, time_ms);
//				osDelay(1000);
//				Servo_SetAngle(&htim12, TIM_CHANNEL_2, 126.0f);
//				osDelay(500);
//				ServoMoveMulti(2, ids, angles4, time_ms);
//				osDelay(500);

void Servo_SetAngle(TIM_HandleTypeDef *htim, uint32_t Channel, float angle) {
    // 约束角度范围
    if (angle < 0) angle = 0;
    else if (angle > 270) angle = 270;
    
    // 计算CCR值（500~2500μs）
    uint32_t ccr = (uint32_t)((angle / 270.0f) * 2000.0f + 500.0f);
    
    // 设置PWM占空比
    __HAL_TIM_SET_COMPARE(htim, Channel, ccr);
}
extern uint8_t mod;

// main
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
//{

//    if (htim->Instance == TIM13) //5ms
//    {
//			ErrorMonitor();
//			pin_switch = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
//			if(pin_switch==1){
//				HAL_GPIO_WritePin(GPIOD ,GPIO_PIN_10 ,GPIO_PIN_RESET);
//			}else{
//				HAL_GPIO_WritePin(GPIOD ,GPIO_PIN_10 ,GPIO_PIN_SET);
//			}
//			Control(remote_t.control.s2_u8);
////			Control(1);
//    } 
//}


