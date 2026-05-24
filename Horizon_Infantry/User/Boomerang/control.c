#include "control.h"
#include "usart.h"
#include "serial_servo.h"
#include "vition.h"
//#include "root.h"
#include "cmsis_os.h"
#include "pid_temp.h"
#include "All_Init.h"
#include "mymath.h"
#include "arm.h"

uint8_t pin_switch_down=0, pin_switch_up=0,pin_switch_power=0;
	uint8_t ids[] = {1, 2};          // 舵机ID列表
	uint16_t angles[] = {140, 700}; // 初始(吸附)
	uint16_t angles2[] = {400, 550};//取下
	uint16_t angles3[] = {900, 460};	// 等待
	uint16_t angles4[] = {730, 600};	// 等待位置
	uint16_t angles5[] = {500, 650};
	uint16_t angles6[] = {500,630};
	uint16_t angles7[] = {450,713};
	uint16_t time_ms = 1000; 
	// uint8_t  b = 1;
	 int32_t  a = 3450;
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
	uint8_t  fifth = 0;
	uint8_t state_launch = 0;
	uint8_t state_power = 0;
	uint8_t state_power_mid_turn = 0;
	uint8_t Servo_996R_angle_close = 60;
	uint8_t Servo_996R_angle_open = 180;
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
			Control_Referee( &User_data);
			break;
		case 3://初始（遥控器拨盘位于中间）测试模式
			Control_test(turn_test2);
			break;
		case 2://手动模式射击		
			ALL_MOTOR.DJI_2006_Trigger.DATA.Aim -= WHW_V_DBUS.Remote.CH3_int16 * 0.4f;
			ALL_MOTOR.DJI_2006_Yaw.DATA.Aim -= WHW_V_DBUS.Remote.CH2_int16 * 0.5;
			ALL_MOTOR.DJI_3508_Pull.DATA.Aim += WHW_V_DBUS.Remote.CH1_int16 * 0.5f;
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
		case 2:
			state_launch = 1;
			ControlServo(state_launch, User_data);
			state_launch = 2;
			break;
		case 3:
			ControlServo(state_launch, User_data);
			// open_cnt = 0;
			break;
		case 0:
			// switch(state_launch)
			// {
			// 	case 0:
			// 		state_launch = 1;
			// 		break;
			// 	case 1:
			// 		//windmill(state, User_data);
			// 		ControlServo(state_launch, User_data);
			// 		osDelay(30000);//防止前两发镖打出后及时进行三四发镖的发射
			// 		switch(second){
			// 			case 1:
			// 				state_launch = 1;
			// 				break;
			// 			case 0:
			// 				state_launch = 2;
			// 				break;
			// 			default:
			// 				break;
			// 		}
			// 		break;
			// 	case 2:
			// 		//windmill(state, User_data);
			// 		ControlServo(state_launch, User_data);
			// 		break;
			// 	default:
			// 		break;
			// }
				break;
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
void ControlServo(uint8_t mod, User_Data_T* User_data)
	{
		switch(mod)
		{
			case 1://第一发和第二发
				if(first==1){
					//电磁铁上磁
					HAL_GPIO_WritePin(GPIOC ,GPIO_PIN_6 ,GPIO_PIN_RESET);
					
					//初始化舵机位置
					//ServoMoveMulti(3, ids, angles, time_ms);
					//初始化yaw位置
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
							//第二发装填完成
							//Dart_put();//放下第二发镖
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
				break;
			case 2://第三发和第四发
				if(third == 1){
					switch(pin_switch_down)
					{
						case 1:
							ALL_MOTOR.DJI_3508_Pull.DATA.Aim =-2500.0;
							break;
						case 0:
							osDelay(25);
							//第三发装填完成
							//Dart_put();//放下第三发镖
							switch(pin_switch_up)
							{
								case 1:
									ALL_MOTOR.DJI_3508_Pull.DATA.Aim =7000.0;
									break;
								case 0:
									ALL_MOTOR.DJI_3508_Pull.DATA.Aim =0;
										Dart_put();//放下第三发镖
										osDelay(300);
										Dart_Trigger_Fire();//发射
										Arm_Action_Sequence(4067.0f);//装填第四发
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
				if(forth ==1){
					switch(pin_switch_down){
						case 1:
							ALL_MOTOR.DJI_3508_Pull.DATA.Aim =-2500.0;
							break;
						case 0:
							osDelay(25);
							//第四发装填完成
							//Dart_put();//放下第四发镖
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

void turn_target(uint8_t mod,User_Data_T User_data,float Aim_base,float Aim_outpost)
{
	// switch(mod)
	// {
	// 	case 1://未标定时的力量确定
	// 		ALL_MOTOR.DJI_2006_Trigger.DATA.Aim -=2.0;
	// 		if(pin_switch_power == 0){
	// 			DJI_2006_trigger_angle_init=ALL_MOTOR.DJI_2006_Trigger.DATA.Angle_now;
	// 			switch(User_data.dart_info.dart_selected_target)
	// 			{
	// 				case 0:
	// 					ALL_MOTOR.DJI_2006_Yaw.DATA.Aim = Aim_outpost+DJI_2006_trigger_angle_init;
	// 					break;
	// 				case 1:
	// 				case 2:
	// 				case 3:
	// 				case 4:
	// 					ALL_MOTOR.DJI_2006_Yaw.DATA.Aim = Aim_base+DJI_2006_trigger_angle_init;
	// 					break;
	// 				default:
	// 					break;
	// 			}
	// 			state_power_mid_turn = 1;
	// 		}
	// 	case 2://已经标定过的目标切换
	// 		switch(User_data.dart_info.dart_selected_target)
	// 			{
	// 				case 0:
	// 					ALL_MOTOR.DJI_2006_Yaw.DATA.Aim = Aim_outpost+DJI_2006_trigger_angle_init;
	// 					break;
	// 				case 1:
	// 				case 2:
	// 				case 3:
	// 				case 4:
	// 					ALL_MOTOR.DJI_2006_Yaw.DATA.Aim = Aim_base+DJI_2006_trigger_angle_init;
	// 					break;
	// 				default:
	// 					break;
	// 			}
	// 		break;
	// }
}

void Control_test(uint8_t mod)
{
	switch(mod)
	{
		case 1://测试，确定机械臂转动到镖的等待位置的编码器值
			if(first==1){
				//电磁铁上磁
				HAL_GPIO_WritePin(GPIOC ,GPIO_PIN_6 ,GPIO_PIN_RESET);
				
				//初始化舵机位置
				//ServoMoveMulti(3, ids, angles, time_ms);
				//初始化yaw位置
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
								Arm_Action_Sequence(2829.0f);//装填第三发
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
							//第二发装填完成
							Dart_put();//放下第二发镖
							switch(pin_switch_up)
							{
								case 1:
									ALL_MOTOR.DJI_3508_Pull.DATA.Aim =7000.0;
									break;
								case 0:
									ALL_MOTOR.DJI_3508_Pull.DATA.Aim =0;
										osDelay(300);
										Dart_Trigger_Fire();//发射
										Arm_Action_Sequence(4067.0f);//装填第四发
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
							//第三发装填完成
							Dart_put();//放下第三发镖
							switch(pin_switch_up)
							{
								case 1:
									ALL_MOTOR.DJI_3508_Pull.DATA.Aim =7000.0;
									break;
								case 0:
										ALL_MOTOR.DJI_3508_Pull.DATA.Aim =0;
										osDelay(300);
										Dart_Trigger_Fire();//发射
										Arm_Action_Sequence(4621.0f);//装填第四发
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
				if(forth ==1){
					switch(pin_switch_down){
						case 1:
							ALL_MOTOR.DJI_3508_Pull.DATA.Aim =-2500.0;
							break;
						case 0:
							osDelay(25);
							//第四发装填完成
							Dart_put();//放下第四发镖
							switch(pin_switch_up){
								case 1:
									ALL_MOTOR.DJI_3508_Pull.DATA.Aim =7000.0;
									break;
								case 0:
									ALL_MOTOR.DJI_3508_Pull.DATA.Aim =0;
										osDelay(300);
									Dart_Trigger_Fire();//发射
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
		case 2://测试，确定发射力度标到基地与前哨的编码器值
			switch(pin_switch_power)
			{
				case  1:
					ALL_MOTOR.DJI_2006_Trigger.DATA.Aim += a;
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
			Arm_Action_Sequence(2829.0f);
			Dart_put();
			break;
		case 7://测试第三发换弹流程与时间
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
			ALL_MOTOR.DJI_6020_turn.DATA.Aim = 3500;
			osDelay(500);
			ServoMoveMulti(2, ids, angles4, time_ms);
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


