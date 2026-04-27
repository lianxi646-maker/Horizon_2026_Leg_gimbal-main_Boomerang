#include "control.h"
#include "usart.h"
#include "serial_servo.h"
#include "vition.h"
//#include "root.h"
#include "cmsis_os.h"
#include "pid_temp.h"
#include "All_Init.h"


int pin_switch;
	uint8_t ids[] = {1, 2};          // 舵机ID列表
	uint16_t angles[] = {450, 700}; // 初始
	uint16_t angles2[] = {500, 630};
	uint16_t angles3[] = {2000, 600};	// 目标
	uint16_t angles4[] = {730, 600};	// 等待位置
	uint16_t angles5[] = {500, 650};
	uint16_t angles6[] = {500,630};
	uint16_t angles7[] = {450,713};
	uint16_t time_ms = 1000; 
	uint8_t  b = 1;
	uint8_t  a = 1;
	uint8_t  c = 1;
	uint8_t  d = 1;
	uint8_t  e = 1;
	uint8_t  f = 1;
	uint8_t  g = 1;
	uint8_t  h = 1;
	uint8_t  j = 1;
	uint8_t  k = 1;
	
	uint8_t  first = 1;
	uint8_t  second = 0;
	uint8_t  third = 0;
	uint8_t  forth = 0;
	uint8_t  fifth = 0;
void ALL_Init()
{
		float PID_P_Pull[3] = {   0.1f,   0.0f,   0.0f   };
    float PID_S_Pull[3] = {   5.0f,   0.0f,   2.0f  };
	PID_init(&ALL_MOTOR.DJI_3508_Pull.PID_P, PID_POSITION,PID_P_Pull, 10000, 0);//拉簧角度环
	PID_init(&ALL_MOTOR.DJI_3508_Pull.PID_S, PID_POSITION,PID_S_Pull, 28000, 0);//拉簧速度环
	
		float PID_P_Trigger[3] = {   0.1f,   0.0f,   0.0f   };
    float PID_S_Trigger[3] = {   10.0f,   0.0f,   5.0f  };
	
	PID_init(&ALL_MOTOR.DJI_2006_Trigger.PID_P, PID_POSITION,PID_P_Trigger, 10000, 0);//扳机角度环
	PID_init(&ALL_MOTOR.DJI_2006_Trigger.PID_S, PID_POSITION,PID_S_Trigger, 7000, 0);//扳机速度环
	
		float PID_P_Yaw[3] = {   0.2f,   0.0f,   0.0f   };
    float PID_S_Yaw[3] = {   2.0f,   0.0f,   1.0f  };
	
	PID_init(&ALL_MOTOR.DJI_2006_Yaw.PID_P, PID_POSITION,PID_P_Yaw, 10000, 0);//YAW轴角度环
	PID_init(&ALL_MOTOR.DJI_2006_Yaw.PID_S, PID_POSITION,PID_S_Yaw, 15000, 0);//YAW轴速度环	

	float PID_P_6020[3] = {   3.0f,   0.0f,   0.0f   };
	float PID_S_6020[3] = {   5.0f,   0.1f,   2.0f  };

	PID_init(&ALL_MOTOR.DJI_6020.PID_P, PID_POSITION,PID_P_6020, 200, 0);//6020角度环
	PID_init(&ALL_MOTOR.DJI_6020.PID_S, PID_POSITION,PID_S_6020, 10000, 0);//6020速度环
}

float qqq = 0.5f;
int temp1;
uint8_t shoot=0;
void Control(uint8_t mod)
{
	if(mod == 1)
	{//视觉标定
		// ProcessReceivedData();
	}else if(mod == 2)
		{
	
	}else if(mod == 3)
	{//手动模式射击		
		ALL_MOTOR.DJI_2006_Trigger.DATA.Aim -= WHW_V_DBUS.Remote.CH3_int16 * 3.0f;
		ALL_MOTOR.DJI_2006_Yaw.DATA.Aim -= WHW_V_DBUS.Remote.CH2_int16 * 3.0;
		ALL_MOTOR.DJI_3508_Pull.DATA.Aim += WHW_V_DBUS.Remote.CH1_int16 * 4.0f;
	}else
	{//清空
		ALL_MOTOR.DJI_2006_Trigger.DATA.Aim = ALL_MOTOR.DJI_2006_Trigger.DATA.Angle_Infinite;
		ALL_MOTOR.DJI_2006_Yaw.DATA.Aim = ALL_MOTOR.DJI_2006_Yaw.DATA.Angle_Infinite;
		ALL_MOTOR.DJI_3508_Pull.DATA.Aim = ALL_MOTOR.DJI_3508_Pull.DATA.Angle_Infinite;
	}	
	
	/*2006扳机PID计算*/
	PID_calc(&ALL_MOTOR.DJI_2006_Trigger.PID_P,ALL_MOTOR.DJI_2006_Trigger.DATA.Angle_Infinite,ALL_MOTOR.DJI_2006_Trigger.DATA.Aim);
	PID_calc(&ALL_MOTOR.DJI_2006_Trigger.PID_S,ALL_MOTOR.DJI_2006_Trigger.DATA.Speed_now,ALL_MOTOR.DJI_2006_Trigger.PID_P.out);
	/*2006YAW轴PID计算*/
	PID_calc(&ALL_MOTOR.DJI_2006_Yaw.PID_P,ALL_MOTOR.DJI_2006_Yaw.DATA.Angle_Infinite,ALL_MOTOR.DJI_2006_Yaw.DATA.Aim);
	PID_calc(&ALL_MOTOR.DJI_2006_Yaw.PID_S,ALL_MOTOR.DJI_2006_Yaw.DATA.Speed_now,ALL_MOTOR.DJI_2006_Yaw.PID_P.out);
	/*3508拉簧PID计算*/
	PID_calc(&ALL_MOTOR.DJI_3508_Pull.PID_P,ALL_MOTOR.DJI_3508_Pull.DATA.Angle_Infinite,ALL_MOTOR.DJI_3508_Pull.DATA.Aim);
	PID_calc(&ALL_MOTOR.DJI_3508_Pull.PID_S,ALL_MOTOR.DJI_3508_Pull.DATA.Speed_now,ALL_MOTOR.DJI_3508_Pull.PID_P.out);
	/*6020PID计算*/
	PID_calc(&ALL_MOTOR.DJI_6020.PID_P,ALL_MOTOR.DJI_6020.DATA.Angle_Infinite,ALL_MOTOR.DJI_6020.DATA.Aim);
	PID_calc(&ALL_MOTOR.DJI_6020.PID_S,ALL_MOTOR.DJI_6020.DATA.Speed_now,ALL_MOTOR.DJI_6020.PID_P.out);
	
	/*CAN发送*/
	DJI_Current_Ctrl(&hcan2,
                     0x1FF,
                     (int16_t)ALL_MOTOR.DJI_2006_Trigger.PID_S.out,
                     (int16_t)ALL_MOTOR.DJI_2006_Yaw.PID_S.out,
                     0,
										 0);
	DJI_Current_Ctrl(&hcan2,
                     0x200,
                     (int16_t)ALL_MOTOR.DJI_3508_Pull.PID_S.out,
                     0,
                     0,
                     0);
	DJI_Current_Ctrl(&hcan1,
					 0x1FF,
					 (int16_t)ALL_MOTOR.DJI_6020.PID_S.out,
					 0,
					 0,
					 0);									 			 
}
	// 将角度（0~180°）转换为 CCR 值
	int  Angle_To_CCR(float angle) 
		{
			// // 高电平时间范围：0.5ms  2.5ms
			// float pulse_min = 0.5;  // 0° 对应 0.5ms
			// float pulse_max = 2.5;  // 180° 对应 2.5ms
			// float pulse_width = pulse_min + (angle / 180.0) * (pulse_max - pulse_min);
				
			// // 计算 CCR 值：CCR = (pulse_width / 20ms) * (ARR + 1)
			return 0;//(uint32_t)((pulse_width / 20.0) * 20000); // ARR+1 = 19999+1=20000
			}		
			
void Controlservo(uint8_t mod)//舵机控制函数和自动控制
{
	
//	if(root_t.motorRoot.feedAmmoD == df_offLine)
//	{
//		ALL_MOTOR.DJI_3508_Pull.DATA.Aim = ALL_MOTOR.DJI_3508_Pull.DATA.Angle_Infinite;
//		return ;
//	}

	//初始化6020位置
	ALL_MOTOR.DJI_6020.DATA.Aim = 0;
	
	if(mod == 2){//第一发和第二发

		if(first==1){
					ALL_MOTOR.DJI_3508_Pull.DATA.Aim -=0.5;
		if(pin_switch == 1&&a == 1){
				for (int i = 0; i < 5; i++) {
					first = 0;
					a = 0;
					ALL_MOTOR.DJI_3508_Pull.DATA.Aim +=445000;
					}
					osDelay(2000);
//					__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, Angle_To_CCR(150));
					osDelay(1000);
//					__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, Angle_To_CCR(90));//第一发打出
					//转动六十度使第二发镖与发射导轨垂直
					ALL_MOTOR.DJI_6020.DATA.Aim = 1365;
					k=2;
					osDelay(500);
					ServoMoveMulti(2, ids, angles, time_ms);
					osDelay(1000);
					ServoMoveMulti(2, ids, angles2, time_ms);
					osDelay(1000);
					ServoMoveMulti(2, ids, angles3, time_ms);
					osDelay(1000);
					//再次转动六十度防止换弹装置影响发射装置
					ALL_MOTOR.DJI_6020.DATA.Aim = 2730;
					osDelay(500);
					ServoMoveMulti(2, ids, angles4, time_ms);
					second = 1;
					first=0;
				}
			
		}
		if(second ==1){
				ALL_MOTOR.DJI_3508_Pull.DATA.Aim -=0.5;
			if(pin_switch == 1&&b == 1){
				for (int o = 0; o < 5; o++) {
					second =0;
					b = 0;
					ALL_MOTOR.DJI_3508_Pull.DATA.Aim +=445000;
					}
					osDelay(2000);
//					__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, Angle_To_CCR(150));
					osDelay(1000);
//				__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, Angle_To_CCR(90));//第二发打出
					second =0;
					third = 1;
				}
			
		}
		if(third == 1){
				if(c==1){
					//使第三发镖与发射导轨垂直
					ALL_MOTOR.DJI_6020.DATA.Aim = 4096;
					k=3;
					osDelay(500);
					ServoMoveMulti(2, ids, angles, time_ms);
					osDelay(1000);
					ServoMoveMulti(2, ids, angles2, time_ms);
					osDelay(1000);
					ServoMoveMulti(2, ids, angles3, time_ms);
					osDelay(1000);
					//防止换弹装置影响发射装置
					ALL_MOTOR.DJI_6020.DATA.Aim = 5461;
					osDelay(500);
					ServoMoveMulti(2, ids, angles4, time_ms);
					c=0;
				}
				ALL_MOTOR.DJI_3508_Pull.DATA.Aim -=0.5;
			if(pin_switch == 1&&d == 1){
				for (int p = 0; p < 5; p++) {
					third = 0;
					d = 0;
					ALL_MOTOR.DJI_3508_Pull.DATA.Aim +=445000;//第三发装完
					}
				third = 0;
				forth  = 1;
				}
			
		}
		
	
	}else if(mod == 1){//第三发和第四发
		
		if(forth ==1){
//				__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, Angle_To_CCR(150));
				osDelay(1000);
//				__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, Angle_To_CCR(90));//第三发打出
				osDelay(500);
			forth =0;
			fifth = 1;
		}
		if(fifth ==1){
			if(j ==1){
				//使第四发镖与发射导轨垂直
				ALL_MOTOR.DJI_6020.DATA.Aim = 6826;
				k=4;
				osDelay(500);
				ServoMoveMulti(2, ids, angles, time_ms);
				osDelay(1000);
				ServoMoveMulti(2, ids, angles6, time_ms);
				osDelay(1000);
				ServoMoveMulti(2, ids, angles3, time_ms);
				osDelay(1000);
				//防止换弹装置影响发射装置
				ALL_MOTOR.DJI_6020.DATA.Aim = 8192;
				osDelay(500);
				ServoMoveMulti(2, ids, angles4, time_ms);
				osDelay(500);
				j =0;
			}
			ALL_MOTOR.DJI_3508_Pull.DATA.Aim -=0.5;
			if(pin_switch == 1&&h == 1){
				for (int l = 0; l < 5; l++) {
					fifth = 0;
					h = 0;
					ALL_MOTOR.DJI_3508_Pull.DATA.Aim +=445000;//第四发装填完成
					}
					osDelay(2000);
//					__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, Angle_To_CCR(150));
					osDelay(1000);
//					__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, Angle_To_CCR(90));//第四发打出
					osDelay(500);
				}
		}
	}else if(mod == 3){//换弹
		
	}
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
}

// void Servo_SetAngle(TIM_HandleTypeDef *htim, uint32_t Channel, float angle) {
//     // 约束角度范围
//     if (angle < 0) angle = 0;
//     else if (angle > 270) angle = 270;
    
//     // 计算CCR值（500~2500μs）
//     uint32_t ccr = (uint32_t)((angle / 270.0f) * 2000.0f + 500.0f);
    
//     // 设置PWM占空比
//     __HAL_TIM_SET_COMPARE(htim, Channel, ccr);
// }
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


