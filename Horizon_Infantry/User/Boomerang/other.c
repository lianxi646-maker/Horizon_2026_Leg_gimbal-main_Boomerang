#include "other.h"
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

void Dart_Trigger_Fire(void)
{
    // 打开扳机
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Angle_To_CCR(Servo_996R_angle_open));
    osDelay(500);

    // 关闭扳机（完成一次发射）
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Angle_To_CCR(Servo_996R_angle_close));
}


//适用于原始换弹的飞镖全自动流程（舵机角度与原始换弹共用，试用前需查看更改）
void ControlServo_test(uint8_t mod, User_Data_T* User_data)
	{
		switch(mod)
		{
			case 1://第一发和第二发
				if(first==1){
					//电磁铁上电
					HAL_GPIO_WritePin(GPIOC ,GPIO_PIN_6 ,GPIO_PIN_RESET);

					ALL_MOTOR.DJI_6020_turn.DATA.Aim = 3435;
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Angle_To_CCR(Servo_996R_angle_close));
					//osDelay(100);
					switch(pin_switch_down)
					{
						case 1:
							ALL_MOTOR.DJI_3508_Pull.DATA.Aim -=300.0;
							break;
						case 0:
							ALL_MOTOR.DJI_3508_Pull.DATA.Aim -=0;
							switch(pin_switch_up)
							{
								case 1:
									osDelay(300);
									ALL_MOTOR.DJI_3508_Pull.DATA.Aim +=600.0;
									break;
								case 0:
									ALL_MOTOR.DJI_3508_Pull.DATA.Aim +=0;
									osDelay(300);
									Dart_Trigger_Fire();
									Motor_TurnTo_Angle(2829.0f);
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
							ALL_MOTOR.DJI_3508_Pull.DATA.Aim -=300.0;
							break;
						case 0:
							ALL_MOTOR.DJI_3508_Pull.DATA.Aim -=0;
							osDelay(25);
							//第二发装填完成
							HAL_GPIO_WritePin(GPIOC ,GPIO_PIN_6 ,GPIO_PIN_SET);
							osDelay(200);
							HAL_GPIO_WritePin(GPIOC ,GPIO_PIN_6 ,GPIO_PIN_RESET);//电磁铁重新上电
							switch(pin_switch_up)
							{
								case 1:
									ALL_MOTOR.DJI_3508_Pull.DATA.Aim +=600.0;
									break;
								case 0:
									if((*(User_data)).dart_info.dart_remaining_time <= 2){

									}else{
										ALL_MOTOR.DJI_3508_Pull.DATA.Aim +=0;
										osDelay(300);
										Dart_Trigger_Fire();
										Motor_TurnTo_Angle(4067.0f);
										second =0;
										third = 1;
									}
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
							ALL_MOTOR.DJI_3508_Pull.DATA.Aim -=300.0;
							break;
						case 0:
							ALL_MOTOR.DJI_3508_Pull.DATA.Aim -=0;
							osDelay(25);
							//第二发装填完成
							HAL_GPIO_WritePin(GPIOC ,GPIO_PIN_6 ,GPIO_PIN_SET);
							osDelay(200);
							HAL_GPIO_WritePin(GPIOC ,GPIO_PIN_6 ,GPIO_PIN_RESET);//电磁铁重新上电
							switch(pin_switch_up)
							{
								case 1:
									ALL_MOTOR.DJI_3508_Pull.DATA.Aim +=600.0;
									break;
								case 0:
									if((*(User_data)).dart_info.dart_remaining_time <= 2){

									}else{
										ALL_MOTOR.DJI_3508_Pull.DATA.Aim +=0;
										osDelay(300);
										Dart_Trigger_Fire();
										Motor_TurnTo_Angle(4621.0f);
										third = 0;
										forth = 1;
									}
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
							ALL_MOTOR.DJI_3508_Pull.DATA.Aim -=300.0;
							break;
						case 0:
							ALL_MOTOR.DJI_3508_Pull.DATA.Aim -=0;
							osDelay(25);
							//第四发装填完成
							HAL_GPIO_WritePin(GPIOC ,GPIO_PIN_6 ,GPIO_PIN_SET);
							switch(pin_switch_up){
								case 1:
									ALL_MOTOR.DJI_3508_Pull.DATA.Aim +=600.0;
									break;
								case 0:
									if((*(User_data)).dart_info.dart_remaining_time <= 2){

									}else{
										ALL_MOTOR.DJI_3508_Pull.DATA.Aim +=0;
										osDelay(300);
										Dart_Trigger_Fire();
									}
									break;
								default:
									break;
							}
							break;
						default:
							break;
					}
					forth = 0;
				}
				break;
			default:
				break;
		}
}