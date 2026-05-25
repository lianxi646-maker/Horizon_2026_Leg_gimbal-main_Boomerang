#include "Boomerang_task.h"
#include "cmsis_os.h"
uint32_t DJI_2006_trigger_angle_init=0;
void MOTOR_PID_Boomerang_INIT(MOTOR_Typedef *motor)
{
	osDelay(100);
    float PID_S_Pull[3] = {   15.0f,   0.7f,   0.0f  };
	PID_init(&ALL_MOTOR.DJI_3508_Pull.PID_S, PID_POSITION,PID_S_Pull, 16384, 7000);//拉簧速度环
	
	float PID_P_Trigger[3] = {   0.8f,   0.0001f,   0.0f   };
    float PID_S_Trigger[3] = {   10.0f,   0.0f,   0.0f  };
	PID_init(&ALL_MOTOR.DJI_2006_Trigger.PID_P, PID_POSITION,PID_P_Trigger, 500, 10);//扳机速度环
	PID_init(&ALL_MOTOR.DJI_2006_Trigger.PID_S, PID_POSITION,PID_S_Trigger, 10000, 2000);//扳机速度环
	
	float PID_P_Yaw[3] = {   -50.0f,   -0.01f,   0.0f   };
    float PID_S_Yaw[3] = {   20.0f,   0.0f,   0.5f  };
	
	PID_init(&ALL_MOTOR.DJI_2006_Yaw.PID_P, PID_POSITION,PID_P_Yaw, 700, 10);//YAW轴视觉位置环
	PID_init(&ALL_MOTOR.DJI_2006_Yaw.PID_S, PID_POSITION,PID_S_Yaw, 10000, 0);//YAW轴速度环	

	float PID_P_6020_turn[3] = {   1.0f,   0.01f,   5.0f   };
	float PID_S_6020_turn[3] = {   40.0f,   0.0f,   0.0f  };

	PID_init(&ALL_MOTOR.DJI_6020_turn.PID_P, PID_POSITION,PID_P_6020_turn, 300, 30);//6020角度环
	PID_init(&ALL_MOTOR.DJI_6020_turn.PID_S, PID_POSITION,PID_S_6020_turn, 16000, 10);//6020速度环

	// DJI_2006_trigger_angle_init=ALL_MOTOR.DJI_2006_Trigger.DATA.Angle_Infinite;
	ALL_MOTOR.DJI_2006_Trigger.DATA.Aim=ALL_MOTOR.DJI_2006_Trigger.DATA.Angle_Infinite;
}

void Boomerang_task()
{
   /*2006扳机PID计算*/
	PID_calc(&ALL_MOTOR.DJI_2006_Trigger.PID_P,ALL_MOTOR.DJI_2006_Trigger.DATA.Angle_Infinite,ALL_MOTOR.DJI_2006_Trigger.DATA.Aim);
	PID_calc(&ALL_MOTOR.DJI_2006_Trigger.PID_S,ALL_MOTOR.DJI_2006_Trigger.DATA.Speed_now,ALL_MOTOR.DJI_2006_Trigger.PID_P.out);
	/*2006YAW轴PID计算*/
	PID_calc(&ALL_MOTOR.DJI_2006_Yaw.PID_P,-(VisionRxData.Data.x0),ALL_MOTOR.DJI_2006_Yaw.DATA.Aim);
	PID_calc(&ALL_MOTOR.DJI_2006_Yaw.PID_S,ALL_MOTOR.DJI_2006_Yaw.DATA.Speed_now,ALL_MOTOR.DJI_2006_Yaw.PID_P.out);
	/*3508拉簧PID计算*/
	PID_calc(&ALL_MOTOR.DJI_3508_Pull.PID_S,ALL_MOTOR.DJI_3508_Pull.DATA.Speed_now,ALL_MOTOR.DJI_3508_Pull.DATA.Aim);
	/*6020_turn换弹PID计算*/
	PID_calc(&ALL_MOTOR.DJI_6020_turn.PID_P,ALL_MOTOR.DJI_6020_turn.DATA.Angle_Infinite,ALL_MOTOR.DJI_6020_turn.DATA.Aim);
	PID_calc(&ALL_MOTOR.DJI_6020_turn.PID_S,ALL_MOTOR.DJI_6020_turn.DATA.Speed_now,ALL_MOTOR.DJI_6020_turn.PID_P.out);

	if	(fabsf(ALL_MOTOR.DJI_6020_turn.DATA.Aim - 3425.0f) >= 2000.0f)
		ALL_MOTOR.DJI_6020_turn.PID_S.out = 0;

    /*CAN发送*/
	if (VisionRxData.Data.isOnline == 1){
	DJI_Current_Ctrl(&hcan1,
                     0x1FF,
                     0,
                     (int16_t)ALL_MOTOR.DJI_2006_Yaw.PID_S.out,
                     0,
					 0);
	}
	DJI_Current_Ctrl(&hcan1,
                     0x200,
                     (int16_t)ALL_MOTOR.DJI_3508_Pull.PID_S.out,
                     (int16_t)ALL_MOTOR.DJI_2006_Trigger.PID_S.out,
                     0,
                     0);
	DJI_Current_Ctrl(&hcan2,
					 0x1FE,
					 (int16_t)ALL_MOTOR.DJI_6020_turn.PID_S.out,
					 0,
					 0,
					 0);									 			 
}