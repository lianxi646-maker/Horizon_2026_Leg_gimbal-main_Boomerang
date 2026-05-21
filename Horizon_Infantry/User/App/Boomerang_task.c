#include "Boomerang_task.h"
uint32_t DJI_2006_trigger_angle_init=0;
void MOTOR_PID_Boomerang_INIT(MOTOR_Typedef *motor)
{
    float PID_P_Pull[3] = {   1.0f,   0.0f,   0.0f   };
    float PID_S_Pull[3] = {   5.0f,   0.0f,   0.0f  };
	PID_init(&ALL_MOTOR.DJI_3508_Pull.PID_P, PID_POSITION,PID_P_Pull, 10000, 0);//拉簧角度环
	PID_init(&ALL_MOTOR.DJI_3508_Pull.PID_S, PID_POSITION,PID_S_Pull, 28000, 0);//拉簧速度环
	
	float PID_P_Trigger[3] = {   0.7f,   0.0f,   0.0f   };
    float PID_S_Trigger[3] = {   1.0f,   0.0f,   0.6f  };
	
	PID_init(&ALL_MOTOR.DJI_2006_Trigger.PID_P, PID_POSITION,PID_P_Trigger, 10000, 0);//扳机角度环
	PID_init(&ALL_MOTOR.DJI_2006_Trigger.PID_S, PID_POSITION,PID_S_Trigger, 18000, 0);//扳机速度环
	
	float PID_P_Yaw[3] = {   0.5f,   0.0f,   0.0f   };
    float PID_S_Yaw[3] = {   2.0f,   0.0f,   0.5f  };
	
	PID_init(&ALL_MOTOR.DJI_2006_Yaw.PID_P, PID_POSITION,PID_P_Yaw, 10000, 0);//YAW轴角度环
	PID_init(&ALL_MOTOR.DJI_2006_Yaw.PID_S, PID_POSITION,PID_S_Yaw, 15000, 0);//YAW轴速度环	

	float PID_P_6020_turn[3] = {   1.0f,   0.0f,   0.0f   };
	float PID_S_6020_turn[3] = {   1.0f,   0.1f,   0.0f  };

	PID_init(&ALL_MOTOR.DJI_6020_turn.PID_P, PID_POSITION,PID_P_6020_turn, 200, 0);//6020角度环
	PID_init(&ALL_MOTOR.DJI_6020_turn.PID_S, PID_POSITION,PID_S_6020_turn, 10000, 0);//6020速度环

	DJI_2006_trigger_angle_init=ALL_MOTOR.DJI_2006_Trigger.DATA.Angle_now;
}

void Boomerang_task()
{
   /*2006扳机PID计算*/
	PID_calc(&ALL_MOTOR.DJI_2006_Trigger.PID_P,ALL_MOTOR.DJI_2006_Trigger.DATA.Angle_Infinite,ALL_MOTOR.DJI_2006_Trigger.DATA.Aim);
	PID_calc(&ALL_MOTOR.DJI_2006_Trigger.PID_S,ALL_MOTOR.DJI_2006_Trigger.DATA.Speed_now,ALL_MOTOR.DJI_2006_Trigger.PID_P.out);
	/*2006YAW轴PID计算*/
	PID_calc(&ALL_MOTOR.DJI_2006_Yaw.PID_P,ALL_MOTOR.DJI_2006_Yaw.DATA.Angle_Infinite,ALL_MOTOR.DJI_2006_Yaw.DATA.Aim);
	PID_calc(&ALL_MOTOR.DJI_2006_Yaw.PID_S,ALL_MOTOR.DJI_2006_Yaw.DATA.Speed_now,ALL_MOTOR.DJI_2006_Yaw.PID_P.out);
	/*3508拉簧PID计算*/
	PID_calc(&ALL_MOTOR.DJI_3508_Pull.PID_P,ALL_MOTOR.DJI_3508_Pull.DATA.Angle_Infinite,ALL_MOTOR.DJI_3508_Pull.DATA.Aim);
	PID_calc(&ALL_MOTOR.DJI_3508_Pull.PID_S,ALL_MOTOR.DJI_3508_Pull.DATA.Speed_now,ALL_MOTOR.DJI_3508_Pull.PID_P.out);
	/*6020_turn换弹PID计算*/
	PID_calc(&ALL_MOTOR.DJI_6020_turn.PID_P,ALL_MOTOR.DJI_6020_turn.DATA.Angle_Infinite,ALL_MOTOR.DJI_6020_turn.DATA.Aim);
	PID_calc(&ALL_MOTOR.DJI_6020_turn.PID_S,ALL_MOTOR.DJI_6020_turn.DATA.Speed_now,ALL_MOTOR.DJI_6020_turn.PID_P.out);

    /*CAN发送*/
	DJI_Current_Ctrl(&hcan1,
                     0x1FF,
                     0,
                     0,//(int16_t)ALL_MOTOR.DJI_2006_Yaw.PID_S.out,
                     0,
					 0);
	DJI_Current_Ctrl(&hcan1,
                     0x200,
                     (int16_t)ALL_MOTOR.DJI_3508_Pull.PID_S.out,
                     0,//(int16_t)ALL_MOTOR.DJI_2006_Trigger.PID_S.out,
                     0,
                     0);
	DJI_Current_Ctrl(&hcan2,
					 0x1FE,
					 (uint16_t)ALL_MOTOR.DJI_6020_turn.PID_S.out,
					 0,
					 0,
					 0);									 			 
}