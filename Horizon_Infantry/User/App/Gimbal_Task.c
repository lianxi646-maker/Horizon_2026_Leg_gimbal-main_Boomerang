#include "Gimbal_Task.h"
#include "controller.h"
#include "All_init.h"
#include "MY_Define.h"
#include "chassisL.h"
#include "VOFA.h"
#include "get_K.h"

uint8_t MOTOR_PID_Gimbal_INIT(MOTOR_Typedef *motor)
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

	float PID_P_6020_Yaw[3] = {   3.0f,   0.0f,   0.0f   };
	float PID_S_6020_Yaw[3] = {   5.0f,   0.1f,   2.0f  };

	PID_init(&ALL_MOTOR.DJI_6020_Yaw.PID_P, PID_POSITION,PID_P_6020_Yaw, 200, 0);//6020角度环
	PID_init(&ALL_MOTOR.DJI_6020_Yaw.PID_S, PID_POSITION,PID_S_6020_Yaw, 10000, 0);//6020速度环

	float PID_P_6020_turn[3] = {   3.0f,   0.0f,   0.0f   };
	float PID_S_6020_turn[3] = {   5.0f,   0.1f,   2.0f  };

	PID_init(&ALL_MOTOR.DJI_6020_turn.PID_P, PID_POSITION,PID_P_6020_turn, 200, 0);//6020角度环
	PID_init(&ALL_MOTOR.DJI_6020_turn.PID_S, PID_POSITION,PID_S_6020_turn, 10000, 0);//6020速度环
//    float PID_S_Pitch[3] = {15.0f, 0.0f, 0.0f};
//    float PID_P_Pitch[3] = {15.0f, 0.0f, 0.0f};

    //PID_Init(&motor->DM4310_Pitch.PID_S, 10000.0f, 0.0f, 
//              PID_S_Pitch, 0.0f, 0.0f, 
//              0.0f, 0.0f, 0, 
//              Integral_Limit);
//    PID_Init(&motor->DM4310_Pitch.PID_P, 1000.0f, 50.0f, 
//              PID_P_Pitch, 0.0f, 0.0f, 
//              0.0f, 0.0f, 0, 
//              Integral_Limit);
//    
//    // float PID_S_Yaw[3] = {35.0f, 0.0f, 0.0f};
//    // float PID_P_Yaw[3] = {25.0f, 0.00001f, 0.0f};
//    float PID_S_Yaw[3] = {15.0f, 0.0f, 0.0f};
//    float PID_P_Yaw[3] = {15.0f, 0.00001f, 0.0f};

//    PID_Init(&motor->DM4310_Yaw.PID_S, 10000.0f, 0.0f, 
//              PID_S_Yaw, 0.0f, 0.0f, 
//              0.0f, 0.0f, 0, 
//              Integral_Limit);
//    PID_Init(&motor->DM4310_Yaw.PID_P, 2000.0f, 50.0f, 
//              PID_P_Yaw, 0.0f, 0.0f, 
//              0.0f, 0.0f, 0, 
//              Integral_Limit);
    return 0;
}

uint8_t Gimbal_AIM_INIT()
{
//    for (int i = 0; i < 10; i++)
//    {
//        motor_mode(&hcan1, 0x01, 0, 0xfc);
//        osDelay(1);
//        motor_mode(&hcan1, 0x02, 0, 0xfc);
//        osDelay(1);
//        RUI_V_CONTAL.HEAD.Pitch_MIN = -20.0f;
//        RUI_V_CONTAL.HEAD.Pitch_MAX =  20.0f;
//    }
    
   
    return 0;
}

uint8_t gimbal_task(CONTAL_Typedef *CONTAL, MOTOR_Typedef *MOTOR, IMU_Data_t *IMU)
{
//    MOTOR->DM4310_Pitch.DATA.Aim = CONTAL->HEAD.Pitch;
//    MOTOR->DM4310_Yaw.DATA.Aim   = CONTAL->HEAD.Yaw;

//    // 平常使用
//    PID_Calculate(&MOTOR->DM4310_Pitch.PID_P, 
//                   IMU_Data.pitch,
//                   MOTOR->DM4310_Pitch.DATA.Aim);
//    PID_Calculate(&MOTOR->DM4310_Pitch.PID_S, 
//                   IMU_Data.gyro[1] * 50.0f,
//                   MOTOR->DM4310_Pitch.PID_P.Output);

//    PID_Calculate(&MOTOR->DM4310_Yaw.PID_P,
//                   IMU_Data.YawTotalAngle,
//                   MOTOR->DM4310_Yaw.DATA.Aim);
//    PID_Calculate(&MOTOR->DM4310_Yaw.PID_S,
//                   QEKF_INS.Gyro[2] * 50.0f,
//                   MOTOR->DM4310_Yaw.PID_P.Output);

//    float board_pitch = boardRxData.dataNeaten.pitch * 0.01f;
//    if (fabsf(board_pitch) > 30.0f)
//    {
//        // 倒地自启使用，防止头疯
//        PID_Calculate(&MOTOR->DM4310_Pitch.PID_P, 
//                    MOTOR->DM4310_Pitch.DATA.pos_infinite * 57.3f,
//                    MOTOR->DM4310_Pitch.DATA.pos_infinite * 57.3f);
//        PID_Calculate(&MOTOR->DM4310_Pitch.PID_S, 
//                    MOTOR->DM4310_Yaw.DATA.vel * 57.3f,
//                    MOTOR->DM4310_Pitch.PID_P.Output);

//        PID_Calculate(&MOTOR->DM4310_Yaw.PID_P,
//                    MOTOR->DM4310_Yaw.DATA.pos_infinite * 57.3f,
//                    MOTOR->DM4310_Yaw.DATA.pos_infinite * 57.3f);
//        PID_Calculate(&MOTOR->DM4310_Yaw.PID_S,
//                    MOTOR->DM4310_Yaw.DATA.vel * 57.3f,
//                    MOTOR->DM4310_Yaw.PID_P.Output);
//    }
//    
//    
//    
//    DM_send_iq(&hcan1, 1, MOTOR->DM4310_Yaw.PID_S.Output);
//    DM_send_iq(&hcan1, 2, MOTOR->DM4310_Pitch.PID_S.Output);

//    if (WHW_V_DBUS.Remote.S1_u8 == 2)
//    {
//        DJI_Current_Ctrl(&hcan1, 0x200, 300, -300, 0, 0);
//    }
//    else
//    {
//        DJI_Current_Ctrl(&hcan1, 0x200, 0, 0, 0, 0);
//    }
//    
//    VOFA_justfloat(MOTOR->DM4310_Yaw.PID_P.Err,
//                    IMU_Data.YawTotalAngle,
//                    MOTOR->DM4310_Yaw.DATA.Aim,
//                    IMU_Data.pitch,
//                    MOTOR->DM4310_Pitch.DATA.Aim,
//                    0,0,0,0,0);

    return 0;
}


