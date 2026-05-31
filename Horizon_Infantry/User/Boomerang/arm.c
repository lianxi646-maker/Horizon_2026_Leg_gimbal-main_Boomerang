#include "arm.h"
#include "main.h"
#include "cmsis_os.h"
#include "All_init.h"
#include "mymath.h"
#include "serial_servo.h"
#include "control.h"
float middle_angle4 = 3404.0f;
float middle_angle3 = 3450.0f;

void Motor_TurnTo_Angle(float target_angle)
{
    while (1)
    {
        // 1. 计算目标值（带缓启动）
        ALL_MOTOR.DJI_6020_turn.DATA.Aim =
            target_ramp((float)ALL_MOTOR.DJI_6020_turn.DATA.Angle_Infinite, target_angle);

        // 2. 延时，给RTOS调度
        osDelay(1);

        // 3. 判断是否到达目标
        if (fabsf(ALL_MOTOR.DJI_6020_turn.DATA.Angle_Infinite - target_angle) < 25.0f)
        {
            break;
        }
    }
}



void Arm_Action_Sequence(float grap_angle)
{
    // 1️⃣ 云台/电机先转到初始位置
    Motor_TurnTo_Angle(grap_angle);

    // 2️⃣ 执行三段舵机动作
    if(grap_angle == 4092.0f){
        Dart_Grab_Action_4();
        Motor_TurnTo_Angle(middle_angle4);
    }
    else if(grap_angle == 2829.0f){
        Dart_Grab_Action_3();
        Motor_TurnTo_Angle(middle_angle3);
    }
    // 3️⃣ 最后转到目标位置
    //Motor_TurnTo_Angle(middle_angle);
}

//飞镖抓取
void Dart_Grab_Action_3()
{
    ServoMoveMulti(2, ids, angles3, time_ms);
    osDelay(1000);

    ServoMoveMulti(2, ids, angles6, time_ms);
    osDelay(1000);

    ServoMoveMulti(2, ids, angles7, time_ms);
    osDelay(1000);

    ServoMoveMulti(2, ids, angles3, time_ms);
    osDelay(1000);


}

void Dart_Grab_Action_4()
{
    ServoMoveMulti(2, ids, angles3, time_ms);
    osDelay(1000);

    ServoMoveMulti(2, ids, angles4, time_ms);
    osDelay(1000);

    ServoMoveMulti(2, ids, angles5, time_ms);
    osDelay(1000);

    ServoMoveMulti(2, ids, angles3, time_ms);
    osDelay(1000);


}


void Dart_put()
{
   ServoMoveMulti(2, ids, angles3, time_ms);
    osDelay(1000);

    ServoMoveMulti(2, ids, angles2, time_ms);
    osDelay(1000);

    ServoMoveMulti(2, ids, angles, time_ms);
    osDelay(1000);

    HAL_GPIO_WritePin(GPIOC ,GPIO_PIN_6 ,GPIO_PIN_SET);//电磁铁下磁
    ServoMoveMulti(2, ids, angles3, time_ms);
    osDelay(1000);
	HAL_GPIO_WritePin(GPIOC ,GPIO_PIN_6 ,GPIO_PIN_RESET);//电磁铁重新上磁
}