# Horizon 2026赛季 飞镖代码

## 控制方式

采用遥控器测试与读取裁判系统进行自动化发射的控制方式；

遥控器只控制同步带上下卷（3508电机控制，改装过减速箱）与自动、遥控、测试模式的切换（关闭遥控器时默认自动），扳机由测试者手动启动；

自动模式下根据裁判系统数据判断镖架行为（有时会失效，未找到原因）

遥控器代码（位置：`User/Boomerang/control.c`）：

```c
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
			ALL_MOTOR.DJI_3508_Pull.DATA.Aim = WHW_V_DBUS.Remote.CH1_int16 * 4.0f;
			break;
		default://关闭遥控器时保持原样不动
			ALL_MOTOR.DJI_2006_Trigger.DATA.Aim = ALL_MOTOR.DJI_2006_Trigger.DATA.Angle_Infinite;
			VisionRxData.Data.isOnline = 0;
			ALL_MOTOR.DJI_3508_Pull.DATA.Aim = ALL_MOTOR.DJI_3508_Pull.DATA.Angle_Infinite;
			break;	
	}
}
```

## 自动模式

### 机械臂部分

三自由度机械臂，采用6020作为云台电机，两个舵机控制大小臂

- `User/Boomerang/arm.c`中是机械臂整体代码

- `User/Boomerang/serial_servo.c`中是给舵机控制板发送舵机控制信息的代码

### 闸门状态检测

通过检测裁判系统开关门数据的上升沿以及上升沿的计数来判断闸门状态与开门次数，并依此进行作业

```c
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
```

### 自动流程

第一发飞镖直接装在滑轨上，第二发飞镖吸附在机械臂上，第三发、第四发放在两侧弹夹，机械臂通过抓放实现飞镖装填。第一次闸门开启打出前两发，然后进行三四发的装填，镖架储能且三四发飞镖状态装填至于一二发状态相同，第二次闸门开启后进行四秒视觉标定后直接发出第三发，后发出第四发。

镖架上与扳机里各有一个微动开关以反馈镖架状态

```c
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
```

该流程存在同步带运动时不断退出到检测闸门状态的函数中的bug，推测原因是：在未触碰到扳机中自带的微动开关时执行完电机命令后流程不堵塞直接执行break命令。

## 视觉标定

视觉标定与其他代码并行作业，上电及进行视觉标定，不受遥控器与裁判系统管控；当视觉离线或未检测到基地绿灯时yaw轴不动，只有视觉在线且检测到基地绿灯时yaw轴才进行移动；由于没有激光雷达，视觉反馈的标定数据具有误差，故添加误差补偿`int16_t vision_error = 193;`以确保到达目标数值时的标定准确。

视觉接收：

```c
void Vision_Rx_Data(uint8_t* buffer, VisionRxDataUnion *VisionRx)
{
    VisionTemp_u16 Union_temp;
    VisionRx->Data.OffCounter = 1;
    uint8_t i = 0;
    //获取头帧
    VisionRx->Data.Head_frame = buffer[i++];
    VisionRx->Data.End_frame = buffer[11];

    if (VisionRx->Data.Head_frame != 0xCD )
    {
        return;
    }
    if (VisionRx->Data.End_frame != 0xDC)
    {
        return;
    }
    Union_temp.Data[1] = buffer[i++];
    Union_temp.Data[0] = buffer[i++];
    VisionRx->Data.x1 = Union_temp.Data_u16;
    
    Union_temp.Data[1] = buffer[i++];
    Union_temp.Data[0] = buffer[i++];
    VisionRx->Data.y1 = Union_temp.Data_u16;

    Union_temp.Data[1] = buffer[i++];
    Union_temp.Data[0] = buffer[i++];
    VisionRx->Data.x2 = Union_temp.Data_u16;

    Union_temp.Data[1] = buffer[i++];
    Union_temp.Data[0] = buffer[i++];
    VisionRx->Data.y2 = Union_temp.Data_u16;

    Union_temp.Data[1] = buffer[i++];
    Union_temp.Data[0] = buffer[i++];
    VisionRx->Data.x0 = Union_temp.Data_u16-640 + vision_error;
    VisionRx->Data.OffCounter = 0;
}
uint8_t vision_offline()
{
    VisionRxData.Data.OffCounter++;
    if ( VisionRxData.Data.OffCounter > 80)
    {
         VisionRxData.Data.OffCounter = 80;
        return 0; // 离线
    }
   
    else
    {
			  if (VisionRxData.Data.x1 == 0 && VisionRxData.Data.x2 == 0) return 0;
        return 1; // 在线
    }
		
    return 2; 
    
}
```

## 总结

本赛季飞镖没有足够的测试次数导致准度不足没有命中基地，不过也很接近了；这套代码的闸门状态检测的bug也是导致本赛季失败结果的原因之一；整体代码水平较低，具有很大优化空间。