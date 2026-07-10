/*
 * @Author: 王举人 11546637+wang-juren@user.noreply.gitee.com
 * @Date: 2024-11-12 20:51:02
 * @LastEditors: 王举人 11546637+wang-juren@user.noreply.gitee.com
 * @LastEditTime: 2024-11-16 10:21:36
 * @FilePath: \Horizon_Infantry\User\App\WHW_IRQN.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*
 * @Author: 王举人 11546637+wang-juren@user.noreply.gitee.com
 * @Date: 2024-11-12 20:51:02
 * @LastEditors: 王举人 11546637+wang-juren@user.noreply.gitee.com
 * @LastEditTime: 2024-11-15 22:47:11
 * @FilePath: \Horizon_Infantry\User\App\WHW_IRQN.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*
 *                        _oo0oo_
 *                       o8888888o
 *                       88" . "88
 *                       (| -_- |)
 *                       0\  =  /0
 *                     ___/`---'\___
 *                   .' \\|     |// '.
 *                  / \\|||  :  |||// \
 *                 / _||||| -:- |||||- \
 *                |   | \\\  - /// |   |
 *                | \_|  ''\---/''  |_/ |
 *                \  .-\__  '-'  ___/-. /
 *              ___'. .'  /--.--\  `. .'___
 *           ."" '<  `.___\_<|>_/___.' >' "".
 *          | | :  `- \`.;`\ _ /`;.`/ - ` : | |
 *          \  \ `_.   \_ __\ /__ _/   .-` /  /
 *      =====`-.____`.___ \_____/___.-`___.-'=====
 *                        `=---='
 * 
 * 
 *      ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * 
 *            佛祖保佑     永不宕机     永无BUG
 * 
 *        佛曰:  
 *                写字楼里写字间，写字间里程序员；  
 *                程序人员写程序，又拿程序换酒钱。  
 *                酒醒只在网上坐，酒醉还来网下眠；  
 *                酒醉酒醒日复日，网上网下年复年。  
 *                但愿老死电脑间，不愿鞠躬老板前；  
 *                奔驰宝马贵者趣，公交自行程序员。  
 *                别人笑我忒疯癫，我笑自己命太贱；  
 *                不见满街漂亮妹，哪个归得程序员？
 */

/*
 * @Descripttion: 
 * @version: 
 * @Author: Eugene
 * @Date: 2024-07-04 15:42:35
 * @LastEditors: Andy
 * @LastEditTime: 2024-07-07 11:27:00
 */

#include "WHW_IRQN.h"
#include "VOFA.h"
#include "chassisL.h"
#include "chassisR.h"
#include "vmc.h"
#include "get_K.h"
#include "Gimbal_Task.h"
#include "control.h"
#include "vision.h"
#include "Boomerang_task.h"
#include "mymath.h"
#include "serial_servo.h"
#include "arm.h"
#include "All_Init.h"

uint8_t move_G, move_S, move_C, move_P;
float t1,t2,dt;
static uint8_t TX[12] = {0xff,0xf1,0xfd,0x90,0x86,0xa7,0xff,0xf1,0xfd,0x90,0x86,0xa7};

uint8_t start_flag = 0;
float time_start = 0.0f;
//发射导轨位置检测及控制模式切换
void StartRobotUITask(void const * argument)
{
    portTickType currentTimeRobotUI;
    currentTimeRobotUI = xTaskGetTickCount();
    
    //扳机舵机PWM初始化
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    //电磁铁初始上磁
    HAL_GPIO_WritePin(GPIOC ,GPIO_PIN_6 ,GPIO_PIN_RESET); 
	RUI_V_CONTAL.DWT_TIME.RobotUI_Dtime = 0;
    for (;;)
     {
        // RUI_V_CONTAL.DWT_TIME.RobotUI_Dtime = DWT_GetDeltaT(&RUI_V_CONTAL.DWT_TIME.RobotUI_DWT_Count);
        if (User_data.game_status.game_progress == 4 )
        {
					start_flag = 1;
//					  static uint8_t time_init = 0;
//						if (time_init == 0)
//						{
//							time_start = DWT_GetTimeline_s();
//							time_init = 1;
//						}
//            RUI_V_CONTAL.DWT_TIME.RobotUI_Dtime = DWT_GetTimeline_s() - time_start;
//            if(RUI_V_CONTAL.DWT_TIME.RobotUI_Dtime < 390.0f){
//                start_flag = 1;
//            }else{
//                start_flag = 0;
//            }
        }
        if (User_data.game_status.game_progress == 5 && open_cnt == 0)
        {
            start_flag = 0;
        }
//        if (User_data.game_status.game_progress == 5 && open_cnt == 1)
//        {
//            start_flag = 0;
//        }
        if(start_flag == 1)
             Control(1); 
        else if (start_flag == 0)
             Control(2);
					// ALL_MOTOR.DJI_6020_turn.DATA.Aim = 0;
        // if (User_data.game_status.game_progress == 4 ){
        //     Control(WHW_V_DBUS.Remote.S1_u8);
        // }
        osDelay(2);
    }
}

//接收并处理视觉数据
void StartMoveTask(void const * argument)
{
    portTickType currentTimeMove;
    currentTimeMove = xTaskGetTickCount();

    for (;;)
    {
        switch(VisionRxData.Data.isOnline)
        {
            case 0:
                //离线
                break;
            case 1:
                //在线
                ALL_MOTOR.DJI_2006_Yaw.DATA.Aim = 0;
                break;
            case 2:
                //离线检测出错
                break;
            default:
                break;
        }
        Vision_Tx_Data(User_data.dart_info.dart_info_bits.dart_selected_target);
        osDelay(2);
    }
}

//飞镖电机任务
void StartDefiantTask(void const * argument)
{
    portTickType currentTimeDefiant;
    currentTimeDefiant = xTaskGetTickCount();
    MOTOR_PID_Boomerang_INIT(&ALL_MOTOR);

    for(;;)
    {
        Boomerang_task();
        osDelay(2);
    }
}

// 陀螺仪解算与自瞄发送任务
void StartIMUTask(void const * argument)
{
    portTickType currentTimeIMU;
    currentTimeIMU = xTaskGetTickCount();

    osDelay(100);
    for(;;)
    {
        RUI_V_CONTAL.DWT_TIME.IMU_Dtime = DWT_GetDeltaT(&RUI_V_CONTAL.DWT_TIME.IMU_DWT_Count);
        pin_switch_down = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_0);
        pin_switch_up = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14);
        pin_switch_power = HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_6);
        osDelay(1);
    }
}

//视觉离线检测任务
void StartRootTask(void const * argument)
{
    portTickType currentTimeRoot;
    currentTimeRoot = xTaskGetTickCount();

    for(;;)
    {
        VisionRxData.Data.isOnline=vision_offline();
        osDelay(1);
    }
}

void BSP_TIM_IRQHandler(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM7) {
		TX[0]++;
		CANSPI_SEND(&hspi2, 0x201, TX);
	}
}

/************************************************************万能分隔符**************************************************************
 * 	@author:			//小瑞
 *	@performance:	    //CAN接收函数
 *	@parameter:		    //
 *	@time:				//22-11-23 20:42
 *	@ReadMe:			//
 *  @LastUpDataTime:    //2023-04-20 02:52    bestrui
 *  @UpData：           //更新成共用体
 *  @LastUpDataTime:    //2023-05-06 20:23    bestrui
 *  @UpData：           //更新判断逻辑
 ************************************************************万能分隔符**************************************************************/
uint8_t test[8];
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
	CAN_RxHeaderTypeDef can_rx = {0};
    uint8_t rx_data[8] = {0};

    HAL_CAN_GetRxMessage(hcan , CAN_RX_FIFO0 , &can_rx , rx_data);
    
	if (hcan == &hcan2)//CAN1总线判断
	{
		switch (can_rx.StdId)//识别ID
		{
			case 0x205://6020_turn
                WHW_F_MOTOR_CAN_RX_6020RM(&ALL_MOTOR.DJI_6020_turn.DATA, rx_data);
                memcpy(test, rx_data, 8);
                break;
            case 0x206://6020_yaw
                WHW_F_MOTOR_CAN_RX_6020RM(&ALL_MOTOR.DJI_6020_Yaw.DATA, rx_data);
                memcpy(test, rx_data, 8);
                break;
			
			
		}
	}
	if (hcan == &hcan1)
	{
		switch (can_rx.StdId)
		{
			case 0x201://拉簧
				// root_t.motorRoot.feedAmmoDTime = 0;
                RUI_F_MOTOR_CAN_RX_3508RM(&ALL_MOTOR.DJI_3508_Pull.DATA, rx_data);
				memcpy(test, rx_data, 8);
                break;
			case 0x202://扳机
                RUI_F_MOTOR_CAN_RX_2006RM(&ALL_MOTOR.DJI_2006_Trigger.DATA, rx_data);
				memcpy(test, rx_data, 8);
                break;
			case 0x206://YAW轴
                RUI_F_MOTOR_CAN_RX_2006RM(&ALL_MOTOR.DJI_2006_Yaw.DATA, rx_data);
				memcpy(test, rx_data, 8);
                break;
           case 0x209://6020_yaw
                WHW_F_MOTOR_CAN_RX_6020RM(&ALL_MOTOR.DJI_6020_Yaw.DATA, rx_data);
                memcpy(test, rx_data, 8);
                break;
		}
	}
}

void WHW_MCP2515_Callback(SPI_HandleTypeDef *hspi, uCAN_MSG *rxMessage)
{
    if (hspi->Instance == SPI2) {
        uint8_t CAN_SPI_2_Data[8];
        memcpy(CAN_SPI_2_Data, &rxMessage->array[6], 8);
        switch (rxMessage->frame.id)
        {
            case 0x201:

                break;
        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == SPI_CAN_2_EXIT_Pin) {
        uint8_t temp = WHW_CANSPI_Receive(&hspi2, &rxMessage1);
        if(temp)
            WHW_MCP2515_Callback(&hspi2, &rxMessage1);
        WHW_MCP2515_IRQHandler(&hspi2);
    }
}

#define  BUFFER_SIZE_6  (255)
#define  BUFFER_SIZE_1  (20)
#define  BUFFER_SIZE_3  (37)

void BSP_UART_IRQHandler(UART_HandleTypeDef *huart)
{
	uint8_t *pData = huart->pRxBuffPtr;
    if(huart->Instance ==USART3)//遥控接收串口
    {
        if (RESET != __HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE))
        {
			uint16_t temp = 0;
            __HAL_UART_CLEAR_IDLEFLAG(&huart3);  //清除空闲中断标志（否则会一直不断进入中断）
			temp = huart3.Instance -> SR; // 清除SR状态寄存器
			temp = huart3.Instance -> DR; // 清除DR数据寄存器，用来清除中断
            // 下面进行空闲中断相关处理
            HAL_UART_DMAStop(&huart3);//暂时停止本次DMA传输，进行数据处理
            
            if(BUFFER_SIZE_3 - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx) == 18)
                RUI_F_DUBS_Resovled(DBUS_RX_DATA, &WHW_V_DBUS);

            HAL_UART_Receive_DMA(&huart3, (uint8_t *)DBUS_RX_DATA,37);  //重启开始DMA传输
        }
        
    }

    if(huart->Instance ==USART6)//裁判系统串口
    {
		Referee_System_Frame_Update(Referee_Rx_Buf, 256);
	    HAL_UARTEx_ReceiveToIdle_DMA(&huart6, Referee_Rx_Buf, REFEREE_RXFRAME_LENGTH);//裁判系统串口
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }

    // if(huart->Instance ==USART1)//调试串口
    // {
	// 	//数据处理
	// 	uint8_t data_length_1;
    //     if (RESET != __HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE))
    //     {
    //         __HAL_UART_CLEAR_IDLEFLAG(&huart1);  //清除空闲中断标志（否则会一直不断进入中断）
    //         // 下面进行空闲中断相关处理
    //         HAL_UART_DMAStop(&huart1);//暂时停止本次DMA传输，进行数据处理
            
    //         data_length_1 = BUFFER_SIZE_1 - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);//计算接收到的数据长度
	// 	    memset((uint8_t *)RX, 0, data_length_1);

    //         HAL_UART_Receive_DMA(&huart1, (uint8_t *)RX, 20);  //重启开始DMA传输
    //     }
    // }
}

