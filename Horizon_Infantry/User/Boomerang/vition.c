#include "vition.h"
#include <stdio.h>
#include "serial_servo.h"
#include "control.h"
#include <string.h>

SensorData gSensorData;
uint8_t uart6_rx_buffer[UART_PACKET_SIZE];
uint8_t uart2_rx_buffer[UART_PACKET_SIZE];
uint8_t uart7_rx_buffer[UART_PACKET_SIZE];

//	int8_t aaa=1;
///* UART6初始化 */
//void UART6_Init(UART_HandleTypeDef *huart) {
//    // 启动DMA接收（循环模式）
//    HAL_UART_Receive_DMA(huart, uart6_rx_buffer, UART_PACKET_SIZE);
//}

//void UART7_Init(UART_HandleTypeDef *huart) {
//    // 启动DMA接收（循环模式）
//    HAL_UART_Receive_DMA(huart, uart7_rx_buffer, UART_PACKET_SIZE);
//}

/* DMA接收完成回调 */
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
////    if (huart->Instance == USART6) {
////        // 帧头校验
////        if ((uart6_rx_buffer[0] == FRAME_HEADE_0) &&
////            (uart6_rx_buffer[1] == FRAME_HEADE_1) &&
////            (uart6_rx_buffer[2] == FRAME_HEADE_2)) {
////            
////            // 解析数据
////            gSensorData.x1 = (uart6_rx_buffer[3] << 8) | uart6_rx_buffer[4];
////            gSensorData.y1 = (uart6_rx_buffer[5] << 8) | uart6_rx_buffer[6];
////            gSensorData.x2 = (uart6_rx_buffer[7] << 8) | uart6_rx_buffer[8];
////            gSensorData.y2 = (uart6_rx_buffer[9] << 8) | uart6_rx_buffer[10];
////            gSensorData.x0 = (uart6_rx_buffer[11] << 8) | uart6_rx_buffer[12];
////						if(gSensorData.x0==0){
////							gSensorData.data_valid = 0; // 标记数据无效
////						}else{
////							gSensorData.data_valid = 1; // 标记数据有效
////						}
////            
////        }
////    }
//		if (huart->Instance == UART7) {
//        // 帧头校验
//        if ((uart7_rx_buffer[0] == FRAME_HEADE_0) &&
//            (uart7_rx_buffer[1] == FRAME_HEADE_1) &&
//            (uart7_rx_buffer[2] == FRAME_HEADE_2)) {
//            
//            // 解析数据
//            gSensorData.x1 = (uart7_rx_buffer[3] << 8) | uart7_rx_buffer[4];
//            gSensorData.y1 = (uart7_rx_buffer[5] << 8) | uart7_rx_buffer[6];
//            gSensorData.x2 = (uart7_rx_buffer[7] << 8) | uart7_rx_buffer[8];
//            gSensorData.y2 = (uart7_rx_buffer[9] << 8) | uart7_rx_buffer[10];
//            gSensorData.x0 = (uart7_rx_buffer[11] << 8) | uart7_rx_buffer[12];
//						if(gSensorData.x0==0){
//							gSensorData.data_valid = 0; // 标记数据无效
//						}else{
//							gSensorData.data_valid = 1; // 标记数据有效
//						}
//            
//        }
//    }
//		
//}

/* 数据处理函数 */
//void ProcessReceivedData(void) {

//    if (gSensorData.data_valid) {
//        // 在这里执行数据保存或后续处理
//      if(gSensorData.x0>641){
//				DJI_2006_Yaw.DATA.Aim -=5000;
//			} 
//      if(gSensorData.x0<639){
//				DJI_2006_Yaw.DATA.Aim +=5000;
//			} 
//			if((539<=gSensorData.x0&&gSensorData.x0<=641) || gSensorData.x0 == 0){
//				aaa=0;
//				DJI_2006_Yaw.DATA.Aim +=0;
//			}
//        // 清除有效标志
//        gSensorData.data_valid = 0;
//    }
//}