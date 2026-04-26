#ifndef	_VITION__H
#define _VITION__H
#include <stdio.h>
#include "serial_servo.h"
#include "control.h"
#ifdef __cplusplus
extern "C" {
#endif
/* 数据包定义 */
#define UART_PACKET_SIZE  13
#define FRAME_HEADE_0    0xAA
#define FRAME_HEADE_1    0xAF
#define FRAME_HEADE_2    0x08

/* 传感器数据结构 */
typedef struct {
    uint16_t x1;
    uint16_t y1;
    uint16_t x2;
    uint16_t y2;
    uint16_t x0;
    uint8_t data_valid;  // 数据有效标志
} SensorData;

/* 全局变量 */
extern SensorData gSensorData;
extern uint8_t uart6_rx_buffer[UART_PACKET_SIZE];
extern uint8_t uart7_rx_buffer[UART_PACKET_SIZE];

/* 函数声明 */
void UART6_Init(UART_HandleTypeDef *huart);
void UART7_Init(UART_HandleTypeDef *huart);
void ProcessReceivedData(void);

#ifdef __cplusplus
}
#endif

#endif 
