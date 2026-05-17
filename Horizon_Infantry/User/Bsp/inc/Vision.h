#ifndef _VISION_H_
#define _VISION_H_

#include "main.h"
#include "usbd_cdc_if.h"

typedef union
{
  uint8_t Data[4];
  float Data_f;
  uint32_t Data_u32;
}VisionTemp;

typedef union
{
  uint8_t Data[2];
  uint16_t Data_u16;
}VisionTemp_u16;

typedef struct
{
    uint8_t OriginData[16];
    struct Data
    {
      uint8_t Head_frame;
      uint8_t End_frame;
      int16_t x0;
      int16_t x1;
      int16_t y1;
      int16_t x2;
      int16_t y2;
      uint16_t OffCounter; // 在线检测
      uint8_t  isOnline;
    } Data;
}VisionRxDataUnion;

typedef struct
{
  uint8_t data[4];

  uint8_t Head_frame;
  uint16_t VisionState;
  uint8_t End_frame;
}VisionTxDataUnion;

void Vision_Rx_Data(uint8_t* buffer, VisionRxDataUnion *VisionRx);
void Vision_Tx_Data(uint16_t State);
uint8_t vision_offline();

#endif
