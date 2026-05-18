#include "Vision.h"
#include "All_Init.h"


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
    VisionRx->Data.x0 = Union_temp.Data_u16;
    VisionRx->Data.x0 -=426;
    VisionRx->Data.OffCounter = 0;
}

void Vision_Tx_Data(uint16_t State)
{
    VisionTxDataUnion VisionTxData;
    uint8_t i = 0;

    VisionTxData.Head_frame = 0xCD;
    VisionTxData.VisionState = State;
    VisionTxData.End_frame = 0xDC;


    VisionTxData.data[i++] = VisionTxData.Head_frame;

    VisionTxData.data[i++] = (uint8_t)(VisionTxData.VisionState & 0xFF);
    VisionTxData.data[i++] = (uint8_t)((VisionTxData.VisionState >> 8) & 0xFF);

    VisionTxData.data[i++] = VisionTxData.End_frame;

   CDC_Transmit_FS(VisionTxData.data, sizeof(VisionTxData.data));
}

uint8_t vision_offline()
{
    // if (VisionRxData.Data.isOnline == 1)
    // {
    VisionRxData.Data.OffCounter++;
    if ( VisionRxData.Data.OffCounter > 50)
    {
         //memset(&VisionRxData.Data, 0, sizeof(VisionRxData.Data));
         VisionRxData.Data.OffCounter = 50;
        return 0; // 离线
    }
   
    else
    {
        return 1; // 在线
    }
    return 2; 
    
}
    
