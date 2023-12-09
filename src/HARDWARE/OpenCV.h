#ifndef __USARTOpenCV_H__
#define __USARTOpenCV_H__

#include "stm32f10x.h"
#include "Usart.h"
#include "HWT101CT.h"

#define ReceivedArrLength 11          // 接收数组的长度
#define FrameHeaderData 0x21          // 帧头
#define FrameTailData 0x41            // 帧尾

#define HEADER_CAMER_CMD 0x45 // 帧头
#define TAIL_CAMER_CMD   0x46 // 帧尾
#define MAIN_MODE        0X30 // 主模式
#define CIRCLE_MODE      0X31 // 寻圆模式
#define CAMERA_LEN       4


void OpenCV_memcpy(void *dest, void *src, int n);
void OpenCV_Init(void);
void OpenCV_Calc(void);
uint8_t Usart3SerialGetRxFlag(void);
uint8_t Usart3SerialGetRxData(void);
int8_t getSignedUsartBuf(unsigned char index);
uint8_t getUsartBuf(unsigned char index);
uint16_t getUsartBuf_16(unsigned char index);
uint32_t getUsartBuf_32(unsigned char index);
float getUsartBuf_float(unsigned char index);
void uart3WriteBuf(uint8_t *buf, uint8_t len);
void Send_CMD(uint8_t main_mode, uint8_t color);

int32_t Get_Excursion(void);
uint8_t Get_Condition(void);
uint8_t Get_Color(void);

#endif
