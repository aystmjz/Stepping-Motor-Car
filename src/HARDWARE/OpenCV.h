#ifndef __USARTOpenCV_H__
#define __USARTOpenCV_H__

#include "stm32f10x.h"
#include "Usart.h"

#define ReceivedArrLength 11          // 接收数组的长度
#define FrameHeaderData 0x21          // 帧头
#define FrameTailData 0x41            // 帧尾


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

int8_t Get_Excursion(void);
uint8_t Get_Condition(void);
uint8_t Get_Color(void);

#define A_ !A
#define B_ !B
#define C_ !C
#define D_ !D
#define E_ !E
#define F_ !F

#define A1_ !A1
#define B1_ !B1
#define C1_ !C1
#define D1_ !D1
#define E1_ !E1
#define F1_ !F1

#define A2_ !A2
#define B2_ !B2
#define C2_ !C2
#define D2_ !D2
#define E2_ !E2
#define F2_ !F2

#endif
