#ifndef __QR_H
#define __QR_H
#include "stm32f10x.h" // Device header
#include <stdio.h>
#include "string.h"
#include "delay.h"
#include "OLED.h"
#include <stdarg.h>
#include "SMOTOR.h"
#define PREIX       0x23 // 前缀
#define SUFIX       0x43 // 后缀

#define First_One   Block_Data[1]
#define First_Two   Block_Data[2]
#define First_Three Block_Data[3]
#define Last_One    Block_Data[5]
#define Last_Two    Block_Data[6]
#define Last_Three  Block_Data[7]

typedef enum {
    NOCOLOR = 0,
    RED     = 1,
    GREEN   = 2,
    BLUE    = 3
} Color;

#define QRBUFLEN 50

void QR_Init(uint32_t bound);
void Serial2_SendByte(uint8_t Byte);
void Serial2_SendArray(uint8_t *Array, uint16_t Length);
void Serial2_SendString(char *String);
void Serial2_SendNumber(uint32_t Number, uint8_t Length);
void Serial2_Printf(char *format, ...);

uint8_t Serial2_GetRxFlag(void);
uint8_t Serial2_GetRxData(void);

uint8_t Usart2SerialGetRxData(void);
void start_scan_QRCode(uint16_t Wait_Time);

#endif
