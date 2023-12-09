#ifndef __HWT101CT_H__
#define __HWT101CT_H__
#include "stm32f10x.h"
#include "OLED.h"
#include <stdio.h>

#define Angle_Speed_Length 9
#define Angle_Length       9
#define ANGLE_SPEED_STATE  0
#define ANGLE_STATE        1

#define HWT_BIRATE_9600    0x02
#define HWT_BIRATE_115200  0x06

void HWTCalc(void);
float HWT_getAngleSpeed(void);
float HWT_getAngle(void);
void HWT_setZero(void);
void HWT_setBaud(uint8_t bitrate);

void HWT101CT_Init(void);
void HWT_Serial_Init(void);
void HWT_Serial_SendByte(uint8_t Byte);
void HWT_Serial_SendArray(uint8_t *Array, uint16_t Length);
void HWT_Serial_SendString(char *String);
void HWT_Serial_SendNumber(uint32_t Number, uint8_t Length);

uint8_t HWT_Serial_GetRxFlag(void);
uint8_t HWT_Serial_GetRxData(void);

#endif
