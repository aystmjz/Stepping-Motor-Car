#ifndef __NIXIE_H__
#define __NIXIE_H__
#include "stm32f10x.h"
#include "sys.h"
#include "Delay.h"

void GPIOMax7219_Init(void);
void Write_Max7219_byte(unsigned char DATA);
void Write_Max7219(unsigned char address, unsigned char dat);
void MAX7219_Init(void);
void Nixie_showNum(unsigned char index, unsigned char num, unsigned char dot);
void Nixie_showPlus(unsigned char index);
void Nixie_reset(void);

#define NIXIE_LIGHT(light) Write_Max7219(0x03, light) // 修改亮度
#define NIXIE_SHUTDOWN     Write_Max7219(0x0C, 0x00)  // 掉电
#define NIXIE_UP           Write_Max7219(0x0C, 0x01)  // 上电

#endif
