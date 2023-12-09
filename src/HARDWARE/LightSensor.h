#ifndef __LIGHTSENSOR_H
#define __LIGHTSENSOR_H
#include "stm32f10x.h"                  // Device header

void LightSensor_Init(void);
uint8_t LightSensor_Lift(void);
uint8_t LightSensor_Spin(void);

#endif
