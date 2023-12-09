#ifndef __Buzzer_H
#define __Buzzer_H
#include "stm32f10x.h"                  // Device header
#include "Delay.h"

void Buzzer_Init(void);
void Buzzer_ON(void);
void Buzzer_OFF(void);
void Buzzer_Turn(void);
void Buzzer_Tow(uint16_t Time);
void Buzzer_One(void);

#endif
