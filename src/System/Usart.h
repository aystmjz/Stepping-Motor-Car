#ifndef __SERIAL_H
#define __SERIAL_H
#include <stdio.h>
#include <stdarg.h>
#include "stm32f10x.h" // Device header
#include "bool.h"

void Usart3SerialInit(void);
uint8_t Usart3SerialGetRxFlag(void);
uint8_t Usart3SerialGetRxData(void);

void Serial1_Init(void);
void uartWriteBuf(uint8_t *buf, uint8_t len);

#endif
