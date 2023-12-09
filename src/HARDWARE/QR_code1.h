#ifndef __QR_H
#define __QR_H
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include "stm32f10x.h"
#include "stdlib.h"
#include "string.h"
#include "Delay.h"


#define USART_REC_LEN 120
#define ATcmd_Set(message) sprintf(AT_cmd,(message))
#define AT_cmd_Buff 128

void QR_Init(uint32_t bound);
void test(void);

#endif
