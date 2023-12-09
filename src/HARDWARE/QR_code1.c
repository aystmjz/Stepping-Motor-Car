// #include "QR_code.h"

// #if 1

// #pragma import(__use_no_semihosting)//标准库需要的支持函数
// struct __FILE
// {
// 	int handle;

// };
// FILE __stdout;
// void _sys_exit(int x)//定义_sys_exit()以避免使用半主机模式
// {
// 	x = x;
// }
// int fputc(int ch, FILE *f)//重定义fputc函数
// {
// 	while((USART2->SR&0X40)==0)
//     {}//循环发送,直到发送完毕
//     USART2->DR = (u8) ch;
// 	return ch;
// }

// #endif

// char RxCounter,RxBuffer[USART_REC_LEN];//接收缓冲
// char AT_cmd[AT_cmd_Buff];

// /// @brief 初始化串口2(EC)
// /// @param bound 波特率
// void QR_Init(uint32_t bound)
// {
//     GPIO_InitTypeDef GPIO_InitStructure;
// 	USART_InitTypeDef USART_InitStructure;
// 	NVIC_InitTypeDef NVIC_InitStructure;

// 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
// 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
//  	USART_DeInit(USART2);//复位串口2
//     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
//     GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
//     GPIO_Init(GPIOA, &GPIO_InitStructure);
//     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
//     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
//     GPIO_Init(GPIOA, &GPIO_InitStructure);


//     NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
// 	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级0
// 	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级3
// 	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
// 	NVIC_Init(&NVIC_InitStructure);

// 	USART_InitStructure.USART_BaudRate = bound;
// 	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
// 	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
// 	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
// 	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
// 	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
//     USART_Init(USART2, &USART_InitStructure);
//     USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
//     USART_Cmd(USART2, ENABLE);
// }




// void Clear_Buffer(void)//清空缓存
// {
// 		u8 i;
// 		for(i=0;i<RxCounter;i++)
// 		RxBuffer[i]=0;//缓存
// 		RxCounter=0;
// }


// void Main_printf(char*SendBuf)
// {
// 	printf(SendBuf);
// }

// void Debug_printf(char*SendBuf)
// {

// }

// void ATcmd_Send(char* ATcmd)
// {
// 	Main_printf(ATcmd);
// 	Delay_ms(100);
// }

// /// @brief 从接收缓存中扫描指定应答
// /// @param Ack 指定应答
// /// @return 存在指定应答返回1
// uint8_t ATcmd_Scan(char* Ack)
// {
// 	char *strx;
// 	strx=strstr((const char*)RxBuffer,(const char*)Ack);
// 	if(strx)return 1;
// 	else return 0;
// }

// /// @brief 从接收缓存中提取指定数据
// /// @param Data 写入位置
// /// @param Flag 数据标志
// /// @param Len  数据长度
// /// @return 成功提取返回1
// uint8_t Data_Scan(char* Data,char* Flag,uint8_t Len)
// {
// 	char* pstr,* pData=Data;
// 	uint8_t FlagLen=strlen(Flag);
// 	pstr=strstr((const char*)RxBuffer,(const char*)Flag);
// 	if(!pstr)return 0;
// 	pstr+=FlagLen;
// 	for(int i=0;i<Len;i++)
// 	{
// 		*pData=*pstr;
// 		pstr++;
// 		pData++;
// 	}
// 	*pData='\0';
// 	return 1;
// }

// /// @brief 从接收缓存中提取指定数据(整数)
// /// @param Flag 数据标志
// /// @param Len  数据长度
// /// @return 提取的整数
// uint16_t Data_ScanInt(char* Flag,uint8_t Len)
// {
// 	char Data_Temp[10];
// 	if(!Data_Scan(Data_Temp,Flag,Len))return 0;
// 	return atoi(Data_Temp);
// }


// /// @brief 发送AT_cmd中指令并等待目标答复
// /// @param Ack 目标答复
// /// @param Error 错误提示
// /// @param Wait_Time 等待答复时间
// /// @param Try_Time 重试次数
// /// @return 成功：1	失败：0
// uint8_t ATcmd_Wait(char* Ack,char* Error,uint16_t Wait_Time,uint8_t Try_Time)
// {
// 	Clear_Buffer();
// 	if(!Try_Time)Try_Time=0xff;
// 	for(uint8_t i=1;i<=Try_Time;i++)
// 	{
// 		ATcmd_Send(AT_cmd);
// 		//Debug_printf("Wait\r\n");
// 		uint16_t timeout = 1;
// 		while(!ATcmd_Scan(Ack))
// 		{
//    	 		Delay_ms(1);
//    			timeout ++;
//     		if(timeout > Wait_Time){if(Error!=NULL)Debug_printf(Error);timeout=0;break;}
// 		}
// 		if(timeout)return 1;
// 	}
// 	return 0;
// }

// /// @brief 生成AT指令到AT_cmd
// /// @param num 指令参数数量
// /// @param cmd 指令名称
// /// @param
// void ATcmd_Make(int num,const char* cmd, ...)
// {
// 	va_list arg_list;
// 	char cmd_temp[AT_cmd_Buff]={0};
// 	char *pstr_temp;
// 	va_start(arg_list, cmd);
// 	strcat(cmd_temp, "AT+");
// 	strcat(cmd_temp, cmd);
// 	if(num)strcat(cmd_temp, "=");
// 	for(int i=0;i<num;i++)
// 	{
// 		pstr_temp=va_arg(arg_list, char*);
// 		if(*pstr_temp<'0'||*pstr_temp>'9')
// 		{
// 			strcat(cmd_temp, "\"");
// 			strcat(cmd_temp, pstr_temp);
// 			strcat(cmd_temp, "\"");
// 		}
// 		else
// 		{
// 			strcat(cmd_temp, pstr_temp);
// 		}
// 		if(i<num-1)strcat(cmd_temp, ",");
// 	}
// 	strcat(cmd_temp, "\r\n");
// 	va_end(arg_list);
// 	sprintf(AT_cmd,cmd_temp);
// }
// /// @brief 生成AT指令并发送
// /// @param num 指令参数数量
// /// @param cmd 指令名称
// /// @param
// void ATcmd_MakeSend(int num,const char* cmd, ...)
// {
// 	va_list arg_list;
// 	char cmd_temp[AT_cmd_Buff]={0};
// 	char *pstr_temp;
// 	va_start(arg_list, cmd);
// 	strcat(cmd_temp, "AT+");
// 	strcat(cmd_temp, cmd);
// 	if(num)strcat(cmd_temp, "=");
// 	for(int i=0;i<num;i++)
// 	{
// 		pstr_temp=va_arg(arg_list, char*);
// 		if(*pstr_temp<'0'||*pstr_temp>'9')
// 		{
// 			strcat(cmd_temp, "\"");
// 			strcat(cmd_temp, pstr_temp);
// 			strcat(cmd_temp, "\"");
// 		}
// 		else
// 		{
// 			strcat(cmd_temp, pstr_temp);
// 		}
// 		if(i<num-1)strcat(cmd_temp, ",");
// 	}
// 	strcat(cmd_temp, "\r\n");
// 	va_end(arg_list);
// 	ATcmd_Send(cmd_temp);
// }



// char CMD[30]="\x7e\x00\x08\x01\x00\x02\x01\xab\xcd";
// char FLAG[5]={0x44,0x41,0x54,0x41};
// char DATA[8];
// char Block_Data[7];

// void test(void)
// {
//     printf("\x7e\x00\x08\x01\x00\x02\x01\xab\xcd");
//     printf(0);
//     ATcmd_Set(CMD);
// 	ATcmd_Wait(FLAG,NULL,10000,0);
//     Data_Scan(DATA,FLAG,7);
//     for(int i=0;i<7;i++)
//     {
//         Block_Data[i]=DATA[i]-'0';
//     }

// }
