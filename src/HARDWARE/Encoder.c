#include "stm32f10x.h"                  // Device header

int16_t Encoder_Count;

void Encoder_Left_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init (GPIOA,&GPIO_InitStructure);


	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period=65536-1;//ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler=1-1;//PSC
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseInitStructure);

	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel=TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICFilter=0xf;
	TIM_ICInit(TIM5, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel=TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICFilter=0xf;
	TIM_ICInit(TIM5, &TIM_ICInitStructure);
	TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI12,TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

	TIM_Cmd(TIM5,ENABLE);
}

void Encoder_Right_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init (GPIOC,&GPIO_InitStructure);


	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period=65536-1;//ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler=1-1;//PSC
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM8,&TIM_TimeBaseInitStructure);

	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel=TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICFilter=0xf;
	TIM_ICInit(TIM8, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel=TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICFilter=0xf;
	TIM_ICInit(TIM8, &TIM_ICInitStructure);
	TIM_EncoderInterfaceConfig(TIM8, TIM_EncoderMode_TI12,TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

	TIM_Cmd(TIM8,ENABLE);
}

void Encoder_Init(void)
{
	Encoder_Left_Init();
	Encoder_Right_Init();
}

int16_t Encoder_Left_Get(void)
{
	return TIM_GetCounter(TIM5);
}

int16_t Encoder_Right_Get(void)
{
	return TIM_GetCounter(TIM8);
}

void Encoder_Right_Clear(void)
{
	TIM_SetCounter(TIM8,0);
}

void Encoder_Left_Clear(void)
{
	TIM_SetCounter(TIM5,0);
}
