#include "LightSensor.h"

void LightSensor_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init (GPIOB,&GPIO_InitStructure);

}

/// @brief 获取举起传感器的数据
/// @return 到位返回0
uint8_t LightSensor_Lift(void)
{
	return GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7);
}

/// @brief 获取旋转传感器的数据
/// @return 到位返回0
uint8_t LightSensor_Spin(void)
{
	return GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6);
}
