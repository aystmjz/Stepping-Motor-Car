#include "LightSensor.h"

void LightSensor_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_11|GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init (GPIOD,&GPIO_InitStructure);

}

/// @brief 获取举起传感器的数据
/// @return 到位返回0
uint8_t LightSensor_Lift(void)
{
	return GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_11);
}

/// @brief 获取旋转传感器的数据
/// @return 到位返回0
uint8_t LightSensor_Spin(void)
{
	return GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_12);
}
