#ifndef __STEPPER_
#define __STEPPER_
#include "sys.h"
#include "stm32f10x.h"

#define STEPPER_ROT_EN_RCC                 RCC_APB2Periph_GPIOA
#define STEPPER_ROT_EN_PORT                GPIOA
#define STEPPER_ROT_EN_PIN                 GPIO_Pin_5

#define STEPPER_ROT_DIR_RCC                 RCC_APB2Periph_GPIOA
#define STEPPER_ROT_DIR_PORT                GPIOA
#define STEPPER_ROT_DIR_PIN                 GPIO_Pin_6

#define STEPPER_ROT_STEP_RCC                 RCC_APB2Periph_GPIOA
#define STEPPER_ROT_STEP_PORT                GPIOA
#define STEPPER_ROT_STEP_PIN                 GPIO_Pin_4



#define ROTATE_EN        PAout(5)
#define ROTATE_DIR       PAout(6)
#define ROTATE_STEP      PAout(4)

void Stepper_Rot_Initial(void);
void stepper_rotate(int dir,int step);

#endif
