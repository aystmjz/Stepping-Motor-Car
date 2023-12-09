#ifndef __SERVOCONTROL_H__
#define __SERVOCONTROL_H__
#include "stm32f10x.h"
#include "LobotServoController.h"
#include "Delay.h"
#include "Usart.h"


#define SERVO_Stretch 0
#define SERVO_Swing   1
#define Time_ms       100
#define Stretch_MIN   1200
#define Stretch_MAX   2100
#define Swing_L       2070
#define Swing_R       730
#define SHRINK        moveServo(SERVO_Stretch, Stretch_MIN, Time_ms)    //机械爪收缩

void Servo_Init(void);
void Init_Status(void);
void Stretch(int16_t Location);
void Swing(int16_t Angle);

#endif
