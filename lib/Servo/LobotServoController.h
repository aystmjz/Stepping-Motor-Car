/*******************************************************************************
 * 文件名: LobotServoController.h
 * 作者: 深圳乐幻索尔科技
 * 日期：20160806
 * LSC系列舵机控制板二次开发示例
 *******************************************************************************/

#ifndef LOBOTSERVOCONTROLLER_H_
#define LOBOTSERVOCONTROLLER_H_

#include <stdarg.h>
#include <string.h>
#include "stm32f10x.h"
#include "bool.h"
#include "Usart.h"


#define FRAME_HEADER            0x55 // 帧头
#define CMD_SERVO_MOVE          0x03 // 舵机移动指令
#define CMD_ACTION_GROUP_RUN    0x06 // 运行动作组指令
#define CMD_ACTION_GROUP_STOP   0x07 // 停止动作组指令
#define CMD_ACTION_GROUP_SPEED  0x0B // 设置动作组运行速度
#define CMD_GET_BATTERY_VOLTAGE 0x0F // 获取电池电压指令

extern bool isUartRxCompleted;
extern uint8_t LobotRxBuf[16];
extern uint16_t batteryVolt;
extern void receiveHandle(void);

typedef struct _lobot_servo_ { // 舵机ID,舵机目标位置
    uint8_t ID;
    uint16_t Position;
} LobotServo;

void moveServo(uint8_t servoID, uint16_t Position, uint16_t Time);
void moveServosByArray(LobotServo servos[], uint8_t Num, uint16_t Time);
void moveServos(uint8_t Num, uint16_t Time, ...);
void runActionGroup(uint8_t numOfAction, uint16_t Times);
void stopActionGroup(void);
void setActionGroupSpeed(uint8_t numOfAction, uint16_t Speed);
void setAllActionGroupSpeed(uint16_t Speed);
void getBatteryVoltage(void);


#endif
