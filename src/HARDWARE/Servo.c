#include "Servo.h"


void Servo_Init(void)
{
    Serial1_Init();
}

/// @brief 舵机回到初始位置
void Init_Status(void)
{
    Stretch(0);
    Swing(0);
}

/// @brief 机械爪控制
/// @param Location 机械爪张开的大小（0~100）
void Stretch(int16_t Location)
{
    if(Location>120)Location=120;
    Location=(Stretch_MAX-Stretch_MIN)*Location/100;
    moveServo(SERVO_Stretch, Stretch_MIN+Location, Time_ms);
}

/// @brief 机械臂控制
/// @param Angle 机械臂角度 右:正
void Swing(int16_t Angle)
{
    Angle=Swing_L+Angle*(Swing_F-Swing_B)/180.0;
    if(Angle>2500)Angle=2500;
    if(Angle<500)Angle=500;
    moveServo(SERVO_Swing, Angle, Time_ms);
}
