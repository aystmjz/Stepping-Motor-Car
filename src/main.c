#include "SMOTOR.h"
#include "OLED.h"
#include "OpenCV.h"
#include "Delay.h"
#include "Servo.h"
#include "QR_code.h"
#include "Encoder.h"
#include "Timer.h"
#include "PID.h"
#include "LightSensor.h"
#include "Buzzer.h"
#include "Nixie.h"

PID PID_L, PID_R;
extern int16_t MOTOR_LeftSpead, MOTOR_RightSpead;
extern double SMOTOR_Long, SMOTOR_Angle, SMOTOR_Height;
extern uint8_t Block_Data[10];

uint8_t Buzzer_Flag      = 0;
uint8_t Buzzer_Debug     = 0;
uint8_t Sensor_Spin_Flag = 0;
uint8_t Sensor_Lift_Flag = 0;
#define Buzzer_Delay  100

#define Data(x)       getUsartBuf((x))

#define E             -(Get_Excursion() / 9)
#define E_            (Get_Excursion() / 9)
#define D             Get_Condition()
#define C             Get_Color()
#define S             getUsartBuf(0)

#define _Distance     7000

#define Spin_Distance 1000

#define Spead         70
#define Distance_     3000
#define Backward      -200

#define DELAY_TIME    100 // 200

#define MOTOR_X(x)    MOTOR_X_(x)

void Display()
{
    OLED_ShowString(1, 1, "X:");
    // OLED_ShowSignedNum(1, 3, getUsartBuf_float(2) * 10, 4);
    OLED_ShowString(2, 1, "Y:");
    OLED_ShowSignedNum(2, 3, getUsartBuf_float(6) * 10, 4);
    OLED_ShowString(3, 1, "Flag:");
    OLED_ShowSignedNum(3, 6, getUsartBuf(10), 2);
    OLED_ShowString(4, 1, "DATE:");
    OLED_ShowNum(4, 6, First_One, 1);
    OLED_ShowNum(4, 8, First_Two, 1);
    OLED_ShowNum(4, 10, First_Three, 1);
    OLED_ShowNum(4, 12, Last_One, 1);
    OLED_ShowNum(4, 14, Last_Two, 1);
    OLED_ShowNum(4, 16, Last_Three, 1);
}
void Display_()
{
    OLED_ShowSignedNum(2, 1, PID_L.e_l, 5);
    OLED_ShowSignedNum(2, 9, PID_R.e_l, 5);
    OLED_ShowSignedNum(3, 1, PID_L.TOTAL_OUT, 5);
    OLED_ShowSignedNum(3, 9, PID_R.TOTAL_OUT, 5);
    OLED_ShowSignedNum(4, 1, MOTOR_LeftSpead, 3);
    OLED_ShowSignedNum(4, 6, PID_L.e_l - PID_R.e_l, 3);
    OLED_ShowSignedNum(4, 11, MOTOR_RightSpead, 3);
    // OLED_ShowSignedNum(1,1,Encoder_Left_Get(),5);
    OLED_ShowSignedNum(1, 9, Encoder_Right_Get(), 5);
    OLED_ShowNum(1, 1, Data(1), 2);
    OLED_ShowNum(1, 3, Data(2), 2);
    OLED_ShowSignedNum(1, 5, E, 1);
    OLED_ShowSignedNum(1, 7, E_, 1);
}

void Go_Spot(int32_t Distance)
{
    while (!D) {
        if (!(MOTOR_CONTROL(Spead + E, Distance_ + Distance, MOTOR_Left) & MOTOR_CONTROL(Spead + E_, Distance_ + Distance, MOTOR_Right))) {
            MOTOR_Clear(MOTOR_Left | MOTOR_Right);
            MOTOR_CONTROL(40, 0, MOTOR_Left | MOTOR_Right);
            return;
        }
    }
    MOTOR_Stop(MOTOR_Left | MOTOR_Right, Spead, Distance);
    Delay_ms(DELAY_TIME);
}

void Go_Corner(int32_t Distance, Direction Vale, int32_t Angle)
{
    while (!S) {
        if (!(MOTOR_CONTROL(Spead, Distance, MOTOR_Left) & MOTOR_CONTROL(Spead, Distance, MOTOR_Right))) {
            MOTOR_Clear(MOTOR_Left | MOTOR_Right);
            MOTOR_CONTROL(40, 0, MOTOR_Left | MOTOR_Right);
            Buzzer_ON();
            Buzzer_Debug = 1;
            Delay_ms(500);
            Buzzer_OFF();
            Buzzer_Debug = 0;
            break;
        }
    }
    Delay_ms(DELAY_TIME);
    MOTOR_Spin(Vale, Angle, Spead);
    Delay_ms(DELAY_TIME);
    MOTOR_Stop(MOTOR_Left | MOTOR_Right, Spead, Distance);
}

void MOTOR_X_(int x)
{
    MOTOR_Clear(MOTOR_Left | MOTOR_Right);
    if (x >= 0) {
        while (MOTOR_CONTROL(Spead + E, x, MOTOR_Left) & MOTOR_CONTROL(Spead + E_ + 2, x, MOTOR_Right)) { Display(); }
    } else {
        while (MOTOR_CONTROL(Spead + E_, x, MOTOR_Left) & MOTOR_CONTROL(Spead + E + 2, x, MOTOR_Right)) { Display(); }
    }
    MOTOR_Clear(MOTOR_Left | MOTOR_Right);
    MOTOR_CONTROL(Spead, 0, MOTOR_Left | MOTOR_Right);
    Delay_ms(DELAY_TIME);
}

#define MOTOR_F(x)                                                                                            \
    MOTOR_Clear(MOTOR_Left | MOTOR_Right);                                                                    \
    while (MOTOR_CONTROL(Spead, (x), MOTOR_Left) | MOTOR_CONTROL(Spead + 1, (x), MOTOR_Right)) { Display(); } \
    Delay_ms(DELAY_TIME)

extern double SMOTOR_SPEED;

void Buzzer_Tow(uint16_t Time)
{
    Buzzer_Debug = 1;
    Buzzer_ON();
    Delay_ms(Time);
    Buzzer_OFF();
    Buzzer_ON();
    Delay_ms(Time);
    Buzzer_OFF();
    Buzzer_Debug = 0;
}

#define Adjust_Slow_Step_K  0.008
#define Adjust_Slow_Step    30
#define Adjust_Slow_Speed_K 1.09
#define Adjust_Slow_Speed   45
extern int32_t SMOTOR_B_Location;
extern double SMOTOR_B_Angle;
extern uint32_t SMOTOR_B_target, SMOTOR_B_step;

void SMOTOR_Angle_Adjust(double Angle, double Speed)
{
    int8_t Adjust_Flag  = 0;
    double Adjust_Error = 0;
    if ((SMOTOR_Angle < 0 && Angle > 0))
        Adjust_Flag = 1;
    else if ((SMOTOR_Angle > 0 && Angle < 0))
        Adjust_Flag = 2;
    angleTypeDef result     = SMOTOR_ANGLE(SMOTOR_Long, SMOTOR_Height, Angle, Speed);
    double SMOTOR_SPEED_B   = SMOTOR_SPEED_K / Adjust_Slow_Speed;
    double Location_Delta_B = result.angle_B * 200 / 9.0 - SMOTOR_B_Location;
    double Location_B       = SMOTOR_B_Location;
    double Slow_Step        = Adjust_Slow_Step * 2;
    double Location_Delta_M = result.angle_B * 200 / 9.0 - SMOTOR_B_Location - (result.angle_B * 200 / 9.0 - SMOTOR_B_Location) * Adjust_Slow_Step_K * Adjust_Slow_Step * 2;
    for (uint8_t i = 1; i <= Adjust_Slow_Step; i++) {
        Slow_Step--;
        if (i != 1) SMOTOR_SPEED_B /= Adjust_Slow_Speed_K;
        Location_B += Location_Delta_B * Adjust_Slow_Step_K;

        SMOTOR_CONTROL_MOVE(Location_B, SMOTOR_SPEED_B, Speed, SMOTOR_B);

        if (Adjust_Flag) {
            while (LightSensor_Spin() && Get_State(SMOTOR_B)) {}
            if (Get_State(SMOTOR_B)) {
                Adjust_Error = SMOTOR_B_step;
                if (Adjust_Flag == 2) {
                    Adjust_Error = -Adjust_Error;
                }
                Location_Delta_B = (result.angle_B * 200 / 9.0 - Location_Delta_M) / (Adjust_Slow_Step_K * Slow_Step);
                Adjust_Flag      = 0;
                Sensor_Spin_Flag = 1;
                Buzzer_ON();
                SMOTOR_Adjust(0);
                SMOTOR_CONTROL_MOVE(Location_B, SMOTOR_SPEED_B, Speed, SMOTOR_B);
                Location_B = Location_Delta_B * Adjust_Slow_Step_K - Adjust_Error;
                while (Get_State(SMOTOR_B)) {};
            }

        } else
            while (Get_State(SMOTOR_B)) {};
    }

    Location_B += Location_Delta_M;
    if (SMOTOR_SPEED_B < SMOTOR_SPEED_K / Speed)
        SMOTOR_CONTROL(SMOTOR_SPEED_K / Speed, Location_B, SMOTOR_B);
    else
        SMOTOR_CONTROL(SMOTOR_SPEED_B, Location_B, SMOTOR_B);
    if (Adjust_Flag) {
        while (LightSensor_Spin() && Get_State(SMOTOR_B)) {}
        if (Get_State(SMOTOR_B)) {
            Adjust_Error = SMOTOR_B_step;
            if (Adjust_Flag == 2) {
                Adjust_Error = -Adjust_Error;
            }
            Location_Delta_B = (result.angle_B * 200 / 9.0 - (Location_Delta_M - Adjust_Error)) / (Adjust_Slow_Step_K * Slow_Step);
            Adjust_Flag      = 0;
            Sensor_Spin_Flag = 1;
            Buzzer_ON();
            SMOTOR_Adjust(0);
            SMOTOR_CONTROL_MOVE(Location_B, SMOTOR_SPEED_B, Speed, SMOTOR_B);
            Location_B = Location_Delta_M - Adjust_Error;
            while (Get_State(SMOTOR_B)) {};
        }

    } else
        while (Get_State(SMOTOR_B)) {};

    for (uint8_t i = 1; i <= Adjust_Slow_Step - 1; i++) {
        Slow_Step--;
        SMOTOR_SPEED_B *= Adjust_Slow_Speed_K;
        Location_B += Location_Delta_B * Adjust_Slow_Step_K;
        if (SMOTOR_SPEED_B < SMOTOR_SPEED_K / Speed)
            SMOTOR_CONTROL(SMOTOR_SPEED_K / Speed, Location_B, SMOTOR_B);
        else
            SMOTOR_CONTROL(SMOTOR_SPEED_B, Location_B, SMOTOR_B);
        if (Adjust_Flag) {
            while (LightSensor_Spin() && Get_State(SMOTOR_B)) {}
            if (Get_State(SMOTOR_B)) {
                Adjust_Error = SMOTOR_B_step;
                if (Adjust_Flag == 2) {
                    Adjust_Error = -Adjust_Error;
                }
                Location_Delta_B = (result.angle_B * 200 / 9.0) / (Adjust_Slow_Step_K * Slow_Step);
                Adjust_Flag      = 0;
                Sensor_Spin_Flag = 1;
                Buzzer_ON();
                SMOTOR_Adjust(0);
                SMOTOR_CONTROL_MOVE(Location_B, SMOTOR_SPEED_B, Speed, SMOTOR_B);
                Location_B = Location_Delta_B * Adjust_Slow_Step_K - Adjust_Error;
                while (Get_State(SMOTOR_B)) {};
            }
        } else
            while (Get_State(SMOTOR_B)) {};
    }
    SMOTOR_CONTROL(SMOTOR_SPEED_B, result.angle_B * 200 / 9, SMOTOR_B);
    while (Get_State(SMOTOR_B)) {};
    Sensor_Spin_Flag = 0;
}

#define Suspend_Delta_R      50
#define Suspend_Delta_L      -10
#define Suspend_Slow_Step_K  0.01
#define Suspend_Slow_Step    30
#define Suspend_Slow_Speed_K 1.09
#define Suspend_Slow_Speed   30 // 45
extern int32_t SMOTOR_R_Location;
extern int32_t SMOTOR_L_Location;

#define Suspend_Long   200 // 160
#define Suspend_Height 90
// #define Suspend_Delta_L   0.1 // 0.4
// #define Suspend_Delta_L_K 1
// #define Suspend_Delta_R   4 // 4
/// @brief 移动到悬浮位置
/// @param Angle
/// @param Speed
void SMOTOR_MOVE_Suspend(double Angle, double Speed)
{
    // angleTypeDef result;
    // Sensor_Lift_Flag = 1;
    // result           = SMOTOR_ANGLE(Suspend_Long, Suspend_Height, Angle, Speed);
    // SMOTOR_CONTROL(result.speed_L * Suspend_Delta_L_K, (result.angle_L - Suspend_Delta_L) * 200 / 9, SMOTOR_L);
    // SMOTOR_CONTROL(result.speed_R, (result.angle_R + Suspend_Delta_R) * 200 / 9, SMOTOR_R);
    // SMOTOR_CONTROL(result.speed_B, result.angle_B * 200 / 9, SMOTOR_B);
    // while (LightSensor_Lift() && Get_State(SMOTOR_L | SMOTOR_R)) {};
    // SMOTOR_RESET(Suspend_Long, Suspend_Height, Angle, SMOTOR_L | SMOTOR_R);
    // while (Get_State(SMOTOR_B)) {};
    // Sensor_Lift_Flag = 0;

    angleTypeDef result       = SMOTOR_ANGLE(Suspend_Long, Suspend_Height, Angle, Speed);
    double SMOTOR_SPEED_R     = SMOTOR_SPEED_K / Suspend_Slow_Speed;
    double SMOTOR_SPEED_L     = SMOTOR_SPEED_K / Suspend_Slow_Speed;
    double Location_Delta_R   = result.angle_R * 200 / 9.0 - SMOTOR_R_Location;
    double Location_Delta_L   = result.angle_L * 200 / 9.0 - SMOTOR_L_Location;
    double Location_R         = SMOTOR_R_Location;
    double Location_L         = SMOTOR_L_Location;
    double Location_Delta_R_M = result.angle_R * 200 / 9.0 - SMOTOR_R_Location - (result.angle_R * 200 / 9.0 - SMOTOR_R_Location) * Adjust_Slow_Step_K * Adjust_Slow_Step;
    double Location_Delta_L_M = result.angle_L * 200 / 9.0 - SMOTOR_L_Location - (result.angle_L * 200 / 9.0 - SMOTOR_L_Location) * Adjust_Slow_Step_K * Adjust_Slow_Step;
    for (uint8_t i = 1; i <= Adjust_Slow_Step; i++) {
        if (i != 1) SMOTOR_SPEED_R /= Suspend_Slow_Speed_K;
        if (i != 1) SMOTOR_SPEED_L /= Suspend_Slow_Speed_K;
        Location_R += Location_Delta_R * Suspend_Slow_Step_K;
        Location_L += Location_Delta_L * Suspend_Slow_Step_K;
        SMOTOR_CONTROL_MOVE(Location_R, SMOTOR_SPEED_R, SMOTOR_SPEED_K / result.speed_R, SMOTOR_R);
        SMOTOR_CONTROL_MOVE(Location_L, SMOTOR_SPEED_L, SMOTOR_SPEED_K / result.speed_L, SMOTOR_L);
        while (Get_State(SMOTOR_L | SMOTOR_R)) {};
    }

    Location_R += Location_Delta_R_M + Suspend_Delta_R;
    Location_L += Location_Delta_L_M + Suspend_Delta_L;
    SMOTOR_CONTROL_MOVE(Location_R, SMOTOR_SPEED_R, SMOTOR_SPEED_K / result.speed_R, SMOTOR_R);
    SMOTOR_CONTROL_MOVE(Location_L, SMOTOR_SPEED_L, SMOTOR_SPEED_K / result.speed_L, SMOTOR_L);
    while (LightSensor_Lift() && Get_State(SMOTOR_L | SMOTOR_R)) {};
    if (Get_State(SMOTOR_L | SMOTOR_R)) {
        Sensor_Spin_Flag = 1;
        Buzzer_ON();
    }
    SMOTOR_RESET(Suspend_Long, Suspend_Height, Angle, SMOTOR_L | SMOTOR_R);
}

#define Get_Angle_R -40
#define Get_Angle_L 40
#define Get_Angle_M 3
#define Get_Angle   40
#define Get_Delay   300
#define Get_Long    200
#define Get_Height  10 // 15
/// @brief 车上取物料
/// @param Location
/// @param Lift_Speed
void Get_Block(Location Location, double Lift_Speed)
{
    switch (Location) {
        case Location_L:
            Swing(Behind);
            SMOTOR_Angle_Adjust(Get_Angle_L, SPEED_B);
            Stretch(Get_Angle);
            Delay_ms(Get_Delay);
            SMOTOR_MOVE(Get_Long, Get_Height, Get_Angle_L, SPEED);
            SHRINK;
            SMOTOR_MOVE_Suspend(Get_Angle_L, Lift_Speed);
            break;

        case Location_M:
            Swing(Behind);
            SMOTOR_Angle_Adjust(Get_Angle_M, SPEED_B);
            Stretch(Get_Angle);
            Delay_ms(Get_Delay);
            SMOTOR_MOVE(Get_Long, Get_Height, Get_Angle_M, SPEED);
            SHRINK;
            SMOTOR_MOVE_Suspend(Get_Angle_M, Lift_Speed);
            break;

        case Location_R:
            Swing(Behind);
            SMOTOR_Angle_Adjust(Get_Angle_R, SPEED_B);
            Stretch(Get_Angle + 40);
            Delay_ms(Get_Delay);
            SMOTOR_MOVE(Get_Long, Get_Height, Get_Angle_R, SPEED);
            SHRINK;
            SMOTOR_MOVE_Suspend(Get_Angle_R, Lift_Speed);
            break;
    }
}

#define CAMERA_Wait        1000
#define CAMERA_Height_Low  -45
#define CAMERA_Height_High -75
#define CAMERA_Long        200
/// @brief 移动摄像头到指定位置
/// @param Height 摄像头高度
void SMOTOR_MOVE_CAMERA(double Height, uint8_t Color)
{
    Send_CMD(CIRCLE_MODE, Color);
    Swing(0);
    SMOTOR_MOVE(CAMERA_Long, Height, Angle_Grasp, SPEED);
    Display();
    Delay_ms(CAMERA_Wait);
}

#define Place_Delay        200
#define Place_Angle        30
#define Place_Delta_Height 30
#define Camera_Delta_Long  0
#define Place_Delta_Long   50
/// @brief 放下物料
void Place_Block(double Lift_Speed)
{
    SMOTOR_Delta_MOVE(Camera_Delta_Long, -Place_Delta_Height, 0, SPEED);
    Delay_ms(Place_Delay);
    Stretch(Place_Angle);
    Delay_ms(Place_Delay);
    SMOTOR_Delta_MOVE(-Place_Delta_Long, 0, 0, 60);
    // SMOTOR_MOVE_Suspend(SMOTOR_Angle, Lift_Speed);
    SMOTOR_MOVE(Suspend_Long, Suspend_Height, SMOTOR_Angle, Lift_Speed);
    SHRINK;
}

#define Install_Angle      20
#define Install_Angle_R    -40
#define Install_Angle_L    40
#define Install_Angle_M    0
#define Install_Delay      300
#define Install_Long       200
#define Install_Delta_Long 0 // 15
#define Install_Height     15
/// @brief 装物料到车
/// @param Location
/// @param Install_Speed
void Install_Block(Location Location, double Lift_Speed)
{
    switch (Location) {
        case Location_L:
            Swing(0);
            Swing(Behind);
            SMOTOR_Angle_Adjust(Install_Angle_L, SPEED_B);
            SMOTOR_MOVE(Install_Long, Install_Height, Install_Angle_L, SPEED);
            Delay_ms(Install_Delay);
            Stretch(Install_Angle);
            SMOTOR_Delta_MOVE(Install_Delta_Long, 0, 0, Lift);
            SMOTOR_MOVE_Suspend(Install_Angle_L, Lift_Speed);
            Swing(Front);
            SHRINK;
            break;

        case Location_M:
            Swing(Behind);
            SMOTOR_Angle_Adjust(Install_Angle_M, SPEED_B);
            SMOTOR_MOVE(Install_Long, Install_Height, Install_Angle_M, SPEED);
            Delay_ms(Install_Delay);
            Stretch(Install_Angle);
            SMOTOR_Delta_MOVE(Install_Delta_Long, 0, 0, Lift);
            SMOTOR_MOVE_Suspend(0, Lift_Speed);
            Swing(Front);
            SHRINK;
            break;

        case Location_R:
            Swing(Behind);
            SMOTOR_Angle_Adjust(Install_Angle_R, SPEED_B);
            SMOTOR_MOVE(Install_Long, Install_Height, Install_Angle_R, SPEED);
            Delay_ms(Install_Delay);
            Stretch(Install_Angle);
            SMOTOR_Delta_MOVE(Install_Delta_Long, 0, 0, Lift);
            SMOTOR_MOVE_Suspend(Install_Angle_R, Lift_Speed);
            Swing(Front);
            SHRINK;
            break;
    }
}

#define Take_Angle        70
#define Take_Delay        500 // 100
#define Take_Long         140 // 160
#define Take_Delta_Long   15
#define Take_Speed        SPEED + 40
#define Take_Height       120 // 122
#define Take_Delta_Height 80
#define Take_X            100 // 80
#define Take_Y            100 // 80
#define Take_Delta_Angle  32
#define Take_Times        3
#define Take_error        5
#define Take_Scan_Time    15
/// @brief 从圆盘取物料
/// @param Color
/// @param Lift_Speed
void Take_Block(uint8_t Color, double Lift_Speed)
{
    int16_t Time_Out = Take_Delay, Flag = 0;
    double Camera_x, Camera_y;
    Send_CMD(CIRCLE_MODE, Color);
    SMOTOR_MOVE(Take_Long, Take_Height, Angle_Grasp + Take_Delta_Angle, SPEED);
    Swing(Front);
    Stretch(Take_Angle);
    while (1) {
        if (!Flag && (Camera_X > 60)) Flag = 1;
        if (Flag && (Camera_X < Take_X && Camera_X > -Take_X && Camera_Y < Take_Y && Camera_Y > -Take_Y && !(Camera_X == 0 && Camera_Y == 0))) {
            if ((Camera_X - Camera_x) < Take_error && (Camera_X - Camera_x) > -Take_error && (Camera_Y - Camera_y) < Take_error && (Camera_Y - Camera_y) > -Take_error)
                Time_Out--;
            else
                Time_Out = Take_Delay;
        }
        if (Time_Out % 10 == 0) {
            Camera_x = Camera_X;
            Camera_y = Camera_Y;
        }
        Delay_ms(1);
        if (!Time_Out) break;
    }
    Buzzer_Tow(20);
    for (uint8_t i = 0; i < Take_Times; i++) {
        if (i) Delay_ms(50);
        if (!(Camera_X == 255 || Camera_Y == 255)) SMOTOR_XY_MOVE(Camera_X * K_x, SMOTOR_Long + Camera_Distance + Camera_Y * K_y - Take_Delta_Long, SMOTOR_Height, SPEED - i * 20);
    }
    SMOTOR_Delta_MOVE(0, -Take_Delta_Height, 0, Take_Speed);
    SHRINK;
    SMOTOR_Delta_MOVE(10, 0, 0, Take_Speed);
    SMOTOR_MOVE(Suspend_Long, Suspend_Height, Angle_Grasp + Take_Delta_Angle, Lift_Speed);
    // SMOTOR_MOVE_Suspend(Angle_Grasp+Take_Delta_Angle, Lift_Speed);
    // SMOTOR_Delta_MOVE(0, 30, 0, Lift_Speed-10);
}

#define Grab_Height       -75
#define Grab_Delta_Height 60
#define Grab_Delta_Long   20
#define Grab_Angle        70
/// @brief 从放置区取物块
/// @param Color
/// @param Lift_Speed
void Grab_Block(CameraTypeDef Camera, double Lift_Speed)
{
    if (Camera.Angle < 20) {
        Camera.Height = CAMERA_Height_Low;
        Camera.Long   = CAMERA_Long;
        SMOTOR_Angle_Adjust(Angle_Grasp, SPEED_B);
    }
    SMOTOR_Angle_Adjust(Camera.Angle, SPEED_B);
    Stretch(Grab_Angle);
    if (!(Camera.Angle || Camera.Height || Camera.Long)) return;
    SMOTOR_MOVE(Camera.Long - Grab_Delta_Long, Grab_Height, SMOTOR_Angle, SPEED);
    SMOTOR_Delta_MOVE(0, -Grab_Delta_Height, 0, SPEED);
    SMOTOR_Delta_MOVE(20, 0, 0, Take_Speed);
    SHRINK;
    SMOTOR_MOVE(Suspend_Long, Suspend_Height, SMOTOR_Angle, Lift_Speed);
    // SMOTOR_MOVE_Suspend(SMOTOR_Angle, Lift_Speed);
}

#define CAMERA_SPEED 110
#define CAMERA_TIMES 6
#define CAMERA_DELAY 100

CameraTypeDef Camera[7];

#define Camera_First_One   Camera[1]
#define Camera_First_Two   Camera[2]
#define Camera_First_Three Camera[3]
#define Camera_Last_One    Camera[4]
#define Camera_Last_Two    Camera[5]
#define Camera_Last_Three  Camera[6]

void Nixie_Show()
{
    for (uint8_t i = 1; i <= 6; i++) {
        if (i > 3)
            Nixie_showNum(i + 2, Block_Data[i + 1], 0);
        else
            Nixie_showNum(i, Block_Data[i], 0);
    }
    Nixie_showPlus(4);
}

int main(void)
{
    Delay_ms(1000);
    SMOTOR_Init();
    OLED_Init();
    LightSensor_Init();
    OpenCV_Init();
    Servo_Init();
    QR_Init(9600);
    Timer_Init();
    Encoder_Init();
    Buzzer_Init();
    MAX7219_Init();
    PID_Init();
    PID_Inint(&PID_L);
    PID_Inint(&PID_R);

    Init_Status();
    Send_CMD(MAIN_MODE, NOCOLOR);
    Display();
    Delay_ms(3000);

    // Send_CMD(CIRCLE_MODE, First_Two);
    // while (1) {}
    //  start_scan_QRCode(5000);
    MOTOR_F(0);
    Block_Data[1] = 1;
    Block_Data[2] = 2;
    Block_Data[3] = 3;
    Block_Data[5] = 3;
    Block_Data[6] = 2;
    Block_Data[7] = 1;
    Nixie_Show();
    Display_();
    switch (1) {
            // 取物料
        case -2:
            SMOTOR_MOVE(115, 0, 0, 60);
            Delay_ms(1000);
            SMOTOR_MOVE_Suspend(0, 300);
            Delay_ms(1000);
            while (1) {

                SMOTOR_Angle_Adjust(-90, 350);
                Delay_ms(500);
                SMOTOR_Angle_Adjust(90, 350);
                Delay_ms(500);
                SMOTOR_Angle_Adjust(-90, 350);
                Delay_ms(500);
                SMOTOR_Angle_Adjust(90, 350);
                Delay_ms(500);
            }
            break;
        case -1:
            SMOTOR_MOVE(115, 0, 0, 60);
            Delay_ms(100);
            SMOTOR_MOVE_Suspend(0, Lift);
            while (1) {

                SMOTOR_Angle_Adjust(90, 100);
                Delay_ms(500);
                SMOTOR_Angle_Adjust(-36, 100);
                Delay_ms(500);
                SMOTOR_Angle_Adjust(90, 200);
                Delay_ms(500);
                SMOTOR_Angle_Adjust(0, 200);
                Delay_ms(500);
                SMOTOR_Angle_Adjust(90, 250);
                Delay_ms(500);
                SMOTOR_Angle_Adjust(40, 250);
                Delay_ms(500);
            }
            break;

        case 1:
            SMOTOR_MOVE(115, 0, 0, 60);
            Delay_ms(100);
            SMOTOR_MOVE_Suspend(0, Lift);
            SMOTOR_Angle_Adjust(Angle_Grasp, SPEED_B);
            Swing(Front);
            Stretch(Take_Angle);
            Delay_ms(1000);
            SHRINK;
            Delay_ms(1000);
            Install_Block(Location_R, Lift - 100);
            SMOTOR_Angle_Adjust(Angle_Grasp, SPEED_B);
            Swing(Front);
            Stretch(Take_Angle);
            Delay_ms(1000);
            SHRINK;
            Delay_ms(1000);
            Install_Block(Location_M, Lift - 100);
            SMOTOR_Angle_Adjust(Angle_Grasp, SPEED_B);
            Swing(Front);
            Stretch(Take_Angle);
            Delay_ms(1000);
            SHRINK;
            Delay_ms(1000);
            Install_Block(Location_L, Lift - 100);

            SMOTOR_Angle_Adjust(Angle_Grasp, 100);

            Get_Block(Location_L, Lift - 100);
            SMOTOR_Angle_Adjust(Angle_Grasp, 100);
            Swing(Front);

            Delay_ms(1000);
            Stretch(Take_Angle);
            Delay_ms(1000);
            SHRINK;

            Get_Block(Location_M, Lift - 100);
            SMOTOR_Angle_Adjust(Angle_Grasp, SPEED_B);
            Swing(Front);

            Delay_ms(1000);
            Stretch(Take_Angle);
            Delay_ms(1000);
            SHRINK;

            Get_Block(Location_R, Lift - 100);
            SMOTOR_Angle_Adjust(Angle_Grasp, SPEED_B);
            Swing(Front);

            Delay_ms(1000);
            Stretch(Take_Angle);
            Delay_ms(1000);
            SHRINK;

            break;

        case 2:

            SMOTOR_MOVE_Suspend(0, Lift);

            Take_Block(First_One, Lift);
            Install_Block(Location_R, Lift);
            Take_Block(First_Two, Lift);
            Install_Block(Location_M, Lift);
            Take_Block(First_Three, Lift);
            Install_Block(Location_L, Lift);

            Get_Block(Location_L, Lift);
            SMOTOR_MOVE_CAMERA(CAMERA_Height_Low, First_One);
            Camera_First_One = SMOTOR_CAMERA_MOVE(CAMERA_TIMES, CAMERA_DELAY, CAMERA_SPEED);
            Place_Block(Lift - 20);

            SMOTOR_Angle_Adjust(0, SPEED_B);
            Get_Block(Location_M, Lift);
            SMOTOR_MOVE_CAMERA(CAMERA_Height_Low, First_Two);
            Camera_First_Two = SMOTOR_CAMERA_MOVE(CAMERA_TIMES, CAMERA_DELAY, CAMERA_SPEED);
            Place_Block(Lift - 20);

            SMOTOR_Angle_Adjust(0, SPEED_B);
            Get_Block(Location_R, Lift);
            SMOTOR_MOVE_CAMERA(CAMERA_Height_Low, First_Three);
            Camera_First_Three = SMOTOR_CAMERA_MOVE(CAMERA_TIMES, CAMERA_DELAY, CAMERA_SPEED);
            Place_Block(Lift - 20);

            Send_CMD(CIRCLE_MODE, NOCOLOR);
            SMOTOR_Angle_Adjust(0, SPEED_B);

            Grab_Block(Camera_First_One, Lift);
            Install_Block(Location_R, Lift);

            Grab_Block(Camera_First_Two, Lift);
            Install_Block(Location_M, Lift);

            Grab_Block(Camera_First_Three, Lift);
            Install_Block(Location_L, Lift);

            Send_CMD(MAIN_MODE, NOCOLOR);

            break;

        case 3:

            Display();
            // break;7
            // case 4:
            SMOTOR_MOVE_Suspend(0, Lift);
            MOTOR_F(-1000);
            MOTOR_Spin(Left, -16, Spead);
            MOTOR_F(-6000);
            MOTOR_Spin(Left, 16, Spead);
            MOTOR_F(500); // 500
            MOTOR_Spin(Left, 90, Spead);

            MOTOR_F(1200);
            MOTOR_X(6200);
            Delay_ms(200); // 二维码
            start_scan_QRCode(5000);
            MOTOR_X(8500); // 12000
            Delay_ms(500); // 圆盘

            Take_Block(First_One, Lift);
            Install_Block(Location_L, Lift);
            Take_Block(First_Two, Lift);
            Install_Block(Location_M, Lift);
            Take_Block(First_Three, Lift);
            Send_CMD(MAIN_MODE, NOCOLOR);
            Install_Block(Location_R, Lift);
            SMOTOR_Angle_Adjust(20, SPEED_B);

            MOTOR_X(4900); // 5300

            MOTOR_F(Spin_Distance);
            MOTOR_Spin(Left, 90, Spead);
            MOTOR_F(Spin_Distance);

            MOTOR_X(9200);

            Delay_ms(300); // 放物料1
            Get_Block(Location_L, Lift);
            SMOTOR_MOVE_CAMERA(CAMERA_Height_Low, First_One);
            Camera_First_One = SMOTOR_CAMERA_MOVE(CAMERA_TIMES, CAMERA_DELAY, CAMERA_SPEED);
            Place_Block(Lift - 10);

            SMOTOR_Angle_Adjust(0, SPEED_B);
            Get_Block(Location_M, Lift);
            SMOTOR_MOVE_CAMERA(CAMERA_Height_Low, First_Two);
            Camera_First_Two = SMOTOR_CAMERA_MOVE(CAMERA_TIMES, CAMERA_DELAY, CAMERA_SPEED);
            Place_Block(Lift - 10);

            SMOTOR_Angle_Adjust(0, SPEED_B);
            Get_Block(Location_R, Lift);
            SMOTOR_MOVE_CAMERA(CAMERA_Height_Low, First_Three);
            Camera_First_Three = SMOTOR_CAMERA_MOVE(CAMERA_TIMES, CAMERA_DELAY, CAMERA_SPEED);
            Place_Block(Lift - 10);

            Send_CMD(CIRCLE_MODE, NOCOLOR);
            SMOTOR_Angle_Adjust(0, SPEED_B); // 取物料

            Grab_Block(Camera_First_One, Lift);
            Install_Block(Location_L, Lift);

            Grab_Block(Camera_First_Two, Lift);
            Install_Block(Location_M, Lift);

            Grab_Block(Camera_First_Three, Lift);
            Install_Block(Location_R, Lift);

            Send_CMD(MAIN_MODE, NOCOLOR);

            MOTOR_X(7500);

            MOTOR_F(Spin_Distance);
            MOTOR_Spin(Left, 90, Spead);
            MOTOR_F(Spin_Distance);

            MOTOR_X(8000);
            Delay_ms(500); // 放物料2
            Get_Block(Location_L, Lift);
            SMOTOR_MOVE_CAMERA(CAMERA_Height_Low, First_One);
            SMOTOR_CAMERA_MOVE(CAMERA_TIMES, CAMERA_DELAY, CAMERA_SPEED);
            Place_Block(Lift - 10);

            SMOTOR_Angle_Adjust(0, SPEED_B);
            Get_Block(Location_M, Lift);
            SMOTOR_MOVE_CAMERA(CAMERA_Height_Low, First_Two);
            SMOTOR_CAMERA_MOVE(CAMERA_TIMES, CAMERA_DELAY, CAMERA_SPEED);
            Place_Block(Lift - 10);

            SMOTOR_Angle_Adjust(0, SPEED_B);
            Get_Block(Location_R, Lift);
            SMOTOR_MOVE_CAMERA(CAMERA_Height_Low, First_Three);
            SMOTOR_CAMERA_MOVE(CAMERA_TIMES, CAMERA_DELAY, CAMERA_SPEED);
            Send_CMD(MAIN_MODE, NOCOLOR);
            Place_Block(Lift - 10);
            SMOTOR_Angle_Adjust(0, SPEED_B);

            MOTOR_X(11500);

            MOTOR_F(Spin_Distance);
            MOTOR_Spin(Left, 90, Spead);
            MOTOR_F(Spin_Distance);

            MOTOR_X(16400);

            MOTOR_F(Spin_Distance);
            MOTOR_Spin(Left, 90, Spead);
            MOTOR_F(Spin_Distance);

            Delay_ms(1500); // 第二轮

            MOTOR_X(16200); // 取物料
            MOTOR_X(3600);

            MOTOR_F(Spin_Distance);
            MOTOR_Spin(Left, 90, Spead);
            MOTOR_F(Spin_Distance);

            MOTOR_X(9200);
            Delay_ms(1500); // 取物料
            MOTOR_X(7500);

            MOTOR_F(Spin_Distance);
            MOTOR_Spin(Left, 90, Spead);
            MOTOR_F(Spin_Distance);

            MOTOR_X(4000);
            Delay_ms(1500); // 放物料2
            MOTOR_X(15500);

            MOTOR_F(1000);
            MOTOR_Spin(Left, 90, Spead);
            MOTOR_F(1000);

            MOTOR_X(16400);

            MOTOR_F(Spin_Distance);
            MOTOR_Spin(Left, 90, Spead);
            MOTOR_F(Spin_Distance);

            break;
    }
    while (1) {
        Display();
    }
}

#if 0
int main_(void)
{

    Init_Status();

    switch (1) {

        case 0xf:

            while (1) {
                Display();
            }

            break;

        case -4:
            MOTOR_Spin(Right, 90, Spead);
            while (1) {
                Display();
            }

        case -3:

            MOTOR_Spin(Right, 20, Spead);
            MOTOR_F(2000);
            MOTOR_Spin(Left, 20, Spead);
            MOTOR_F(-2400);

            break;

        case -2:

            while (1) {
                // MOTOR_Spin(Left,20,Spead+30);
                MOTOR_F(2000);
                // MOTOR_Spin(Right,20,Spead+30);
                MOTOR_F(-2000);
            }

            break;

            MOTOR_F(2000);
            MOTOR_Spin(Right, 90, Spead);
            MOTOR_F(-700);
            MOTOR_F(700);
            MOTOR_Spin(Left, 90, Spead);
            MOTOR_F(-2000);

        case -1:

            break;

        case 0:
            MOTOR_CONTROL(50, 10000, MOTOR_Left);
            MOTOR_CONTROL(50, -10000, MOTOR_Right);

            while (1) {
                if (Data(0)) {
                    MOTOR_CONTROL(50, 10000, MOTOR_Left);
                    MOTOR_CONTROL(50, -10000, MOTOR_Right);
                }
                Display();
            }

            break;

        case 1:

            Display();
            MOTOR_F(-1000);
            MOTOR_Spin(Left, -16, Spead);
            MOTOR_F(-6000);
            MOTOR_Spin(Left, 16, Spead);
            MOTOR_F(500);
            MOTOR_Spin(Left, 90, Spead);
            MOTOR_F(1200);
            MOTOR_X(6200);
            Delay_ms(1500); // 二维码
            MOTOR_X(10800);
            Delay_ms(1500); // 圆盘
            MOTOR_X(2500);

            MOTOR_F(1000);
            MOTOR_Spin(Left, 90, Spead);
            MOTOR_F(1000);

            MOTOR_X(5200);
            Delay_ms(1500); // 取物料
            MOTOR_X(11500);

            MOTOR_F(1000);
            MOTOR_Spin(Left, 90, Spead);
            MOTOR_F(1000);

            MOTOR_X(4000);
            Delay_ms(1500); // 放物料
            MOTOR_X(15500);

            MOTOR_F(1000);
            MOTOR_Spin(Left, 90, Spead);
            MOTOR_F(1000);

            MOTOR_X(16400);

            MOTOR_F(1000);
            MOTOR_Spin(Left, 90, Spead);
            MOTOR_F(1000);

            while (1) {
                Delay_ms(1500);
                MOTOR_X(6200);  // 第二轮
                Delay_ms(1500); // 二维码
                MOTOR_X(10800);
                Delay_ms(1500); // 圆盘
                MOTOR_X(2800);

                MOTOR_F(1000);
                MOTOR_Spin(Left, 90, Spead);
                MOTOR_F(1000);

                MOTOR_X(5200);
                Delay_ms(1500); // 取物料
                MOTOR_X(11500);

                MOTOR_F(1000);
                MOTOR_Spin(Left, 90, Spead);
                MOTOR_F(1000);

                MOTOR_X(4000);
                Delay_ms(1500); // 放物料
                MOTOR_X(15500);

                MOTOR_F(1000);
                MOTOR_Spin(Left, 90, Spead);
                MOTOR_F(1000);

                MOTOR_X(16400);

                MOTOR_F(1000);
                MOTOR_Spin(Left, 90, Spead);
                MOTOR_F(1000);
            }
            // MOTOR_F(19200);
            break;

        case 11:

            while (1) {
                Display();
                MOTOR_F(1000);
                MOTOR_Spin(Left, 30, 55);
                MOTOR_F(18000);
                MOTOR_Spin(Right, 30, Spead);
                MOTOR_F(15000);
                Delay_ms(15000);
                MOTOR_X(35000);
                Delay_ms(2000); // 圆盘
                // MOTOR_X(1000);
                MOTOR_F(5000);
                // MOTOR_F(_Distance);
                MOTOR_Spin(Left, 90, Spead);
                MOTOR_X(53000);
                MOTOR_F(10000);
                MOTOR_Spin(Left, 90, Spead);
                MOTOR_X(45000);
                Delay_ms(1500); // 放置
                MOTOR_X(18000);
                MOTOR_F(10000);
                MOTOR_Spin(Left, 90, Spead);
                MOTOR_X(78000);
                MOTOR_Spin(Left, 90, Spead);
                MOTOR_Spin(Left, 90, 40);

                // MOTOR_X(_Distance);
                // MOTOR_Spin(Left,90,Spead);
                // MOTOR_X(_Distance);
                // MOTOR_Spin(Left,90,Spead);
                // MOTOR_X(_Distance);
                // MOTOR_Spin(Left,174,Spead);
                // MOTOR_X(_Distance);
                // MOTOR_Spin(Right,90,Spead);
                // MOTOR_X(_Distance);
                // MOTOR_Spin(Right,90,Spead);
                // MOTOR_X(_Distance);
                // MOTOR_Spin(Right,90,Spead);
                // MOTOR_X(_Distance);
                // MOTOR_Spin(Right,174,Spead);
            }

            break;

    while (1) {
        Display();
    }

    }
}
#endif

void TIM6_IRQHandler(void)
{
    static uint16_t MOTOR_Spead_Counter = 0, Buzzer_Counter = 0;
    if (TIM_GetITStatus(TIM6, TIM_IT_Update) == SET) {
        if (MOTOR_Spead_Counter >= 28) // 13
        {
            MOTOR_Spead_Counter = 0;
            MOTOR_Spead_calc();
        } else

            PID_L.current_l = Encoder_Left_Get();
        PID_L.current_s = MOTOR_LeftSpead;
        PID_R.current_l = Encoder_Right_Get();
        PID_R.current_s = MOTOR_RightSpead;
        MOTOR_Run_all();
        PID_calc_all();

        Buzzer_Counter++;
        if (!Buzzer_Flag && (Sensor_Lift_Flag && (!LightSensor_Lift()) || (Sensor_Spin_Flag && (!LightSensor_Spin())))) {
            Buzzer_Counter = 1;
            Buzzer_Flag    = 1;
        }
        if (Buzzer_Counter <= 40)
            Buzzer_ON();
        else if (!Buzzer_Debug)
            Buzzer_OFF();
        if (Buzzer_Counter >= Buzzer_Delay) {
            Buzzer_Counter = Buzzer_Delay;
            Buzzer_Flag    = 0;
        }
        MOTOR_Spead_Counter++;

        TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
    }
}
