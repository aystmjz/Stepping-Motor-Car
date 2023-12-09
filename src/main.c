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

PID PID_L, PID_R;
extern int16_t MOTOR_LeftSpead, MOTOR_RightSpead;
extern double SMOTOR_Long, SMOTOR_Angle, SMOTOR_Height;
extern uint8_t Block_Data[10];

uint8_t Buzzer_Flag  = 0;
uint8_t Buzzer_Debug = 0;
#define Buzzer_Delay 500

#define Data(x)    getUsartBuf((x))

#define E          -(Get_Excursion() / 9)
#define E_         (Get_Excursion() / 9)
#define D          Get_Condition()
#define C          Get_Color()
#define S          getUsartBuf(0)

#define _Distance  7000

#define Spin_Distance  1200

#define Spead      80
#define Distance_  3000
#define Backward   -200

#define DELAY_TIME 200

#define MOTOR_X(x) MOTOR_X_(x)

void Display_()
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
void Display()
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
        if (!(MOTOR_CONTROL(Spead ,Distance, MOTOR_Left) & MOTOR_CONTROL(Spead , Distance, MOTOR_Right))) {
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

#define Suspend_Delta_M 3 // 0.4

void SMOTOR_Angle_Adjust(double Angle, double Speed)
{
    if (!Angle) {
        angleTypeDef result;
        result = SMOTOR_ANGLE(SMOTOR_Long, SMOTOR_Height, Angle, Speed);
        SMOTOR_CONTROL(result.speed_B, (result.angle_B - Suspend_Delta_M) * 200 / 9, SMOTOR_B);

        while (LightSensor_Spin() && Get_State(SMOTOR_B)) {}
        SMOTOR_RESET(SMOTOR_Long, SMOTOR_Height, Angle, SMOTOR_B);
        return;
    }
    SMOTOR_MOVE(SMOTOR_Long, SMOTOR_Height, Angle, Speed);
}

#define Suspend_Long    160 // 160
#define Suspend_Height  110
#define Suspend_Delta_R 1 // 0.4
#define Suspend_Delta_L 3
/// @brief 移动到悬浮位置
/// @param Angle
/// @param Speed
void SMOTOR_MOVE_Suspend(double Angle, double Speed)
{
    angleTypeDef result;
    result = SMOTOR_ANGLE(Suspend_Long, Suspend_Height, Angle, Speed);
    SMOTOR_CONTROL(result.speed_L, (result.angle_L + Suspend_Delta_L) * 200 / 9, SMOTOR_L);
    SMOTOR_CONTROL(result.speed_R, (result.angle_R - Suspend_Delta_R) * 200 / 9, SMOTOR_R);
    SMOTOR_CONTROL(result.speed_B, result.angle_B * 200 / 9, SMOTOR_B);

    while (LightSensor_Lift() && Get_State(SMOTOR_L | SMOTOR_R)) {};
    // while (Get_State(SMOTOR_L | SMOTOR_R)) {};
    SMOTOR_RESET(Suspend_Long, Suspend_Height, Angle, SMOTOR_L | SMOTOR_R);
    while (Get_State(SMOTOR_B)) {};
}

#define Get_Angle_R       -20
#define Get_Angle_L       20
#define Get_Angle_M       -2
#define Get_Angle         30
#define Get_Delay         500
#define Get_Long          130
#define Get_Long_Adjust_M 30
#define Get_Height        20
/// @brief 车上取物料
/// @param Location
/// @param Lift_Speed
void Get_Block(Location Location, double Lift_Speed)
{
    switch (Location) {
        case Location_L:
            SMOTOR_Angle_Adjust(Get_Angle_L, SPEED_B);
            Swing(Get_Angle_L);
            Stretch(Get_Angle);
            Delay_ms(Get_Delay);
            SMOTOR_MOVE(Get_Long, Get_Height, Get_Angle_L, SPEED);
            SHRINK;
            SMOTOR_MOVE_Suspend(Get_Angle_L, Lift_Speed);
            break;

        case Location_M:
            SMOTOR_Angle_Adjust(Get_Angle_M, SPEED_B);
            Swing(Get_Angle_M);
            Stretch(Get_Angle);
            Delay_ms(Get_Delay);
            SMOTOR_MOVE(Get_Long - Get_Long_Adjust_M, Get_Height, Get_Angle_M, SPEED);
            SHRINK;
            SMOTOR_MOVE_Suspend(Get_Angle_M, Lift_Speed);
            break;

        case Location_R:
            SMOTOR_Angle_Adjust(Get_Angle_R, SPEED_B);
            Swing(Get_Angle_R);
            Stretch(Get_Angle);
            Delay_ms(Get_Delay);
            SMOTOR_MOVE(Get_Long, Get_Height, Get_Angle_R, SPEED);
            SHRINK;
            SMOTOR_MOVE_Suspend(Get_Angle_R, Lift_Speed);
            break;
    }
}

#define CAMERA_Wait        1000
#define CAMERA_Height_Low  50
#define CAMERA_Height_High 120
#define CAMERA_Long        150
/// @brief 移动摄像头到指定位置
/// @param Height 摄像头高度
void SMOTOR_MOVE_CAMERA(double Height, uint8_t Color)
{
    Send_CMD(CIRCLE_MODE, Color);
    Swing(-90);
    SMOTOR_MOVE(CAMERA_Long, Height, Angle_Grasp, SPEED);
    Display();
    Delay_ms(CAMERA_Wait);
}

#define Place_Delay        200
#define Place_Angle        30
#define Place_Delta_Height 80
#define Place_Delta_Long   50
/// @brief 放下物料
void Place_Block(double Lift_Speed)
{
    SMOTOR_Delta_MOVE(0, -Place_Delta_Height, 0, SPEED);
    Delay_ms(Place_Delay);
    Stretch(Place_Angle);
    Delay_ms(Place_Delay);
    SMOTOR_Delta_MOVE(-Place_Delta_Long, 0, 0, 60);
    // SMOTOR_MOVE_Suspend(SMOTOR_Angle, Lift_Speed);
    SMOTOR_MOVE(Suspend_Long, Suspend_Height, SMOTOR_Angle, Lift_Speed);
    SHRINK;
}

#define Install_Angle         20
#define Install_Angle_R       -23
#define Install_Angle_L       23
#define Install_Angle_M       0
#define Install_Delay         500
#define Install_Long          160 // 170
#define Install_Long_Adjust_M 20
#define Install_Delta_Long    15
#define Install_Height        30
/// @brief 装物料到车
/// @param Location
/// @param Install_Speed
void Install_Block(Location Location, double Install_Speed)
{
    switch (Location) {
        case Location_L:
            SMOTOR_Angle_Adjust(0, SPEED_B);
            SMOTOR_Angle_Adjust(Install_Angle_L, SPEED_B - 50);
            Swing(Install_Angle_L);
            SMOTOR_MOVE(Install_Long, Install_Height, Install_Angle_L, SPEED);
            Stretch(Install_Angle);
            Delay_ms(Install_Delay);
            SMOTOR_Delta_MOVE(-Install_Delta_Long, 0, 0, Lift);
            SMOTOR_MOVE_Suspend(Install_Angle_L, Install_Speed);
            Swing(0);
            SHRINK;
            break;

        case Location_M:
            SMOTOR_Angle_Adjust(Install_Angle_M, SPEED_B);
            Swing(Install_Angle_M);
            SMOTOR_MOVE(Install_Long - Install_Long_Adjust_M, Install_Height, Install_Angle_M, SPEED);
            Stretch(Install_Angle);
            Delay_ms(Install_Delay);
            SMOTOR_Delta_MOVE(-Install_Delta_Long, 0, 0, Lift);
            SMOTOR_MOVE_Suspend(0, Install_Speed);
            Swing(0);
            SHRINK;
            break;

        case Location_R:
            SMOTOR_Angle_Adjust(0, SPEED_B);
            SMOTOR_Angle_Adjust(Install_Angle_R, SPEED_B - 50);
            Swing(Install_Angle_R);
            SMOTOR_MOVE(Install_Long, Install_Height, Install_Angle_R, SPEED);
            Stretch(Install_Angle);
            Delay_ms(Install_Delay);
            SMOTOR_Delta_MOVE(-Install_Delta_Long, 0, 0, Lift);
            SMOTOR_MOVE_Suspend(Install_Angle_R, Install_Speed);
            Swing(0);
            SHRINK;
            break;
    }
}

#define Take_Angle        70
#define Take_Delay        500//100
#define Take_Long         140//160
#define Take_Delta_Long   15
#define Take_Speed        SPEED + 40
#define Take_Height       120//122
#define Take_Delta_Height 80
#define Take_X            100//80
#define Take_Y            100//80
#define Take_Delta_Angle  32
#define Take_Times         3
#define Take_error         5
/// @brief 从圆盘取物料
/// @param Color
/// @param Lift_Speed
void Take_Block(uint8_t Color, double Lift_Speed)
{
    int16_t Time_Out   = Take_Delay,Flag=0;
    double Camera_x, Camera_y;
    Send_CMD(CIRCLE_MODE, Color);
    SMOTOR_MOVE(Take_Long, Take_Height, Angle_Grasp+Take_Delta_Angle, SPEED);
    Swing(0);
    Stretch(Take_Angle);
    while (1)
    {
        if(!Flag&&(Camera_X>50))Flag=1;
        if(Flag&&(Camera_X < Take_X && Camera_X > -Take_X && Camera_Y < Take_Y && Camera_Y > -Take_Y && !(Camera_X == 0 && Camera_Y == 0)))
        {
            if((Camera_X-Camera_x)<Take_error&&(Camera_X-Camera_x)>-Take_error&&(Camera_Y-Camera_y)<Take_error&&(Camera_Y-Camera_y)>-Take_error)Time_Out--;
            else Time_Out=Take_Delay;
        }
        if(Time_Out%10==0)
        {
            Camera_x=Camera_X;
            Camera_y=Camera_Y;
        }
    Delay_ms(1);
    if(!Time_Out)break;
    }
    Buzzer_ON();
    Buzzer_Debug = 1;
    Delay_ms(20);
    Buzzer_OFF();
    Buzzer_ON();
    Delay_ms(20);
    Buzzer_OFF();
    Buzzer_Debug = 0;
    for (uint8_t i = 0; i < Take_Times; i++) {
        if (i) Delay_ms(100);
        if (!(Camera_X == 255 || Camera_Y == 255)) SMOTOR_XY_MOVE(Camera_X * K_x, SMOTOR_Long + Camera_Distance + Camera_Y * K_y-Take_Delta_Long, SMOTOR_Height, SPEED-i*20);
    }
    //Delay_ms(Take_Delay);
    SMOTOR_Delta_MOVE(0, -Take_Delta_Height, 0, Take_Speed);
    SHRINK;
    SMOTOR_Delta_MOVE(10, 0, 0, Take_Speed);
    SMOTOR_MOVE(Suspend_Long, Suspend_Height, Angle_Grasp+Take_Delta_Angle, Lift_Speed);
    //SMOTOR_MOVE_Suspend(Angle_Grasp+Take_Delta_Angle, Lift_Speed);
    //SMOTOR_Delta_MOVE(0, 30, 0, Lift_Speed-10);
}

#define Grab_Height       30
#define Grab_Delta_Height 60
#define Grab_Delta_Long   20
#define Grab_Angle        40
/// @brief 从放置区取物块
/// @param Color
/// @param Lift_Speed
void Grab_Block(CameraTypeDef Camera, double Lift_Speed)
{
    SMOTOR_Angle_Adjust(Camera.Angle, SPEED_B);
    Stretch(Grab_Angle);
    if (!(Camera.Angle || Camera.Height || Camera.Long)) return;
    SMOTOR_MOVE(Camera.Long - Grab_Delta_Long, Grab_Height, SMOTOR_Angle, SPEED);
    SMOTOR_Delta_MOVE(0, -Grab_Delta_Height, 0, SPEED);
    SHRINK;
    SMOTOR_MOVE(Suspend_Long - 10, Suspend_Height, SMOTOR_Angle, Lift_Speed);
    // SMOTOR_MOVE_Suspend(SMOTOR_Angle, Lift_Speed);
}

#define CAMERA_SPEED 110
#define CAMERA_TIMES 5
#define CAMERA_DELAY 100

CameraTypeDef Camera[7];

#define Camera_First_One   Camera[1]
#define Camera_First_Two   Camera[2]
#define Camera_First_Three Camera[3]
#define Camera_Last_One    Camera[4]
#define Camera_Last_Two    Camera[5]
#define Camera_Last_Three  Camera[6]

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
    Block_Data[1] = 1;
    Block_Data[2] = 2;
    Block_Data[3] = 3;

    switch (1) {
            // 取物料
        case 1:
            SMOTOR_MOVE_Suspend(0, Lift);

            Take_Block(First_One, Lift);
            Install_Block(Location_R, Lift);
            Take_Block(First_Two, Lift);
            Install_Block(Location_M, Lift);
            Take_Block(First_Three, Lift);
            Install_Block(Location_L, Lift);

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
            //             break;
            // case 4:
            MOTOR_F(-1000);
            MOTOR_Spin(Left, -16, Spead);
            MOTOR_F(-6000);
            MOTOR_Spin(Left, 16, Spead);
            MOTOR_F(800);//500
            MOTOR_Spin(Left, 90, Spead);


            MOTOR_F(1200);
            MOTOR_X(6200);
            Delay_ms(1500); // 二维码
            MOTOR_X(12000);
            Delay_ms(5000); // 圆盘

            // SMOTOR_MOVE_Suspend(0, Lift);
            // Take_Block(First_One, Lift);
            // Install_Block(Location_R, Lift);
            // Take_Block(First_Two, Lift);
            // Install_Block(Location_M, Lift);
            // Take_Block(First_Three, Lift);
            // Install_Block(Location_L, Lift);

            Send_CMD(MAIN_MODE, NOCOLOR);

            MOTOR_X(1300);

            MOTOR_F(Spin_Distance);
            MOTOR_Spin(Left, 90, Spead);
            MOTOR_F(Spin_Distance);
            //Go_Corner(1000,Left, 90);

            MOTOR_X(9200);

            Delay_ms(5000); // 放物料1
            //  Get_Block(Location_L, Lift);
            // SMOTOR_MOVE_CAMERA(CAMERA_Height_Low, First_One);
            // Camera_First_One = SMOTOR_CAMERA_MOVE(CAMERA_TIMES, CAMERA_DELAY, CAMERA_SPEED);
            // Place_Block(Lift - 20);

            // SMOTOR_Angle_Adjust(0, SPEED_B);
            // Get_Block(Location_M, Lift);
            // SMOTOR_MOVE_CAMERA(CAMERA_Height_Low, First_Two);
            // Camera_First_Two = SMOTOR_CAMERA_MOVE(CAMERA_TIMES, CAMERA_DELAY, CAMERA_SPEED);
            // Place_Block(Lift - 20);

            // SMOTOR_Angle_Adjust(0, SPEED_B);
            // Get_Block(Location_R, Lift);
            // SMOTOR_MOVE_CAMERA(CAMERA_Height_Low, First_Three);
            // Camera_First_Three = SMOTOR_CAMERA_MOVE(CAMERA_TIMES, CAMERA_DELAY, CAMERA_SPEED);
            // Place_Block(Lift - 20);

            // Send_CMD(CIRCLE_MODE, NOCOLOR);
            // SMOTOR_Angle_Adjust(0, SPEED_B);// 取物料

            // Grab_Block(Camera_First_One, Lift);
            // Install_Block(Location_R, Lift);

            // Grab_Block(Camera_First_Two, Lift);
            // Install_Block(Location_M, Lift);

            // Grab_Block(Camera_First_Three, Lift);
            // Install_Block(Location_L, Lift);

            // Send_CMD(MAIN_MODE, NOCOLOR);

            MOTOR_X(7500);

            MOTOR_F(Spin_Distance);
            MOTOR_Spin(Left, 90, Spead);
            MOTOR_F(Spin_Distance);
            //Go_Corner(1000,Left, 90);

            MOTOR_X(9750);
            Delay_ms(1500); // 放物料2
            MOTOR_X(9750);

            MOTOR_F(Spin_Distance);
            MOTOR_Spin(Left, 90, Spead);
            MOTOR_F(Spin_Distance);
            //Go_Corner(1000,Left, 90);

            MOTOR_X(16400);

            MOTOR_F(Spin_Distance);
            MOTOR_Spin(Left, 90, Spead);
            MOTOR_F(Spin_Distance);



            Delay_ms(1500);// 第二轮

            MOTOR_X(16200);//取物料
            MOTOR_X(3600);

            MOTOR_F(Spin_Distance);
            MOTOR_Spin(Left, 90, Spead);
            MOTOR_F(Spin_Distance);
            //Go_Corner(1000,Left, 90);

            MOTOR_X(9200);
            Delay_ms(1500); // 取物料
            MOTOR_X(7500);

            MOTOR_F(Spin_Distance);
            MOTOR_Spin(Left, 90, Spead);
            MOTOR_F(Spin_Distance);
            //Go_Corner(1000,Left, 90);

            MOTOR_X(4000);
            Delay_ms(1500); // 放物料2
            MOTOR_X(15500);

            MOTOR_F(1000);
            MOTOR_Spin(Left, 90, Spead);
            MOTOR_F(1000);
            //Go_Corner(1000,Left, 90);

            MOTOR_X(16400);

            MOTOR_F(Spin_Distance);
            MOTOR_Spin(Left, 90, Spead);
            MOTOR_F(Spin_Distance);
            //Go_Corner(1000,Left, 90);

            break;
    }
    while (1) {
        Display();
    }
}

#ifdef M_MOTOR_
void Display2()
{
    OLED_ShowSignedNum(2, 1, PID_L2.e_l, 5);
    OLED_ShowSignedNum(2, 9, PID_R2.e_l, 5);
    OLED_ShowSignedNum(3, 1, PID_L2.TOTAL_OUT, 5);
    OLED_ShowSignedNum(3, 9, PID_R2.TOTAL_OUT, 5);
    OLED_ShowSignedNum(4, 1, MOTOR_Left2Spead, 3);
    OLED_ShowSignedNum(4, 6, PID_L2.e_l - PID_R2.e_l, 3);
    OLED_ShowSignedNum(4, 11, MOTOR_Right2Spead, 3);
    // OLED_ShowSignedNum(1,1,Encoder_Left_Get(),5);
    OLED_ShowSignedNum(1, 9, Encoder_Right2_Get(), 5);
    OLED_ShowNum(1, 1, Data(1), 2);
    OLED_ShowNum(1, 3, Data(0), 2);
    OLED_ShowSignedNum(1, 5, E, 1);
    OLED_ShowSignedNum(1, 7, E_, 1);
}
#endif
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
    }

    while (1) {
        Display();
    }
}



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
        if (!Buzzer_Flag && (!LightSensor_Lift() || !LightSensor_Spin())) {
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
