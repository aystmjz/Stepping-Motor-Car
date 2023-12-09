#include "SMOTOR.h"
#include "OLED.h"
#include "OpenCV.h"
#include "Delay.h"
#include "Servo.h"

extern double SMOTOR_Long, SMOTOR_Angle, SMOTOR_Height;

void display()
{
    OLED_ShowString(1, 1, "X:");
    OLED_ShowSignedNum(1, 3, getUsartBuf_float(2) * 10, 4);
    OLED_ShowString(2, 1, "Y:");
    OLED_ShowSignedNum(2, 3, getUsartBuf_float(6) * 10, 4);
    OLED_ShowString(3, 1, "Flag:");
    OLED_ShowSignedNum(3, 6, getUsartBuf(10), 2);
    OLED_ShowString(4, 1, "DATE:");
}
int main(void)
{
    SMOTOR_Init();
    OLED_Init();
    OpenCV_Init();
    Servo_Init();

    //while (1) {

        Init_Status();
        // Delay_ms(1000);
        // Stretch(150);
        // Delay_ms(1000);
        // Delay_ms(1000);
        // SHRINK;
        // Delay_ms(1000);
        // double h=11,l=13.5,r=17.3,angle_R_start=24;
        // 初始点 0，7，8.7
    //}
    SMOTOR_MOVE(96.1, 0, 0,SPEED);
    Delay_ms(1000);
    SMOTOR_MOVE(96.1, 10, 0,SPEED-50);
    Delay_ms(1000);
    SMOTOR_MOVE(160, 80, 0,SPEED-60);
    Delay_ms(1000);
    Swing(0);
    Stretch(100);
    Delay_ms(500);
    SMOTOR_MOVE(160, 0, 0,SPEED);
    SHRINK;

    SMOTOR_MOVE(160, 80, 0,SPEED);

    SMOTOR_MOVE(150, 60, Angle_Grasp,SPEED);
    Swing(-90);
    SMOTOR_MOVE(150, 30, Angle_Grasp,SPEED);
    display();
    Delay_ms(2000);
    //CAMERA_ANGLE(getUsartBuf_float(2), getUsartBuf_float(6), SMOTOR_Long, -20);
    //CAMERA_ANGLE_(160,160, SMOTOR_Long, -20);
    Swing(0);
    CAMERA_ANGLE(80, 80,SPEED);
    Delay_ms(1000);
    SMOTOR_Delta_MOVE(0, -80, 0,SPEED);
    Delay_ms(200);
    Stretch(100);
    SMOTOR_Delta_MOVE(-40, 0, 0,SPEED);
    SMOTOR_Delta_MOVE(0, 80, 0,SPEED);
    SHRINK;
    //SMOTOR_MOVE(96.1, 20, 0,SPEED);

    SMOTOR_MOVE(160, 100, 0,SPEED-50);
    //SMOTOR_ResetLocation(SMOTOR_L | SMOTOR_R | SMOTOR_B);

    //Init_Status();
while (1) {
    }
    Delay_ms(1000);
    display();
    SMOTOR_MOVE(96.1, 0, 0,SPEED);
    Delay_ms(1000);
    SMOTOR_MOVE(96.1, 10, 0,SPEED);
    SMOTOR_MOVE(150, 20, Angle_Grasp,SPEED);
    SMOTOR_MOVE(150, -20, Angle_Grasp,SPEED);
    display();
    Delay_ms(2000);
    //CAMERA_ANGLE(getUsartBuf_float(2), getUsartBuf_float(6), SMOTOR_Long, -20);
    //CAMERA_ANGLE_(160,160, SMOTOR_Long, -20);
    CAMERA_ANGLE(160, 160,SPEED);
    Delay_ms(1000);
    SMOTOR_Delta_MOVE(0, -50, 0,SPEED);
    SMOTOR_Delta_MOVE(-40, 0, 0,SPEED);
    SMOTOR_Delta_MOVE(0, 50, 0,SPEED);
    SMOTOR_MOVE(96.1, 10, 0,SPEED);
    SMOTOR_ResetLocation(SMOTOR_L | SMOTOR_R | SMOTOR_B);
    // Delay_ms(1000);
    // SMOTOR_MOVE(150,0,0);
    // Delay_ms(1000);
    // SMOTOR_MOVE(150,10,0);
    // Delay_ms(1000);
    // SMOTOR_MOVE(220,10,0);
    // Delay_ms(1000);
    // SMOTOR_MOVE(220,0,0);
    // Delay_ms(1000);

    // Delay_ms(1000);
    // SMOTOR_MOVE(15,0,0);
    // Delay_ms(1000);
    // SMOTOR_MOVE(15,2,0);
    // Delay_ms(1000);
    // SMOTOR_MOVE(15,4,0);
    // Delay_ms(1000);
    // SMOTOR_MOVE(15,6,0);
    // Delay_ms(1000);
    // SMOTOR_MOVE(15,8,0);
    // Delay_ms(1000);
    // SMOTOR_MOVE(15,10,0);
    // Delay_ms(1000);
    // SMOTOR_MOVE(15,12,0);
    // Delay_ms(1000);

    // SMOTOR_MOVE(20,0,0);
    // Delay_ms(1000);
    // SMOTOR_MOVE(25,0,0);
    // Delay_ms(1000);
    // SMOTOR_MOVE(30,0,0);

    // Delay_ms(1000);
    // SMOTOR_MOVE(12,0,0);
    // Delay_ms(1000);
    // SMOTOR_MOVE(14,0,0);
    // Delay_ms(1000);
    // SMOTOR_MOVE(16,0,0);
    // Delay_ms(1000);
    // SMOTOR_MOVE(18,0,0);
    // Delay_ms(1000);
    // SMOTOR_MOVE(20,0,0);
    // Delay_ms(1000);
    // SMOTOR_MOVE(22,0,0);
    // Delay_ms(1000);

    // SMOTOR_MOVE(14,10,2);
    // Delay_ms(1000);
    // SMOTOR_MOVE(16,20,2);

    // SMOTOR_MOVE(10,20,15);
    // Delay_ms(1000);
    // SMOTOR_MOVE(10,15,15);
    // Delay_ms(1000);
    // SMOTOR_MOVE(10,20,15);
    while (1) {
    }
}
