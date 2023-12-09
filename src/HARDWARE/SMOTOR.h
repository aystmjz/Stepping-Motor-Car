#ifndef _MOTOR_H_
#define _MOTOR_H_

#include <math.h>
#include "stm32f10x.h"
#include "Delay.h"
#include "OpenCV.h"
#include "PID.h"
#include "Servo.h"
#include "Buzzer.h"

#define MOTOR_FULL_STEP      0 // 满步
#define MOTOR_HALF_STEP      1 // 二分之一步
#define MOTOR_QUARTER_STEP   2 // 四分之一步
#define MOTOR_EIGHTH_STEP    3 // 八分之一步
#define MOTOR_SIXTEENTH_STEP 4 // 十六分之一步

#define TIM2_OCInit          TIM_OC2Init // PB3 1 G15
#define TIM3_OCInit          TIM_OC1Init // PA6 2 A5
#define TIM4_OCInit          TIM_OC4Init // PB9 3 B8

#define TIM2_SetCompare      TIM_SetCompare2
#define TIM3_SetCompare      TIM_SetCompare1
#define TIM4_SetCompare      TIM_SetCompare4

#define DIR_GPIO_G           GPIO_Pin_15
#define DIR_GPIO_A           GPIO_Pin_5
#define DIR_GPIO_B           GPIO_Pin_8

#define STEP_GPIO_A          GPIO_Pin_6
#define STEP_GPIO_B          GPIO_Pin_3 | GPIO_Pin_9

#define START_DEFALTX        0
#define START_DEFALTY        8

#define SMOTOR_R             ((uint8_t)0x01)
#define SMOTOR_L             ((uint8_t)0x02)
#define SMOTOR_B             ((uint8_t)0x08)
#define ALL_SMOTOR           SMOTOR_R | SMOTOR_L | SMOTOR_B

typedef enum {
    Tim2 = 2,
    Tim3 = 3,
    Tim4 = 4
} Tim;

typedef enum {
    Location_L = 1,
    Location_M = 2,
    Location_R = 3
} Location;

typedef struct angles {
    double angle_L;
    double angle_R;
    double angle_B;
    double speed_L;
    double speed_R;
    double speed_B;

} angleTypeDef;

typedef struct Camera {
    double Long;
    double Height;
    double Angle;

} CameraTypeDef;

#define SPEED                            110
#define Lift                             70
#define SPEED_B                          180
#define SPEED_B_MAX                      180
#define SMOTOR_SPEED_K                   200000

#define K_x                              (120.0 / 160)
#define K_y                              (120.0 / 160)
#define Camera_Distance                  155.0
#define Angle_Grasp                      90.0
#define Camera_X                         getUsartBuf_float(2)
#define Camera_Y                         getUsartBuf_float(6)

#define Angle_Clculate(angle_x, angle_y) (Radian_Angle(atan((angle_x) / (angle_y))))

#define PI                               3.1415926

#define l                                170.0
#define r                                140.23

#define SMOTOR_L_Init                    45 // 34.4
#define SMOTOR_R_Init                    -3
#define SMOTOR_B_Init                    0

#define Angle_Radian(angle)              ((angle) * (PI / 180.0))                       // 角度转弧度
#define Radian_Angle(angle)              ((angle) * (180.0 / PI))                       // 弧度转角度
#define CosineLaw_Angle(a, b, c)         (acos((b * b + c * c - a * a) / (2 * b * c)))  // 余弦定理计算角度
#define CosineLaw_Edge(b, c, angle)      (sqrt(b * b + c * c - 2 * b * c * cos(angle))) // 余弦定理计算对边(angle是弧度)

#define Clculate(angle)                  (180.0 - Radian_Angle(CosineLaw_Angle(20.0, 140.23, CosineLaw_Edge(20.0, 140.23, Angle_Radian(angle))) + CosineLaw_Angle(140.0, CosineLaw_Edge(20.0, 140.23, Angle_Radian(angle)), 30.0)))

#define Modify(angle_l, angle_r)         (180.0 - Clculate(180.0 - (angle_l) - (angle_r)) - (angle_r))

#define Modify_(angle_l, angle_r)        (angle_l)

#define Clculate_Angle_L_Z(Long)         (90.0 - Radian_Angle(CosineLaw_Angle(r, l, Long)))
#define Clculate_Angle_R_Z(Long)         (Radian_Angle(CosineLaw_Angle(Long, l, r) + CosineLaw_Angle(r, l, Long)) - 90.0)

#define Clculate_Angle_L_P(Long, Height) (180.0 - Radian_Angle(CosineLaw_Angle(Long, Height, sqrt(Long * Long + Height * Height)) + CosineLaw_Angle(r, sqrt(Long * Long + Height * Height), l)))
#define Clculate_Angle_R_P(Long, Height) (Radian_Angle(CosineLaw_Angle(sqrt(Long * Long + Height * Height), l, r) + CosineLaw_Angle(Long, Height, sqrt(Long * Long + Height * Height)) + CosineLaw_Angle(r, sqrt(Long * Long + Height * Height), l)) - 180.0)

#define Clculate_Angle_L_N(Long, Height) (Radian_Angle(CosineLaw_Angle(Long, sqrt(Long * Long + Height * Height), Height) - CosineLaw_Angle(r, l, sqrt(Long * Long + Height * Height))))
#define Clculate_Angle_R_N(Long, Height) (Radian_Angle(CosineLaw_Angle(sqrt(Long * Long + Height * Height), l, r) - CosineLaw_Angle(Long, sqrt(Long * Long + Height * Height), Height) + CosineLaw_Angle(r, l, sqrt(Long * Long + Height * Height))))

void clock_config(void);
void NVIC_IQR_Confing(uint8_t nvic_IRQChannel, uint8_t nvic_PreemptionPriority, uint8_t nvic_SubPriority);
void nvic_config(void);
void SMOTOR_TIM_Init(void);
void SMOTOR_Init(void);
void SET_TIM(Tim Vale, uint16_t ARR, uint16_t CCR);
void SET_Rotation(uint8_t SMOTOR, Rotation Dir);
void SMOTOR_STOP(uint8_t SMOTOR);
void SMOTOR_START(uint8_t SMOTOR);
uint8_t Get_State(uint8_t SMOTOR);
void SMOTOR_RESET(double Long, double Height, double Angleu, int8_t SMOTOR);
void SMOTOR_CONTROL(uint32_t Spead, int32_t Location, uint8_t SMOTOR);
void SMOTOR_MOVE(double Long, double Height, double Angle, double Speed);
void SMOTOR_Delta_MOVE(double Delta_Long, double Delta_Height, double Delta_Angle, double Speed);
void SMOTOR_XY_MOVE(double Location_X, double Location_Y, double Height, double Speed);
void SMOTOR_ResetLocation(uint8_t SMOTOR);
void TIM2_IRQHandler(void);
void TIM3_IRQHandler(void);
void TIM4_IRQHandler(void);
angleTypeDef SMOTOR_ANGLE(double Long, double Height, double Angle, double Speed);
CameraTypeDef SMOTOR_CAMERA_MOVE(uint8_t Times, uint16_t Delay, double Speed);

#endif
