#ifndef __PID_H
#define __PID_H

#include "stm32f10x.h" // Device header
#include "Encoder.h"
#include "OpenCV.h"
#include "Delay.h"
#include "HWT101CT.h"
#include "Buzzer.h"


#define MOTOR_NULL  ((uint8_t)0x00)
#define MOTOR_Left  ((uint8_t)0x01)
#define MOTOR_Right ((uint8_t)0x02)

#ifdef M_MOTOR_
#define MOTOR_Left2  ((uint8_t)0x04)
#define MOTOR_Right2 ((uint8_t)0x08)
#endif


#define MOTOR_MAX   150

typedef enum {
    Forward  = 1,
    Backward = 0
} Rotation;

typedef enum {
    Left  = 1,
    Right = 0
} Direction;


typedef struct
{
    int32_t e_l, last_e_l, target_l, current_l;
    int32_t e_s, last_e_s, target_s, current_s;
    double KP_L, KI_L, KD_L;
    double KP_S, KI_S, KD_S;
    int32_t p_l, i_l, d_l, total_l;
    int32_t p_s, i_s, d_s, total_s;
    int32_t TOTAL_OUT;

} PID;


void PID_Inint(PID* PID);
uint8_t MOTOR_CONTROL(int32_t Spead,int32_t Location,uint16_t MOTOR);
void PID_LocationSet(PID* PID,int32_t Location);
void PID_SpeadSet(PID* PID,int32_t Spead);
void PID_calc_all(void);
void PID_calc(PID* PID);
void PID_Init(void);
void PWM_TIM1_Init(void);
void MOTOR_Rotation_Init(void);
void MOTOR_Rotation(Rotation Vale,uint16_t MOTOR);
void MOTOR_Set(uint16_t Spead,Rotation Vale,uint16_t MOTOR);
void MOTOR_Spead_calc(void);
uint16_t MOTOR_Spead_Get(uint8_t MOTOR);
void MOTOR_Run_all(void);
void MOTOR_Clear(uint16_t MOTOR);
void MOTOR_Stop(uint16_t MOTOR,int32_t Spead,int32_t Distance);
void MOTOR_Spin(Direction Vale,int32_t Angle,int32_t Spead);


#endif
