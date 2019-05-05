#ifndef __MY_API_H__
#define __MY_API_H__

#include <stm32f4xx.h>
#include "mydata.h"

#define PREPARE_TIME_TICK_MS 4500

typedef enum
{
   PREPARE_STATE,     		//上电后初始化状态 4s钟左右
   STANDBY_STATE,			//云台停止不转状态
   NORMAL_STATE,			//无输入状态
   STOP_STATE,        	//停止运动状态
   CALI_STATE,    			//校准状态
}WorkState_e;


#define SteeringEngine  TIM1->CCR1  //PA8
#define ShootMotor_Left   TIM12->CCR1  //PH6
#define ShootMotor_Right  TIM12->CCR2	//PH9


#define CH0_Speed 1.0f
#define CH1_Speed 1.0f
#define CH2_Speed 0.5f
#define CH3_Speed 0.01f

#define Gimbal_Init 0
#define Gimbal_Init_output 800
#define Gimbal_work_output 4500

#define Pitch_Position_kp 8.5f
#define Pitch_Velocity_kp  18.0f

#define Yaw_Position_kp 5.3f
#define Yaw_Velocity_kp  13.0f

#define Pitch_Encoder_Init 3000   
#define Yaw_Encoder_Init  6841

#define SetFireMotorSpeed 2000
#define Fire_kp 0.05f

#define Pitch_Move_Up    500
#define Pitch_Move_Dowm -500



void GimbalMotorOutput(void);
WorkState_e GetWorkState(void);

void Track_Motor_Pidparams_init(void);
void Fire_Motor_Pidparams_init(void);
void Gimbal_Params_Init(void);

extern WorkState_e workState;
extern int time_1ms;
extern u8 start;
extern volatile unsigned int fire_fre ;

#endif


