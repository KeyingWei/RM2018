#ifndef __MY_API_H__
#define __MY_API_H__

#include <stm32f4xx.h>
#include "mydata.h"
#include "cmtask.h"


typedef enum
{
   PREPARE_STATE,     		//上电后初始化状态 4s钟左右
   STANDBY_STATE,			//云台停止不转状态
   NORMAL_STATE,			//无输入状态
   STOP_STATE,        	//停止运动状态
   CALI_STATE,    			//校准状态
}WorkState_e;


#define CAMERA_LR TIM1->CCR1  //PA8
#define CAMERA_UD TIM1->CCR2   //PA9
#define LOCKCAR   TIM8->CCR4	 //PI2
#define DANCANG_L TIM8->CCR2  //PI6
#define DANCANG_R TIM8->CCR3  //PI7


#define CH0_Speed 1.0f
#define CH1_Speed 1.0f
#define CH2_Speed 0.5f
#define CH3_Speed 0.01f

#define Gimbal_Init 0
#define Gimbal_Init_output 800
#define Gimbal_work_output 4500

#define Pitch_Position_kp 7.0f
#define Pitch_Velocity_kp  35.0f

#define Yaw_Position_kp 8.3f
#define Yaw_Velocity_kp  40.0f

#define Pitch_Encoder_Init  2120  
#define Yaw_Encoder_Init   5698

#define SetFireMotorSpeed 2000
#define Fire_kp 0.05f

#define Pitch_Move_Up    500
#define Pitch_Move_Dowm -500



void GimbalMotorOutput(void);
void CMControlLoop(void);
WorkState_e GetWorkState(void);


extern WorkState_e workState;
extern int time_1ms;
extern u8 start;
extern volatile unsigned int fire_fre ;
extern  CMCONTORL CmCotrol;

#endif


