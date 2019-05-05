#ifndef __CMTASK_H__
#define __CMTASK_H__

#include <stm32f4xx.h>


#define CMcontrolconfig \
{\
	.Angle = 0,\
	.Control_CH0 = 0,\
	.Control_CH1 = 0,\
	.Control_CH2 = 0,\
	.Encoder_Init = 5663,\
	.Encoder = 0,\
	.time = 0,\
}\

typedef struct _CMCONTORL
{
	volatile float Angle;
	volatile	float Control_CH0;
	volatile	float Control_CH1;
	volatile	float Control_CH2;
	volatile int16_t Encoder_Init;
	volatile int16_t Encoder;
	int time;
}CMCONTORL;


typedef struct _POSITION_PID
{
  float position_kp ;
	float velocity_kp;
	long int ange_get;
	long int ange_set;
	long int  init_angle;
	long int lang_init_angle;
	int16_t  encoder_val_new_data;
	int16_t  encoder_val_old_data;
	long int encoder_count;
	int16_t speed_get;
	int16_t out;
}position_pid ,*p_position_pid;


typedef enum{
     FourWheelUp_SATA, //0
	   GoForward_SATA,   //1
	   LeftAdjust_SATA,  //2
	   RightAdjust_SATA, //3
	   WairtDown_SATA,  //4
	   HandWheelDown_SATA, //5
	   FrontWheelUp_SATA, //6
	   BackWheelUp_SATA,	//7   
}Lang_state;



typedef enum{
    KEY_UP = 0,
	  KEY_DOWN ,
	  KEY_HOLE ,
	  KEY_CLICK,
	  KEY_ONE,
	  KEY_TWO,
	  KEY_LONG
}key_state;

typedef enum{
	  KEY_NO_CLICK = 0 ,
    KEY_ONE_CLICK    ,
	  KEY_DOUBLE_CLICK ,
	  KEY_LONG_CLICK   ,
}key_event;

typedef struct KEY_
{
  key_event Key_Event;
	key_state Key_State;
	int16_t cnt;
}Key_ide,*p_Key_ide;

void CMControlCalc(void);
void CM_Up_Down_Motor_Pidparams_init(void);
void Motor_Position_Params_init(void);

void CM_test(void);
void catcher_control(float set_angle,long int secont_p,long int limit_h);
void bullet_control(long int position);
void Go_Island(int16_t Up_Down_speed,int16_t CM_speed,long int maxP);
float Moter_position_pid(p_position_pid position);
float catch_position_pid(void);
void Front_Wheel_Down_Back_Up(long int pisition,int16_t speed,long int back_pisition);
void Four_Wheel_Down(long int pisition ,int16_t speed);
void Fornt_Wheel_Up(int16_t speed,long int position);
void Back_Wheel_Down(long int pisition,int16_t speed);
void Back_Wheel_Up(int16_t speed,long int position);
void Fornt_Wheel_Down(long int pisiton,int16_t speed);

void CM_U_D_A_Catch_Control(long int pisition,int16_t speed);
void Camera_Control(int16_t speed);


void Key_Scan(uint16_t Key_Value,p_Key_ide key_id);

extern int   limit_ch1  ;
extern int   limit_ch0 ;
extern int   limit_ch2 ;
extern position_pid CAN1_Moter_Position_PID[7];
extern position_pid CAN2_Moter_Position_PID[7];
extern Lang_state state ;
extern char g_Lang_flag;

extern key_event Key_event;
extern key_state Key_state ;
extern int watch_val1;
#endif
