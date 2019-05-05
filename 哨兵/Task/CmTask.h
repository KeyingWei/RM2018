#ifndef __CMTASK_H__
#define __CMTASK_H__

#include <stm32f4xx.h>
#include "stdbool.h"


#define CMcontrolconfig \
{\
	.Angle = 0,\
	.Control_CH0 = 0,\
	.Control_CH1 = 0,\
	.Control_CH2 = 0,\
	.Encoder_Init = 5663,\
	.Encoder = 0,\
	.time = 0,\
	.Calc = &CMControlCalc,\
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
	void (*Calc)();
}CMCONTORL; 

void CAN1_ID_1_4_Send(bool status,long int left_limit,long int right_limit,int16_t track_speed,int16_t fire_speed);
float Fire_fre_control(int16_t speed);

extern  CMCONTORL CmCotrol;
extern  volatile int16_t g_fire_speed ;
extern long int encoder_position ;
extern  long int position;
extern char Encoder_Coli_flag;
#endif
