#ifndef __BSP_H__
#define __BSP_H__

#include "headfile.h"
#include "stdbool.h"

#define BSPConfig \
{\
	.OpenTheGreenLed = &OpenTheGreenLed,\
	.OpenTheRedLed = &OpenTheRedLed,\
	.OpenTheLaser = &OpenTheLaser,\
	.TurnOnTheShootMotor = &TurnOnTheShootMotor,\
	.OpenTheCatcher_ = &OpenTheCatcher,\
	.bullet_control_ = &OpenTheMagazine,\
	.OpenTheBall_=&OpenTheBall,\
	.LangSmallWheel =&LangSmallWheel,\
}\
	
typedef struct BSP
{
	void (*OpenTheGreenLed)(bool status);
	void (*OpenTheRedLed)(bool status);
	void (*OpenTheLaser)(bool status);
	void (*TurnOnTheShootMotor)(bool status);
	void (*OpenTheCatcher_)(bool status);
	void(*bullet_control_)(bool status);
	void(*OpenTheBall_)(bool status);
	void (*LangSmallWheel)(char status, int position);
}BSP;
	
extern BSP Bsp;
extern char init_flag;
extern int16_t g_smallwheel_speed ;
void OpenTheCatcher(bool status);
void OpenTheGreenLed(bool status);
void OpenTheRedLed(bool status);
void OpenTheLaser(bool status);
void TurnOnTheShootMotor(bool status);
void OpenTheMagazine(bool status);
void OpenTheBall(bool status);
void LangSmallWheel(char status, int position);
void BSP_Init(void);
#endif 

