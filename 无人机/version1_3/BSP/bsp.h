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
}\
	
typedef struct BSP
{
	void (*OpenTheGreenLed)(bool status);
	void (*OpenTheRedLed)(bool status);
	void (*OpenTheLaser)(bool status);
	void (*TurnOnTheShootMotor)(bool status,int speed);
}BSP;
	
extern BSP Bsp;
void OpenTheGreenLed(bool status);
void OpenTheRedLed(bool status);
void OpenTheLaser(bool status);
void TurnOnTheShootMotor(bool status,int speed);
void BSP_Init(void);
#endif 

