#ifndef __KEY_H_
#define __KEY_H_
#include "sys.h"

void Key_Init(void);
void Auto_Control(void);

#define key_up 	GPIO_ReadInputDataBit(GPIOI,GPIO_Pin_5)
#define key_dowm 	GPIO_ReadInputDataBit(GPIOI,GPIO_Pin_6)
#define key_right 	GPIO_ReadInputDataBit(GPIOI,GPIO_Pin_7)
#define key_left 	GPIO_ReadInputDataBit(GPIOI,GPIO_Pin_2)

#define key_rightup 	GPIO_ReadInputDataBit(GPIOH,GPIO_Pin_10)
#define key_leftup 	GPIO_ReadInputDataBit(GPIOH,GPIO_Pin_11)
#define key_leftdowm 	GPIO_ReadInputDataBit(GPIOH,GPIO_Pin_12)
#define key_rightdowm 	GPIO_ReadInputDataBit(GPIOI,GPIO_Pin_0)

extern u8 Auto_flag;
#endif

