#ifndef __TIMER_H__
#define __TIMER_H__

#include <stm32f4xx.h>
#include "cmTask.h"

void TimConfig(void);
uint32_t Get_Time_Micros(void);

extern p_Key_ide  P_Key_ide1[8];

#endif
