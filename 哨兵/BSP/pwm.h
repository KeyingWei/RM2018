#ifndef __PWM_H__
#define __PWM_H__

void PwmConfig(void);

#define UNDERCARRIAGE_PWM   TIM1->CCR1 

#define UP   20;
#define DOWN 8

#endif

