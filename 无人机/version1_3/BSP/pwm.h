#ifndef __PWM_H__
#define __PWM_H__

void PwmConfig(void);

#define UNDERCARRIAGE_PWM   TIM1->CCR1 

#define UP   10;
#define DOWN 20

#endif

