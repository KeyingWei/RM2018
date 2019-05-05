#ifndef __CAN2_H__
#define __CAN2_H__

#include <stm32f4xx.h>

void CAN2Config(void);
void CAN2_SendCommand(int16_t Cur_201,int16_t Cur_202,int16_t Cur_203,int16_t Cur_204);
void  CAN2_Send_Msg(int16_t control_201,int16_t control_202);

#endif
