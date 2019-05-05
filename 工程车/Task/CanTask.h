#ifndef __CANTASK_H__
#define __CANTASK_H__
#include <stm32f4xx.h>

#define CAN_BUS1_MOTOR1           0x201
#define CAN_BUS1_MOTOR2           0x202 
#define CAN_BUS1_MOTOR3           0x203
#define CAN_BUS1_MOTOR4           0x204
#define CAN_BUS1_MOTOR5           0x205
#define CAN_BUS1_MOTOR6           0x206
#define CAN_BUS1_MOTOR7           0x207

#define CAN_BUS2_MOTOR1           0x201
#define CAN_BUS2_MOTOR2           0x202 
#define CAN_BUS2_MOTOR3           0x203
#define CAN_BUS2_MOTOR4           0x204 
#define CAN_BUS2_MOTOR5           0x205 
#define CAN_BUS2_MOTOR6           0x206
#define CAN_BUS2_MOTOR7           0x207 


void CAN1_Moter_5_7_Send(CAN_TypeDef *CANx, int16_t gimbal_yaw_iq, int16_t gimbal_pitch_iq, int16_t gimbal_fire_iq);
void CAN1_Moter_1_4_Send(CAN_TypeDef *CANx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq);
void CAN2_Moter_1_4_Send( int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq);
void CAN2_Moter_5_7_send( int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq);

extern volatile int CAN1_Motor[8];
extern int16_t CAN2_Motor[8];
extern  int16_t  encoder_init_val ;
extern long int catch_encoder_value;
#endif



