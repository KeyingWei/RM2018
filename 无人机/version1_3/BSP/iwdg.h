#ifndef __IWDG_H__
#define __IWDG_H__

#include <stm32f4xx.h>
void IWDG_Init(void);
void DogFeed(void);
void GetLostError(u8 code);
void check(uint32_t * cout);
typedef struct
{
	u8 DbusReceive;
	u8 Dbus;
	u8 Imu;
	u8 Reset;
	u8 motor1;
	u8 motor2;
	u8 motor3;
	u8 motor4;
	u8 motor5;
	u8 motor6;
	u8 motor7;
	u8 rc;
}DOG;


#define DogConfig \
{\
	.DbusReceive = 0,\
	.Dbus = 1,\
	.Imu = 1,\
	.Reset = 1,\
	.motor1 = 0,\
	.motor2 = 0,\
	.motor3 = 0,\
	.motor4 = 0,\
	.motor5 = 0,\
	.motor6 = 0,\
	.motor7 = 0,\
	.rc = 0,\
}\


#define LOST_ERROR_RC									(1<<0)		//rc lost 
#define LOST_ERROR_IMU									(1<<1)		//mpu6050 error
#define LOST_ERROR_ZGYRO								(1<<2)		//can1 zyro error
#define LOST_ERROR_MOTOR1								(1<<3)		//motor1 error
#define LOST_ERROR_MOTOR2								(1<<4)		//
#define LOST_ERROR_MOTOR3								(1<<5)		//
#define LOST_ERROR_MOTOR4								(1<<6)		//
#define LOST_ERROR_MOTOR5								(1<<7)		//
#define LOST_ERROR_MOTOR6								(1<<8)		//
#define LOST_ERROR_DEADLOCK								(1<<9)		//deadlock error
#define LOST_ERROR_NOCALI  						        (1<<10)		//nocali error


#define LOST_COUNTER_INDEX_MOTOR1                    0u    //green:red:green 1 1 1 
#define LOST_COUNTER_INDEX_MOTOR2                    1u    //green:red:green 1 2 1 
#define LOST_COUNTER_INDEX_MOTOR3                    2u    //green:red:green 1 3 1 
#define LOST_COUNTER_INDEX_MOTOR4                    3u    //green:red:green 1 4 1 
#define LOST_COUNTER_INDEX_MOTOR5                    4u    //green:red:green 1 5 1 
#define LOST_COUNTER_INDEX_MOTOR6                    5u    //green:red:green 1 6 1 
#define LOST_COUNTER_INDEX_MOTOR7                    6u    //green:red:green 1 6 1 
#define LOST_COUNTER_INDEX_RC                    		 7u    //green:red:green 1 6 1 
#define LOST_COUNTER_INDEX_MOTOR8                    8u    //green:red:green 1 1 1 
#define LOST_COUNTER_INDEX_MOTOR9                    9u    //green:red:green 1 2 1 
#define LOST_COUNTER_INDEX_MOTOR10                   10u    //green:red:green 1 2 1 
#define LOST_COUNTER_INDEX_MOTOR11                   11u    //green:red:green 1 2 1 
#define LOST_COUNTER_INDEX_MOTOR12                   12u    //green:red:green 1 2 1 
#define LOST_COUNTER_INDEX_MOTOR13                   13u    //green:red:green 1 2 1 

#define Online 0

#define Lose 0
#define OL 1


extern DOG Dog;
extern uint32_t global_cout[12];
#endif
