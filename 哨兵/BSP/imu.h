#ifndef __IMU_H__
#define __IMU_H__

#include "mydata.h"

#define M_PI  (float)3.1415926535

void IMU_getValues(volatile float * values);

float invSqrt(float x);

void IMU_getYawPitchRoll(volatile float * angles, volatile float *mygetqval);
	
void IMU_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);

void GetPitchYawGxGyGz(MyData *mydata);
	
extern MyData mydata;
#endif

