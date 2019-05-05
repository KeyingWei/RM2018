#include "headfile.h"
#include "pid_modify.h"

int main()
{
	BSP_Init();
	Gimbal_Params_Init();
	Fire_Motor_Pidparams_init();
	Track_Motor_Pidparams_init();
	
	Dog.DbusReceive = 1;
	
	if(!RemoteSelfCheck())//上电检查遥控是否都在中档
	{
		Dog.Dbus = 0;  
	//	printf("rc data error \r\n");
	}
	else Dog.Dbus = 1;  
	while(1)
	{		
		DogFeed();
		
//		send_data(0,1);
		 
		
	//	printf("angle=%lf  control_val=%lf\r\n",YawControl.Angle,YawControl.ControlData );
		
	// 	printf("pitch = %d  yaw = %d\r\n",origion_pith_encoder_val,origion_yaw_encoder_val);
		// printf("Gx_offset:%d  Gy_offset:%d  Gz_offset:%d   gy_data:%d  angle=%f\r\n",Gx_offset,Gy_offset,Gz_offset,gy_data,mydata.imudata.Angle_Yaw);  //陀螺仪校准		
	
		//printf("  GetInputMode = %d  state=%d   position=%d  \r\n",GetInputMode(),workState,TIM2->CNT);
		
	//	printf("yaw_angle= %f  pitch_angle= %f,mode=%d\r\n",AutoData.YawAxiaAngle,AutoData.PitchAxiaAngle,AutoData.Mode);
		
	//	printf("P_angle=%f,P_speed=%f Y_angle=%f,Y_speed=%f  y_out=%d p_out=%d  dbus_r=%d\r\n",AutoData.PitchAxiaAngle,\
		mydata.imudata.Gyro_X,AutoData.YawAxiaAngle,mydata.imudata.Gyro_Z,YawControl.output,PitchControl.output,Dog.rc);
		
		//printf("%f\r\n",mydata.Cortol_ch3);
		
		// printf("Level=%d Heat = %d  bullet_speed=%f,bullet_fre=%d   \r\n",Referee_date1.GameRobotState_t.robotLevel,Referee_date1.powerHeatData.shooterHeat0 ,Referee_date1.ShootData.bulletSpeed,Referee_date1.ShootData.bulletFreq);
	//	printf("%d   %d   mydata.imudata.Gyro_Z=%f  mydata.imudata.Gyro_X =%f\r\n",workState, GetInputMode(),mydata.imudata.Gyro_Z,mydata.imudata.Gyro_X);
		//printf("%f  %f  %d   %d\r\n ",CAN1_Motor_PID[2].out,CAN1_Motor_PID[3].out,CAN1_Motor_PID[2].get,CAN1_Motor_PID[3].get);
		delay_ms(20);
	}
} 
