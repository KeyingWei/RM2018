#include "headfile.h"
#include "pid_modify.h"
 #include "Auto_attackTask.h"

int main()
{
	Gimbal_Params_Init();
	BSP_Init();
	Fire_Motor_Pidparams_init(); 

	
	if(!RemoteSelfCheck())//上电检查遥控是否都在中档
	{
		Dog.Dbus = 0;  
	//	printf("rc data error \r\n");
	}
	else Dog.Dbus = 1;  
	while(1)
	{   		
     DogFeed();  
   
//	 send_data(send_data_mode, Get_angle());	
		
   //		printf("o_y=%d  r_y=%d  i_y=%d  cnt=%d\r\n",origion_yaw_encoder_val,mydata.usemotor.rm6623_y.encoder,YawControl.Encoder_Init,watch);
	 //  	printf("o_p=%d  r_y=%d  i_p=%d  cnt1=%d\r\n",origion_pith_encoder_val,mydata.usemotor.rm6623_p.encoder,	PitchControl.Encoder_Init,watch);
		
//	printf("yaw_angle= %f  pitch_angle= %f,mode=%d\r\n",AutoData.YawAxiaAngle,AutoData.PitchAxiaAngle,AutoData.Mode);
//		printf("P_angle=%f,P_speed=%f Y_angle=%f,Y_speed=%f  y_out=%d p_out=%d\r\n",AutoData.PitchAxiaAngle,mydata.imudata.Gyro_X,AutoData.YawAxiaAngle,mydata.imudata.Gyro_Z,YawControl.output,PitchControl.output);
		
//    printf("Gx_offset:%d  Gy_offset:%d  Gz_offset:%d   gy_data:%d  angle=%f\r\n",Gx_offset,Gy_offset,Gz_offset,gy_data,mydata.imudata.Angle_Yaw);  //陀螺仪校准		
     printf("pitch = %d  yaw = %d\r\n", origion_pith_encoder_val, origion_yaw_encoder_val);  //云台校准
		
	//	printf();
		
	//	printf("state=%d \r\n",GetWorkState());
		
	//	printf("%d  %d  %d %d  %d\r\n",mydata.dbus.remote.ch0,mydata.dbus.remote.ch1,mydata.dbus.remote.ch2,mydata.dbus.remote.ch3,Control_Mode);
	//	printf("%f\r\n",mydata.Cortol_ch3); //调云台上下限位
	//	printf("set=%f,get=%d,origion=%d   out=%d\r\n",mydata.Cortol_ch3,mydata.usemotor.rm6623_p.encoder,origion_pith_encoder_val,PitchControl.output);
	//	printf("%d  %f   %f  %d \r\n",time_1ms,PitchControl.ControlData,mydata.imudata.Gyro_X,GetWorkState());
		// printf("%f   %f  %d  %d  %d  %d\r\n",power.curr_power,mydata.Cortol_ch1,limit_ch0,limit_ch1,limit_ch2,power.output);  
		//	 printf("Level=%d Heat = %d  bullet_speed=%f,bullet_fre=%d  Moter_Out= %f   Dog.rc=%d\r\n",\
	           	Referee_date1.GameRobotState_t.robotLevel,Referee_date1.powerHeatData.shooterHeat0 ,Referee_date1.ShootData.bulletSpeed,Referee_date1.ShootData.bulletFreq,CAN1_Motor_PID[6].out,Dog.rc);
		
	//	printf("%d   %d   %d  %f\r\n",mydata.dbus.key.v , mydata.dbus.mouse.x,mydata.dbus.mouse.y,mydata.Cortol_ch3);		
	//	delay_ms(20); 
	}
} 
