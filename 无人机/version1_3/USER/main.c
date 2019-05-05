#include "headfile.h"
#include "pid_modify.h"

int main()
{
	BSP_Init();
	Fire_Moter_Init();   
	
	if(!RemoteSelfCheck())//上电检查遥控是否都在中档
	{
		Dog.Dbus = 0;  
		printf("rc data error \r\n");
	}
	else Dog.Dbus = 1;  
	while(1)
	{		
		//DogFeed();
		 //  printf("Gx_offset:%d  Gy_offset:%d  Gz_offset:%d   gy_data:%d\r\n",Gx_offset,Gy_offset,Gz_offset,gy_data);  //陀螺仪校准		 
   // printf("pitch = %d  yaw = %d\r\n", origion_pith_encoder_val, origion_yaw_encoder_val);  //云台校准
			printf("ch3=%f ch2 =%f \r\n",mydata.Cortol_ch3,mydata.Cortol_ch2);//调云台运动位置极限值
		
	//	printf("o_y=%d  c_y=%d  i_y=%d out=%d\r\n",origion_yaw_encoder_val,mydata.usemotor.rm6623_y.encoder,YawControl.Encoder_Init,YawControl.output);
		
		// printf("%f   %f  %d  %d  %d  %d\r\n",power.curr_power,mydata.Cortol_ch1,limit_ch0,limit_ch1,limit_ch2,power.output); 
		//printf(" speed =%d  p_get=%ld   init_angle=%d  ch3=%f \r\n",CAN1_Motor_PID[5].get,CAN1_Motor_PID[5].position_get,CAN1_Motor_PID[5].init_position,mydata.Cortol_ch3);
		
	//Set_Gimbal_Current(CAN1,100,0,0);
		
	//	printf("%f  %f  %f\r\n",   );
		delay_ms(20); 
	}
} 
