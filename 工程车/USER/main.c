#include "headfile.h"
#include "pid_modify.h"

int main()
{
	
	CM_Up_Down_Motor_Pidparams_init(); 
	Motor_Position_Params_init();
	
	BSP_Init();
	
	if(!RemoteSelfCheck())//上电检查遥控是否都在中档
	{
		Dog.Dbus = 0;  
		printf("rc data error \r\n");
	}
	else Dog.Dbus = 1;  
	while(1)
	{		
	//	DogFeed();		
	//	delay_ms(20);
	// CAN2_Set_CM_UpDown(500,0,0,0);
	
		printf("get = %ld  set = %ld out =%d get = %ld  set = %ld out =%d  init_angle5:%ld init_angle4 :%ld\r\n",CAN2_Moter_Position_PID[6].ange_get,\
	       	CAN2_Moter_Position_PID[6].ange_set,CAN2_Moter_Position_PID[6].out,\
		      CAN2_Moter_Position_PID[4].ange_get,CAN2_Moter_Position_PID[4].ange_set,CAN2_Moter_Position_PID[4].out,CAN2_Moter_Position_PID[5].init_angle,CAN2_Moter_Position_PID[4].init_angle);
		//printf("%ld  %ld  %ld  201speed=%ld  catch_state=%d CAN2_Moter_Position_PID[0].ange_set=%ld  %f\r\n",CAN1_Moter_Position_PID[5].init_angle,CAN1_Moter_Position_PID[5].ange_get,CAN1_Moter_Position_PID[5].ange_set,CAN2_Moter_Position_PID[0].ange_set,watch_val1,CAN2_Moter_Position_PID[0].ange_set,8192*14-8192*6.5);
	//	printf("g_lang_flag:%d   lang_state=%d ch0=%f  ch1=%f ch2=%f  angle=%f  1_s=%ld 2_s=%ld 3_s=%ld 4_s=%ld  %ld\r\n",g_Lang_flag,state,  CmCotrol.Control_CH0, CmCotrol.Control_CH1,CmCotrol.Control_CH2,mydata.imudata.Angle_Yaw ,CAN2_Moter_Position_PID[0].ange_get,CAN2_Moter_Position_PID[1].ange_get,CAN2_Moter_Position_PID[2].ange_get,CAN2_Moter_Position_PID[3].ange_get,8192*14);	//	printf(" mydata.dbus.key.v=%d  Key_event=%d    Key_state =%d\r\n",mydata.dbus.key.v,P_Key_ide1[6]->Key_Event,P_Key_ide1[6]->Key_State);
	//	  printf("%f  %d  %d  %d  %d\r\n",mydata.imudata.Angle_Yaw,Gx_offset,Gy_offset,Gz_offset,gy_data);
	//	printf("speed=%d  angle_get:%ld   agnle_set%ld  mydata.dbus.key.v=%d\r\n" ,CAN1_Moter_Position_PID[6].speed_get ,CAN1_Moter_Position_PID[6].ange_get,CAN1_Moter_Position_PID[6].ange_set,mydata.dbus.key.v);
	//	printf("state =%d    g_Lang_flag=%d   mydata.dbus.key.v =%d  catch_state=%d  angle_set=%ld  get=%ld\r\n",state,g_Lang_flag,mydata.dbus.key.v ,watch_val1,CAN1_Moter_Position_PID[5].ange_set,CAN1_Moter_Position_PID[5].ange_get);
	//	printf("CAN2_201speed=%d  CAN2_201init_angle=%ld  CAN2_201angle=%ld   CAN2_201angle_set=%ld   out=%d state=%d\r\n",CAN2_Moter_Position_PID[0].speed_get,	CAN2_Moter_Position_PID[0].init_angle,CAN2_Moter_Position_PID[0].ange_get,CAN2_Moter_Position_PID[0].ange_set,CAN2_Moter_Position_PID[0].out,state);
	}
} 
