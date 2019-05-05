#include "headfile.h"
#include <math.h>
#include "pid_modify.h"


CMCONTORL CmCotrol = CMcontrolconfig;//底盘控制参数赋值 
GIMBALCONTORL YawControl = yawcontrolconfig; //云台Y轴控制参数赋值
GIMBALCONTORL PitchControl = pitchcontrolconfig;//云台P轴控制参数赋值

WorkState_e workState = PREPARE_STATE;//上电初始化状态，4-5s左右

WorkState_e GetWorkState(void)//获取工作状态
{
		return workState;
}

void WorkState(void)
{

}
/***************************************************
函数名:CMControlLoop
入口参数：无
出口参数：无
功能： 获取遥控参数值和云台Y轴码盘值，更新底盘控制参数
**************************************************/
void CMControlLoop(void)
{	
	CmCotrol.Control_CH0 = mydata.Cortol_ch0;//控制底盘前后移动
	CmCotrol.Control_CH1 = mydata.Cortol_ch1;//控制底盘左右横移

	
//	if(mydata.Cortol_ch2 != 0)
//	{
//	   mydata.imudata.Angle_Yaw = 0;  
		 CmCotrol.Control_CH2 = mydata.Cortol_ch2;
//	}
//	else
//	{
//	   CmCotrol.Control_CH2 = mydata.imudata.Angle_Yaw * 100;
//	}
}


u8 start =	1;
int time_1ms = 0;
volatile unsigned int fire_fre = 0;
 /***************************************************
函数名:GimbalMotorOutput
入口参数：无
出口参数：无
功能： 根据状态控制云台电机，当遥控值无法接收或模块离线时，云
台电机电流清零
**************************************************/
void GimbalMotorOutput(void)
{

	if(start == 1)
	{
		time_1ms ++;
		
	//	CAN1_Moter_Position_PID[5].ange_set -= 50;//负值向右，正值向左
	
	
		bullet_control(1800);  //控制放弹动作
	 // Camera_Control(100);  //控制摄像头运动
		
		if(time_1ms >=5500)
		{
		  time_1ms = 5500;
			LOCKCAR = 13;
		}
		else
		{
		  	LOCKCAR = 25;
		}
				
   if(SPACE_LIMIT7 == 0)
	 {
	     Bsp.OpenTheRedLed (1); 
	 }
	 else 
	 {
	   Bsp.OpenTheRedLed(0); 
	 }
		
   if(SPACE_LIMIT3 == 0)
	 {
	     Bsp.OpenTheGreenLed (0); 
	 }
	 else 
	 {
	   Bsp.OpenTheGreenLed(1); 
	 }	 
	 
		
		 if(GetInputMode() == STOP)
		 {
					CAN1_Moter_5_7_Send(CAN1,0,0,0);
					
					CAN1_Moter_1_4_Send(CAN1,0,0,0,0);
						
					CAN2_Moter_1_4_Send(0,0,0,0);		    
		 }
		 else
		 {
			 
				//	CAN1_Moter_5_7_Send(CAN1,catch_position_pid(),Moter_position_pid(&CAN1_Moter_Position_PID[5]),Moter_position_pid(&CAN1_Moter_Position_PID[6]));
				   CAN1_Moter_5_7_Send(CAN1,catch_position_pid(),CAN1_Motor_PID[5].out,Moter_position_pid(&CAN1_Moter_Position_PID[6]));	
			 
					CAN1_Moter_1_4_Send(CAN1,CAN1_Motor_PID[0].out, CAN1_Motor_PID[1].out, CAN1_Motor_PID[2].out, CAN1_Motor_PID[3].out);
						
					CAN2_Moter_1_4_Send(CAN2_Moter_Position_PID[0].out	,CAN2_Moter_Position_PID[1].out	,CAN2_Moter_Position_PID[2].out	,CAN2_Moter_Position_PID[3].out	);	

      //    CAN2_Moter_5_7_send(g_smallwheel_speed,-g_smallwheel_speed,0,0); 
		   CAN2_Moter_5_7_send( -Moter_position_pid(&CAN2_Moter_Position_PID[4]),-Moter_position_pid(&CAN2_Moter_Position_PID[5]),Moter_position_pid(&CAN2_Moter_Position_PID[6]),0);
		 }
 //   CAN2_Moter_5_7_send( CAN2_Moter_Position_PID[4].out, 0, 0, 0);		
	 // CAN2_Set_CM_UpDown(CAN2_Motor_PID[0].out,CAN2_Motor_PID[1].out,CAN2_Motor_PID[2].out,CAN2_Motor_PID[3].out);	 
		
	}		 
}





