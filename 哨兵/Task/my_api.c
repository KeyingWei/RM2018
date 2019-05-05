#include "headfile.h"
#include <math.h>
#include "pid_modify.h"

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

void Gimbal_Params_Init()
{
	 	YawControl.Encoder_Init = 3895;//获取初始码盘值
	  YawControl.Position_kp = 8.2f;//设定位置环控制参数
	  YawControl.Velocity_kp = 45.0f;//设定速度环控制参数
	
	  PitchControl.Encoder_Init = 6620        ;	
	  PitchControl.Position_kp = 9.0f;
	  PitchControl.Velocity_kp = 4.4f;	
}

void Fire_Motor_Pidparams_init()
{
   int i =0;	
 for(i=0;i<2;i++)
	{
	  PID_struct_init(&CAN1_Motor_PID[i],DELTA_PID,7000,500,9,0.9,0); 
	}
	
	  PID_struct_init(&CAN1_Motor_PID[6],DELTA_PID,5000,500,1,0.01,0); 	
}

/***************************************************
函数名:YawControlLoop
入口参数：无
出口参数：无
功能： 获取Y轴电机码盘值和陀螺仪Z轴角速度，Z轴积分绕轴角
度值后进行串级控制
**************************************************/
void YawControlLoop(void)
{
	YawControl.Angle = mydata.imudata.Angle_Yaw; //获取绕轴角度
	YawControl.Gyro = mydata.imudata.Gyro_Z;//获取绕轴角速度
	
	static  char  flag = 0;
	  
	
	
	YawControl.Encoder = mydata.usemotor.rm6623_y.encoder;//获取Y轴码盘值
	
	if(Dog.rc == 1)		
	{
	    YawControl.ControlData = mydata.Cortol_ch2;//获取绕轴控制值     
	}
	else
	{
		  if(flag == 0)
			{
			  YawControl.ControlData += 0.02f;
				if(YawControl.ControlData >= 500)
				{
				  flag= 1;
				}
			}
			else
			{
				 YawControl.ControlData -= 0.02f;
					if(YawControl.ControlData <= -500)
				{
				  flag= 0;
				}			
			}
}
	YawControl.Calc(&YawControl);//云台Y轴控制
}
/***************************************************
函数名:PitchControlLoop
入口参数：无
出口参数：无
功能： 获取P轴电机码盘值和陀螺仪X轴角速度
度值后进行串级控制
**************************************************/
void PitchControlLoop(void)
{
	PitchControl.Gyro = mydata.imudata.Gyro_X;
	
	PitchControl.ControlData  = mydata.Cortol_ch3;
	PitchControl.Encoder      = mydata.usemotor.rm6623_p.encoder;

	PitchControl.Calc(&PitchControl);
}

void WorkStateFSM(void);

u8 debug_mode = 0;
int time_1ms = 0;
volatile unsigned int fire_fre = 0;
 /***************************************************
函数名:GimbalMotorOutput
入口参数：无
出口参数：无
功能： 控制轨道运动和云台控制
**************************************************/
void GimbalMotorOutput(void)
{
	if(debug_mode == 0)
	{
		
	  WorkStateFSM();
	//	YawControlLoop();
	//	PitchControlLoop();
		
		if(workState == STOP_STATE )
		{	
 
       CAN1_Send_Motor_data(CAN1,0,0,0,0);
		   Set_Gimbal_Current(CAN1, 0 , 0, 0);				
			  
		}
		else
		{
			
			 Heat2SetFire();
			
			 if(AutoData.Mode == 1)
			 {
			    CAN1_ID_1_4_Send(0 ,-6200,6200,5000,g_fire_speed);			  
			 }
			 else
			 {		  
						 if(Dog.rc == 1)
							{					 
									 CAN1_Motor_PID[2].set =   mydata.Cortol_ch1; 	
									 CAN1_Motor_PID[3].set =  -mydata.Cortol_ch1; 
								
									 pid_calc(&CAN1_Motor_PID[2] );
									 pid_calc(&CAN1_Motor_PID[3] );
								
								
									CAN1_Send_Motor_data(CAN1,CAN1_Motor_PID[0].out,CAN1_Motor_PID[1].out,CAN1_Motor_PID[2].out,CAN1_Motor_PID[3].out); 
										 
							}
							else
							{
								     
									 CAN1_ID_1_4_Send(1 ,-6200,6200,5000,g_fire_speed);
							}						
			  }
		//		 Set_Gimbal_Current(CAN1, YawControl.output , PitchControl.output, CAN1_Motor_PID[6].out);				 
		}

	}		
}



void WorkStateFSM(void)
{
//    lastWorkState =   workState;
//	  static char cali_success_flag = 0;
//    static char cnt = 0,cnt1 = 0;
	 switch(workState) 
	 {
		 case PREPARE_STATE:
			   time_1ms ++;
			   if(GetInputMode() == STOP)
				 {
				     workState =  STOP_STATE;
				 
				 }
				 else if(time_1ms > PREPARE_TIME_TICK_MS)
				 {
					  workState =  NORMAL_STATE;
            time_1ms = 0;				 
				 }
			 break;
		 case STANDBY_STATE:
			  if(GetInputMode() == STOP)
				 {
				     workState =  STOP_STATE;			 
				 }
			 
		 break;
		 case  NORMAL_STATE:
			 time_1ms = 0 ;
			 if(GetInputMode() == STOP)
				 {
				     workState =  STOP_STATE;
				 }
			 break;
		 
		 case STOP_STATE:
		       time_1ms = 0 ;
			     if(GetInputMode() != STOP)
					 {
					     workState =  PREPARE_STATE;  
						  //   workState = CALI_STATE;						   
					 }
			 break;
		 case CALI_STATE:
  /*          do{
						  	MPU6500_InitGyro_offset();
							  delay_ms(10);
							  cnt1++;
                if(gy_data > -3 && gy_data  < 3)
								{
								   cnt++;
								   cali_success_flag = 0;
									 if(cnt >=6)
									 {
									    cnt = 0;
										  cali_success_flag = 1;
									 }										 
								}						     							  
						  }			 
          while(cali_success_flag == 0 && cnt1 < 100);
					workState =  PREPARE_STATE; 
          time_1ms = 0;								
					cnt1 = 0; */
			 break;
		 default:break;
	 }
	
}



