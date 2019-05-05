#include "headfile.h"
#include <math.h>
#include "pid_modify.h"

CMCONTORL CmCotrol = CMcontrolconfig;//底盘控制参数赋值
GIMBALCONTORL YawControl = yawcontrolconfig; //云台Y轴控制参数赋值
GIMBALCONTORL PitchControl = pitchcontrolconfig;//云台P轴控制参数赋值

volatile int16_t g_fire_speed = 0; 

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
	
	CM_Motor_Pidparams_init();
	
	CmCotrol.Control_CH0 = mydata.Cortol_ch0;//控制底盘前后移动
	CmCotrol.Control_CH1 = mydata.Cortol_ch1;//控制底盘左右横移
	CmCotrol.Encoder = mydata.usemotor.rm6623_y.encoder;//获取Y轴码盘值（经过连续化处理），用于计算角度差值，实现底盘跟随
	CmCotrol.Encoder_Init = Yaw_Encoder_Init;//获取云台初始化位置
	CmCotrol.Calc(); 
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
	
	YawControl.ControlData = mydata.Cortol_ch2;//获取绕轴控制值
	
	YawControl.Encoder = mydata.usemotor.rm6623_y.encoder;//获取Y轴码盘值
	YawControl.Encoder_Init = 2500;//获取初始码盘值
	
	YawControl.Position_kp = Yaw_Position_kp;//设定位置环控制参数
	YawControl.Velocity_kp = Yaw_Velocity_kp;//设定速度环控制参数
	
	
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
	PitchControl.Encoder_Init = Pitch_Encoder_Init;
	
	PitchControl.Position_kp = Pitch_Position_kp;
	PitchControl.Velocity_kp = Pitch_Velocity_kp;
	
	PitchControl.Calc(&PitchControl);
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
   time_1ms++;
		if(time_1ms >=4500)
		{
		  time_1ms = 4500;
			workState = STANDBY_STATE;
		}
		
		YawControlLoop();
		
		if(GetInputMode() == STOP)
		{
		    Set_Gimbal_Current(CAN1, 0 , 0, 0);
		}
		else
		{
		  Set_Gimbal_Current(CAN1, YawControl.output , (int16_t)PitchControl_Position(), CAN1_Motor_PID[6].out);
			//  Set_Gimbal_Current(CAN1,0 , (int16_t)PitchControl_Position(), CAN1_Motor_PID[6].out);
		}
				
		
		

	}		
}

