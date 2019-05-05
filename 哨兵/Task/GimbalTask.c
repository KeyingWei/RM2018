#include "headfile.h"
#include "pid_modify.h"


u8 GimbalInit = 0;
/*************************************************************************
函数名：YawControlLoop_Init
入口参数：
         GIMBALCONTORL *YawControl：传入Y轴的控制结构体指针，得到控制参数
				 
出口参数：Y轴速度环限流后的输出
功能：上电云台Y轴归中控制
说明：云台采用串级控制，有位置环（控制角度），速度环（控制速度），电流环（控制电流）
位置环相当于队长，指挥调节速度环，速度环相当于组长， 指挥调节电流环，层层指导，
Y轴编码器提供位置反馈，MPU6500提供角速度反馈，形成双环PID控制，电流环由驱动板控制。
云台的稳定性由结构本身确定，可通过调节PID参数提高稳定性，步兵采用纯比例控制
*************************************************************************/
float YawControlLoop_Init(GIMBALCONTORL *YawControl) //计算
{
	float PositionOutPut = 0.0f, VelocityOutPut = 0.0f, yawCal = 0.0f;
	YawControl->Limit += 1;//让电流值递增，防止归中时电流过大
	if(YawControl->Limit > 800)		YawControl->Limit = 800;

	PositionOutPut = 	- (
												(
													YawControl->Encoder //现在的机械角度（已进行连续化处理）
													- YawControl->Encoder_Init  //初始化的机械角度
												) 
												* 360 / 8192 //角度转换
											) 
											* YawControl->Position_kp;//Y轴位置环，注意符号的选择，应使误差向着减小的方向
		
	VelocityOutPut = -(PositionOutPut + YawControl->Gyro) * YawControl->Velocity_kp;//Y轴速度环，注意符号的选择，应使误差向着减小的方向
		
	yawCal = constrain(VelocityOutPut, -YawControl->Limit, YawControl->Limit);//限制电流输出
	
	return yawCal;
}
/*************************************************************************
函数名：YawControlLoop_Work
入口参数：
         GIMBALCONTORL *YawControl：传入Y轴的控制结构体指针，得到控制参数
				 
出口参数：Y轴速度环限流后的输出
功能：云台Y轴正常工作时的控制
说明：云台采用串级控制，有位置环（控制角度），速度环（控制速度），电流环（控制电流）
位置环相当于队长，指挥调节速度环，速度环相当于组长， 指挥调节电流环，层层指导，
陀螺仪Z轴积分得到位置反馈，MPU6500提供角速度反馈，形成双环PID控制，电流环由驱动板控制。
云台的稳定性由结构本身确定，可通过调节PID参数提高稳定性，步兵采用纯比例控制
*************************************************************************/
float YawControlLoop_Work(GIMBALCONTORL *YawControl) //计算
{
	float PositionOutPut = 0.0f, VelocityOutPut = 0.0f, yawCal = 0.0f;
	YawControl->Limit = 4500;
	
	if(AutoData.Mode == 0)
	{
	//	YawControl->ControlData += 0.2;
				
	  PositionOutPut = -(YawControl->ControlData + YawControl->Angle) * YawControl->Position_kp;//Y轴位置环控制      
		
	  VelocityOutPut = (PositionOutPut - YawControl->Gyro) * YawControl->Velocity_kp;//Y轴速度环控制	  
	}
	else
	{
		YawControl->ControlData = YawControl->Angle  = 0;
			
	  PositionOutPut = (AutoData.YawAxiaAngle -3.5) * 9;
	VelocityOutPut = (PositionOutPut - YawControl->Gyro) * 40.0f;//Y轴速度环控制
		
	}
	
	yawCal = constrain(VelocityOutPut, -YawControl->Limit, YawControl->Limit);//限制电流值
	
	return yawCal;
}
/*************************************************************************
函数名：YawControlCalc
入口参数：
         GIMBALCONTORL *YawControl：传入Y轴的控制结构体指针，得到控制参数
				 
出口参数：
功能：云台Y轴根据状态选择输出并对初始遥控值清零
*************************************************************************/
void YawControlCalc(GIMBALCONTORL *YawControl)
{
	if(GetWorkState() != NORMAL_STATE)
	{
		mydata.Cortol_ch2 = mydata.imudata.Angle_Yaw = 0; //遥控值清零
		YawControl->output = YawControlLoop_Init(YawControl);//上电得到Y轴输出
	}
	else 
		YawControl->output = YawControlLoop_Work(YawControl);//正常工作时的Y轴输出
}

/*************************************************************************
函数名：PitchControlCalc
入口参数：
         GIMBALCONTORL *PitchControl：传入P轴的控制结构体指针，得到控制参数
				 
出口参数：无
功能：上电云台P轴归中控制
说明：云台采用串级控制，有位置环（控制角度），速度环（控制速度），电流环（控制电流）
位置环相当于队长，指挥调节速度环，速度环相当于组长， 指挥调节电流环，层层指导，
P轴编码器提供位置反馈，MPU6500提供角速度反馈，形成双环PID控制，电流环由驱动板控制。
云台的稳定性由结构本身确定，可通过调节PID参数提高稳定性，步兵采用纯比例控制
*************************************************************************/
void PitchControlCalc(GIMBALCONTORL *PitchControl) //计算
{
		float PositionOutPut = 0.0f, VelocityOutPut = 0.0f;
		static float Encoder_init = 0, Gimbal_MaxOutPut = 0;

		
  if(AutoData.Mode  == 0)	
	{
		Encoder_init = PitchControl->Encoder_Init;
		Gimbal_MaxOutPut += 1;
		if(Gimbal_MaxOutPut > 4500)		Gimbal_MaxOutPut = 4500;
		PositionOutPut = 	(
													(
															PitchControl->Encoder
															- Encoder_init  	//初始化的机械角度
															+ PitchControl->ControlData //控制量
													) 
													* 360 / 8192   //角度转换
											) 
											* PitchControl->Position_kp;
	
		VelocityOutPut = (PositionOutPut - PitchControl->Gyro) * PitchControl->Velocity_kp;//P轴角速度	   
		 
	}
 else 
 {
     PitchControl->ControlData =  Encoder_init- PitchControl->Encoder;
	   PositionOutPut = (float)(AutoData.PitchAxiaAngle +1.00) * 9.1;
	   VelocityOutPut = -(float)(PositionOutPut - PitchControl->Gyro) * 25.2;
 }	 
	
		PitchControl->output = constrain(VelocityOutPut, -Gimbal_MaxOutPut, Gimbal_MaxOutPut);
}


int16_t FireControlLoop(int16_t SetSpeed)//拨弹电机控制 
{		
		return (SetSpeed + CAN1_Motor_PID[6].get / 20) * 1.0f; 	
}


void FiresSpeedControlLoop(int16_t SetSpeed)//摩擦电机控制 
{
	  mydata.usemotor.rm3510_fire_L.control_val  =  constrain((mydata.usemotor.rm3510_fire_L.speed- SetSpeed * 0.1f),-6000,6000);
	  mydata.usemotor.rm3510_fire_R.control_val  = constrain(-mydata.usemotor.rm3510_fire_L.control_val,-6000,6000); 	
}


int16_t FireControlLoop_New(int16_t SetSpeed)//拨弹电机控制 
{
	   	CAN1_Motor_PID[6].set  =  SetSpeed;
	    pid_calc(&CAN1_Motor_PID[6]);	
	    CAN1_Motor_PID[6].out = constrain(CAN1_Motor_PID[6].out,-8000,8000);
	    return CAN1_Motor_PID[6].out;
	//return -(mydata.usemotor.rm2310_fire.speed - SetSpeed) * 0.60f;
}


void Set_Fire_Fre(int16_t fre)
{
	static int cnt,   cnt1 = 0;
	static char flag = 0;
	
	int16_t fire_set_speed =  fre;

	if(fire_set_speed != 0)
	{
		if(	CAN1_Motor_PID[6].get < 100)
				{
					 cnt++;
					 if(cnt > 40)
					 {
							flag = 1;
							cnt = 0;
					 }
				}
				
		if(flag == 1)
					 {
							 cnt1++;
							 fire_set_speed = -1500; //电机反转
							 if(cnt1 > 20)
							 {
								 flag = 0;
								 cnt1 = 0;
								 fire_set_speed = fre;
							 }
					 }	
	}
	
	FireControlLoop_New(fire_set_speed);
	
}


static void Fire_speed_Control_New(int16_t speed)
{
    CAN1_Motor_PID[0].set =  -speed;
	  CAN1_Motor_PID[1].set = speed;
	 
	  pid_calc(&CAN1_Motor_PID[0] );
	  pid_calc(&CAN1_Motor_PID[1] );
	
	  
	  CAN1_Motor_PID[0].out = constrain(CAN1_Motor_PID[0].out,-7000,7000);
	  CAN1_Motor_PID[1].out = constrain(CAN1_Motor_PID[1].out,-7000,7000);
	 
}


void Set_Fire_Speed(int16_t speed)
{
  Fire_speed_Control_New(speed);
}

void SetFireSpeddAndFre(int16_t speed,int16_t fre)
{
  Set_Fire_Speed(speed);
	Set_Fire_Fre(fre);
}


typedef enum{
	H_SPEED_A_L_FRE = 1,
	L_SPEED_A_H_FRE ,
	M_SPEED_A_M_FRE ,
	H_SPEED_A_H_FRE ,
	Fire_STOP,
}Fire_Mode;

void Set_Fire_Mode(Fire_Mode mode)
{
	
   switch(mode)
	 {
	   case H_SPEED_A_L_FRE:
         SetFireSpeddAndFre(1200,6500);
			 break;
	   case L_SPEED_A_H_FRE:
         SetFireSpeddAndFre(2500,5000);
			 break;
	   case M_SPEED_A_M_FRE:
       SetFireSpeddAndFre(2000,6000);
			 break;
	   case H_SPEED_A_H_FRE:
       SetFireSpeddAndFre(2500,6500);
			 break;
		 case Fire_STOP:
			 			 	Set_Fire_Fre(0);
			 break;			 
		 
	 }		 
}


enum{
	LEVEL1=1,
	LEVEL2,
	LEVEL3,
};

void Heat2SetFire()
{ 
	      
	 		if(AutoData.Mode == 1)			
			{
			  // if( AutoData.YawAxiaAngle <=  5.6 && AutoData.YawAxiaAngle >= -1.75 && AutoData.PitchAxiaAngle <= 3.83 && AutoData.PitchAxiaAngle >= -1.32 )
			//	 {  		
						Set_Fire_Mode(H_SPEED_A_H_FRE);		
			//	 }
				// else
				// {
			//	    Set_Fire_Mode(Fire_STOP);
			//		  Set_Fire_Speed(5500);
				// }
			}
			else
			{
					if(mydata.dbus.remote.s1 == MOVE_UP)
					{  	
						 SetFireSpeddAndFre(0,0);		
						 Bsp.TurnOnTheShootMotor(0);
						 Bsp.OpenTheLaser(1);
					}
					else if(mydata.dbus.remote.s1 == MOVE_MID)
					{
						 SetFireSpeddAndFre(5000,0);		
						 Bsp.TurnOnTheShootMotor(1);
					}
					else  if(mydata.dbus.remote.s1 == MOVE_DOWM)
					{		 
						SetFireSpeddAndFre(5000,2000);					
						Bsp.TurnOnTheShootMotor(1);
					
					}
         else
				 {
				     SetFireSpeddAndFre(5000,0);		
						 Bsp.TurnOnTheShootMotor(1);
				 }					 
			}
	

}

