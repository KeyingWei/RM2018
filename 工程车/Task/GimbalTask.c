#include "headfile.h"


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

	PositionOutPut = -(YawControl->ControlData + YawControl->Angle) * YawControl->Position_kp;//Y轴位置环控制      
		
	VelocityOutPut = (PositionOutPut - YawControl->Gyro) * YawControl->Velocity_kp;//Y轴速度环控制
		
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
	if(GetWorkState() == PREPARE_STATE)
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
		static float Encoder_init = 0, Encoder_E, Gimbal_MaxOutPut = 0;
		static u8 mode = 0;
		static int cout = 0;
	
		Encoder_E = PitchControl->Encoder_Init - PitchControl->Encoder;
		cout ++;
		if(cout <= 1000)
		{
			Encoder_init = PitchControl->Encoder;
			PitchControl->ControlData = 0;
		}
		if(mode == 0)
		{
			Encoder_init += (Encoder_E / 1500);  //?
			if(abs(Encoder_init - PitchControl->Encoder_Init) < 50)
			{	
				Encoder_init = PitchControl->Encoder_Init;	
				mode = stop;
				workState = STANDBY_STATE;
			}
		}
		
		Gimbal_MaxOutPut += 4;
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
	
		PitchControl->output = constrain(VelocityOutPut, -Gimbal_MaxOutPut, Gimbal_MaxOutPut);
}

int16_t FireControlLoop(int16_t SetSpeed)//拨弹电机控制 
{
	return -(mydata.usemotor.rm2310_fire.speed - SetSpeed) * 0.60f;
}


void FiresSpeedControlLoop(int16_t SetSpeed)//摩擦电机控制 
{
	  mydata.usemotor.rm3510_fire_L.control_val  =  constrain((mydata.usemotor.rm3510_fire_L.speed- SetSpeed * 0.1f),-6000,6000);
	  mydata.usemotor.rm3510_fire_R.control_val  = constrain(-mydata.usemotor.rm3510_fire_L.control_val,-6000,6000); 	
}
