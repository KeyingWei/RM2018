#include "headfile.h"
#include "pid_modify.h"


u8 GimbalInit = 0;
/*************************************************************************
��������YawControlLoop_Init
��ڲ�����
         GIMBALCONTORL *YawControl������Y��Ŀ��ƽṹ��ָ�룬�õ����Ʋ���
				 
���ڲ�����Y���ٶȻ�����������
���ܣ��ϵ���̨Y����п���
˵������̨���ô������ƣ���λ�û������ƽǶȣ����ٶȻ��������ٶȣ��������������Ƶ�����
λ�û��൱�ڶӳ���ָ�ӵ����ٶȻ����ٶȻ��൱���鳤�� ָ�ӵ��ڵ����������ָ����
Y��������ṩλ�÷�����MPU6500�ṩ���ٶȷ������γ�˫��PID���ƣ�����������������ơ�
��̨���ȶ����ɽṹ����ȷ������ͨ������PID��������ȶ��ԣ��������ô���������
*************************************************************************/
float YawControlLoop_Init(GIMBALCONTORL *YawControl) //����
{
	float PositionOutPut = 0.0f, VelocityOutPut = 0.0f, yawCal = 0.0f;
	YawControl->Limit += 1;//�õ���ֵ��������ֹ����ʱ��������
	if(YawControl->Limit > 800)		YawControl->Limit = 800;

	PositionOutPut = 	- (
												(
													YawControl->Encoder //���ڵĻ�е�Ƕȣ��ѽ�������������
													- YawControl->Encoder_Init  //��ʼ���Ļ�е�Ƕ�
												) 
												* 360 / 8192 //�Ƕ�ת��
											) 
											* YawControl->Position_kp;//Y��λ�û���ע����ŵ�ѡ��Ӧʹ������ż�С�ķ���
		
	VelocityOutPut = -(PositionOutPut + YawControl->Gyro) * YawControl->Velocity_kp;//Y���ٶȻ���ע����ŵ�ѡ��Ӧʹ������ż�С�ķ���
		
	yawCal = constrain(VelocityOutPut, -YawControl->Limit, YawControl->Limit);//���Ƶ������
	
	return yawCal;
}
/*************************************************************************
��������YawControlLoop_Work
��ڲ�����
         GIMBALCONTORL *YawControl������Y��Ŀ��ƽṹ��ָ�룬�õ����Ʋ���
				 
���ڲ�����Y���ٶȻ�����������
���ܣ���̨Y����������ʱ�Ŀ���
˵������̨���ô������ƣ���λ�û������ƽǶȣ����ٶȻ��������ٶȣ��������������Ƶ�����
λ�û��൱�ڶӳ���ָ�ӵ����ٶȻ����ٶȻ��൱���鳤�� ָ�ӵ��ڵ����������ָ����
������Z����ֵõ�λ�÷�����MPU6500�ṩ���ٶȷ������γ�˫��PID���ƣ�����������������ơ�
��̨���ȶ����ɽṹ����ȷ������ͨ������PID��������ȶ��ԣ��������ô���������
*************************************************************************/
float YawControlLoop_Work(GIMBALCONTORL *YawControl) //����
{
	float PositionOutPut = 0.0f, VelocityOutPut = 0.0f, yawCal = 0.0f;
	YawControl->Limit = 4500;
	
	if(AutoData.Mode == 0)
	{
	//	YawControl->ControlData += 0.2;
				
	  PositionOutPut = -(YawControl->ControlData + YawControl->Angle) * YawControl->Position_kp;//Y��λ�û�����      
		
	  VelocityOutPut = (PositionOutPut - YawControl->Gyro) * YawControl->Velocity_kp;//Y���ٶȻ�����	  
	}
	else
	{
		YawControl->ControlData = YawControl->Angle  = 0;
			
	  PositionOutPut = (AutoData.YawAxiaAngle -3.5) * 9;
	VelocityOutPut = (PositionOutPut - YawControl->Gyro) * 40.0f;//Y���ٶȻ�����
		
	}
	
	yawCal = constrain(VelocityOutPut, -YawControl->Limit, YawControl->Limit);//���Ƶ���ֵ
	
	return yawCal;
}
/*************************************************************************
��������YawControlCalc
��ڲ�����
         GIMBALCONTORL *YawControl������Y��Ŀ��ƽṹ��ָ�룬�õ����Ʋ���
				 
���ڲ�����
���ܣ���̨Y�����״̬ѡ��������Գ�ʼң��ֵ����
*************************************************************************/
void YawControlCalc(GIMBALCONTORL *YawControl)
{
	if(GetWorkState() != NORMAL_STATE)
	{
		mydata.Cortol_ch2 = mydata.imudata.Angle_Yaw = 0; //ң��ֵ����
		YawControl->output = YawControlLoop_Init(YawControl);//�ϵ�õ�Y�����
	}
	else 
		YawControl->output = YawControlLoop_Work(YawControl);//��������ʱ��Y�����
}

/*************************************************************************
��������PitchControlCalc
��ڲ�����
         GIMBALCONTORL *PitchControl������P��Ŀ��ƽṹ��ָ�룬�õ����Ʋ���
				 
���ڲ�������
���ܣ��ϵ���̨P����п���
˵������̨���ô������ƣ���λ�û������ƽǶȣ����ٶȻ��������ٶȣ��������������Ƶ�����
λ�û��൱�ڶӳ���ָ�ӵ����ٶȻ����ٶȻ��൱���鳤�� ָ�ӵ��ڵ����������ָ����
P��������ṩλ�÷�����MPU6500�ṩ���ٶȷ������γ�˫��PID���ƣ�����������������ơ�
��̨���ȶ����ɽṹ����ȷ������ͨ������PID��������ȶ��ԣ��������ô���������
*************************************************************************/
void PitchControlCalc(GIMBALCONTORL *PitchControl) //����
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
															- Encoder_init  	//��ʼ���Ļ�е�Ƕ�
															+ PitchControl->ControlData //������
													) 
													* 360 / 8192   //�Ƕ�ת��
											) 
											* PitchControl->Position_kp;
	
		VelocityOutPut = (PositionOutPut - PitchControl->Gyro) * PitchControl->Velocity_kp;//P����ٶ�	   
		 
	}
 else 
 {
     PitchControl->ControlData =  Encoder_init- PitchControl->Encoder;
	   PositionOutPut = (float)(AutoData.PitchAxiaAngle +1.00) * 9.1;
	   VelocityOutPut = -(float)(PositionOutPut - PitchControl->Gyro) * 25.2;
 }	 
	
		PitchControl->output = constrain(VelocityOutPut, -Gimbal_MaxOutPut, Gimbal_MaxOutPut);
}


int16_t FireControlLoop(int16_t SetSpeed)//����������� 
{		
		return (SetSpeed + CAN1_Motor_PID[6].get / 20) * 1.0f; 	
}


void FiresSpeedControlLoop(int16_t SetSpeed)//Ħ��������� 
{
	  mydata.usemotor.rm3510_fire_L.control_val  =  constrain((mydata.usemotor.rm3510_fire_L.speed- SetSpeed * 0.1f),-6000,6000);
	  mydata.usemotor.rm3510_fire_R.control_val  = constrain(-mydata.usemotor.rm3510_fire_L.control_val,-6000,6000); 	
}


int16_t FireControlLoop_New(int16_t SetSpeed)//����������� 
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
							 fire_set_speed = -1500; //�����ת
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

