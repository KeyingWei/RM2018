#include "headfile.h"
#include "pid_modify.h"


u8 GimbalInit = 0;


void Fire_Moter_Init()
{
    PID_struct_init(&CAN1_Motor_PID[6],DELTA_PID,7000,500,5,0.50f,0.1);
}


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
	if(YawControl->Limit > 8000)		YawControl->Limit = 8000;

	PositionOutPut = 	- (
												(
													YawControl->Encoder //���ڵĻ�е�Ƕȣ��ѽ�������������
													- YawControl->Encoder_Init  //��ʼ���Ļ�е�Ƕ�
												) 
												* 360 / 8192 //�Ƕ�ת��
											) 
											* YawControl->Position_kp;//Y��λ�û���ע����ŵ�ѡ��Ӧʹ������ż�С�ķ���
		
	VelocityOutPut = (PositionOutPut - YawControl->Gyro) * YawControl->Velocity_kp;//Y���ٶȻ���ע����ŵ�ѡ��Ӧʹ������ż�С�ķ���
		
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
	YawControl->Limit = 30000;

	PositionOutPut = -(YawControl->ControlData + YawControl->Angle) * YawControl->Position_kp;//Y��λ�û�����      
		
	VelocityOutPut = (PositionOutPut - YawControl->Gyro) * YawControl->Velocity_kp;//Y���ٶȻ�����
		
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
	if(GetWorkState() == PREPARE_STATE)
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
															- Encoder_init  	//��ʼ���Ļ�е�Ƕ�
															+ PitchControl->ControlData //������
													) 
													* 360 / 8192   //�Ƕ�ת��
											) 
											* PitchControl->Position_kp;
	
		VelocityOutPut = (PositionOutPut - PitchControl->Gyro) * PitchControl->Velocity_kp;//P����ٶ�
	
		PitchControl->output = constrain(VelocityOutPut, -Gimbal_MaxOutPut, Gimbal_MaxOutPut);
}

int16_t FireControlLoop(int16_t SetSpeed)//����������� 
{
	return -(mydata.usemotor.rm2310_fire.speed - SetSpeed) * 0.80f;
}


float  PitchControl_Position()
{

	  float position_out = 0;
	  float velocity_out =0;
	
	  PitchControl.Position_kp = 8.0f;
	  PitchControl.Velocity_kp = 10.0f;
	  PitchControl.Angle = mydata.Cortol_ch3;
	
	  int16_t curr_speed = CAN1_Motor_PID[5].get;
	  position_out = PitchControl.Position_kp * (PitchControl.Angle -(CAN1_Motor_PID[5].position_get- CAN1_Motor_PID[5].init_position)); 
	  velocity_out = PitchControl.Velocity_kp * (position_out - curr_speed);
	
	   velocity_out= constrain(velocity_out,-8000,8000);
	
	  return velocity_out;  
}



int16_t FireControlLoop_New(int16_t SetSpeed)//����������� 
{
	   	CAN1_Motor_PID[6].set  =  SetSpeed;
	    pid_calc(&CAN1_Motor_PID[6]);	
	  
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
							 if(cnt1 > 25)
							 {
								 flag = 0;
								 cnt1 = 0;
								 fire_set_speed = fre;
							 }
					 }	
	}
	
	FireControlLoop_New(fire_set_speed);
	
}





void Set_Fire_Speed(int16_t speed)
{
   Bsp.TurnOnTheShootMotor(1,speed);
}

void SetFireSpeddAndFre(int16_t speed,int16_t fre)
{
  Bsp.TurnOnTheShootMotor(1,speed);
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
       SetFireSpeddAndFre(147,1000);
			 break;
	   case L_SPEED_A_H_FRE:
       SetFireSpeddAndFre(130,1500);
			 break;
	   case M_SPEED_A_M_FRE:
          SetFireSpeddAndFre(142,1800);
			 break;
	   case H_SPEED_A_H_FRE:
           SetFireSpeddAndFre(147,2500);
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
			  if( Referee_date1.powerHeatData.shooterHeat0 > ( 180 - 30))
				{
				     Set_Fire_Mode(Fire_STOP);   
				}
				else if(Referee_date1.powerHeatData.shooterHeat0 > ( 180 *2/3))
				{
				   Set_Fire_Mode(H_SPEED_A_L_FRE);
				
				}
				else
				{
				    Set_Fire_Mode(M_SPEED_A_M_FRE);
				}
}

