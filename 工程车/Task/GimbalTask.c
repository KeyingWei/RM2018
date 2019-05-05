#include "headfile.h"


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
	return -(mydata.usemotor.rm2310_fire.speed - SetSpeed) * 0.60f;
}


void FiresSpeedControlLoop(int16_t SetSpeed)//Ħ��������� 
{
	  mydata.usemotor.rm3510_fire_L.control_val  =  constrain((mydata.usemotor.rm3510_fire_L.speed- SetSpeed * 0.1f),-6000,6000);
	  mydata.usemotor.rm3510_fire_R.control_val  = constrain(-mydata.usemotor.rm3510_fire_L.control_val,-6000,6000); 	
}
