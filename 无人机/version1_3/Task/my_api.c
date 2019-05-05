#include "headfile.h"
#include <math.h>
#include "pid_modify.h"

CMCONTORL CmCotrol = CMcontrolconfig;//���̿��Ʋ�����ֵ
GIMBALCONTORL YawControl = yawcontrolconfig; //��̨Y����Ʋ�����ֵ
GIMBALCONTORL PitchControl = pitchcontrolconfig;//��̨P����Ʋ�����ֵ

volatile int16_t g_fire_speed = 0; 

WorkState_e workState = PREPARE_STATE;//�ϵ��ʼ��״̬��4-5s����

WorkState_e GetWorkState(void)//��ȡ����״̬
{
		return workState;
}

void WorkState(void)
{

}
/***************************************************
������:CMControlLoop
��ڲ�������
���ڲ�������
���ܣ� ��ȡң�ز���ֵ����̨Y������ֵ�����µ��̿��Ʋ���
**************************************************/
void CMControlLoop(void)
{
	
	CM_Motor_Pidparams_init();
	
	CmCotrol.Control_CH0 = mydata.Cortol_ch0;//���Ƶ���ǰ���ƶ�
	CmCotrol.Control_CH1 = mydata.Cortol_ch1;//���Ƶ������Һ���
	CmCotrol.Encoder = mydata.usemotor.rm6623_y.encoder;//��ȡY������ֵ���������������������ڼ���ǶȲ�ֵ��ʵ�ֵ��̸���
	CmCotrol.Encoder_Init = Yaw_Encoder_Init;//��ȡ��̨��ʼ��λ��
	CmCotrol.Calc(); 
}
/***************************************************
������:YawControlLoop
��ڲ�������
���ڲ�������
���ܣ� ��ȡY��������ֵ��������Z����ٶȣ�Z����������
��ֵ����д�������
**************************************************/
void YawControlLoop(void)
{
	YawControl.Angle = mydata.imudata.Angle_Yaw; //��ȡ����Ƕ�
	YawControl.Gyro = mydata.imudata.Gyro_Z;//��ȡ������ٶ�
	
	YawControl.ControlData = mydata.Cortol_ch2;//��ȡ�������ֵ
	
	YawControl.Encoder = mydata.usemotor.rm6623_y.encoder;//��ȡY������ֵ
	YawControl.Encoder_Init = 2500;//��ȡ��ʼ����ֵ
	
	YawControl.Position_kp = Yaw_Position_kp;//�趨λ�û����Ʋ���
	YawControl.Velocity_kp = Yaw_Velocity_kp;//�趨�ٶȻ����Ʋ���
	
	
	YawControl.Calc(&YawControl);//��̨Y�����
}
/***************************************************
������:PitchControlLoop
��ڲ�������
���ڲ�������
���ܣ� ��ȡP��������ֵ��������X����ٶ�
��ֵ����д�������
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
������:GimbalMotorOutput
��ڲ�������
���ڲ�������
���ܣ� ����״̬������̨�������ң��ֵ�޷����ջ�ģ������ʱ����
̨�����������
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

