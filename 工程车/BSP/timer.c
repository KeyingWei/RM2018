#include "headfile.h"
#include "pid_modify.h"

/******************************
��������TIM2_Config
��ڲ�������
���ڲ�������
���ܣ�����TIM2������Ԥ��Ƶֵ����װ��ֵ
��ʱʱ��time=(90/psc)*arr )us=4294
*******************************/
void TIM2_Config(void)
{
    TIM_TimeBaseInitTypeDef tim;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
    tim.TIM_Period = 0xFFFFFFFF;
    tim.TIM_Prescaler = 90 - 1;	          //1M ��ʱ��  
    tim.TIM_ClockDivision = TIM_CKD_DIV1;	//����Ƶ
    tim.TIM_CounterMode = TIM_CounterMode_Up;  //���ϼ���
    TIM_ARRPreloadConfig(TIM2, ENABLE);	//ʹ����װ��
    TIM_TimeBaseInit(TIM2, &tim);

    TIM_Cmd(TIM2,ENABLE);	
}
/******************************
��������TIM3_Config
��ڲ�������
���ڲ�������
���ܣ�����TIM2������Ԥ��Ƶֵ����װ��ֵ
��ʱʱ��time=(90/90)*1000 )us=1ms
*******************************/
void TIM3_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	TIM_DeInit(TIM3);
	TIM_TimeBaseStructure.TIM_Period = 10000-1; //100hz
	TIM_TimeBaseStructure.TIM_Prescaler = 90-1; 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM3, ENABLE);  
}

/******************************
��������TIM4_Config
��ڲ�������
���ڲ�������
���ܣ�����TIM2������Ԥ��Ƶֵ����װ��ֵ
��ʱʱ��time=(90/psc)*arr )us=1ms
*******************************/
void TIM4_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); 
	
	TIM_DeInit(TIM4);
	TIM_TimeBaseStructure.TIM_Period = 1000-1; 
	TIM_TimeBaseStructure.TIM_Prescaler = 90-1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); 
	
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM4, ENABLE);  
}


/********************************************
��������TIMExNVIC_Config
��ڲ�������
���ڲ�������
���ܣ�����TIM3,4�жϣ�������Ӧ���ȼ�����ռ���ȼ�
˵��������ԽС���ȼ�Խ�ߣ���ռ���ȼ��ߵĿɴ����
�ȼ��͵��жϣ���ռ���ȼ���ͬʱ���жϲ��ɴ�ϣ�ͬ
ʱ�����ж�ʱ����Ӧ���ȼ��ߵĿ����Ƚ����ж�
**********************************************/
void TIMExNVIC_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//�������ȼ����2
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn; //���ö�ʱ��3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //������ռ���ȼ�Ϊ1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  //������Ӧ���ȼ�Ϊ2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure);  
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;   //���ö�ʱ��4�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //������ռ���ȼ�Ϊ0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //������Ӧ���ȼ�Ϊ0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure); 
	
}
/********************************************
��������TimConfig
��ڲ�������
���ڲ�������
���ܣ�����TIM2,TIM3,TIM4
**********************************************/
void TimConfig(void)
{
	TIM2_Config();
	TIM3_Config();
	TIM4_Config();
	TIMExNVIC_Config();
}

/********************************************
��������TIM2_IRQHandler
��ڲ�������
���ڲ�������
���ܣ� Ԥ��
**********************************************/
void TIM2_IRQHandler(void)
{
	  if (TIM_GetITStatus(TIM2,TIM_IT_Update)!= RESET) 
		{
			  TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
        TIM_ClearFlag(TIM2, TIM_FLAG_Update);
			  //BOTH_LED_TOGGLE();
		}
} 

uint32_t Get_Time_Micros(void)
{
	return TIM2->CNT;
}

/********************************************
��������TIM3_IRQHandler
��ڲ�������
���ڲ�������
���ܣ�Ԥ��
**********************************************/

Key_ide key_ide[8]={
	{0},{0},{0},{0},{0},{0},{0},{0}
	} ;
p_Key_ide P_Key_ide1[8] ={&key_ide[0],&key_ide[1],&key_ide[2],&key_ide[3],&key_ide[4],&key_ide[5],&key_ide[6],&key_ide[7]} ; 
	
void TIM3_IRQHandler(void)
{	
	if(TIM_GetITStatus(TIM3,TIM_IT_Update) != RESET) //����ж�
	{
            Key_Scan(KEY_PRESSED_OFFSET_F,P_Key_ide1[0]);
		    Key_Scan(KEY_PRESSED_OFFSET_G,P_Key_ide1[1]);
		    Key_Scan(KEY_PRESSED_OFFSET_R,P_Key_ide1[2]);
		    Key_Scan(KEY_PRESSED_OFFSET_Z,P_Key_ide1[3]);
		    Key_Scan(KEY_PRESSED_OFFSET_X,P_Key_ide1[4]);
		    Key_Scan(KEY_PRESSED_OFFSET_B,P_Key_ide1[5]);
		    Key_Scan(KEY_PRESSED_OFFSET_C,P_Key_ide1[6]);
		    Key_Scan(KEY_PRESSED_OFFSET_V,P_Key_ide1[7]);
		
		
		    //����R��
		    if(P_Key_ide1[2]->Key_Event == KEY_ONE_CLICK || P_Key_ide1[2]->Key_Event == KEY_LONG_CLICK || P_Key_ide1[2]->Key_Event == KEY_NO_CLICK) //
				{
					GPIO_ResetBits(GPIOH, GPIO_Pin_11);	
				}
				else if(P_Key_ide1[2]->Key_Event == KEY_DOUBLE_CLICK )
				{
					GPIO_SetBits(GPIOH, GPIO_Pin_11); 			
				}		    
				
				//����F��	
		    if(P_Key_ide1[0]->Key_Event == KEY_ONE_CLICK || P_Key_ide1[0]->Key_Event == KEY_LONG_CLICK || P_Key_ide1[0]->Key_Event == KEY_NO_CLICK)
				{
				     Bsp.OpenTheGreenLed(1);
							
					 g_Lang_flag = 0;//���ϵ���־
			
					 state = FourWheelUp_SATA;
					
					 CMControlLoop();
					 pid_calc(&CAN1_Motor_PID[5]);
         
					if(P_Key_ide1[5]->Key_Event == KEY_ONE_CLICK || P_Key_ide1[5]->Key_Event == KEY_LONG_CLICK || P_Key_ide1[5]->Key_Event == KEY_NO_CLICK) 
							{
								LangSmallWheel(0,900);
							}
					else if(P_Key_ide1[5]->Key_Event == KEY_DOUBLE_CLICK )
							{ 
								
								LangSmallWheel(1,1000);		
							}						
					
					CM_U_D_A_Catch_Control(8192 *14,700);	 	//�����µ������ͼ�ȡ��� params1:˿������λ�ã�˿�������ٶ�  
				}
				else if(P_Key_ide1[0]->Key_Event == KEY_DOUBLE_CLICK )
				{
				   Bsp.OpenTheGreenLed(0); 
					
					 	LangSmallWheel(1,1000);
				
					 CmCotrol.Control_CH0 = mydata.Cortol_ch0;
					 Go_Island(700,2200,8192 *14);  // ����1��˿�������ٶȣ�����2�������ƶ��ٶ� ��˿������λ��
			     if(g_Lang_flag == 2)
					 {
					    P_Key_ide1[0]->Key_Event = KEY_ONE_CLICK;
					 }
				}
					
			  CMControlCalc();//���̿���	
				
	}
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);  //����жϱ�־λ
}
 
void TIM4_IRQHandler(void)
{

	if(TIM_GetITStatus(TIM4,TIM_IT_Update) != RESET) //����ж�
	{
		GimbalMotorOutput();
	}
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);  //����жϱ�־λ
}





