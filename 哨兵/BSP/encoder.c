#include "headfile.h"

void Encoder_GPIO_init()
{
    GPIO_InitTypeDef GPIO_InitStructure;
      

    //PD12 ch1  A,PD13 ch2 
	
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 

	  	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM2); 
	    GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM2);
	
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;           //GPIOA8,A9
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;       
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;      
		  GPIO_Init(GPIOA,&GPIO_InitStructure); 

}


void Encoder_Timer_init()
{
	  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
	
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);//ʹ��TIM4ʱ��  
	
	  TIM_DeInit(TIM2);
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = 0xffffffff;  //�趨��������װֵ   TIMx_ARR = 359*4
    TIM_TimeBaseStructure.TIM_Prescaler = 0; //TIM3ʱ��Ԥ��Ƶֵ
    TIM_TimeBaseStructure.TIM_ClockDivision =TIM_CKD_DIV1 ;//����ʱ�ӷָ� T_dts = T_ck_int    
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM���ϼ��� 
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);              

    TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_BothEdge ,TIM_ICPolarity_BothEdge);//ʹ�ñ�����ģʽ3�������½�������
    TIM_ICStructInit(&TIM_ICInitStructure);//���ṹ���е�����ȱʡ����
    TIM_ICInitStructure.TIM_ICFilter = 6;  //ѡ������Ƚ��˲��� 
    TIM_ICInit(TIM2, &TIM_ICInitStructure);//��TIM_ICInitStructure�е�ָ��������ʼ��TIM4

//  TIM_ARRPreloadConfig(TIM4, ENABLE);//ʹ��Ԥװ��
   // TIM_ClearFlag(TIM4, TIM_FLAG_Update);//���TIM4�ĸ��±�־λ
    //TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);//���и����ж�
    //Reset counter
    TIM2->CNT = 2000000000;//

    TIM_Cmd(TIM2, ENABLE);   //����TIM4��ʱ��
  
}




static void Encoder_Exti_Gpio_Config()
{
	 GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);// GPIOEʱ��
 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14; // MPU_INT����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//��ͨ����ģʽ
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//����
  GPIO_Init(GPIOD, &GPIO_InitStructure);//��ʼ��GPIOE1

}

static void Encoder_NVIC_Config()
{
	NVIC_InitTypeDef   NVIC_InitStructure;
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;//�ⲿ�ж�14
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;//��ռ���ȼ�0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;//�����ȼ�2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ʹ���ⲿ�ж�ͨ��
  NVIC_Init(&NVIC_InitStructure);//����
	
}

static void Encoder_EXTI_Config()
{
  EXTI_InitTypeDef   EXTI_InitStructure;
	
	/* ����EXTI_Line1 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource14);//�����ж���1
	EXTI_InitStructure.EXTI_Line = EXTI_PinSource14;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//�ж��¼�
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //�½��ش���
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;//�ж���ʹ��
  EXTI_Init(&EXTI_InitStructure);//����
}



void Encoder_Config()
{ 
	 Encoder_GPIO_init();
	 Encoder_Timer_init();
	
	
   // Encoder_Exti_Gpio_Config();
  //  Encoder_NVIC_Config();
	//  Encoder_EXTI_Config();
}


void EXTI15_10_IRQHandler(void)
{
  	if(EXTI_GetITStatus(EXTI_Line14)!=RESET)
		{
		   TIM2->CNT = 0;
      // printf("%d\r\n",TIM4->CR1 & (1<<4));			 
		}
	   EXTI_ClearITPendingBit(EXTI_Line14);//���LINE1�ϵ��жϱ�־λ 
}

