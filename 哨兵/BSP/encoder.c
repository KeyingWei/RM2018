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
	
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);//使能TIM4时钟  
	
	  TIM_DeInit(TIM2);
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = 0xffffffff;  //设定计数器重装值   TIMx_ARR = 359*4
    TIM_TimeBaseStructure.TIM_Prescaler = 0; //TIM3时钟预分频值
    TIM_TimeBaseStructure.TIM_ClockDivision =TIM_CKD_DIV1 ;//设置时钟分割 T_dts = T_ck_int    
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数 
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);              

    TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_BothEdge ,TIM_ICPolarity_BothEdge);//使用编码器模式3，上升下降都计数
    TIM_ICStructInit(&TIM_ICInitStructure);//将结构体中的内容缺省输入
    TIM_ICInitStructure.TIM_ICFilter = 6;  //选择输入比较滤波器 
    TIM_ICInit(TIM2, &TIM_ICInitStructure);//将TIM_ICInitStructure中的指定参数初始化TIM4

//  TIM_ARRPreloadConfig(TIM4, ENABLE);//使能预装载
   // TIM_ClearFlag(TIM4, TIM_FLAG_Update);//清除TIM4的更新标志位
    //TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);//运行更新中断
    //Reset counter
    TIM2->CNT = 2000000000;//

    TIM_Cmd(TIM2, ENABLE);   //启动TIM4定时器
  
}




static void Encoder_Exti_Gpio_Config()
{
	 GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);// GPIOE时钟
 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14; // MPU_INT引脚
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//下拉
  GPIO_Init(GPIOD, &GPIO_InitStructure);//初始化GPIOE1

}

static void Encoder_NVIC_Config()
{
	NVIC_InitTypeDef   NVIC_InitStructure;
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;//外部中断14
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;//抢占优先级0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;//子优先级2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
  NVIC_Init(&NVIC_InitStructure);//配置
	
}

static void Encoder_EXTI_Config()
{
  EXTI_InitTypeDef   EXTI_InitStructure;
	
	/* 配置EXTI_Line1 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource14);//连接中断线1
	EXTI_InitStructure.EXTI_Line = EXTI_PinSource14;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //下降沿触发
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;//中断线使能
  EXTI_Init(&EXTI_InitStructure);//配置
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
	   EXTI_ClearITPendingBit(EXTI_Line14);//清除LINE1上的中断标志位 
}

