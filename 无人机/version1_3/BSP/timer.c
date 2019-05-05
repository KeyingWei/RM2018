#include "headfile.h"

/******************************
函数名：TIM2_Config
入口参数：无
出口参数：无
功能：配置TIM2，设置预分频值，重装载值
定时时间time=(90/psc)*arr )us=4294
*******************************/
void TIM2_Config(void)
{
    TIM_TimeBaseInitTypeDef tim;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
    tim.TIM_Period = 0xFFFFFFFF;
    tim.TIM_Prescaler = 90 - 1;	          //1M 的时钟  
    tim.TIM_ClockDivision = TIM_CKD_DIV1;	//不分频
    tim.TIM_CounterMode = TIM_CounterMode_Up;  //向上计数
    TIM_ARRPreloadConfig(TIM2, ENABLE);	//使能重装载
    TIM_TimeBaseInit(TIM2, &tim);

    TIM_Cmd(TIM2,ENABLE);	
}
/******************************
函数名：TIM3_Config
入口参数：无
出口参数：无
功能：配置TIM2，设置预分频值，重装载值
定时时间time=(90/90)*20000 )us=20ms
*******************************/
void TIM3_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	TIM_DeInit(TIM3);
	TIM_TimeBaseStructure.TIM_Period = 20000-1; 
	TIM_TimeBaseStructure.TIM_Prescaler = 90-1; 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM3, ENABLE);  
}

/******************************
函数名：TIM4_Config
入口参数：无
出口参数：无
功能：配置TIM2，设置预分频值，重装载值
定时时间time=(90/psc)*arr )us=1ms
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
函数名：TIMExNVIC_Config
入口参数：无
出口参数：无
功能：配置TIM3,4中断，设置响应优先级和抢占优先级
说明：数字越小优先级越高，抢占优先级高的可打断优
先级低的中断，抢占优先级相同时，中断不可打断，同
时产生中断时，响应优先级高的可优先进行中断
**********************************************/
void TIMExNVIC_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置优先级组号2
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn; //设置定时器3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //设置抢占优先级为1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  //设置响应优先级为2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure);  
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;   //设置定时器4中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //设置抢占优先级为0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //设置响应优先级为0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure); 
	
}
/********************************************
函数名：TimConfig
入口参数：无
出口参数：无
功能：配置TIM2,TIM3,TIM4
**********************************************/
void TimConfig(void)
{
	TIM2_Config();
	TIM3_Config();
	TIM4_Config();
	TIMExNVIC_Config();
}

/********************************************
函数名：TIM2_IRQHandler
入口参数：无
出口参数：无
功能： 预留
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
函数名：TIM3_IRQHandler
入口参数：无
出口参数：无
功能：预留
**********************************************/
void TIM3_IRQHandler(void)
{
	
	if(TIM_GetITStatus(TIM3,TIM_IT_Update) != RESET) //溢出中断
	{

	}
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);  //清除中断标志位
}
 
void TIM4_IRQHandler(void)
{

	if(TIM_GetITStatus(TIM4,TIM_IT_Update) != RESET) //溢出中断
	{
		GimbalMotorOutput();
	}
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);  //清除中断标志位
}





