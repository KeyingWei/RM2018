#include "headfile.h"
#include "pid_modify.h"

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
定时时间time=(90/90)*1000 )us=1ms
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

Key_ide key_ide[8]={
	{0},{0},{0},{0},{0},{0},{0},{0}
	} ;
p_Key_ide P_Key_ide1[8] ={&key_ide[0],&key_ide[1],&key_ide[2],&key_ide[3],&key_ide[4],&key_ide[5],&key_ide[6],&key_ide[7]} ; 
	
void TIM3_IRQHandler(void)
{	
	if(TIM_GetITStatus(TIM3,TIM_IT_Update) != RESET) //溢出中断
	{
            Key_Scan(KEY_PRESSED_OFFSET_F,P_Key_ide1[0]);
		    Key_Scan(KEY_PRESSED_OFFSET_G,P_Key_ide1[1]);
		    Key_Scan(KEY_PRESSED_OFFSET_R,P_Key_ide1[2]);
		    Key_Scan(KEY_PRESSED_OFFSET_Z,P_Key_ide1[3]);
		    Key_Scan(KEY_PRESSED_OFFSET_X,P_Key_ide1[4]);
		    Key_Scan(KEY_PRESSED_OFFSET_B,P_Key_ide1[5]);
		    Key_Scan(KEY_PRESSED_OFFSET_C,P_Key_ide1[6]);
		    Key_Scan(KEY_PRESSED_OFFSET_V,P_Key_ide1[7]);
		
		
		    //按下R键
		    if(P_Key_ide1[2]->Key_Event == KEY_ONE_CLICK || P_Key_ide1[2]->Key_Event == KEY_LONG_CLICK || P_Key_ide1[2]->Key_Event == KEY_NO_CLICK) //
				{
					GPIO_ResetBits(GPIOH, GPIO_Pin_11);	
				}
				else if(P_Key_ide1[2]->Key_Event == KEY_DOUBLE_CLICK )
				{
					GPIO_SetBits(GPIOH, GPIO_Pin_11); 			
				}		    
				
				//按下F键	
		    if(P_Key_ide1[0]->Key_Event == KEY_ONE_CLICK || P_Key_ide1[0]->Key_Event == KEY_LONG_CLICK || P_Key_ide1[0]->Key_Event == KEY_NO_CLICK)
				{
				     Bsp.OpenTheGreenLed(1);
							
					 g_Lang_flag = 0;//清上岛标志
			
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
					
					CM_U_D_A_Catch_Control(8192 *14,700);	 	//控制下岛动作和夹取物块 params1:丝杆上限位置，丝杆升降速度  
				}
				else if(P_Key_ide1[0]->Key_Event == KEY_DOUBLE_CLICK )
				{
				   Bsp.OpenTheGreenLed(0); 
					
					 	LangSmallWheel(1,1000);
				
					 CmCotrol.Control_CH0 = mydata.Cortol_ch0;
					 Go_Island(700,2200,8192 *14);  // 参数1：丝杆升降速度，参数2：底盘移动速度 ，丝杆上限位置
			     if(g_Lang_flag == 2)
					 {
					    P_Key_ide1[0]->Key_Event = KEY_ONE_CLICK;
					 }
				}
					
			  CMControlCalc();//底盘控制	
				
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





