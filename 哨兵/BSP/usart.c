#include "headfile.h"

#include "stm32f4xx.h"
#if 1
#pragma import(__use_no_semihosting)                          
struct __FILE 
{ 
	int handle; 
}; 
//串口输出函数重定向
FILE __stdout;        
void _sys_exit(int x) 
{ 
	x = x; 
} 

int fputc(int ch, FILE *f)
{ 	
	while((USART6->SR&0X40)==0);
	USART6->DR = (u8) ch;      
	return ch;
}
#endif
/****************************
USART6_Gpio_Config
入口参数：无
出口参数：无
功能：配置串口6GPI0
说明：  PG9--USART6_RX
			  PG14--UASRT6_TX
******************************/
void USART6_Gpio_Config(void)
{
	GPIO_InitTypeDef  gpio;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE);
	
	gpio.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_14;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOG,&gpio);
	
	GPIO_PinAFConfig(GPIOG,GPIO_PinSource14 ,GPIO_AF_USART6); //tx
  GPIO_PinAFConfig(GPIOG,GPIO_PinSource9,GPIO_AF_USART6);   //rx
}
/****************************
USART6_Gpio_Config
入口参数：无
出口参数：无
功能：配置串口6GPI0
说明：  PD9--USART3_RX
				PD8--UASRT3_TX
******************************/
/*void USART3_Gpio_Config(void)
{
	GPIO_InitTypeDef  gpio;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	
	gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD,&gpio);
	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource8 ,GPIO_AF_USART3);
  GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_USART3); 
} */
/****************************
函数名：USART3_Config
入口参数：无
出口参数：无
功能：配置USART3参数
说明：  波特率：115200
				数据位：8
				停止位：1
				奇偶校验位：无				 
******************************/
/*void USART3_Config(void)
{
	USART_InitTypeDef usart;
	
	USART3_Gpio_Config();

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	
	usart.USART_BaudRate = 115200;
	usart.USART_WordLength = USART_WordLength_8b;
	usart.USART_StopBits = USART_StopBits_1;
	usart.USART_Parity = USART_Parity_No;
	usart.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
	usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART3,&usart);
	
	USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);
	USART_Cmd(USART3,ENABLE);
	
	while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) != SET)
    ;
}*/
/****************************
函数名：USART6_Config
入口参数：无
出口参数：无
功能：配置USART6参数
说明：  波特率：115200
				数据位：8
				停止位：1
				奇偶校验位：无				 
******************************/
void USART6_Config(void)
{
	USART_InitTypeDef usart;
	
	USART6_Gpio_Config();
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);
	
	usart.USART_BaudRate = 115200;
	usart.USART_WordLength = USART_WordLength_8b;
	usart.USART_StopBits = USART_StopBits_1;
	usart.USART_Parity = USART_Parity_No;
	usart.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
	usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART6,&usart);
	
	USART_ITConfig(USART6,USART_IT_RXNE,ENABLE);
	USART_Cmd(USART6,ENABLE);
	
	while (USART_GetFlagStatus(USART6, USART_FLAG_TXE) != SET)
    ;
}
/********************************************
函数名：USARTxNVIC_Config
入口参数：无
出口参数：无
功能：配置USART3，USART6中断优先级
说明： USART3: 抢占优先级：3 响应优先级：0
		  USART6:  抢占优先级：3  响应优先级 1
******************************************/
void USARTxNVIC_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure_USART6;
	NVIC_InitTypeDef NVIC_InitStructure_USART3;
	
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
  NVIC_InitStructure_USART6.NVIC_IRQChannelPreemptionPriority = 3;
  NVIC_InitStructure_USART6.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure_USART6.NVIC_IRQChannel = USART6_IRQn;
  NVIC_InitStructure_USART6.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure_USART6);

  
  NVIC_InitStructure_USART3.NVIC_IRQChannelPreemptionPriority =  3;
  NVIC_InitStructure_USART3.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure_USART3.NVIC_IRQChannel = USART3_IRQn;
  NVIC_InitStructure_USART3.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure_USART3);
}
/*********************************************
函数名：UsartConfig
入口参数：无
出口参数：无
功能：配置USART3接收妙算数据，USART6向PC打印数据
***********************************************/
void UsartConfig(void)
{
	//USART3_Config();//妙算 115200
	USART6_Config();//打印数据 115200
	USARTxNVIC_Config();
}

TerminalCommand myTerminal;
/*********************************************
USART3_IRQHandler
入口参数：无
出口参数：无
功能：USART3中断函数，接收妙算数据
说明：数据帧开头：0XFF
			数据帧结尾：0xFE
***********************************************/
/*void USART3_IRQHandler(void)                	//MINIPC
	{
		if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  
		{
				uint8_t oneByte = USART_ReceiveData(USART3);	
				if (myTerminal.rxIndex == 0)	
				{
					if (oneByte == 0xFF)
					{
						myTerminal.cmdIn[myTerminal.rxIndex] = oneByte;
						myTerminal.rxIndex = 1;
					}
					else
						{;}
				}
				else
				{
					if (oneByte == 0xFE)    //receive a 0xFE would lead to a command-execution
					{
						myTerminal.cmdIn[myTerminal.rxIndex] = oneByte;
						myTerminal.rxLength = myTerminal.rxIndex + 1;
						myTerminal.rxIndex = 0;
						myTerminal.cmdReadyFlag = 1;
					}
					else
					{
						myTerminal.cmdIn[myTerminal.rxIndex] = oneByte;
						myTerminal.rxIndex++;
					}
				}
    } 
}  */
	
/*********************************************
USART6_IRQHandler
入口参数：无
出口参数：无
功能：USART3中断函数，接收上位机数据
说明：数据帧开头：0XFF
			数据帧结尾：0xFE
***********************************************/
void USART6_IRQHandler(void)                	 
	{
	
	if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)  
		{
				uint8_t oneByte = USART_ReceiveData(USART6);	
				if (myTerminal.rxIndex == 0)	
				{
					if (oneByte == 0xFF)
					{
						myTerminal.cmdIn[myTerminal.rxIndex] = oneByte;
						myTerminal.rxIndex = 1;
					}
					else
						{;}
				}
				else
				{
					if (oneByte == 0xFE)    //receive a 0xFE would lead to a command-execution
					{
						myTerminal.cmdIn[myTerminal.rxIndex] = oneByte;
						myTerminal.rxLength = myTerminal.rxIndex + 1;
						myTerminal.rxIndex = 0;
						myTerminal.cmdReadyFlag = 1;
					}
					else
					{
						myTerminal.cmdIn[myTerminal.rxIndex] = oneByte;
						myTerminal.rxIndex++;
					}
				}
    } 
} 

