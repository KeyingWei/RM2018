#include "headfile.h"
//VP230---CAN_TX---PA12(CANTX) 
//VP230---CAN_RX---PA11(CANRX) 

/*------------------------------------------------------------------------------
                             CAN1接收电机数据
显性电平对应逻辑0，CAN_H为3.5v,CAN_L为1.5v,压差2v，
隐性电平对应逻辑1，CAN_H，CAN_L都为2.5v,压差0v
STM32F4有28个筛选器组，一个筛选器组有2个32位的寄存器，1个32的寄存器可拆分为2个16的寄存器
筛选器模式可设定为位屏蔽模式和列表模式，屏蔽位模式分为1个32位的期望ID和一个32位的掩码，或
者2个16位的期望ID和两个16位的掩码，筛选器只关心掩码为1的位，
列表模式可设定2个32位的期望ID或4个16位的期望ID，要求必须完全匹配

------------------------------------------------------------------------------*/
/******************************
函数名：CAN1_Gpio_Config
入口参数：无
出口参数：无
功能：初始化CAN1_GPIO
说明：PD1:CAN1_TX
      PD0:CAN1_RX
*******************************/
static void CAN1_Gpio_Config(void)
{
	GPIO_InitTypeDef       gpio;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);	//开启GPIOD时钟
	
	gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  gpio.GPIO_Mode = GPIO_Mode_AF;		       //开启复用功能										
  gpio.GPIO_Speed = GPIO_Speed_100MHz; //设置IO速度
	GPIO_Init(GPIOD, &gpio);
	
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_CAN1);//PDO映射为CAN1_L
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_CAN1);//PD0映射为CAN1_H
}
/******************************
函数名：CANxNVIC_Config
入口参数：无
出口参数：无
功能：设置CAN1的接收中断
*******************************/
static void CANxNVIC_Config(void)
{
	NVIC_InitTypeDef       nvic;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置中断优先级分组2
	
	nvic.NVIC_IRQChannel = CAN1_RX0_IRQn;					//开启CAN1接收中断
  nvic.NVIC_IRQChannelPreemptionPriority = 1;   //响应优先级1
  nvic.NVIC_IRQChannelSubPriority = 1;          //抢占优先级2
  nvic.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic);
	
  nvic.NVIC_IRQChannel = CAN1_TX_IRQn;					//开启CAN1接收中断
  nvic.NVIC_IRQChannelPreemptionPriority = 0;   //响应优先级1
  nvic.NVIC_IRQChannelSubPriority = 1;          //抢占优先级2
  nvic.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic);
}
/******************************
函数名：CAN1_Config
入口参数：无
出口参数：无
功能：配置CAN1的位时序即设置通信波特率和CAN1的筛选器
波特率计算 baud(Mbps) =（45/(CAN_SJW_1tq+CAN_BS2_6tq+CAN_BS1_8tq)）/CAN_Prescaler 
*******************************/
static void CAN1_Config(void)
{
	CAN_InitTypeDef        can;
	CAN_FilterInitTypeDef  can_filter;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	
	CAN_DeInit(CAN1);
  CAN_StructInit(&can);
  /************CAN总线的配置*******************/
  can.CAN_TTCM = DISABLE;			//非时间触发通信模式
  can.CAN_ABOM = DISABLE;			//软件自动离线管理模式
  can.CAN_AWUM = DISABLE;			//自动唤醒模式，睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
  can.CAN_NART = DISABLE;			//非自动重传输模式，禁止报文自动传送 
  can.CAN_RFLM = DISABLE;			//接收FIFO锁定模式，报文不锁定,新的覆盖旧的 
  can.CAN_TXFP = ENABLE;			//发送FIFO优先迹，优先级由报文标识符决定 
  can.CAN_Mode = CAN_Mode_Normal;		//模式设置： mode:0,普通模式;1,回环模式;
  can.CAN_SJW  = CAN_SJW_1tq; 
  can.CAN_BS1 = CAN_BS2_6tq;
  can.CAN_BS2 = CAN_BS1_8tq;
  can.CAN_Prescaler = 3;   //CAN BaudRate 45/(1+6+8)/3=1Mbps
  CAN_Init(CAN1, &can);
	
	can_filter.CAN_FilterNumber=0;//选择筛选器组0，F4共有28个筛选器组
  can_filter.CAN_FilterMode=CAN_FilterMode_IdMask;
  can_filter.CAN_FilterScale=CAN_FilterScale_32bit;//32位掩码模式，筛选器的R1寄存器用来存放期望的ID,R2存放掩码，筛选器只关心掩码为1的位
  can_filter.CAN_FilterIdHigh=0x0000;
  can_filter.CAN_FilterIdLow=0x0000;
  can_filter.CAN_FilterMaskIdHigh=0x0000;                      //选择接收所有的ID
  can_filter.CAN_FilterMaskIdLow=0x0000;
  can_filter.CAN_FilterFIFOAssignment=0;//the message which pass the filter save in fifo0
  can_filter.CAN_FilterActivation=ENABLE;
  CAN_FilterInit(&can_filter);			//滤波器初始化
    
  CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);		//接收中断
  CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE); 		//发送中断
}
/*****************************************
函数名：CAN1Config
入口参数：无
出口参数：无
功能：配置CAN1的GPIO,NVIC,筛选器，位时序
******************************************/
void CAN1Config(void)
{
	CAN1_Gpio_Config();
	CANxNVIC_Config();
	CAN1_Config();
}


