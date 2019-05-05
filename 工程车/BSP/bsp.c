#include "headfile.h"

BSP Bsp = BSPConfig;
     
char init_flag=0;
void BSP_Init(void)
{
	PwmConfig(); 
	LEDConfig();
	LaserConfig();
	BeepConfig();
	Infrared_init();
  Spaing_Limit_Init();	
	DelayConfig(168);
	UsartConfig();	
	MPU6500Config();
	TimConfig();
	DbusConfig();
	
	CAN1Config();
	CAN2Config();
	
	Catch_Gpio_Config();
	delay_ms(500);

//	StmflashRead();
	MPU6500_InitGyro_offset();
	IMU_INT_Config();
	//IWDG_Init();
	
//	Referee_init();
}


void OpenTheCatcher(bool status)//夹取气缸
{
	if(status)
		GPIO_ResetBits(GPIOI, GPIO_Pin_0);
	else 
		GPIO_SetBits(GPIOI, GPIO_Pin_0);
}

void OpenTheBall(bool status)//弹仓气缸控制
{
	if(status)
		GPIO_ResetBits(GPIOH, GPIO_Pin_12);
	else 
		GPIO_SetBits(GPIOH, GPIO_Pin_12);
}


int16_t g_smallwheel_speed = 0;
void LangSmallWheel(char status,int position)//小轮子控制2310电机
{
	if(status == 1)
	{
		
		if(CAN2_Moter_Position_PID[5].ange_set > -position)
			  CAN2_Moter_Position_PID[5].ange_set -= 50;
		
   if(CAN2_Moter_Position_PID[4].ange_set < position)
			  CAN2_Moter_Position_PID[4].ange_set += 50;
	}
	else
	{
			if(CAN2_Moter_Position_PID[5].ange_set < 150)
			  CAN2_Moter_Position_PID[5].ange_set += 50;
		
     if(CAN2_Moter_Position_PID[4].ange_set > -150)
			  CAN2_Moter_Position_PID[4].ange_set -= 50;	  
	}

}


void OpenTheGreenLed(bool status)
{
	if(status)
		GPIO_ResetBits(GPIOF, GPIO_Pin_14);
	else 
		GPIO_SetBits(GPIOF, GPIO_Pin_14);
}

void OpenTheRedLed(bool status)
{
	if(status)
		GPIO_ResetBits(GPIOE, GPIO_Pin_7);
	else 
		GPIO_SetBits(GPIOE, GPIO_Pin_7);
}

void OpenTheLaser(bool status)
{
	if(status)
		GPIO_SetBits(GPIOG, GPIO_Pin_13);
	else 
		GPIO_ResetBits(GPIOG, GPIO_Pin_13);
}

void OpenTheMagazine(bool status)
{
//	if(status)
	//	BULLET_PWM = 24 ;
//	else 
	//	BULLET_PWM = 15;
}

void TurnOnTheShootMotor(bool status)
{
//	if(status)
	//	ShootMotor_Left = ShootMotor_Right = 147;
//	else 
	//	ShootMotor_Left = ShootMotor_Right = 100;
}

