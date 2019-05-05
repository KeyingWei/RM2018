#include "headfile.h"

BSP Bsp = BSPConfig;
     

void BSP_Init(void)
{
	PwmConfig(); 
	LEDConfig();
	LaserConfig();
	BeepConfig();
	DelayConfig(168);
	MainFocusConfig();
	UsartConfig();	
	MPU6500Config();
	TimConfig();
	DbusConfig();
	CAN1Config();
	//CAN2Config();
	StmflashRead();
	MPU6500_InitGyro_offset();
	IMU_INT_Config();
	IWDG_Init();
	
	
	delay_ms(500);
	
	Referee_init();
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
	if(status)
		SteeringEngine = 24;
	else 
		SteeringEngine = 15;
}

void TurnOnTheShootMotor(bool status)
{
	if(status)
		ShootMotor_Left = ShootMotor_Right = 147;
	else 
		ShootMotor_Left = ShootMotor_Right = 100;
}

