#include "headfile.h"
/****************************
��������LEDConfig
��ڲ�������
���ڲ�������
���ܣ���ʼ��LED_GPIO
˵����PE7 --��ɫLED
      PE14 --��ɫLED
			�͵�ƽ����
******************************/
void LEDConfig(void)
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOF, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;		 //PE7                                                                                                                                                                                                                                                                                         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOE,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;		//PF14  
	GPIO_Init(GPIOF,&GPIO_InitStructure);  
}



void Catch_Gpio_Config(void)
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;		 //PI0 ����ץȡ��ҩ��ļ̵���                                                                                                                                                                                                                                                                                    
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOI,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;		
	GPIO_Init(GPIOI,&GPIO_InitStructure);  
	GPIO_SetBits(GPIOI, GPIO_Pin_0);
	
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 |GPIO_Pin_11 ;		 //PH12   ����������ֹܵļ̵���                                                                                                                                                                                                                                                                                   
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP  ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOH,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 ;	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 ;		
	GPIO_Init(GPIOH,&GPIO_InitStructure); 
	GPIO_SetBits(GPIOH, GPIO_Pin_11);	
	GPIO_SetBits(GPIOH, GPIO_Pin_12);
	
}
