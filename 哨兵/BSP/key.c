#include "main.h"

void Key_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);//使能GPIOA,GPIOE时钟
 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_2 | GPIO_Pin_0; //KEY0 KEY1 KEY2对应引脚
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOI, &GPIO_InitStructure);//初始化GPIOI2,5,6,7

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12; //KEY0 KEY1 KEY2对应引脚
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOH, &GPIO_InitStructure);//初始化GPIOI2,5,6,7	
} 

u8 AutoLR_flag = 0;
u8 AutoUD_flag = 0;
u8 AutoSP_flag = 0;

int16_t LR_Speed = 300;
int16_t UD_Speed = 300;
int16_t Speed = 300;
/*-------------------------------------------------------------------------------
如果你想理解这段代码的话，建议先画个图。
虽然感觉很烂，但是勉强能凑合着用用。懒得修改了=_=
鬼知道了我经历了什么
-------------------------------------------------------------------------------*/
void Auto_Control(void)
{
	if(AutoUD_flag == 0)
	{
			Control_data.RC_ch1 = UD_Speed;  //up
	}
	else
	{
			Control_data.RC_ch1 = -UD_Speed;  //buck
	}
	
	if(AutoLR_flag == 0)
	{
			Control_data.RC_ch0 = -LR_Speed;  
	}
	else 
	{
			Control_data.RC_ch0 = LR_Speed; 
	}
	
	if(AutoSP_flag == 0)
	{
			Control_data.RC_ch2 = -Speed;  
	}
	else 
	{
			Control_data.RC_ch2 = Speed; 
	}
	
	if(key_up == 0)
	{
			AutoUD_flag = 1;
	}
	if(key_dowm == 0)
	{
			AutoUD_flag = 0;
	}

	if((key_right == 0) | (key_rightup == 0) | (key_leftdowm == 0))
	{
			AutoLR_flag = 0;
	}
	
	if((key_left == 0) | (key_leftup == 0) | (key_rightdowm == 0))
	{
			AutoLR_flag = 1;
	}

	
	if(((key_left == 0) && (key_up == 0)) | ((key_right == 0) && (key_dowm == 0)))
	{
			AutoSP_flag = 1;
	}
	else if(((key_left == 0) && (key_dowm == 0)) | ((key_right == 0) && (key_up == 0)))
	{
			AutoSP_flag = 0;
	}

	
	
}
