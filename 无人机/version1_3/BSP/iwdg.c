#include "headfile.h"

DOG Dog = DogConfig;

void IWDG_Init(void)
{
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); //使能对IWDG->PR IWDG->RLR的写 取消写保护
	
	IWDG_SetPrescaler(4); //设置IWDG分频系数 ,设置计数频率为 0.5KHz  tout = ((4*2^prer)*rlr)/32=1s（内部LSI为32K）

	IWDG_SetReload(750);   //设置IWDG装载值

	IWDG_ReloadCounter(); //reload
	
	IWDG_Enable();       //使能看门狗
}

uint32_t global_cout[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/*--------------------------------------
检查遥控前4个通道数据是否正常
检查dbus是否掉线
数据正常则喂狗
数据不正常则不喂狗将重新初始化
--------------------------------------*/
void DogFeed(void)
{
	static u8 DogFlag = 0;
	if((mydata.dbus.remote.ch0 > 363 || mydata.dbus.remote.ch1 > 363 || mydata.dbus.remote.ch2 > 363 || mydata.dbus.remote.ch3 > 363) && 
			(mydata.dbus.remote.ch0 < 1685 || mydata.dbus.remote.ch1 < 1685 || mydata.dbus.remote.ch2 < 1685 || mydata.dbus.remote.ch3 < 1685))
	{
		DogFlag = (Dog.DbusReceive | Dog.Dbus << 1 | 1 << 2 | Dog.Reset << 3);
		if((DogFlag & 0x0f) == 0x0f)
		{
			IWDG_ReloadCounter();//喂狗
		}
	}
	check(global_cout);
}

void check(uint32_t * local_cout)
{
	u8 i = 0;
	for(i = 0; i < 8; i++)
	{
		if(local_cout[i] < 250)
		{
			local_cout[i] ++;
		}
	}
	for(i = 0; i < 8; i++)
	{
		if(local_cout[i] < 8)
		{
			Dog.rc = OL;
		}
		if(local_cout[i] > 10)
		{
			GetLostError(i);
		}
	}
}

void GetLostError(u8 code)
{
	switch(code)
	{
		case LOST_COUNTER_INDEX_MOTOR1: printf("lost motor ID201\r\n");
			break;
		case LOST_COUNTER_INDEX_MOTOR2: printf("lost motor ID202\r\n");
			break;
		case LOST_COUNTER_INDEX_MOTOR3: printf("lost motor ID203\r\n");
			break;
		case LOST_COUNTER_INDEX_MOTOR4: printf("lost motor ID204\r\n");
			break;
		case LOST_COUNTER_INDEX_MOTOR5: printf("lost motor ID205\r\n");
			break;
		case LOST_COUNTER_INDEX_MOTOR6: printf("lost motor ID206\r\n");
			break;
		case LOST_COUNTER_INDEX_MOTOR7: printf("lost motor ID207\r\n");
			break;
		case LOST_COUNTER_INDEX_RC: 		printf("lost rc\r\n");
																		Dog.rc = Lose;
			break;
		default:break;
	}
}
