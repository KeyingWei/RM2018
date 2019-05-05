#include "main.h"

u8 sign = 0;
void System_Init(void)
{
	SPI5_Init();
	SPI5_SetSpeed(SPI_BaudRatePrescaler_128);
	MPU6500_Init();
	
	Underpan_Pid_Init(); //µ×ÅÌPID
}






