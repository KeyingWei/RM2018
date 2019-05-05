#ifndef __USART_H__
#define __USART_H__

#include <stm32f4xx.h>

#define MAX_RECEIVE 32

typedef struct _TerminalCommand_
{
  uint32_t cmdReadyFlag; //Rx_Handle_Flag
  uint8_t  cmdIn[MAX_RECEIVE]; //Rx_buff
  int32_t  rxIndex;  //Rx_adr
  int32_t  rxLength; //Rx_length
	uint8_t  SendFlag;
}TerminalCommand;

void UsartConfig(void); //´®¿Ú³õÊ¼»¯


#endif

