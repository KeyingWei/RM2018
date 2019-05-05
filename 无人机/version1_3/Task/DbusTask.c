#include "headfile.h"

static volatile InputMode_e inputmode = REMOTE_INPUT;   //输入模式设定
static void MouseKeyboardControlPress(MyData *mydata);

InputMode_e GetInputMode()
{
	return inputmode;
}

void ReceiveDbusData(MyData *mydata)			
{
	Dog.DbusReceive = 1;
	global_cout[LOST_COUNTER_INDEX_RC] = Online; 
	mydata->dbus.remote.ch0 = (sbus_rx_buffer[0]| (sbus_rx_buffer[1] << 8)) & 0x07ff; 						//!< Channel 0
	mydata->dbus.remote.ch1 = ((sbus_rx_buffer[1] >> 3) | (sbus_rx_buffer[2] << 5)) & 0x07ff; 		//!< Channel 1
	mydata->dbus.remote.ch2 = ((sbus_rx_buffer[2] >> 6) | (sbus_rx_buffer[3] << 2) | (sbus_rx_buffer[4] << 10)) & 0x07ff;//!< Channel 2
	mydata->dbus.remote.ch3 = ((sbus_rx_buffer[4] >> 1) | (sbus_rx_buffer[5] << 7)) & 0x07ff; //!< Channel 3
	mydata->dbus.remote.s1 = ((sbus_rx_buffer[5] >> 4)& 0x000C) >> 2;															//!< Switch left
	mydata->dbus.remote.s2 = ((sbus_rx_buffer[5] >> 4)& 0x0003); 																	//!< Switch right	
	mydata->dbus.mouse.x = sbus_rx_buffer[6] | (sbus_rx_buffer[7] << 8); 											//!< Mouse X axis
	mydata->dbus.mouse.y = sbus_rx_buffer[8] | (sbus_rx_buffer[9] << 8);										  //!< Mouse Y axis
	mydata->dbus.mouse.z = sbus_rx_buffer[10] | (sbus_rx_buffer[11] << 8); 										//!< Mouse Z axis
	mydata->dbus.mouse.press_l = sbus_rx_buffer[12]; 																					//!< Mouse Left Is Press ?
	mydata->dbus.mouse.press_r = sbus_rx_buffer[13]; 																					//!< Mouse Right Is Press ?
	mydata->dbus.key.v = sbus_rx_buffer[14] | (sbus_rx_buffer[15] << 8); 											//!< KeyBoard value
	
	SetInputMode(mydata);
	switch(GetInputMode())
	{
		case REMOTE_INPUT:
		{
			RemoteControlProcess(mydata);
			Bsp.OpenTheLaser(1);
			UNDERCARRIAGE_PWM = DOWN;
		}break;
		case KEY_MOUSE_INPUT:
		{
      MouseKeyboardControlPress(mydata);
		  Bsp.OpenTheLaser(1);
		}break;
		case STOP:
		{
		//	 UNDERCARRIAGE_PWM = UP;	
		}break; 
	}
}

void RemoteControlProcess(MyData *mydata)
{
    if(GetWorkState()!=PREPARE_STATE)
		{
				mydata->Cortol_ch0 = (mydata->dbus.remote.ch0 - 1024) * 22;
				mydata->Cortol_ch1 = (mydata->dbus.remote.ch1 - 1024) * 22;
				mydata->Cortol_ch2 -= (mydata->dbus.remote.ch2 - 1024) * 0.01f;
				mydata->Cortol_ch3 -= (mydata->dbus.remote.ch3 - 1024) * 0.1f;
			
			 if(mydata->Cortol_ch3 < -280) 	//上下移动限位 
						mydata->Cortol_ch3 = -280;
	     else 	if(mydata->Cortol_ch3 > 180) 
						mydata->Cortol_ch3 = 180;	 
			 
			 if(mydata->Cortol_ch2 < -130)  //左右移动限位
						mydata->Cortol_ch2 = -130;
	     else 	if(mydata->Cortol_ch2 > 50) 
						mydata->Cortol_ch2 = 50;	 
	 
		}
		
		
		if(mydata->dbus.remote.s1 == MOVE_UP)
		{
			 Set_Fire_Fre(0);  	  
			 Bsp.TurnOnTheShootMotor(0,0);
		}
		else if(mydata->dbus.remote.s1 == MOVE_MID)
		{
	
     		Set_Fire_Fre(0);		
			 Bsp.TurnOnTheShootMotor(1,120);
		}
		else  if(mydata->dbus.remote.s1 == MOVE_DOWM)
		{
 			Set_Fire_Fre(1000);	
			Bsp.TurnOnTheShootMotor(1,120);
		
		}
		
}

static void MouseKeyboardControlPress(MyData *mydata)
{

	static char middle_flag =0;
	 
	 switch(mydata->dbus.key.v)
	 {
	    case  KEY_PRESSED_OFFSET_W:
		                         mydata->Cortol_ch3  -= 5.9f;break; //调节云台左右移动速度
			case  KEY_PRESSED_OFFSET_S:
														 mydata->Cortol_ch3  += 5.9f;;break;
			case  KEY_PRESSED_OFFSET_A:
														mydata->Cortol_ch2 += 1.5f;break;//调节云台上下移动速度
		  case  KEY_PRESSED_OFFSET_D:
														mydata->Cortol_ch2  -= 1.5f;break;
			default:  
				   ;break;
				
	 }	
	
	    if(mydata->dbus.key.v == KEY_PRESSED_OFFSET_Q)
			{
				  middle_flag = 1;
			}
			
			if(middle_flag == 1)
			{
							if(mydata->Cortol_ch2 > 2)
							 {
									 mydata->Cortol_ch2 -=0.8f;   
							 }
							 else if(mydata->Cortol_ch2 < -2)
							 {
								  mydata->Cortol_ch2 +=0.8f;    
							 }
						  else
						   {
							    middle_flag = 0;
						   }						 
			}
			
			if(mydata->dbus.mouse.press_l  == 1)
			{
	 			Set_Fire_Fre(400);	//调射频
		  	Bsp.TurnOnTheShootMotor(1,125);		 //调射速 
			}
			else
			{
 		   	Set_Fire_Fre(0);	
			  Bsp.TurnOnTheShootMotor(1,125);			  
			}
	 
	 
	 

			 if(mydata->Cortol_ch3 < -320) 	//上下移动限位 
						mydata->Cortol_ch3 = -320;
	     else 	if(mydata->Cortol_ch3 > 220) 
						mydata->Cortol_ch3 = 220;	 
			 
			 if(mydata->Cortol_ch2 < -130)  //左右移动限位
						mydata->Cortol_ch2 = -130;
	     else 	if(mydata->Cortol_ch2 > 50) 
						mydata->Cortol_ch2 = 50;	 
	 
	 
}

void SetInputMode(MyData *mydata)
{
	if(mydata->dbus.remote.s2 == MOVE_UP)
	{
		inputmode = REMOTE_INPUT;
	}
	else if(mydata->dbus.remote.s2 == MOVE_MID)
	{
		inputmode = KEY_MOUSE_INPUT;
	}
	else if(mydata->dbus.remote.s2 == MOVE_DOWM)
	{
		inputmode = STOP;
	}	
}

u8 RemoteSelfCheck(void)
{
	if((mydata.dbus.remote.ch0 != 1024) || (mydata.dbus.remote.ch1 != 1024) ||(mydata.dbus.remote.ch2 != 1024) || (mydata.dbus.remote.ch3 != 1024))
	{
		return false;
	}
	else 
		return true;
}

void DMA2_Stream5_IRQHandler(void)	
{

  if(DMA_GetFlagStatus(DMA2_Stream5,DMA_IT_TCIF5)==SET) 
	{
		DMA_ClearFlag(DMA2_Stream5,DMA_IT_TCIF5); 
		DMA_ClearITPendingBit(DMA2_Stream5,DMA_IT_TCIF5);
		ReceiveDbusData(&mydata);				
	}
}
