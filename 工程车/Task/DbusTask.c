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
		}break;
		case KEY_MOUSE_INPUT:
		{
			MouseKeyboardControlPress(mydata);

		}break;
		case STOP:
		{
				
		}break; 
	}
}

void RemoteControlProcess(MyData *mydata)
{

				mydata->Cortol_ch0 = (mydata->dbus.remote.ch0 - 1024) * 10;
				mydata->Cortol_ch1 = (mydata->dbus.remote.ch1 - 1024) * 10;
	     	mydata->Cortol_ch2 = (mydata->dbus.remote.ch2 - 1024) * 15;					
}

static void MouseKeyboardControlPress(MyData *mydata)
{
	
	 static char flag1 = 1;
	 static int cnt = 0,cnt1 =0,controlval= 15,controlval1 = 15;
   mydata->Cortol_ch2 = mydata->dbus.mouse.x   *  180.0f;
	 
	if(mydata->dbus.mouse.press_l == 1)
	{
	   flag1 = 1;
	}
	else if(mydata->dbus.mouse.press_r == 1)
	{
	  flag1 = 2;
	}
	
	if(flag1 == 1)
	{
			 switch(mydata->dbus.key.v)
			 {
					case  KEY_PRESSED_OFFSET_W:
																		mydata->Cortol_ch0=0;if(mydata->Cortol_ch1 <= 9000) mydata->Cortol_ch1  += 400;break; 
					case  KEY_PRESSED_OFFSET_S:
																		mydata->Cortol_ch0=0;if(mydata->Cortol_ch1 >= -9000) mydata->Cortol_ch1 -= 400;break;
					case  KEY_PRESSED_OFFSET_A:
																		mydata->Cortol_ch1=0;if(mydata->Cortol_ch0 >= -9000) mydata->Cortol_ch0  -= 400;break;
					case  KEY_PRESSED_OFFSET_D:
																		mydata->Cortol_ch1=0;if(mydata->Cortol_ch0 <= 9000)  mydata->Cortol_ch0 += 400;break;
					default:           				
									 if( mydata->Cortol_ch0 >0) 	  mydata->Cortol_ch0 -= 400	;
									 else if( mydata->Cortol_ch0 <0)mydata->Cortol_ch0 += 400	;	
									 if( mydata->Cortol_ch1 >0) 	  mydata->Cortol_ch1 -= 400	;
									 else if( mydata->Cortol_ch1 <0)mydata->Cortol_ch1 += 400	;
									 break;			
			 }		  
	}
	else if(flag1 == 2)
	{
 
			 switch(mydata->dbus.key.v)
			 {
					case  KEY_PRESSED_OFFSET_W:
							cnt++;
							if(cnt >= 10)
							{
								 cnt =0;
								 controlval++;
							}
							if(controlval >=25 ) controlval=25; 
						                         ;break;
					case  KEY_PRESSED_OFFSET_S:
							cnt++;
							if(cnt >= 10)
							{
								 cnt =0;
								 controlval--;
							}
							if(controlval <=5 ) controlval=5;						
																		 ;break;
					case  KEY_PRESSED_OFFSET_A:
							cnt++;
							if(cnt >= 10)
							{
								 cnt =0;
								 controlval1++;
							}
							if(controlval1 >=25 ) controlval1=25;						
																		 ;break;
					case  KEY_PRESSED_OFFSET_D:
							cnt++;
							if(cnt >= 10)
							{
								 cnt =0;
								 controlval1--;
							}
							if(controlval1 <=5 ) controlval1=5;
							break;
					default:           				
									 break;			
        }

	}
		CAMERA_LR = controlval;
		CAMERA_UD = controlval1;
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
