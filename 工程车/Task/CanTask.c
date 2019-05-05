#include "headfile.h"
#include "pid_modify.h"
static int16_t cout_p = 0, cout_y = 0;
static int16_t new_data_p, old_data_p;
static int16_t new_data_y, old_data_y;

int16_t CAN1_Motor_Speed[8]={0};
int16_t CAN2_Motor_Speed[8]={0};

long int catch_encoder_value = 0;

/* Get the value of the continuous encoder of the pitch axis */
/*************************************************************************
函数名：encoder_value_for_pitch
入口参数：
         input_data：P轴接收到的机械角
出口参数：无
功能：把P轴的机械角进行连续化处理
说明：当码盘值到临界时，比如上个数据是10,下个数据是-8000，说明已经过了临界角，
需进行连续化处理，否则得到的码盘值是错的，连续化处理的方法是当到达临界角时，顺
时针经过临界角时，圈数加1，逆时针转过时减1
*************************************************************************/
int encoder_value_for_pitch(int16_t input_data)  
{
	old_data_p = new_data_p;
	new_data_p = input_data;
	
	if((new_data_p - old_data_p) < -7000)
		cout_p++;
	else if((new_data_p - old_data_p) > 7000)
		cout_p--;
	return new_data_p + cout_p * 8191;
}

/* Get the value of the continuous encoder of the yaw axis */
/*************************************************************************
函数名：encoder_value_for_yaw
入口参数：
         input_data：Y轴接收到的机械角
出口参数：无
功能：把Y轴的机械角进行连续化处理
*************************************************************************/
int encoder_value_for_yaw(int16_t input_data)
{
	old_data_y = new_data_y;
	new_data_y = input_data;
	
	if((new_data_y - old_data_y) < -7000)
		cout_y++;
	else if((new_data_y - old_data_y) > 7000)
		cout_y--;
	return new_data_y + cout_y * 8191;
}

int encoder_value_for_catch(int16_t input_data)
{
	old_data_y = new_data_y;
	new_data_y = input_data;
	
	if((new_data_y - old_data_y) < -7000)
		cout_y++;
	else if((new_data_y - old_data_y) > 7000)
		cout_y--;
	return new_data_y + cout_y * 8191;
}


long int encoder_value_for_Moter(p_position_pid encoder,int16_t input_data)
{
	encoder->encoder_val_old_data = encoder->encoder_val_new_data;
	encoder->encoder_val_new_data = input_data;
	
	if((encoder->encoder_val_new_data - encoder->encoder_val_old_data) < -6000)
		encoder->encoder_count++;
	else if((encoder->encoder_val_new_data - encoder->encoder_val_old_data) > 6000)
		encoder->encoder_count--;
	return encoder->encoder_val_new_data + encoder->encoder_count * 8191;
}



/* Get the motor all the motor parameters */
/*************************************************************************
函数名：ReceiveCAN1Data
入口参数：无
出口参数：无
功能：根据电调ID接收电调的转速及机械角数据
注意：
电调发送频率1kHz；
机械角度值的范围：0~8191(0x1FFF)
实际电流测量值范围：-13000 ~ 13000
*************************************************************************/

  int16_t  encoder_init_val = 0 ;

void ReceiveCAN1Data(MyData *mydata)
{
	CanRxMsg rx_message; 
	CAN_Receive(CAN1, CAN_FIFO0, &rx_message);
	
	switch(rx_message.StdId)
	{	
		case CAN_BUS1_MOTOR1:
		{             
			global_cout[LOST_COUNTER_INDEX_MOTOR1] = Online;
			CAN1_Motor_PID[0].get = (rx_message.Data[2]<<8) + rx_message.Data[3];	
		}
		break;
		
		case CAN_BUS1_MOTOR2:
		{ 
			global_cout[LOST_COUNTER_INDEX_MOTOR2] = Online;
			CAN1_Motor_PID[1].get= (rx_message.Data[2]<<8) + rx_message.Data[3];
		}	
		break;
			
		case CAN_BUS1_MOTOR3:
		{ 
			global_cout[LOST_COUNTER_INDEX_MOTOR3] = Online;
			CAN1_Motor_PID[2].get = (rx_message.Data[2]<<8) + rx_message.Data[3];			
		}
		break;
			
		case CAN_BUS1_MOTOR4:
		{
			global_cout[LOST_COUNTER_INDEX_MOTOR4] = Online;
			CAN1_Motor_PID[3].get = (rx_message.Data[2]<<8) + rx_message.Data[3];
		}
		break;			
		
		case CAN_BUS1_MOTOR5:
		{
			global_cout[LOST_COUNTER_INDEX_MOTOR5] = Online;
			CAN1_Motor_PID[4].get = (rx_message.Data[2]<<8) + rx_message.Data[3];
			catch_encoder_value = (encoder_value_for_catch((rx_message.Data[0]<<8) + rx_message.Data[1]))/19;
			if(init_flag==0)
			{
			   encoder_init_val = catch_encoder_value;	
				 init_flag  = 1;
			}
				
		}
		break;
		
		case CAN_BUS1_MOTOR6:
		{
			global_cout[LOST_COUNTER_INDEX_MOTOR6] = Online;
			
			CAN1_Motor_PID[5].get = (rx_message.Data[2]<<8) + rx_message.Data[3];
			CAN1_Moter_Position_PID[5].speed_get = (rx_message.Data[2]<<8) + rx_message.Data[3];
			CAN1_Moter_Position_PID[5].ange_get  = encoder_value_for_Moter(&CAN1_Moter_Position_PID[5],(rx_message.Data[0] << 8) | rx_message.Data[1])/19;//码盘值连续化处理
			if(init_flag == 2)
			{
			   init_flag = 3;
				 CAN1_Moter_Position_PID[5].init_angle =  CAN1_Moter_Position_PID[5].ange_get;
			}
		}
		break;
		
		case CAN_BUS1_MOTOR7:
		{
			global_cout[LOST_COUNTER_INDEX_MOTOR7] = Online; 
		  CAN1_Moter_Position_PID[6].speed_get = (rx_message.Data[2]<<8) + rx_message.Data[3];
			CAN1_Moter_Position_PID[6].ange_get  = encoder_value_for_Moter(&CAN1_Moter_Position_PID[6],(rx_message.Data[0] << 8) | rx_message.Data[1])/19;//码盘值连续化处理
			if(init_flag == 1)
			{
			   init_flag = 2;
				 CAN1_Moter_Position_PID[6].init_angle =  CAN1_Moter_Position_PID[6].ange_get;
			}
		}
		break;
		
		default:break;
	}
}

/*       CAN1_RX0_IRQHandler           */
/*************************************************************************
函数名：CAN1_RX0_IRQHandler
入口参数：无
出口参数：无
功能：CAN接收中断接收电调发送的机械角及转速数据
注意：
电调发送频率1kHz；
机械角度值的范围：0~8191(0x1FFF)
实际电流测量值范围：-13000 ~ 13000
*************************************************************************/
void CAN1_RX0_IRQHandler(void)
{
	if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET) 
	{
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);		
		CAN_ClearFlag(CAN1, CAN_FLAG_FF0); 
		ReceiveCAN1Data(&mydata);
	}
}

void CAN1_TX_IRQHandler()
{
  if(CAN_GetITStatus(CAN1,CAN_IT_TME) !=RESET)
	{
	   CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
	 }
}
/****************************************************************************************************************************
函数原型：void Set_Gimbal_Current(CAN_TypeDef *CANx, int16_t gimbal_yaw_iq, int16_t gimbal_pitch_iq, int16_t gimbal_fire_iq)
入口参数：
1.CAN_TypeDef *CANx 指定是主CAN1还是从CAN2
2. int16_t gimbal_yaw_iq  云台ID为205的Y轴电调发送的转矩电流数据
3. int16_t gimbal_pitch_iq云台ID为206的Y轴电调发送的转矩电流数据
4. int16_t gimbal_fire_iq 云台ID为207的拨弹电调发送的转矩电流数据
5.int16_t cm4_iq 底盘ID为204的电调发送的转矩电流数据
出口参数：无
功能：发送云台电调的转矩电流数据
*****************************************************************************************************************************/
void CAN1_Moter_5_7_Send(CAN_TypeDef *CANx, int16_t gimbal_yaw_iq, int16_t gimbal_pitch_iq, int16_t gimbal_fire_iq)
{
    CanTxMsg tx_message;  

      
    tx_message.StdId = 0x1FF;//发送标准ID
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;//发送数据帧
    tx_message.DLC = 0x08;//数据帧发送8个字节的数据
    
    tx_message.Data[0] = (unsigned char)(gimbal_yaw_iq >> 8);
    tx_message.Data[1] = (unsigned char)gimbal_yaw_iq;
    tx_message.Data[2] = (unsigned char)(gimbal_pitch_iq >> 8);
    tx_message.Data[3] = (unsigned char)gimbal_pitch_iq;
    tx_message.Data[4] = (unsigned char)(gimbal_fire_iq >> 8);
    tx_message.Data[5] = (unsigned char)gimbal_fire_iq;
    tx_message.Data[6] = 0x00;
    tx_message.Data[7] = 0x00;
    CAN_Transmit(CANx,&tx_message);
}

/*************************************************************************************************************
函数原型：void Set_CM_Speed(CAN_TypeDef *CANx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq)
入口参数：
1.CAN_TypeDef *CANx 指定是主CAN1还是从CAN2
2.int16_t cm1_iq 底盘ID为201的电调发送的转矩电流数据
3.int16_t cm2_iq 底盘ID为202的电调发送的转矩电流数据
4.int16_t cm3_iq 底盘ID为203的电调发送的转矩电流数据
5.int16_t cm4_iq 底盘ID为204的电调发送的转矩电流数据
出口参数：无
功能：发送底盘电调的转矩电流数据
**************************************************************************************************************/

void CAN1_Moter_1_4_Send(CAN_TypeDef *CANx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq)
{
    CanTxMsg tx_message;
    tx_message.StdId = 0x200;//标准ID
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;//发送数据帧
    tx_message.DLC = 0x08;//发送八个字节的数据
    
    tx_message.Data[0] = (uint8_t)(cm1_iq >> 8);//数据高八位
    tx_message.Data[1] = (uint8_t)cm1_iq;         //数据低八位
    tx_message.Data[2] = (uint8_t)(cm2_iq >> 8);
    tx_message.Data[3] = (uint8_t)cm2_iq;
    tx_message.Data[4] = (uint8_t)(cm3_iq >> 8);
    tx_message.Data[5] = (uint8_t)cm3_iq;
    tx_message.Data[6] = (uint8_t)(cm4_iq >> 8);
    tx_message.Data[7] = (uint8_t)cm4_iq;
	
	
//	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, DISABLE);
    CAN_Transmit(CANx,&tx_message);
	//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);
}


void ReceiveCAN2Data()
{
	CanRxMsg rx_message; 
	CAN_Receive(CAN2, CAN_FIFO0, &rx_message);     
	switch(rx_message.StdId)
	{	
		case CAN_BUS2_MOTOR1:
		{             
	//		global_cout[LOST_COUNTER_INDEX_MOTOR8] = Online;
			CAN2_Motor_PID[0].get = (rx_message.Data[2]<<8) + rx_message.Data[3];
      CAN2_Moter_Position_PID[0].speed_get = (rx_message.Data[2]<<8) + rx_message.Data[3];
			CAN2_Moter_Position_PID[0].ange_get  = encoder_value_for_Moter(&CAN2_Moter_Position_PID[0],(rx_message.Data[0] << 8) | rx_message.Data[1])/19;//码盘值连续化处理
			if(init_flag == 0)
			{
			   init_flag = 1;
				 CAN2_Moter_Position_PID[0].init_angle =  CAN2_Moter_Position_PID[0].ange_get;
			}
		}
		break;
		
		case CAN_BUS2_MOTOR2:
		{ 
		//	global_cout[LOST_COUNTER_INDEX_MOTOR9] = Online;
			CAN2_Motor_PID[1].get = (rx_message.Data[2]<<8) + rx_message.Data[3];
			CAN2_Moter_Position_PID[1].speed_get = (rx_message.Data[2]<<8) + rx_message.Data[3];
			CAN2_Moter_Position_PID[1].ange_get  = encoder_value_for_Moter(&CAN2_Moter_Position_PID[1],(rx_message.Data[0] << 8) | rx_message.Data[1])/19;//码盘值连续化处理
			if(init_flag == 4)
			{
			   init_flag = 5;
				 CAN2_Moter_Position_PID[1].init_angle =  CAN2_Moter_Position_PID[1].ange_get;
			}
			
		}	
		break;
				case CAN_BUS2_MOTOR3:
		{ 
		//	global_cout[LOST_COUNTER_INDEX_MOTOR9] = Online;
			CAN2_Motor_PID[2].get = (rx_message.Data[2]<<8) + rx_message.Data[3];
		  CAN2_Moter_Position_PID[2].speed_get = (rx_message.Data[2]<<8) + rx_message.Data[3];
			CAN2_Moter_Position_PID[2].ange_get  = encoder_value_for_Moter(&CAN2_Moter_Position_PID[2],(rx_message.Data[0] << 8) | rx_message.Data[1])/19;//码盘值连续化处理
			if(init_flag == 5)
			{
			   init_flag = 6;
				 CAN2_Moter_Position_PID[2].init_angle =  CAN2_Moter_Position_PID[2].ange_get;
			}
			
		}	
		break;
	case CAN_BUS2_MOTOR4:
		{ 
			//global_cout[LOST_COUNTER_INDEX_MOTOR9] = Online;
			CAN2_Motor_PID[3].get = (rx_message.Data[2]<<8) + rx_message.Data[3];
			CAN2_Moter_Position_PID[3].speed_get = (rx_message.Data[2]<<8) + rx_message.Data[3];
			CAN2_Moter_Position_PID[3].ange_get  = encoder_value_for_Moter(&CAN2_Moter_Position_PID[3],(rx_message.Data[0] << 8) | rx_message.Data[1])/19;//码盘值连续化处理
			if(init_flag == 6)
			{
			   init_flag = 7;
				 CAN2_Moter_Position_PID[3].init_angle =  CAN2_Moter_Position_PID[3].ange_get;
			}			
		}	
		break;
		
	case CAN_BUS2_MOTOR5:
		{ 
			CAN2_Moter_Position_PID[4].speed_get = (rx_message.Data[2]<<8) + rx_message.Data[3];
			CAN2_Moter_Position_PID[4].ange_get  = encoder_value_for_Moter(&CAN2_Moter_Position_PID[4],(rx_message.Data[0] << 8) | rx_message.Data[1])/96;//码盘值连续化处理
			if(init_flag == 7)
			{
			   init_flag = 8;
				 CAN2_Moter_Position_PID[4].init_angle =  CAN2_Moter_Position_PID[4].ange_get;
			}			
		}	
		break;
		
	case CAN_BUS2_MOTOR6:
		{ 
			CAN2_Moter_Position_PID[5].speed_get = (rx_message.Data[2]<<8) + rx_message.Data[3];
			CAN2_Moter_Position_PID[5].ange_get  = encoder_value_for_Moter(&CAN2_Moter_Position_PID[5],(rx_message.Data[0] << 8) | rx_message.Data[1])/96;//码盘值连续化处理
			if(init_flag == 8)
			{
			   init_flag = 9;
				 CAN2_Moter_Position_PID[5].init_angle =  CAN2_Moter_Position_PID[5].ange_get;
			}			
		}	
		break;

	case CAN_BUS2_MOTOR7:
		{ 
			CAN2_Moter_Position_PID[6].speed_get = (rx_message.Data[2]<<8) + rx_message.Data[3];
			CAN2_Moter_Position_PID[6].ange_get  = encoder_value_for_Moter(&CAN2_Moter_Position_PID[6],(rx_message.Data[0] << 8) | rx_message.Data[1])/19;//码盘值连续化处理
			if(init_flag == 9)
			{
			   init_flag = 10;
				 CAN2_Moter_Position_PID[6].init_angle =  CAN2_Moter_Position_PID[6].ange_get;
			}			
		}	
		break;
		
		default:break;
	}
}


void CAN2_RX0_IRQHandler(void)
{   
  if (CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET) 
	{
		CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
		CAN_ClearFlag(CAN2, CAN_FLAG_FF0); 
    ReceiveCAN2Data();
	}
	
}


void CAN2_TX_IRQHandler()
{
		if(CAN_GetITStatus(CAN2,CAN_IT_TME) !=RESET)
		{
			 CAN_ClearITPendingBit(CAN2,CAN_IT_TME);
		 }  
}


void CAN2_Moter_1_4_Send( int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq)
{
	  int i=0;
	  u8 mbox;
    CanTxMsg tx_message;
    tx_message.StdId = 0x200;//标准ID
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;//发送数据帧
    tx_message.DLC = 0x08;//发送八个字节的数据
    
    tx_message.Data[0] = (uint8_t)(cm1_iq >> 8);//数据高八位
    tx_message.Data[1] = (uint8_t)cm1_iq;         //数据低八位
    tx_message.Data[2] = (uint8_t)(cm2_iq >> 8);
    tx_message.Data[3] = (uint8_t)cm2_iq;
    tx_message.Data[4] = (uint8_t)(cm3_iq >> 8);
    tx_message.Data[5] = (uint8_t)cm3_iq;
    tx_message.Data[6] = (uint8_t)(cm4_iq >> 8);
    tx_message.Data[7] = (uint8_t)cm4_iq;
    mbox = CAN_Transmit(CAN2,&tx_message);
	
	 while(CAN_TransmitStatus(CAN2,mbox) ==CAN_TxStatus_Failed && i++<0xfff);
}

void CAN2_Moter_5_7_send( int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq)
{
	  int i=0;
	  u8 mbox;
    CanTxMsg tx_message;
    tx_message.StdId = 0x1FF;//标准ID
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;//发送数据帧
    tx_message.DLC = 0x08;//发送八个字节的数据
    
    tx_message.Data[0] = (uint8_t)(cm1_iq >> 8);//数据高八位
    tx_message.Data[1] = (uint8_t)cm1_iq;         //数据低八位
    tx_message.Data[2] = (uint8_t)(cm2_iq >> 8);
    tx_message.Data[3] = (uint8_t)cm2_iq;
    tx_message.Data[4] = (uint8_t)(cm3_iq >> 8);
    tx_message.Data[5] = (uint8_t)cm3_iq;
    tx_message.Data[6] = (uint8_t)(cm4_iq >> 8);
    tx_message.Data[7] = (uint8_t)cm4_iq;
    mbox = CAN_Transmit(CAN2,&tx_message);
	
	 while(CAN_TransmitStatus(CAN2,mbox) ==CAN_TxStatus_Failed && i++<0xfff);
}


