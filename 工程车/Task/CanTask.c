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
��������encoder_value_for_pitch
��ڲ�����
         input_data��P����յ��Ļ�е��
���ڲ�������
���ܣ���P��Ļ�е�ǽ�������������
˵����������ֵ���ٽ�ʱ�������ϸ�������10,�¸�������-8000��˵���Ѿ������ٽ�ǣ�
�������������������õ�������ֵ�Ǵ�ģ�����������ķ����ǵ������ٽ��ʱ��˳
ʱ�뾭���ٽ��ʱ��Ȧ����1����ʱ��ת��ʱ��1
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
��������encoder_value_for_yaw
��ڲ�����
         input_data��Y����յ��Ļ�е��
���ڲ�������
���ܣ���Y��Ļ�е�ǽ�������������
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
��������ReceiveCAN1Data
��ڲ�������
���ڲ�������
���ܣ����ݵ��ID���յ����ת�ټ���е������
ע�⣺
�������Ƶ��1kHz��
��е�Ƕ�ֵ�ķ�Χ��0~8191(0x1FFF)
ʵ�ʵ�������ֵ��Χ��-13000 ~ 13000
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
			CAN1_Moter_Position_PID[5].ange_get  = encoder_value_for_Moter(&CAN1_Moter_Position_PID[5],(rx_message.Data[0] << 8) | rx_message.Data[1])/19;//����ֵ����������
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
			CAN1_Moter_Position_PID[6].ange_get  = encoder_value_for_Moter(&CAN1_Moter_Position_PID[6],(rx_message.Data[0] << 8) | rx_message.Data[1])/19;//����ֵ����������
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
��������CAN1_RX0_IRQHandler
��ڲ�������
���ڲ�������
���ܣ�CAN�����жϽ��յ�����͵Ļ�е�Ǽ�ת������
ע�⣺
�������Ƶ��1kHz��
��е�Ƕ�ֵ�ķ�Χ��0~8191(0x1FFF)
ʵ�ʵ�������ֵ��Χ��-13000 ~ 13000
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
����ԭ�ͣ�void Set_Gimbal_Current(CAN_TypeDef *CANx, int16_t gimbal_yaw_iq, int16_t gimbal_pitch_iq, int16_t gimbal_fire_iq)
��ڲ�����
1.CAN_TypeDef *CANx ָ������CAN1���Ǵ�CAN2
2. int16_t gimbal_yaw_iq  ��̨IDΪ205��Y�������͵�ת�ص�������
3. int16_t gimbal_pitch_iq��̨IDΪ206��Y�������͵�ת�ص�������
4. int16_t gimbal_fire_iq ��̨IDΪ207�Ĳ���������͵�ת�ص�������
5.int16_t cm4_iq ����IDΪ204�ĵ�����͵�ת�ص�������
���ڲ�������
���ܣ�������̨�����ת�ص�������
*****************************************************************************************************************************/
void CAN1_Moter_5_7_Send(CAN_TypeDef *CANx, int16_t gimbal_yaw_iq, int16_t gimbal_pitch_iq, int16_t gimbal_fire_iq)
{
    CanTxMsg tx_message;  

      
    tx_message.StdId = 0x1FF;//���ͱ�׼ID
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;//��������֡
    tx_message.DLC = 0x08;//����֡����8���ֽڵ�����
    
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
����ԭ�ͣ�void Set_CM_Speed(CAN_TypeDef *CANx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq)
��ڲ�����
1.CAN_TypeDef *CANx ָ������CAN1���Ǵ�CAN2
2.int16_t cm1_iq ����IDΪ201�ĵ�����͵�ת�ص�������
3.int16_t cm2_iq ����IDΪ202�ĵ�����͵�ת�ص�������
4.int16_t cm3_iq ����IDΪ203�ĵ�����͵�ת�ص�������
5.int16_t cm4_iq ����IDΪ204�ĵ�����͵�ת�ص�������
���ڲ�������
���ܣ����͵��̵����ת�ص�������
**************************************************************************************************************/

void CAN1_Moter_1_4_Send(CAN_TypeDef *CANx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq)
{
    CanTxMsg tx_message;
    tx_message.StdId = 0x200;//��׼ID
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;//��������֡
    tx_message.DLC = 0x08;//���Ͱ˸��ֽڵ�����
    
    tx_message.Data[0] = (uint8_t)(cm1_iq >> 8);//���ݸ߰�λ
    tx_message.Data[1] = (uint8_t)cm1_iq;         //���ݵͰ�λ
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
			CAN2_Moter_Position_PID[0].ange_get  = encoder_value_for_Moter(&CAN2_Moter_Position_PID[0],(rx_message.Data[0] << 8) | rx_message.Data[1])/19;//����ֵ����������
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
			CAN2_Moter_Position_PID[1].ange_get  = encoder_value_for_Moter(&CAN2_Moter_Position_PID[1],(rx_message.Data[0] << 8) | rx_message.Data[1])/19;//����ֵ����������
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
			CAN2_Moter_Position_PID[2].ange_get  = encoder_value_for_Moter(&CAN2_Moter_Position_PID[2],(rx_message.Data[0] << 8) | rx_message.Data[1])/19;//����ֵ����������
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
			CAN2_Moter_Position_PID[3].ange_get  = encoder_value_for_Moter(&CAN2_Moter_Position_PID[3],(rx_message.Data[0] << 8) | rx_message.Data[1])/19;//����ֵ����������
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
			CAN2_Moter_Position_PID[4].ange_get  = encoder_value_for_Moter(&CAN2_Moter_Position_PID[4],(rx_message.Data[0] << 8) | rx_message.Data[1])/96;//����ֵ����������
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
			CAN2_Moter_Position_PID[5].ange_get  = encoder_value_for_Moter(&CAN2_Moter_Position_PID[5],(rx_message.Data[0] << 8) | rx_message.Data[1])/96;//����ֵ����������
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
			CAN2_Moter_Position_PID[6].ange_get  = encoder_value_for_Moter(&CAN2_Moter_Position_PID[6],(rx_message.Data[0] << 8) | rx_message.Data[1])/19;//����ֵ����������
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
    tx_message.StdId = 0x200;//��׼ID
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;//��������֡
    tx_message.DLC = 0x08;//���Ͱ˸��ֽڵ�����
    
    tx_message.Data[0] = (uint8_t)(cm1_iq >> 8);//���ݸ߰�λ
    tx_message.Data[1] = (uint8_t)cm1_iq;         //���ݵͰ�λ
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
    tx_message.StdId = 0x1FF;//��׼ID
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;//��������֡
    tx_message.DLC = 0x08;//���Ͱ˸��ֽڵ�����
    
    tx_message.Data[0] = (uint8_t)(cm1_iq >> 8);//���ݸ߰�λ
    tx_message.Data[1] = (uint8_t)cm1_iq;         //���ݵͰ�λ
    tx_message.Data[2] = (uint8_t)(cm2_iq >> 8);
    tx_message.Data[3] = (uint8_t)cm2_iq;
    tx_message.Data[4] = (uint8_t)(cm3_iq >> 8);
    tx_message.Data[5] = (uint8_t)cm3_iq;
    tx_message.Data[6] = (uint8_t)(cm4_iq >> 8);
    tx_message.Data[7] = (uint8_t)cm4_iq;
    mbox = CAN_Transmit(CAN2,&tx_message);
	
	 while(CAN_TransmitStatus(CAN2,mbox) ==CAN_TxStatus_Failed && i++<0xfff);
}


