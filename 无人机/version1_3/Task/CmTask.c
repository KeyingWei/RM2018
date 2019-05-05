#include "headfile.h"
#include "pid_modify.h"


 /*
void CMControlCalc(CMCONTORL *CmCotrol)
{
	if(GetWorkState() == PREPARE_STATE) //上电初始化时，设定底盘初始角度为0
	{
		CmCotrol->Angle = 0;
	}
	else //初始化完成后根据Y轴电机的机械角得到底盘相对云台的角度值
	{
		CmCotrol->Angle = (CmCotrol->Encoder_Init - CmCotrol->Encoder) * 360 / 8192;
	}
	
	CmCotrol->Control_CH2 = CmCotrol->Angle * 220;//角度值*200后得到底盘电机的控制值
	
	Power_Limit(CmCotrol);
	
	ID201_PID.SetSpeed = CmCotrol->Control_CH1 + CmCotrol->Control_CH2 + CmCotrol->Control_CH0; //麦克纳姆轮运动公式
	ID202_PID.SetSpeed = -(CmCotrol->Control_CH1 - CmCotrol->Control_CH2 - CmCotrol->Control_CH0);
	ID203_PID.SetSpeed = CmCotrol->Control_CH1 + CmCotrol->Control_CH2 - CmCotrol->Control_CH0;
	ID204_PID.SetSpeed = -(CmCotrol->Control_CH1 - CmCotrol->Control_CH2 + CmCotrol->Control_CH0);
	
	ID201_PID.Calc(&ID201_PID);//进行增量式PID计算
	ID202_PID.Calc(&ID202_PID);
	ID203_PID.Calc(&ID203_PID);
	ID204_PID.Calc(&ID204_PID);
	
	
	if((Dog.rc == Lose) || (GetWorkState() == PREPARE_STATE) || (GetWorkState() == STOP_STATE))//有模块离线时，底盘速度清零
	{
		Set_CM_Speed(CAN1, 0, 0, 0, 0);
	}
	else 
	{
		Set_CM_Speed(CAN1, ID201_PID.output, ID202_PID.output, ID203_PID.output, ID204_PID.output);//发送电机转矩电流控制值
	}
}
*/




void Power_Limit(CMCONTORL *CmCotrol);



void Fire_Motor_Pidparams_init()
{
   int i =0;	
 for(i=0;i<4;i++)
	{
	  PID_struct_init(&CAN2_Motor_PID[i],DELTA_PID,7000,500,9,0.1,0); 
	}
	
}

void CM_Motor_Pidparams_init()
{
  int i =0;
	for(i=0;i<4;i++)
	{
	  PID_struct_init(&CAN1_Motor_PID[i],DELTA_PID,7000,500,5,0.3,0.1);
	}
}

void CMControlCalc()
{

	int i=0;
	
		if(GetWorkState() == PREPARE_STATE) //上电初始化时，设定底盘初始角度为0
	{
		CmCotrol.Angle = 0;
	}
	else //初始化完成后根据Y轴电机的机械角得到底盘相对云台的角度值
	{
		CmCotrol.Angle = (CmCotrol.Encoder_Init - CmCotrol.Encoder) * 360 / 8192;
	}
	
	CmCotrol.Control_CH2 = CmCotrol.Angle * 380;
//	Power_Limit(&CmCotrol);
	
	
	CAN1_Motor_PID[0].set = CmCotrol.Control_CH1 + CmCotrol.Control_CH2 + CmCotrol.Control_CH0; //麦克纳姆轮运动公式
	CAN1_Motor_PID[1].set = -(CmCotrol.Control_CH1 - CmCotrol.Control_CH2 - CmCotrol.Control_CH0);
	CAN1_Motor_PID[2].set = CmCotrol.Control_CH1 + CmCotrol.Control_CH2 - CmCotrol.Control_CH0;
	CAN1_Motor_PID[3].set = -(CmCotrol.Control_CH1 - CmCotrol.Control_CH2 + CmCotrol.Control_CH0);
	
	
	for(i=0;i<4;i++)
	{
    pid_calc(&CAN1_Motor_PID[i]);	
	
	}
	
	 Set_CM_Speed(CAN1,CAN1_Motor_PID[0].out, CAN1_Motor_PID[1].out, CAN1_Motor_PID[2].out, CAN1_Motor_PID[3].out);
	

}

int   limit_ch1 =0;
int   limit_ch0 =0;
int   limit_ch2 =0;


void Power_Limit(CMCONTORL *CmCotrol)
{
		 g_Referee_flag = 0 ; 
       
		 
	   power.SetPower   = 40;	
	
	
			  power.Kp   = 0.05f;
			  power.Ki   = 0;
			  power.Kd   = 0;
			  power.error[0] = (power.SetPower   - power.average);
			
			  power.pout =  power.Kp *(power.error[0]-power.error[1]);
			  power.iout =  power.Ki *(power.error[0]);
			  power.dout  = power.Kd *(power.error[0]-2*power.error[1]+power.error[2]);
			
			 
	
	    // CmCotrol->Control_CH0 = CmCotrol->Control_CH0*power.output;
      
	   //  CmCotrol->Control_CH1 = CmCotrol->Control_CH2*power.output;
	
//		 
	  if(CmCotrol->Control_CH1 > 0  )
		{
			 power.output +=power.pout + power.iout + power.dout ;
			 if(power.output< 0)power.output=0;
			 CmCotrol->Control_CH1 = CmCotrol->Control_CH1*power.output;
//			  power.Kp   = 0.01f;
//			  power.Ki   = 0;
//			  power.Kd   = 0;
//			  power.error[0] = (power.SetPower   - power.average);
//			
//			  power.pout =  power.Kp *(power.error[0]-power.error[1]);
//			  power.iout =  power.Ki *(power.error[0]);
//			  power.dout  = power.Kd *(power.error[0]-2*power.error[1]+power.error[2]);
//			
//			  power.output +=power.pout + power.iout + power.dout ;
//			  
//			  if(power.output>0)limit_ch1=5000;
//			  else
//				{
//					limit_ch1 =  limit_ch1 +  power.output;
//					limit_ch1 = constrain(limit_ch1,1500,5000);
//					if(CmCotrol->Control_CH1>limit_ch1 ) CmCotrol->Control_CH1  = limit_ch1;				
//				}
//				
				power.error[1] = power.error[0];
				power.error[2] = power.error[1];
//		 		 
		}
//		
//		else if(CmCotrol->Control_CH1 < 0)
//		{	
//	
//		  power.Kp   = 0.78;
//			power.output =(int16_t) (power.SetPower   - power.average) * power.Kp ;
//			
//      if(power.output>0)limit_ch1=-10000;
//			  else
//				{
//					limit_ch1 = limit_ch1 -power.output;	
//					limit_ch1 = constrain(limit_ch1,-10000,-1500);		
//					if(CmCotrol->Control_CH1< limit_ch1 ) CmCotrol->Control_CH1  = limit_ch1;				
//				}
//		

//		}
//		
//		
//		
//	  if(CmCotrol->Control_CH0 > 0  )
//		{
//			  power.Kp   = 0.1;
//			  power.output =(int16_t) (power.SetPower   - power.average) * power.Kp ;
//			
//			 if(power.output>0)limit_ch0=10000;
//			  else
//				{
//					limit_ch0 =  limit_ch0 +  power.output;
//					limit_ch0 = constrain(limit_ch0,1500,10000);
//					if(CmCotrol->Control_CH0>limit_ch0 ) CmCotrol->Control_CH0  = limit_ch0;			
//				}

//			 
//		}
//		
//		else if(CmCotrol->Control_CH0 < 0)
//		{	
//			  power.Kp   = 0.1;
//			  power.output =(int16_t) (power.SetPower   - power.average) * power.Kp ;
//			
//			 if(power.output>0)limit_ch0=-10000;
//			  else
//				{
//						limit_ch0 = limit_ch0 -power.output;	
//						limit_ch0 = constrain(limit_ch0,-10000,-1500);		
//					 if(CmCotrol->Control_CH0< limit_ch0 ) CmCotrol->Control_CH0  = limit_ch0;		
//				}
//		  
//		}
//		
//		
//	if(CmCotrol->Control_CH2 > 0  )
//		{
//			  power.Kp   = 0.4;
//			  power.output =(int16_t) (power.SetPower   - power.average) * power.Kp ;
//			
//				if(power.output>0)limit_ch2=10000;
//			  else
//				{
//					limit_ch2 =  limit_ch2 +  power.output;
//					limit_ch2 = constrain(limit_ch2,9500,10000);
//					if(CmCotrol->Control_CH2>limit_ch2 ) CmCotrol->Control_CH2  = limit_ch2;
//				}

//			 
//		}
//		
//		else if(CmCotrol->Control_CH2 < 0)
//		{	
//			  power.Kp   = 0.4f;
//			  power.output =(int16_t) (power.SetPower   - power.average) * power.Kp ;
//			
//			if(power.output>0)limit_ch2=-10000;
//			  else
//				{
//						limit_ch2 = limit_ch2 -power.output;	
//						limit_ch2 = constrain(limit_ch2,-10000,-9500);		
//					 if(CmCotrol->Control_CH2< limit_ch2 ) CmCotrol->Control_CH2  = limit_ch2;
//				}
//		 }
		

			
}

void Fire_speed_Control(int16_t speed)
{
    CAN2_Motor_PID[0].set =  speed;
	  CAN2_Motor_PID[1].set = -speed;
	  pid_calc(&CAN2_Motor_PID[0] );
	  pid_calc(&CAN2_Motor_PID[1] );

	  CAN2_Set_FireSpeed(CAN2_Motor_PID[0].out,CAN2_Motor_PID[1].out,0,0);
	 
}

