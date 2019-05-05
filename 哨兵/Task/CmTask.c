#include "headfile.h"
#include "pid_modify.h"

#define TRACK_SPEED 4000



void Track_Motor_Pidparams_init()
{
  int i ;
	for(i=2;i<4;i++)
	{
	  PID_struct_init(&CAN1_Motor_PID[i],DELTA_PID,7000,500,5,0.3,0.1);
	}
}

 long int encoder_position = 0;
 long int position = 0;

static void bubble_sort(int16_t *p,char num)
{
  char i,j;
	float temp;
	for(i=0;i< num-1;i++)
	  for(j=0;j<num-1-i;j++)
	  { 
	     if(p[j]>p[j+1])
			 {
			      temp = p[j];
						p[j] = p[j+1];
				    p[j+1] = temp;
			 }
	  }
}


char Encoder_Coli_flag = 0;

static  void Track_Move(long int left_limit,long int right_limit,int16_t init_speed)
{
  static  char flag = 0,speed_positive2left = 0;
	static char init_flag = 0;
	static int cnt= 0;
	static int16_t set_speed  = TRACK_SPEED;
	int16_t curr_p_sort[50];
	int i=0;
	long int sum;

	
	encoder_position  = TIM2->CNT  / 2;
	
	
	if(Encoder_Coli_flag == 0)
	{
	  if(ENCODER_CALI == 0)
	  {
		  Encoder_Coli_flag = 1;
			Bsp.OpenTheGreenLed(1);
	  }	
	}
	else if(Encoder_Coli_flag == 1)
	{
	     TIM2->CNT = 2000000000;
		   Encoder_Coli_flag = 2;
	}
	
	for(i=0;i<50;i++)
		   {
		      curr_p_sort[i] = TIM2->CNT  / 2  - 2000000000 / 2; 			 
		   }
			 
			 bubble_sort(curr_p_sort,50);  
			 
  for(i=5;i<45;i++)
			 {
			     sum += curr_p_sort[i];
			 }
			 		 
			  position = (long int)sum / 40;  
			 
	if(init_flag == 0){         //先让电机运动一下好判断电机的方向
			cnt ++;
			CAN1_Motor_PID[2].set =  set_speed; 
			CAN1_Motor_PID[3].set =  -set_speed;
				if(cnt==5200)  //电机启动需要时间
				{
					cnt = 0;
					init_flag =1;
					flag = 1;
					if(position < 0 )speed_positive2left = 1;
					else speed_positive2left = 0;
					Bsp.OpenTheGreenLed(1);
				}		
		}
  else
	{ 	
		Bsp.OpenTheGreenLed(0); 
		 	
      if(flag == 1)
			{
				 
				if( position  <= left_limit)
					 {
						 flag = 2;
						 Encoder_Coli_flag = 0;
					 }	
				
				if( position  < left_limit + 1000)
				  {
						    set_speed -=3.5;
                if(set_speed <= 2000) set_speed = 2000;							
				  }
				else if( position  > right_limit - 1000)
				   {
						    set_speed +=3.5;
                if(set_speed >= init_speed) set_speed = init_speed;					
				   }						
			}				
		else
		{
				 if( position  >=right_limit)
					{
							 flag =1;
						   Encoder_Coli_flag = 0;
					}	
						
				if( position  < left_limit + 1000)
					{
									 set_speed +=3.5;
									if(set_speed >= init_speed) set_speed = init_speed;							
					}
					else if( position  > right_limit - 1000)
					{
									set_speed -=3.5;
									if(set_speed <= 2000)  set_speed = 2000;					
					}
		  }	

    			
			
			
		 if(speed_positive2left ==1)
		 {
				if(flag == 1)
				 {		 
					 CAN1_Motor_PID[2].set =  set_speed; 
					 CAN1_Motor_PID[3].set =  -set_speed; 	     
				 }
				 else 
				 {
					 CAN1_Motor_PID[2].set =  -set_speed; 	
					 CAN1_Motor_PID[3].set =  set_speed; 	 
				 }	 
		 }
		 else
		 {
				if(flag == 1)
					 {		 
						 CAN1_Motor_PID[2].set =  -set_speed; 
						 CAN1_Motor_PID[3].set =  set_speed; 	     
					 }
				else 
					 {
						 CAN1_Motor_PID[2].set =  set_speed; 	
						 CAN1_Motor_PID[3].set =  -set_speed; 	 
					 }	 		 
		 }
	}
	    	
	 pid_calc(&CAN1_Motor_PID[2] );
	 pid_calc(&CAN1_Motor_PID[3] );
}


volatile int16_t g_fire_speed = 0; 
static volatile int16_t move_speed = 1000;


/*
static void Fire_speed_Control(int16_t speed)
{
    CAN1_Motor_PID[0].set =  speed;
	  CAN1_Motor_PID[1].set = -speed;
	 
	  pid_calc(&CAN1_Motor_PID[0] );
	  pid_calc(&CAN1_Motor_PID[1] );
	  
	 
}
*/

void CAN1_ID_1_4_Send(bool status,long int left_limit,long int right_limit,int16_t  track_speed,int16_t fire_speed)
{

		
//	Fire_speed_Control(fire_speed);
	Track_Move(left_limit,right_limit,track_speed);
	
	if(!status)
	{
	     CAN1_Send_Motor_data(CAN1,CAN1_Motor_PID[0].out,CAN1_Motor_PID[1].out,0,0);//在轨道上不运动
	}
  else
	{
	   CAN1_Send_Motor_data(CAN1,CAN1_Motor_PID[0].out,CAN1_Motor_PID[1].out,CAN1_Motor_PID[2].out,CAN1_Motor_PID[3].out); 
	} 
}



/*
float Fire_fre_control(int16_t speed)
{
    CAN1_Motor_PID[6].set =  speed;
	  pid_calc(&CAN1_Motor_PID[6] );
	
	  return CAN1_Motor_PID[6].out;
	
}

*/
