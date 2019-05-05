#include "headfile.h"
#include "pid_modify.h"



position_pid CAN1_Moter_Position_PID[7];
position_pid CAN2_Moter_Position_PID[7];


 char g_CM_Up_Down_staus = 5;

void CM_Up_Down_Motor_Pidparams_init() //底盘移动电机
{
  int i =0;
	for(i=0;i<4;i++)
	{
	  PID_struct_init(&CAN1_Motor_PID[i],DELTA_PID,8000,500,5,0.4f,0);
	}


	for(i=0;i<4;i++)
	{
	  PID_struct_init(&CAN2_Motor_PID[i],DELTA_PID,8000,500,1,0.02f,0);
	}
	
	PID_struct_init(&CAN1_Motor_PID[5],DELTA_PID,8000,500,6,1.5f,0); 
		
}

void Motor_Position_Params_init()
{
   int i = 0;
	  for(i = 0; i < 4 ;i ++)  //底盘升降电机
			{
			    CAN2_Moter_Position_PID[i].position_kp = 2.0f;
				  CAN2_Moter_Position_PID[i].velocity_kp = 3.5f;
				  CAN2_Moter_Position_PID[i].encoder_count = 0;
				  CAN2_Moter_Position_PID[i].encoder_val_new_data = 4096;
			}
			
		 for(i = 4; i < 6 ;i ++)  //底盘升降电机
			{
			    CAN2_Moter_Position_PID[i].position_kp = 4.0f;
				  CAN2_Moter_Position_PID[i].velocity_kp = 10.5f;
				  CAN2_Moter_Position_PID[i].encoder_count = 0;
				  CAN2_Moter_Position_PID[i].encoder_val_new_data = 4096;
			}
			
			    CAN2_Moter_Position_PID[6].position_kp = 4.0f;
				  CAN2_Moter_Position_PID[6].velocity_kp = 8.5f;
				  CAN2_Moter_Position_PID[6].encoder_count = 0;
				  CAN2_Moter_Position_PID[6].encoder_val_new_data = 4096;

			
				  CAN1_Moter_Position_PID[5].position_kp = 6.0f;  //抓手移动电机
				  CAN1_Moter_Position_PID[5].velocity_kp = 10.5f;
				  CAN1_Moter_Position_PID[5].encoder_count = 0;
				  CAN1_Moter_Position_PID[5].encoder_val_new_data = 4096;	

				  CAN1_Moter_Position_PID[6].position_kp = 10.0f;  //弹仓升降电机
				  CAN1_Moter_Position_PID[6].velocity_kp = 25.5f;
				  CAN1_Moter_Position_PID[6].encoder_count = 0;
				  CAN1_Moter_Position_PID[6].encoder_val_new_data = 4096;				
			
}


void CMControlCalc()
{

	int i=0;
	
	CAN1_Motor_PID[0].set = CmCotrol.Control_CH1 + CmCotrol.Control_CH2 + CmCotrol.Control_CH0; //麦克纳姆轮运动公式
	CAN1_Motor_PID[1].set = -(CmCotrol.Control_CH1 - CmCotrol.Control_CH2 - CmCotrol.Control_CH0);
	CAN1_Motor_PID[2].set = CmCotrol.Control_CH1 + CmCotrol.Control_CH2 - CmCotrol.Control_CH0;
	CAN1_Motor_PID[3].set = -(CmCotrol.Control_CH1 - CmCotrol.Control_CH2 + CmCotrol.Control_CH0);
	
	
	for(i=0;i<4;i++)
	{
    pid_calc(&CAN1_Motor_PID[i]);	
	
	}
	
	for(i=0;i<4;i++)
	{
    CAN1_Motor_PID[i].out = constrain(CAN1_Motor_PID[i].out,-9000,9000 );	
	}
	
	
}



int watch_val1=0;

float catch_angle=0;

float catch_position_pid()
{
    float position_kp = 6.0f;
	  float velocity_kp = 10.0f;
	  float position_out = 0;
	  float velocity_out =0;
	
	  int contorol_val = catch_angle;
	
	  int16_t curr_speed = CAN1_Motor_PID[4].get;
	  // watch_val1 = catch_encoder_value - encoder_init_val;
	   position_out = position_kp * (contorol_val - (catch_encoder_value - encoder_init_val)); 
	   velocity_out = velocity_kp * (position_out - curr_speed);
	
	   velocity_out= constrain(velocity_out,-7000,7000);
	
	  return velocity_out;
}



float Moter_position_pid(p_position_pid position)
{
	
	 
	  float position_out = 0;
	  float velocity_out = 0;
	
	  position_out =  position->position_kp * (position->ange_set - (position->ange_get - position->init_angle)); 
	  velocity_out =  position->velocity_kp * (position_out - position->speed_get);
	
	  velocity_out= constrain(velocity_out,-9000,9000);
	
	   position->out =  velocity_out ;
	  return velocity_out;
}



char catch_flag =  0;

enum 
{
  INIT_SATA,
	WAIT_SATA,
	CATCH_SATA,
	THROW_BOLLET_SATA,
	THROW_SATA,
	CATCH_TWO,
};

char cather_status  = INIT_SATA;
void catcher_control(float set_angle,long int secont_p,long int limit_h)
{
	//static char status = INIT_SATA;
	static char flag = 0 ,flag1 =0,flag2 =0,flag3 =0;
	static int  count=0,count1=0,count2=0;

	
	watch_val1 = cather_status;
	
	if( mydata.dbus.key.v  == KEY_PRESSED_OFFSET_Q)
	{
	       flag  = 1;
	}

	if( mydata.dbus.key.v  == KEY_PRESSED_OFFSET_E)
	{
				 flag1 = 1;
	}
		 switch(cather_status)  //状态机切换状态
				{
					case  INIT_SATA:

                    if(flag2 == 0)
						{
							catch_angle -=(float)100.0f;		          
							if(catch_angle <=0)
								{ 
									 catch_flag = 0;				
									 catch_angle = 0;
									 Bsp.OpenTheCatcher_(1);
																								 
									if(flag == 1)	 
											cather_status = WAIT_SATA; 																															 																																																											
								}												   
						}
                    else
						{
							catch_angle += 100.0f;
							  if(catch_angle >= 2 * set_angle)
								{
									 catch_angle =  2 * set_angle;				
									 catch_flag = 0;
									 Bsp.OpenTheCatcher_(1);
																								 
									if(flag == 1)	 
											cather_status = WAIT_SATA; 													    
								}
						}											
					 break;
					case  WAIT_SATA:	
                     if(flag2 == 0)
					 {
							 catch_angle +=(float)100.0f;
							 if(catch_angle >= set_angle)
									 {
										 catch_angle= set_angle;
										 Bsp.OpenTheCatcher_(1);														
											if(flag1==1)	
												{
													cather_status = CATCH_SATA;//按下E键进入自动夹取状态
												}
									}											  
					 }
                     else
					 {
						catch_angle -=(float)100.0f; 
						  if(catch_angle <= set_angle)
							{
										 catch_angle= set_angle;
										 Bsp.OpenTheCatcher_(1);														
											if(flag1==1)	
												{
													cather_status = CATCH_SATA;//按下E键进入自动夹取状态
												}												  
							}
					 }											 
					 break;
					case 	CATCH_SATA:
									Bsp.OpenTheCatcher_(0);
									flag=0;
									flag1 =0;
									count2++;
					       				        
									if(count2>=95)
									{
										 count2 = 95;
										 catch_flag = 2;
										 
									  if(CAN2_Moter_Position_PID[0].ange_set >= limit_h  -100  &&  CAN2_Moter_Position_PID[1].ange_set >= limit_h  -100  &&  CAN2_Moter_Position_PID[2].ange_set  >= limit_h  -100&&  CAN2_Moter_Position_PID[3].ange_set >= limit_h  -100)
											{												
												   cather_status =   THROW_BOLLET_SATA; 
                           	count2 = 0;											
											}																			 
									}
									break;							
										
					case 	 THROW_BOLLET_SATA:
							 catch_angle +=(float)100.0f;
								if(catch_angle >= 2 * set_angle)
								{												
										catch_angle =   2 * set_angle;
										Bsp.OpenTheCatcher_(0);
										count++;
										if(count == 130)
										{
											count =0 ;
											cather_status = THROW_SATA;
										}														
								}
								break;
					case   	THROW_SATA:
							 catch_angle -=(float)100.0f;
							 if(catch_angle <= set_angle)
							 {
									
									 catch_angle = set_angle;
									 Bsp.OpenTheCatcher_(1);
									 count1++;
									 if(count1== 100)
									 {
											 count1=0;
											 cather_status = INIT_SATA;	
											 flag2++;													 
									 }					  
							 }				 
									 break;				
					default:break;					 									              								           					
				}
		
}

void Fornt_Wheel_Down(long int pisiton,int16_t speed)
{
     if(CAN2_Moter_Position_PID[0].ange_set <=  pisiton )
		{
		      CAN2_Moter_Position_PID[0].ange_set += speed;
		}

    if(CAN2_Moter_Position_PID[1].ange_set <=  pisiton)
		{
		      CAN2_Moter_Position_PID[1].ange_set += speed;
		}           			 
}

void Fornt_Wheel_Up(int16_t speed,long int position)
{
    if(CAN2_Moter_Position_PID[0].ange_set >=  position)
		{
		      CAN2_Moter_Position_PID[0].ange_set -= speed;
		}

    if(CAN2_Moter_Position_PID[1].ange_set >=  position)
		{
		      CAN2_Moter_Position_PID[1].ange_set -= speed;
		}	
		
}
 



void Back_Wheel_Down(long int pisition,int16_t speed)
{
     if(CAN2_Moter_Position_PID[2].ange_set <=  pisition)
		 {
		      CAN2_Moter_Position_PID[2].ange_set += speed;
		 }

    if(CAN2_Moter_Position_PID[3].ange_set <=  pisition)
		{
		      CAN2_Moter_Position_PID[3].ange_set += speed;
		} 			 
}

void Back_Wheel_Up(int16_t speed,long int position)
{
    if(CAN2_Moter_Position_PID[2].ange_set >=  position)
		{
		      CAN2_Moter_Position_PID[2].ange_set -= speed;
		}

    if(CAN2_Moter_Position_PID[3].ange_set >=position)
		{
		      CAN2_Moter_Position_PID[3].ange_set -= speed;
		}			 
}
 

void Four_Wheel_Up(int16_t speed,long int position)
{
   Fornt_Wheel_Up(speed,position);  
   Back_Wheel_Up(speed,position);	
}



void Four_Wheel_Down(long int pisition ,int16_t speed)
{
	     Fornt_Wheel_Down(pisition,speed);  
       Back_Wheel_Down(pisition,speed);		 
}

void Front_Wheel_Down_Back_Up(long int pisition,int16_t speed,long int back_pisition)
{
	     Fornt_Wheel_Down(pisition,speed);  
       Back_Wheel_Up(speed,back_pisition);    		 
}

void CM_Go_Forware(int16_t speed)
{
	  				CAN1_Motor_PID[0].set = speed;  //底盘前进
				    CAN1_Motor_PID[1].set = -speed;
				    CAN1_Motor_PID[2].set = speed;
				    CAN1_Motor_PID[3].set = -speed;	
}

void CM_Turn_Left(int16_t speed)
{
   CAN1_Motor_PID[0].set = 700;
	 CAN1_Motor_PID[1].set = -speed;
	 CAN1_Motor_PID[2].set = 700;
	 CAN1_Motor_PID[3].set = -speed; 
}

void CM_Turn_Right(int16_t speed)
{
				CAN1_Motor_PID[0].set = speed;
			  CAN1_Motor_PID[1].set = -700;
				CAN1_Motor_PID[2].set = speed;
		    CAN1_Motor_PID[3].set = -700;	
}

void CM_Speed2Zero()
{
				CAN1_Motor_PID[0].set = 0;
			  CAN1_Motor_PID[1].set = 0;
				CAN1_Motor_PID[2].set = 0;
		    CAN1_Motor_PID[3].set = 0;	
}

Lang_state state = FourWheelUp_SATA;	

char g_Lang_flag = 0;

void Go_Island(int16_t Up_Down_speed,int16_t CM_speed,long int maxP)
{
	static char flag1 =0;
	
	static int cnt = 0,cnt1 = 0,cnt2=0;
	
	int i = 0;
	
	if(g_Lang_flag <= 1)
	{
			switch(state)
			{
				case  FourWheelUp_SATA:
					           
						 CmCotrol.Control_CH1 = 0;
						 if(g_Lang_flag == 0)	
						 {
							CmCotrol.Control_CH2  = mydata.Cortol_ch2;
							  mydata.imudata.Angle_Yaw = 0;
						 }
						 else
						 {										    
							  CmCotrol.Control_CH2  = mydata.imudata.Angle_Yaw * 180;
						 }											 
						 Four_Wheel_Down(maxP + 10,Up_Down_speed);
						 
					   if(CAN2_Moter_Position_PID[0].ange_set >= maxP - 100  &&  CAN2_Moter_Position_PID[1].ange_set >= maxP - 100  &&  CAN2_Moter_Position_PID[2].ange_set >= maxP - 100  &&  CAN2_Moter_Position_PID[3].ange_set >= maxP  -100   )
							{												
								   state =   GoForward_SATA;                            											
							}	
								 break;
											
				 case  GoForward_SATA:			                  
				 
				           if(g_Lang_flag == 0)	
							 {
								CmCotrol.Control_CH2  = mydata.Cortol_ch2;
								  mydata.imudata.Angle_Yaw = 0;
							 }
						 else
						    {										    
							  CmCotrol.Control_CH2  = mydata.imudata.Angle_Yaw * 180;
						    }
						 
									 if( Infrared_1 == 0 )
										  {
											  cnt2++;
											  CmCotrol.Control_CH1 = CM_speed;
										      CmCotrol.Control_CH2 = 0;
											  if(cnt2 >= 80)  
												 {
													 cnt2 = 0;
												   state =   FrontWheelUp_SATA;
													 if(g_Lang_flag == 0)
														 mydata.imudata.Angle_Yaw = 0;
												 }														        	
										  }	
									else
											{
												 CmCotrol.Control_CH1 = CM_speed;
											}														
	
												break;			
																										
						case  FrontWheelUp_SATA:	
							             											
	                      Fornt_Wheel_Up(Up_Down_speed,-20);
                        CmCotrol.Control_CH1 = 0;
						            CmCotrol.Control_CH2 = 0;
                    	if(CAN2_Moter_Position_PID[0].ange_set <=  50 && CAN2_Moter_Position_PID[1].ange_set <=   50 )		
														{
																	cnt2++; 
																	
																	if(cnt2 >=60)
																	{
																			 cnt2 = 61;
																			if(Infrared_2 == 0)  
																			{
																				cnt1++;
																				CmCotrol.Control_CH1 = CM_speed;
																				CmCotrol.Control_CH2 = 0; 
																				
																				if(cnt1 >=50)
																					{
																							 cnt1 = 0;
																						   cnt2 = 0;
																							 state =   BackWheelUp_SATA;																	         
																					}
																			}
																		 else 
																			{
																				 CmCotrol.Control_CH2 = mydata.imudata.Angle_Yaw * 180;
																				 CmCotrol.Control_CH1 = CM_speed; 
																			}																 
																 }	
													  }
                                break;																			 
																					
						case  BackWheelUp_SATA:		
						
					                Back_Wheel_Up(Up_Down_speed,-20);
						              CmCotrol.Control_CH1 = 0;
													if(CAN2_Moter_Position_PID[2].ange_set <=   50 && CAN2_Moter_Position_PID[2].ange_set <=  50 )		
															{   
                                  cnt2++;
                                 if(cnt2 >=50)
																	{
																		 cnt2  = 51;
																		  if(Infrared_3 == 0)
																			  {																					 
																				   flag1 = 1;
																			  }
																    else 
																				{
																					  CmCotrol.Control_CH1 = CM_speed;
                                            CmCotrol.Control_CH2 = mydata.imudata.Angle_Yaw * 180;																					 
																				}
																				 
																		 if(flag1 == 1)
																				 {
																				   	 cnt++;
																						 CmCotrol.Control_CH1 = CM_speed;  
																					   CmCotrol.Control_CH2 = mydata.imudata.Angle_Yaw * 180;
																					 if(cnt == 2)
																					  {
																						    cnt   = 0;
																							  cnt2  = 0;
																								flag1 = 0;
																						 		 CmCotrol.Control_CH1 = 0; 
                                                 CmCotrol.Control_CH2 = 0;																							
																								g_Lang_flag += 1 ;
																						 
																								state  =  FourWheelUp_SATA; 
																							  if(g_Lang_flag == 2) 
																							  {
																									g_Lang_flag = 2;	
                                                  																									
																							  }																					 
																					  } 
																				 }																										 
																	}																				 
								                }	
                          break;
            default :break;																				 
			 }	
			
		for(i=0;i<4;i++)
		{
		    CAN2_Moter_Position_PID[i].out = 	Moter_position_pid(&CAN2_Moter_Position_PID[i]);		
		}
	}
}



void CM_U_D_A_Catch_Control(long int pisition,int16_t speed)
{
	int i = 0;
	static char init_flag = 0;
	static int cnt  = 0;
	  if(P_Key_ide1[1]->Key_Event == KEY_ONE_CLICK || P_Key_ide1[1]->Key_Event == KEY_LONG_CLICK || P_Key_ide1[1]->Key_Event == KEY_NO_CLICK)
				{
				  // Four_Wheel_Up(speed); 
					 if(init_flag == 0)  //初始时让4个电机抬升 和夹手复位
					 {
						 cnt++;
						    if(cnt >=150)
								{
								   cnt =151;
								 if(SPACE_LIMIT1 != 0)
										{
											 CAN2_Moter_Position_PID[0].init_angle -= 100 ;     
										}
										

										if(SPACE_LIMIT2 != 0)
										{
											 CAN2_Moter_Position_PID[1].init_angle -= 100 ;     
										}

										if(SPACE_LIMIT3 != 0)
										{
											 CAN2_Moter_Position_PID[2].init_angle -= 100 ;     
										}
										
										if(SPACE_LIMIT4 != 0)
										{
											 CAN2_Moter_Position_PID[3].init_angle -= 100 ;     
										}										

//										if(SPACE_LIMIT7 != 0)
//										{
//											 CAN2_Moter_Position_PID[5].init_angle += 10 ; 
//                                             CAN2_Moter_Position_PID[4].init_angle -= 10 ;  											
//										}	
										
											
										
									 if(SPACE_LIMIT1 == 0 && SPACE_LIMIT2 == 0 && SPACE_LIMIT3 == 0 && SPACE_LIMIT4 == 0)
									 {
										    
									      init_flag = 1;    
										 CAN2_Moter_Position_PID[5].init_angle = CAN2_Moter_Position_PID[5].ange_get;
										 CAN2_Moter_Position_PID[4].init_angle = CAN2_Moter_Position_PID[4].ange_get;										 
									 }
																				
								}																				
					   }
					else
						{										
							if(P_Key_ide1[3]->Key_Event == KEY_ONE_CLICK || P_Key_ide1[3]->Key_Event == KEY_LONG_CLICK )		
							{
							    
								 g_CM_Up_Down_staus = 1;
								 P_Key_ide1[3]->Key_Event = KEY_NO_CLICK;
							}
							else if(P_Key_ide1[3]->Key_Event == KEY_DOUBLE_CLICK)	
							{
							  g_CM_Up_Down_staus =2; 
								P_Key_ide1[3]->Key_Event = KEY_NO_CLICK;
							}								
							
							
							if(P_Key_ide1[4]->Key_Event == KEY_ONE_CLICK || P_Key_ide1[4]->Key_Event == KEY_LONG_CLICK )		
							{
								g_CM_Up_Down_staus =3;							   
								   P_Key_ide1[4]->Key_Event = KEY_NO_CLICK;
							}
							else if(P_Key_ide1[4]->Key_Event == KEY_DOUBLE_CLICK)	
							{
								g_CM_Up_Down_staus = 4;
							  P_Key_ide1[4]->Key_Event = KEY_NO_CLICK;
							}		
              
							
							if(P_Key_ide1[6]->Key_Event == KEY_ONE_CLICK || P_Key_ide1[6]->Key_Event == KEY_LONG_CLICK )		
							{
								g_CM_Up_Down_staus = 5;
								P_Key_ide1[6]->Key_Event = KEY_NO_CLICK;
                   								
							}
							else if(P_Key_ide1[6]->Key_Event == KEY_DOUBLE_CLICK)	
							{
								g_CM_Up_Down_staus = 6;
								P_Key_ide1[6]->Key_Event = KEY_NO_CLICK;        								
							}

							if(P_Key_ide1[7]->Key_Event == KEY_ONE_CLICK || P_Key_ide1[7]->Key_Event == KEY_LONG_CLICK )		
							{
								g_CM_Up_Down_staus = 7;
								P_Key_ide1[7]->Key_Event = KEY_NO_CLICK;
                   								
							}
							else if(P_Key_ide1[7]->Key_Event == KEY_DOUBLE_CLICK)	
							{
								g_CM_Up_Down_staus = 8;
								P_Key_ide1[7]->Key_Event = KEY_NO_CLICK;        								
							}							
              							
              
							
							switch(g_CM_Up_Down_staus)
							{
								case 1 : Fornt_Wheel_Up(speed,-20); break;
								
								case 2 : Fornt_Wheel_Down(pisition,speed); break;
								
								case 3 : Back_Wheel_Up(speed,-20);break;
								
								case 4 : Back_Wheel_Down(pisition,speed);  break;
								
								case 7 : catch_flag = 0;
								         cather_status = INIT_SATA;
								         Fornt_Wheel_Up(speed,-20); 
							           Back_Wheel_Up(speed,-20);break;
								
								case 5 : Fornt_Wheel_Up(speed,-20); 
							           Back_Wheel_Up(speed,-20);break;
								
								case 6 : Fornt_Wheel_Down(pisition,speed);  
							           Back_Wheel_Down(pisition,speed); break;
								
								case 8 : 
									       catcher_control(9500,8192 * 95,pisition);//控制抓弹药箱动作 params1:夹取位置 params2:第二个块的位置		
								          
									       if(catch_flag == 0)
												 {
													Fornt_Wheel_Down(pisition - 8192 * 5.0,speed);  
													Back_Wheel_Down(pisition  - 8192 * 5.0,speed); 												    
												 }
												 else if(catch_flag == 1)
												 {
													Fornt_Wheel_Up(speed, 8192 *5.0); 
													Back_Wheel_Up(speed,  8192 *5.0);break;  
												 }
												 else
												 {
													Fornt_Wheel_Down(pisition  ,speed);  
													Back_Wheel_Down(pisition  ,speed);
                             												   
												 }
								default:break;
							}					
              													  
						}					
				}
				else if(P_Key_ide1[1]->Key_Event == KEY_DOUBLE_CLICK)
				{
				    Front_Wheel_Down_Back_Up(pisition,speed,-20); 
				}
								
			for(i=0;i<4;i++)
					{
					  	Moter_position_pid(&CAN2_Moter_Position_PID[i]);										
					}
}

static void Magazine_UpDown_Control(char flag ,long int angle_set) //弹仓升降控制 
{
	static char init_flag = 0;
	static int cnt = 0;
	
	if(init_flag == 0)
	{
		  cnt++;
      if(cnt >=150)	
			{
			   cnt =151;
				
				 if(SPACE_LIMIT5 != 0)
				 {
				    CAN1_Moter_Position_PID[6].init_angle -= 600 ; 
				 }
				 else
				 {
				    init_flag = 1;
				 }
			}				   
	 }
	else
	{ 
			if(flag == 1)
			{
				   if(CAN1_Moter_Position_PID[6].ange_set <= angle_set)
						 {
								 CAN1_Moter_Position_PID[6].ange_set += 500; 															   										 															  															 																  																															  
						 }	
			}	 
     else
		 {
 				  if(CAN1_Moter_Position_PID[6].ange_set >= -20)
							 {
										 CAN1_Moter_Position_PID[6].ange_set -= 500; 															   										 															  															 																  																															  
							}				    
		 }			 
	 }
}


void bullet_control(long int position)
{
	 static int cnt  =0,cnt1 = 0;
	static char flag = 0;
	if(mydata.dbus.key.v == KEY_PRESSED_OFFSET_SHIFT)
	{     
        flag = 1;
	 }
	else if(mydata.dbus.key.v == KEY_PRESSED_OFFSET_CTRL)
	{  
      flag  =0;		
	}
	
	if(flag == 1)
	{
//  		    DANCANG_L = 20;
//		      DANCANG_R = 10;
//		      
//		      cnt ++;
//		      cnt1=0;
//		       if(cnt >= 400)
//					 {
//						   cnt =401;
//						  
					     Bsp.OpenTheBall_(1);    
//						   OpenTheBall(1);

									   if(CAN2_Moter_Position_PID[6].ange_set >= -position )
										 {
										    CAN2_Moter_Position_PID[6].ange_set -= 10;
										 }
									
						
					// }	   
	}
	else
	{
	  if(CAN2_Moter_Position_PID[6].ange_set <= 0 )
		{
		   CAN2_Moter_Position_PID[6].ange_set  +=5;   
		}

		
		if(CAN2_Moter_Position_PID[6].ange_set  >= 0)
		{
		   Bsp.OpenTheBall_(0);
		}	  
	}
}



void Camera_Control(int16_t speed)
{
	  static int control_val = 15,cnt = 0,cnt1 = 0;
 
	  if(mydata.dbus.mouse.press_l == 1)
		{ 
			  cnt++;
			  if(cnt >= speed)
				{
				   cnt =0;
					 control_val++;
				}
				if(control_val >=25 ) control_val=25; 			
		}
		else if(mydata.dbus.mouse.press_r == 1)
		{
			cnt1++;
			if(cnt1 >= speed)
			{
			   cnt1 = 0;
				 control_val--;
			} 
			 if(control_val <=5 ) control_val=5; 		
		}
	//	CAMERA = control_val;
		
}



void Key_Scan(uint16_t Key_Value,p_Key_ide key_id)
{
  
			switch(key_id->Key_State)
					{

						case KEY_UP :
							 if(mydata.dbus.key.v  == Key_Value)
								       key_id->Key_State =    KEY_DOWN;			
							break;
						
				  	case KEY_DOWN :
							 if(mydata.dbus.key.v  == Key_Value)
								       key_id->Key_State =    KEY_HOLE;
               else
                       key_id->Key_State =    KEY_UP; 	//干扰信号							 											
							break;
						
						case KEY_HOLE :
               				  
						     key_id->cnt++;
						     if(key_id->cnt < 300)
								 {
								    if(mydata.dbus.key.v  != Key_Value)
										{ 
											 key_id->cnt = 0;
										   key_id->Key_State =    KEY_CLICK;       
										}
								 }
								 else 
								 {
								    key_id->cnt = 0;
                    key_id->Key_State =    KEY_LONG;									 
								 }
						
							break;

						case KEY_CLICK:
							   (key_id->cnt)++;
						     if(key_id->cnt < 30)
								 {
								    if(mydata.dbus.key.v  == Key_Value)
										{
											   key_id->cnt = 0;
										     key_id->Key_State =    KEY_TWO;  
										}
								 }
								 else
								 {
								    key_id->cnt = 0;
									  key_id->Key_State =    KEY_ONE;  
								 }
						
							
						   break;
								 
								 
						case KEY_ONE :
							  key_id->Key_Event = KEY_ONE_CLICK;
						    key_id->Key_State =    KEY_UP;							
							break;								 
						
						case KEY_TWO :
							   key_id->Key_Event = KEY_DOUBLE_CLICK;
						    if(mydata.dbus.key.v  != Key_Value)
						         key_id->Key_State =    KEY_UP;							
							break;
						
						
					  case KEY_LONG :
							   key_id->Key_Event = KEY_LONG_CLICK;
						    if(mydata.dbus.key.v  != Key_Value)
						         key_id->Key_State =    KEY_UP;								
							break;					
				
					}				
}


