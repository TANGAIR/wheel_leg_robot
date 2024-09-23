#include "main.h"

/*************************************************数据定义区*************************************************/
//腿数据结构体,
Leg_Data_Typedef Left_Front_Leg_Struct;
Leg_Data_Typedef Left_Back_Leg_Struct;
Leg_Data_Typedef Right_Back_Leg_Struct;
Leg_Data_Typedef Right_Front_Leg_Struct;


//步态控制标志位
 char Left_Front_Start_Flag=0;
 char Left_Back_Start_Flag=0;
 char Right_Back_Start_Flag=0;
 char Right_Front_Start_Flag=0;

 char count_num=RIGHT_FRONT_LEG;
 char wheel_turn=LEFT_FRONT_LEG;


 //计时参数
 int TIM_t=0;

/*************************************************函数定义区**************************************************/


/********************************************滑行***************************************************/
/**
  * @brief  轮式滑行控制函数
  * @param  无                    
  * @retval void
  * @notes  用遥控器控制机械狗前进，转向
  */
void Slide(void)
{
	float RC_Motor_Ra=10;
	
      Wheel_LF.Target_Speed =(DBUS.RC.ch1+DBUS.RC.ch0)*RC_Motor_Ra;
			Wheel_LB.Target_Speed =(DBUS.RC.ch1+DBUS.RC.ch0)*RC_Motor_Ra;
			Wheel_RB.Target_Speed =(DBUS.RC.ch1-DBUS.RC.ch0)*RC_Motor_Ra;
			Wheel_RF.Target_Speed =(DBUS.RC.ch1-DBUS.RC.ch0)*RC_Motor_Ra;
}


/********************************************步行***************************************************/
/**
  * @brief  坐标反解,发送控制函数
  * @param  Leg_Data：腿部数据结构体指针                    
  * @retval void
  * @notes  使用反解公式将X,Y坐标反解为电机角速位置值，并且输出
                                            
  */
void Coordinate_Inverse(Leg_Data_Typedef* Leg_Data)
{
	//角度定义,单位弧度
	double  seigamar=0;
	double  gama=0;
  
  
  //坐标反解公式
	seigamar=asin(sqrt(Leg_Data->Leg_X*Leg_Data->Leg_X+Leg_Data->Leg_Y*Leg_Data->Leg_Y)/2/CLUB_LENGTH)+atan(Leg_Data->Leg_X/Leg_Data->Leg_Y);//长，上,外
  gama    =asin(sqrt(Leg_Data->Leg_X*Leg_Data->Leg_X+Leg_Data->Leg_Y*Leg_Data->Leg_Y)/2/CLUB_LENGTH)-atan(Leg_Data->Leg_X/Leg_Data->Leg_Y);//短，下，内
  
	
	//CAN1控制输出
	
	switch(Leg_Data->Leg_Type)
	{
		//左腿
		case LEFT_FRONT_LEG :
		{
			Joint_LF_U.send.send_Position =-seigamar;
			Joint_LF_D.send .send_Position =-gama;

	    CAN1_TX_Jiont(&Joint_LF_U);
    
			CAN1_TX_Jiont(&Joint_LF_D);
    
			break;
		}
	  case LEFT_BACK_LEG :
		{
			Joint_LB_U.send.send_Position =-seigamar;
			Joint_LB_D.send .send_Position =-gama;

	    CAN1_TX_Jiont(&Joint_LB_U);
     
			CAN1_TX_Jiont(&Joint_LB_D);
     
			break;
		}
		
		
		//右腿
	 case  RIGHT_BACK_LEG:
		{
			Joint_RB_U.send.send_Position =seigamar;
			Joint_RB_D.send .send_Position =gama;

	    CAN1_TX_Jiont(&Joint_RB_U);
     
			CAN1_TX_Jiont(&Joint_RB_D);
     
			break;
		}
		case RIGHT_FRONT_LEG :
		{
			Joint_RF_U.send.send_Position =seigamar;
			Joint_RF_D.send .send_Position =gama;

	    CAN1_TX_Jiont(&Joint_RF_U);//发送控制7号
     
			CAN1_TX_Jiont(&Joint_RF_D);//发送控制8号
     
			break;
		}
		
		default :
			break;
	} 

}






/**
  * @brief  腿部坐标控制函数
  * @param  void
  * @retval void
  * @notes  使用坐标控制足端位置,X轴向前为正，Y轴向下为正
            用于对单腿进行调试
  */
void Leg_Diretion_Control(Leg_Data_Typedef* Leg_Data,float foot_x,float foot_y)
{
	
	
//	//限位
//	if(foot_x>180)foot_x=180;
//	if(foot_x<-180)foot_x=-180;
//	
//	//限位
//	if(foot_y>=360)foot_y=360;
//	if(foot_y<200)foot_y=200;
//	
	switch (Leg_Data->Leg_Type)
	{		
		 //左前腿迈步
			case LEFT_FRONT_LEG :
			{					
					Left_Front_Leg_Struct.Leg_X =foot_x;
					Left_Front_Leg_Struct.Leg_Y =foot_y;
					
			}
			break;
			//左后腿迈步
			case LEFT_BACK_LEG :
			{	
				Left_Back_Leg_Struct.Leg_X =foot_x;
				Left_Back_Leg_Struct.Leg_Y =foot_y;
				
			}	
			break;
			//右后腿
			case RIGHT_BACK_LEG :
			{			
				Right_Back_Leg_Struct.Leg_X =foot_x;
				Right_Back_Leg_Struct.Leg_Y =foot_y;
				
			}	
			break;
			//右前腿
			case RIGHT_FRONT_LEG :
			{
				Right_Front_Leg_Struct.Leg_X =foot_x;
				Right_Front_Leg_Struct.Leg_Y =foot_y;
				
			}
			break;
			
			default :
			break;	
	}
	//坐标反解，控制
	Coordinate_Inverse(Leg_Data);
	
}



/**
  * @brief  单腿足端轨迹控制函数
  * @param  Leg_Data：腿部数据结构体指针
            trail：轨迹，用于选择修正摆线和直线
            direction：机器狗移动方向，前或后
  * @retval void
  * @notes  使用函数运行周期作为基础轨迹参数累加周期，使用各态历经算法，
             以极径为范围，只有到允许偏差之内点才能继续累加
  */
void Foot_Control(Leg_Data_Typedef* Leg_Data,char trail,char direction )
{
	//
	double fai=0;
  double deta_distance_2=0;//真实足端坐标与目标坐标的距离的平方

  
  //由电机的反馈角度计算足端坐标
   Leg_Data->real_x =CLUB_LENGTH*(cos(Leg_Data->Jiont_Motor_D->recieve.recieve_Position)-cos(Leg_Data->Jiont_Motor_U->recieve.recieve_Position));
   Leg_Data->real_y =CLUB_LENGTH*(sin(Leg_Data->Jiont_Motor_D->recieve.recieve_Position)+sin(Leg_Data->Jiont_Motor_U->recieve.recieve_Position));
  //计算距离偏差的平方,距离单位mm，
   deta_distance_2 = (Leg_Data->real_x-Leg_Data->Leg_X)*(Leg_Data->real_x-Leg_Data->Leg_X)+(Leg_Data->real_y-Leg_Data->Leg_Y)*(Leg_Data->real_y-Leg_Data->Leg_Y);
   

	//大于允许偏差则电机一直输出之前的数值，防止轨迹不完整，！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
// if(deta_distance_2<=(DISTANCE_IGNORE*DISTANCE_IGNORE))
// {
 //迈步前进
  if(trail==STEP_FORWARD)
	{ 
		 //控时参数转换,每次运行的时候进行转换
	   if(direction==DIRECTION_FORWARD) 
			 Leg_Data->step_forward_time +=1*2*pi/STEP_FORWARD_TIME;// (t-last_t)		
		 else if(direction==DIRECTION_BACK)
			 Leg_Data->step_forward_time -=1*2*pi/STEP_FORWARD_TIME; //(t-last_t)
		 
     //数据类型转换		 
		 fai=Leg_Data->step_forward_time;
	
		 //限位
		 if(fai>2*pi)fai=2*pi;
		 if(fai<0)fai=0;
		 	 
		 //轨迹坐标计算
		 Leg_Data->Leg_X =STEP_LENTH/2/pi*(fai-sin(fai))+FOOT_X;
		 Leg_Data->Leg_Y =(STEP_HIGHT/2*(1-cos(fai)))*(-1)+FOOT_Y;
		 
		 //轨迹衔接,防止计算出错
		 if(fai==2*pi) 
		 {
			 Leg_Data->Leg_X =(FOOT_X+STEP_LENTH);
			 Leg_Data->Leg_Y = FOOT_Y;
		 }
		 if(fai==0)    
		 {
			 Leg_Data->Leg_X = FOOT_X;
			 Leg_Data->Leg_Y = FOOT_Y;
		 }
	}
	//静退
  else if(trail==QUIESECENCE)
	{
		 //控时参数转换//轨迹坐标计算
	   if(direction==DIRECTION_FORWARD)
			 Leg_Data->Leg_X  -=1*STEP_LENTH/QUIESECENCE_TIME;
	   else if(direction==DIRECTION_BACK)
			 Leg_Data->Leg_X  +=1*STEP_LENTH/QUIESECENCE_TIME;
		
  //限位，X
		 if(Leg_Data->Leg_X > (FOOT_X+STEP_LENTH))Leg_Data->Leg_X = (FOOT_X+STEP_LENTH);
		 if(Leg_Data->Leg_X < (FOOT_X))Leg_Data->Leg_X = FOOT_X;
 //设置Y
		 Leg_Data->Leg_Y=FOOT_Y;
		 
		//轨迹衔接
		if(Leg_Data->Leg_X == (FOOT_X+STEP_LENTH))
		  Leg_Data->step_forward_time=2*pi;
	  if(Leg_Data->Leg_X <= FOOT_X+10)
			Leg_Data->step_forward_time=0;
	
	}
// }

	//坐标反解输出控制
	Coordinate_Inverse(Leg_Data);

}


/**
  * @brief  单腿循环前进函数
  * @param  Leg_Data：腿部数据结构体指针
            
  * @retval void
  * @notes  调用Foot_Control函数实现单腿循环
  */
void signal_leg(Leg_Data_Typedef *Leg_Data)
{
	 
	 TIM_t++;
	 
   if(TIM_t<=STEP_FORWARD_TIME)
	 {

	   Foot_Control(Leg_Data,STEP_FORWARD,DIRECTION_FORWARD);

	 }
	 else if(TIM_t<=(STEP_FORWARD_TIME+QUIESECENCE_TIME+QUIESECENCE_TIME/5))
	 {
	    
		 Foot_Control(Leg_Data,QUIESECENCE,DIRECTION_FORWARD);
	 
	 }
   else
	 {
		 
		 TIM_t=0;
	 
	 }
 }


/**
  * @brief  四足步态控制函数
  * @param  gait：步态，前进，后退，收缩
  * @retval void
  * @notes  采用四循环静态步态，即四条腿依次逆时针迈出，但最后一条腿迈出时，其他三条腿收回
            使用轮转参数控制迈出腿的类型，使用计次参数控制当哪一条腿迈出时其他三条腿收回
            后退时也能适应
  */
void Gait_Control(char gait)
{
	
	//如果步态为前进或者后退
	if(gait!=SHRINK)
	{
		//控时参数递增
	  TIM_t++;
		
		
		
		
		//设置迈腿摆线周期
	  if(TIM_t%(STEP_FORWARD_TIME)==0)
		{
			//当四条腿都迈步时，清空标志位但保留当前腿标志，让计数参数更改，
			if((Left_Front_Start_Flag==1)&&(Left_Back_Start_Flag==1)&&(Right_Back_Start_Flag==1)&&(Right_Front_Start_Flag==1))
			{
				Left_Front_Start_Flag=0;
	      Left_Back_Start_Flag=0;
	      Right_Back_Start_Flag=0;
        Right_Front_Start_Flag=0;	
				//设置当前迈步的腿的标志为1
				switch(count_num)
				{
					case LEFT_FRONT_LEG:
						Left_Front_Start_Flag=1;
					 break;
					case LEFT_BACK_LEG:
						Left_Back_Start_Flag=1;
					 break;
				  case RIGHT_BACK_LEG:
						Right_Back_Start_Flag=1;
					 break;
				  case RIGHT_FRONT_LEG:
						Right_Front_Start_Flag=1;
					break;
				}
				//换计数用腿
			  count_num-=11;
			  //循环衔接
				if(count_num<11)count_num=44;
			}
			//更改轮转用腿
		  wheel_turn+=11;	
		  //循环衔接
		  if(wheel_turn>44)wheel_turn=11;
		}
	  
		
		
		
		switch(wheel_turn)
		{
			//左前腿迈步
			case LEFT_FRONT_LEG :
			{					
				//更新标志位
				Left_Front_Start_Flag=1;
				
		
				
				//左前腿前进
				Foot_Control(&Left_Front_Leg_Struct,STEP_FORWARD,gait);
			  //如果计数为当前腿，则其他腿后退
				if(count_num==LEFT_FRONT_LEG)
				{
					Foot_Control(&Left_Back_Leg_Struct,QUIESECENCE,gait);
				  Foot_Control(&Right_Front_Leg_Struct,QUIESECENCE,gait);
				  Foot_Control(&Right_Back_Leg_Struct,QUIESECENCE,gait);
				}		
			}
			break;
			//左后腿迈步
			case LEFT_BACK_LEG :
			{
				//更新左后腿开始标志
				Left_Back_Start_Flag=1;
			
				
				//左后腿前进
				Foot_Control(&Left_Back_Leg_Struct,STEP_FORWARD,gait);
				//如果计数为当前腿，则其他腿后退
				if(count_num==LEFT_BACK_LEG)
				{
					Foot_Control(&Left_Front_Leg_Struct,QUIESECENCE,gait);
				  Foot_Control(&Right_Front_Leg_Struct,QUIESECENCE,gait);
				  Foot_Control(&Right_Back_Leg_Struct,QUIESECENCE,gait);
				}		
			}	
			break;
			//右后腿
			case RIGHT_BACK_LEG :
			{
			  //更新右后腿标志
				Right_Back_Start_Flag=1;
			
				
				//右后腿前进
				Foot_Control(&Right_Back_Leg_Struct,STEP_FORWARD,gait);
				//如果计数为当前腿，则其他腿后退
				if(count_num==RIGHT_BACK_LEG)
				{
					Foot_Control(&Left_Front_Leg_Struct,QUIESECENCE,gait);
				  Foot_Control(&Right_Front_Leg_Struct,QUIESECENCE,gait);
				  Foot_Control(&Left_Back_Leg_Struct,QUIESECENCE,gait);
				}		
			}	
			break;
			//右前腿
			case RIGHT_FRONT_LEG :
			{
				//更新右前腿标志
				Right_Front_Start_Flag=1;
				
			
				
				//右前腿前进
				Foot_Control(&Right_Front_Leg_Struct,STEP_FORWARD,gait);
				//如果计数为当前腿，则其他腿后退
				if(count_num==RIGHT_FRONT_LEG)
				{
					Foot_Control(&Left_Front_Leg_Struct,QUIESECENCE,gait);
				  Foot_Control(&Right_Back_Leg_Struct,QUIESECENCE,gait);
				  Foot_Control(&Left_Back_Leg_Struct,QUIESECENCE,gait);
				}
       				
			}
			break;
			
			default :
			break;
		}
	}
	//若步态为收缩	
  else if(gait==SHRINK)
	{
		//收缩并初始化腿
	  Shrink();
	}
}




/********************************************轮腿切换与初始化***************************************************/
/**
  * @brief  腿部收缩函数
  * @param  void
  * @retval void
  * @notes  四腿标志位归零，计数参数和轮转参数复位，迈步时间周期归零，足端坐标初始化，足端回位。
  */
void Shrink(void)
{

	 Left_Front_Start_Flag=0;
	 Left_Back_Start_Flag=0;
	 Right_Back_Start_Flag=0;
	 Right_Front_Start_Flag=0;	
		
	 count_num=RIGHT_FRONT_LEG;
	 wheel_turn=LEFT_FRONT_LEG;
	
	 //左前
   Leg_Deinit(&Left_Front_Leg_Struct);
	 //右前
	 Leg_Deinit(&Right_Front_Leg_Struct);
	 //右后
   Leg_Deinit(&Right_Back_Leg_Struct);
   //左后
   Leg_Deinit(&Left_Back_Leg_Struct);
}




/**
  * @brief  腿部自适应初始化
  * @param  void
  * @retval void
  * @notes  四腿标志位归零，计数参数和轮转参数复位，迈步时间周期归零，足端坐标初始化，足端回位。
  */
void Leg_Deinit(Leg_Data_Typedef *Leg_Struct_Point)
{
  Leg_Struct_Point->step_forward_time=0;
	Leg_Struct_Point->Leg_X =FOOT_X;
	Leg_Struct_Point->Leg_Y =FOOT_Y+Leg_Struct_Point->Y_Add ;

	Coordinate_Inverse(Leg_Struct_Point);

}






























