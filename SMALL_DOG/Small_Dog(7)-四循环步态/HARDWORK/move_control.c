#include "main.h"

/*************************************************数据定义区*************************************************/
//腿数据结构体
Leg_Data_Typedef Left_Front_Leg_Struct;
Leg_Data_Typedef Left_Back_Leg_Struct;
Leg_Data_Typedef Right_Front_Leg_Struct;
Leg_Data_Typedef Right_Back_Leg_Struct;

//
 char Left_Front_Start_Flag=0;
 char Left_Back_Start_Flag=0;
 char Right_Back_Start_Flag=0;
 char Right_Front_Start_Flag=0;	
	
 char count_num=RIGHT_FRONT_LEG;
 char wheel_turn=LEFT_FRONT_LEG;

//计时参数
int TIM_t=0;


/*************************************************函数定义区**************************************************/
/**
  * @brief  移动控制任务函数
  * @param  void
  * @retval void
  * @notes  根据遥控器的模式选择不同的方式
  */
void Move_Control_Task(void *pvParameters)
{
	
	//陀螺仪初始化，开启陀螺仪中断
	IMU_Init();
	vTaskDelay(500);
	//舵机初始化，头舵机设置为90°
  Steering_Init();
	//足端位置初始化，站立
	Foot_Direction_Init();

	vTaskDelay(1500);	
	
	Beep_ON ();
	delay_ms(500);
	Beep_OFF();
	
  TIM_t=0;
	while(1)
	{
	
		Gait_Control(DIRECTION_FORWARD);
		
//		Leg_Diretion_Control(&Left_Front_Leg_Struct,0,190);
//		Leg_Diretion_Control(&Left_Back_Leg_Struct,0,130);
//		Leg_Diretion_Control(&Right_Front_Leg_Struct,0,150);
//		Leg_Diretion_Control(&Right_Back_Leg_Struct,0,150);
		
//		if(DBUS.RC.ch0>0)
//		{
//	  Foot_Control(&Left_Front_Leg_Struct,STEP_FORWARD,DIRECTION_FORWARD);
//		Foot_Control(&Left_Back_Leg_Struct,STEP_FORWARD,DIRECTION_FORWARD);
//		Foot_Control(&Right_Front_Leg_Struct,STEP_FORWARD,DIRECTION_FORWARD);
//		Foot_Control(&Right_Back_Leg_Struct,STEP_FORWARD,DIRECTION_FORWARD);
//		}		
//		
//		else 
//		{
//  	Foot_Control(&Left_Front_Leg_Struct,QUIESECENCE,DIRECTION_FORWARD);
//		Foot_Control(&Left_Back_Leg_Struct,QUIESECENCE,DIRECTION_FORWARD);
//		Foot_Control(&Right_Front_Leg_Struct,QUIESECENCE,DIRECTION_FORWARD);
//		Foot_Control(&Right_Back_Leg_Struct,QUIESECENCE,DIRECTION_FORWARD);
//		}
		
		
	  vTaskDelay(10);
	}

}


/**
  * @brief  轮式滑行控制函数
  * @param  无                    
  * @retval void
  * @notes  用遥控器控制机械狗前进，转向
  */
void Slide(void)
{
  int speed=0;
	char turn_flag=0;
	
	speed=abs(DBUS.RC.ch3)*1/2+1000;
	
	Left_Front_Speed(speed-70)//950
	Left_Back_Speed(speed)//1041
	Right_Front_Speed(speed)//1033
	Right_Back_Speed(speed+30)//1036
	
  if(DBUS.RC.ch0>0)
	{
		speed=abs(DBUS.RC.ch0) *1/3+1000;
	  Left_Front_Speed(speed-70)
	  Left_Back_Speed(speed)
		Right_Front_Speed(WHEEL_STOP)
	  Right_Back_Speed(WHEEL_STOP)
		
		Right_Front_Leg_Struct.Leg_X =FOOT_X;
		Right_Front_Leg_Struct.Leg_Y =FOOT_Y+20;
		Coordinate_Inverse(&Right_Front_Leg_Struct);
		Right_Front_IN(Right_Front_Leg_Struct.Steerint_IN )
		Right_Front_OUT(Right_Front_Leg_Struct.Steerint_OUT )	
			
		Right_Back_Leg_Struct.Leg_X =FOOT_X+15;
		Right_Back_Leg_Struct.Leg_Y =FOOT_Y-STEP_HIGHT;
		Coordinate_Inverse(&Right_Back_Leg_Struct);
		Right_Back_In(Right_Back_Leg_Struct.Steerint_IN )
		Right_Back_OUT(Right_Back_Leg_Struct.Steerint_OUT )
		
		turn_flag=1;
	
	}
  else if(DBUS.RC.ch0<0)
	{
	  speed=abs(DBUS.RC.ch0) *1/3+1000;
	  Left_Front_Speed(WHEEL_STOP-70)
	  Left_Back_Speed(WHEEL_STOP)
		Right_Front_Speed(speed)
	  Right_Back_Speed(speed)
	
	  Left_Front_Leg_Struct.Leg_X =FOOT_X;
	  Left_Front_Leg_Struct.Leg_Y =FOOT_Y+20;
    Coordinate_Inverse(&Left_Front_Leg_Struct);
	  Left_Front_IN(Left_Front_Leg_Struct.Steerint_IN )
    Left_Front_OUT(Left_Front_Leg_Struct.Steerint_OUT )
		
  	Left_Back_Leg_Struct.Leg_X =FOOT_X+15;
  	Left_Back_Leg_Struct.Leg_Y =FOOT_Y-STEP_HIGHT;
  	Coordinate_Inverse(&Left_Back_Leg_Struct);
  	Left_Back_In(Left_Back_Leg_Struct.Steerint_IN )
    Left_Back_OUT(Left_Back_Leg_Struct.Steerint_OUT )
		
		turn_flag=1;
	
	}	
  else
  {
		//if(turn_flag==1)
		{
			
	  	Shrink();
			vTaskDelay(500); 
			turn_flag=0;
		}
		
	}		
}



/**
  * @brief  坐标反解函数
  * @param  Leg_Data：腿部数据结构体指针                    
  * @retval void
  * @notes  使用反解公式将X,Y坐标反解为舵机的输出控制值，
            将解算的值存在腿数据结构体中的成员。
  */
void Coordinate_Inverse(Leg_Data_Typedef* Leg_Data)
{
	//角度定义
	double  seigamar=0;
	double  gama=0;
	
	
	//腿部误差修正,右后腿
	if(Leg_Data->Leg_Type ==RIGHT_BACK_LEG)
	{
	  Leg_Data->Leg_Y+=30;
	}
	
	
  //坐标反解公式
	seigamar=asin(sqrt(Leg_Data->Leg_X*Leg_Data->Leg_X+Leg_Data->Leg_Y*Leg_Data->Leg_Y)/2/CLUB_LENGTH)+atan(Leg_Data->Leg_X/Leg_Data->Leg_Y);
  gama    =asin(sqrt(Leg_Data->Leg_X*Leg_Data->Leg_X+Leg_Data->Leg_Y*Leg_Data->Leg_Y)/2/CLUB_LENGTH)-atan(Leg_Data->Leg_X/Leg_Data->Leg_Y);
  
	
	
	//舵机输出控制值解算
	if(Leg_Data->Leg_Type <=LEFT_BACK_LEG)//左侧腿
	{
		Leg_Data->Steerint_OUT =100*seigamar/pi+100;
		Leg_Data->Steerint_IN  =100*gama/pi+100;
	}
	else//右侧腿
	{
	   Leg_Data->Steerint_OUT =200-100*seigamar/pi;
	   Leg_Data->Steerint_IN =200-100*gama/pi;
	}
}

/**
  * @brief  腿部坐标控制函数
  * @param  void
  * @retval void
  * @notes  使用坐标控制足端位置
  */
void Leg_Diretion_Control(Leg_Data_Typedef* Leg_Data,double foot_x,double foot_y)
{
	switch (Leg_Data->Leg_Type)
	{		
		 //左前腿迈步
			case LEFT_FRONT_LEG :
			{					
					Left_Front_Leg_Struct.Leg_X =foot_x;
					Left_Front_Leg_Struct.Leg_Y =foot_y;
					Coordinate_Inverse(&Left_Front_Leg_Struct);
					Left_Front_IN(Left_Front_Leg_Struct.Steerint_IN )
					Left_Front_OUT(Left_Front_Leg_Struct.Steerint_OUT )
			}
			break;
			//左后腿迈步
			case LEFT_BACK_LEG :
			{	
				Left_Back_Leg_Struct.Leg_X =foot_x;
				Left_Back_Leg_Struct.Leg_Y =foot_y;
				Coordinate_Inverse(&Left_Back_Leg_Struct);
				Left_Back_In(Left_Back_Leg_Struct.Steerint_IN )
				Left_Back_OUT(Left_Back_Leg_Struct.Steerint_OUT )
			}	
			break;
			//右后腿
			case RIGHT_BACK_LEG :
			{			
				Right_Back_Leg_Struct.Leg_X =foot_x;
				Right_Back_Leg_Struct.Leg_Y =foot_y;
				Coordinate_Inverse(&Right_Back_Leg_Struct);
				Right_Back_In(Right_Back_Leg_Struct.Steerint_IN )
				Right_Back_OUT(Right_Back_Leg_Struct.Steerint_OUT )
			}	
			break;
			//右前腿
			case RIGHT_FRONT_LEG :
			{
				Right_Front_Leg_Struct.Leg_X =foot_x;
				Right_Front_Leg_Struct.Leg_Y =foot_y;
				Coordinate_Inverse(&Right_Front_Leg_Struct);
				Right_Front_IN(Right_Front_Leg_Struct.Steerint_IN )
				Right_Front_OUT(Right_Front_Leg_Struct.Steerint_OUT )
			}
			break;
			
			default :
			break;	
	}

}

/**
  * @brief  足端轨迹控制函数
  * @param  Leg_Data：腿部数据结构体指针
            trail：轨迹，用于选择修正摆线和直线
            direction：机器狗移动方向，前或后
            t：时基参数，100ms增加1,只能递增
  * @retval void
  * @notes  将周期进行转换让任务时基成为控制足端轨迹的参数，当一条轨迹到达端点时限位，同时更新另一条轨迹的坐标
            然后进行控制数据解算，再根据腿的类型进行控制
  */
void Foot_Control(Leg_Data_Typedef* Leg_Data,char trail,char direction )
{
	//
	double fai=0;
	
	
	//迈步前进
	if(trail==STEP_FORWARD)
	{ 
		 //控时参数转换
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
		 Leg_Data->Leg_X =STEP_LENTH/2/pi*(fai-sin(fai));
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
		 
		 if(Leg_Data->Leg_X > (FOOT_X+STEP_LENTH))Leg_Data->Leg_X = (FOOT_X+STEP_LENTH);
		 if(Leg_Data->Leg_X < (FOOT_X))Leg_Data->Leg_X = FOOT_X;

		 Leg_Data->Leg_Y=FOOT_Y;
		 
		//轨迹衔接
		if(Leg_Data->Leg_X == (FOOT_X+STEP_LENTH))
		  Leg_Data->step_forward_time=2*pi;
	  if(Leg_Data->Leg_X == FOOT_X)
			Leg_Data->step_forward_time=0;
	
	}
	
	
	//反解
  Coordinate_Inverse(Leg_Data);
	
	//控制
	switch(Leg_Data->Leg_Type)
	{
		case LEFT_FRONT_LEG :
		{
			Left_Front_IN(Leg_Data->Steerint_IN )
			Left_Front_OUT(Leg_Data->Steerint_OUT )
			break;
		}
	  case LEFT_BACK_LEG :
		{
			Left_Back_In(Leg_Data->Steerint_IN )
			Left_Back_OUT(Leg_Data->Steerint_OUT )
			break;
		}
	 case RIGHT_FRONT_LEG :
		{
			Right_Front_IN(Leg_Data->Steerint_IN )
			Right_Front_OUT(Leg_Data->Steerint_OUT )
			break;
		}
		case RIGHT_BACK_LEG :
		{
			Right_Back_In(Leg_Data->Steerint_IN )
			Right_Back_OUT(Leg_Data->Steerint_OUT )
			break;
		}
		
		default :
			break;
	} 
}

/**
  * @brief  步态控制函数
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
	  if(TIM_t%(STEP_FORWARD_TIME+100)==0)
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
	
	 
	Left_Front_Leg_Struct.step_forward_time=0;
	Left_Front_Leg_Struct.Leg_X =FOOT_X;
	Left_Front_Leg_Struct.Leg_Y =FOOT_Y;
	Coordinate_Inverse(&Left_Front_Leg_Struct);
	Left_Front_IN(Left_Front_Leg_Struct.Steerint_IN )
  Left_Front_OUT(Left_Front_Leg_Struct.Steerint_OUT )
  
	Left_Back_Leg_Struct.step_forward_time=0;
	Left_Back_Leg_Struct.Leg_X =FOOT_X;
	Left_Back_Leg_Struct.Leg_Y =FOOT_Y;
	Coordinate_Inverse(&Left_Back_Leg_Struct);
	Left_Back_In(Left_Back_Leg_Struct.Steerint_IN )
  Left_Back_OUT(Left_Back_Leg_Struct.Steerint_OUT )

  Right_Back_Leg_Struct.step_forward_time=0;
  Right_Back_Leg_Struct.Leg_X =FOOT_X;
	Right_Back_Leg_Struct.Leg_Y =FOOT_Y;
	Coordinate_Inverse(&Right_Back_Leg_Struct);
	Right_Back_In(Right_Back_Leg_Struct.Steerint_IN )
  Right_Back_OUT(Right_Back_Leg_Struct.Steerint_OUT )
	
	Right_Front_Leg_Struct.step_forward_time=0;
	Right_Front_Leg_Struct.Leg_X =FOOT_X;
	Right_Front_Leg_Struct.Leg_Y =FOOT_Y;
	Coordinate_Inverse(&Right_Front_Leg_Struct);
	Right_Front_IN(Right_Front_Leg_Struct.Steerint_IN )
  Right_Front_OUT(Right_Front_Leg_Struct.Steerint_OUT )
	
	
}

/**
  * @brief  足端位置初始化函数
  * @param  void
  * @retval void
  * @notes  足端回位，电机速度归零，
  */
void Foot_Direction_Init(void)
{
	//腿部类型标定
	Left_Front_Leg_Struct.Leg_Type  = LEFT_FRONT_LEG;
	Left_Back_Leg_Struct.Leg_Type   = LEFT_BACK_LEG;
	Right_Front_Leg_Struct.Leg_Type = RIGHT_FRONT_LEG;
  Right_Back_Leg_Struct.Leg_Type  = RIGHT_BACK_LEG;
	
  //腿部回原点
	Shrink();
	
	
	//轮电机激活	
	Left_Front_Speed(400)
	Left_Back_Speed(400)
  Right_Back_Speed(400)
  Right_Front_Speed(400)
	
	vTaskDelay(1500);	
  //轮电机速度归0,左前轮要减50
  Left_Front_Speed(WHEEL_STOP-70)
	Left_Back_Speed(WHEEL_STOP)
	Right_Front_Speed(WHEEL_STOP)
	Right_Back_Speed(WHEEL_STOP+30)
	

}











