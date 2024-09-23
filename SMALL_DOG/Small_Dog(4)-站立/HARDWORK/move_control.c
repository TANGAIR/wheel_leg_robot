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
	
 char count_num=RIGHT_BACK_LEG;
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
  vTaskDelay(500);
	//足端位置初始化
	Foot_Direction_Init();
	vTaskDelay(2000);
	
	while(1)
	{
		
	  
	
	
	
	
	
	
	
	
	
	  vTaskDelay(1);
	}

}

/**
  * @brief  坐标反解函数
  * @param  void
  * @retval void
  * @notes  使用反解公式将X,YZ坐标反解为舵机的输出控制值，将解算的值存在结构体中的成员。
  */
void Coordinate_Inverse(Leg_Data_Typedef* Leg_Data)
{
	//角度定义
	double  seigamar=0;
	double  gama=0;
	
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
  * @brief  足端轨迹控制函数
  * @param  void
  * @retval void
  * @notes  void
  */
void Foot_Control(Leg_Data_Typedef* Leg_Data,char trail,char direction ,int t)
{
	
	double fai=0;
	static int last_t=0;
	
	
	if(trail==STEP_FORWARD)
	{ 
		 //控时参数转换
	   if(direction==DIRECTION_FORWARD) 
			 Leg_Data->step_forward_time +=(t-last_t)*2*pi/STEP_FORWARD_TIME; 		
		 else if(direction==DIRECTION_BACK)
			 Leg_Data->step_forward_time -=(t-last_t)*2*pi/STEP_FORWARD_TIME; 
		 
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
  else if(trail==DIRECTION_BACK)
	{
		 //控时参数转换//轨迹坐标计算
	   if(direction==DIRECTION_FORWARD)
			 Leg_Data->Leg_X  -=(t-last_t)/QUIESECENCE_TIME*STEP_LENTH;
	   else if(direction==DIRECTION_BACK)
			 Leg_Data->Leg_X  +=(t-last_t)/QUIESECENCE_TIME*STEP_LENTH;
		 
		 Leg_Data->Leg_Y = FOOT_Y;
		 
		//限位，x坐标应该为FOOT_X~FOOT_X+STEP_LENTH
		if(Leg_Data->Leg_X >(FOOT_X+STEP_LENTH))Leg_Data->Leg_X =(FOOT_X+STEP_LENTH);
		if(Leg_Data->Leg_X <FOOT_X)Leg_Data->Leg_X =FOOT_X;
		
		if(Leg_Data->Leg_X == (FOOT_X+STEP_LENTH))
		  Leg_Data->step_forward_time=2*pi;
	  if(Leg_Data->Leg_X == FOOT_X)
			Leg_Data->step_forward_time=0;
	
	}
	//更新时基
  last_t=t;
	
	
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


void Gait_Control(char gait)
{
	
	
	if(gait!=SHRINK)
	{
	  TIM_t++;
	  if(TIM_t%STEP_FORWARD_TIME==0)wheel_turn+=11;
		if(wheel_turn>RIGHT_BACK_LEG)wheel_turn=LEFT_FRONT_LEG;
		
		switch(wheel_turn)
		{
			case LEFT_FRONT_LEG :
			{
				//
				if(Left_Front_Start_Flag==0)
					{
					}						
				//
				Left_Front_Start_Flag=1;
				//
				Foot_Control(&Left_Front_Leg_Struct,STEP_FORWARD,gait,TIM_t);
				if(count_num==LEFT_FRONT_LEG)
				{
					Foot_Control(&Left_Back_Leg_Struct,QUIESECENCE,gait,TIM_t);
				  Foot_Control(&Right_Front_Leg_Struct,QUIESECENCE,gait,TIM_t);
				  Foot_Control(&Right_Back_Leg_Struct,QUIESECENCE,gait,TIM_t);
				}		
			}
			case LEFT_BACK_LEG :
			{
				//
				if(Left_Back_Start_Flag==0)
				{
				}
				//
				Left_Back_Start_Flag=1;
				//
				Foot_Control(&Left_Back_Leg_Struct,STEP_FORWARD,gait,TIM_t);
				if(count_num==LEFT_BACK_LEG)
				{
					Foot_Control(&Left_Front_Leg_Struct,QUIESECENCE,gait,TIM_t);
				  Foot_Control(&Right_Front_Leg_Struct,QUIESECENCE,gait,TIM_t);
				  Foot_Control(&Right_Back_Leg_Struct,QUIESECENCE,gait,TIM_t);
				}		
			}	
			case RIGHT_BACK_LEG :
			{
				//
				if(Right_Back_Start_Flag==0)
				{
				}
				//
				Right_Back_Start_Flag=1;
				//
				Foot_Control(&Right_Back_Leg_Struct,STEP_FORWARD,gait,TIM_t);
				if(count_num==RIGHT_BACK_LEG)
				{
					Foot_Control(&Left_Front_Leg_Struct,QUIESECENCE,gait,TIM_t);
				  Foot_Control(&Right_Front_Leg_Struct,QUIESECENCE,gait,TIM_t);
				  Foot_Control(&Left_Back_Leg_Struct,QUIESECENCE,gait,TIM_t);
				}		
			}	
			case RIGHT_FRONT_LEG :
			{
				//
				if(Right_Front_Start_Flag==0)
				{
				}
				//
				Right_Front_Start_Flag=1;
				//
				Foot_Control(&Right_Front_Leg_Struct,STEP_FORWARD,gait,TIM_t);
				if(count_num==RIGHT_BACK_LEG)
				{
					Foot_Control(&Left_Front_Leg_Struct,QUIESECENCE,gait,TIM_t);
				  Foot_Control(&Right_Back_Leg_Struct,QUIESECENCE,gait,TIM_t);
				  Foot_Control(&Left_Back_Leg_Struct,QUIESECENCE,gait,TIM_t);
				}		
			}
		  if((Left_Front_Start_Flag==1)&&(Left_Back_Start_Flag==1)&&(Right_Back_Start_Flag==1)&&(Right_Front_Start_Flag=1))
			{
			  count_num+=11;
				if(count_num>44)count_num=11;
				
				Left_Front_Start_Flag=0;
	      Left_Back_Start_Flag=0;
	      Right_Back_Start_Flag=0;
        Right_Front_Start_Flag=0;	
	
			}
		
		}
	}
	if(gait==SHRINK)
	{
	  Shrink();
	
	}
  






}



void Shrink(void)
{
	
	 Left_Front_Start_Flag=0;
	 Left_Back_Start_Flag=0;
	 Right_Back_Start_Flag=0;
	 Right_Front_Start_Flag=0;	
		
	 count_num=RIGHT_BACK_LEG;
	 wheel_turn=LEFT_FRONT_LEG;
	
	 
	//Left_Front_Leg_Struct.step_forward_time =FOOT_X;
	
	
	
	Left_Front_Leg_Struct.Leg_X =FOOT_X;
	Left_Front_Leg_Struct.Leg_Y =FOOT_Y;
	Coordinate_Inverse(&Left_Front_Leg_Struct);
	Left_Front_IN(Left_Front_Leg_Struct.Steerint_IN )
  Left_Front_OUT(Left_Front_Leg_Struct.Steerint_OUT )
  
	Left_Back_Leg_Struct.Leg_X =FOOT_X;
	Left_Back_Leg_Struct.Leg_Y =FOOT_Y;
	Coordinate_Inverse(&Left_Back_Leg_Struct);
	Left_Back_In(Left_Back_Leg_Struct.Steerint_IN )
  Left_Back_OUT(Left_Back_Leg_Struct.Steerint_OUT )

  Right_Back_Leg_Struct.Leg_X =FOOT_X;
	Right_Back_Leg_Struct.Leg_Y =FOOT_Y;
	Coordinate_Inverse(&Right_Back_Leg_Struct);
	Right_Back_In(Right_Back_Leg_Struct.Steerint_IN )
  Right_Back_OUT(Right_Back_Leg_Struct.Steerint_OUT )
	
	Right_Front_Leg_Struct.Leg_X =FOOT_X;
	Right_Front_Leg_Struct.Leg_Y =FOOT_Y;
	Coordinate_Inverse(&Right_Front_Leg_Struct);
	Right_Front_IN(Right_Front_Leg_Struct.Steerint_IN )
  Right_Front_OUT(Right_Front_Leg_Struct.Steerint_OUT )
	
	
}


void Foot_Direction_Init(void)
{
	//腿部类型标定
	Left_Front_Leg_Struct.Leg_Type  = LEFT_FRONT_LEG;
	Left_Back_Leg_Struct.Leg_Type   = LEFT_BACK_LEG;
	Right_Front_Leg_Struct.Leg_Type = RIGHT_FRONT_LEG;
  Right_Back_Leg_Struct.Leg_Type  = RIGHT_BACK_LEG;
  //轮电机速度归0
  Left_Front_Speed(WHEEL_STOP)
	Left_Back_Speed(WHEEL_STOP)
	Right_Front_Speed(WHEEL_STOP)
	Right_Back_Speed(WHEEL_STOP)
	//腿部回原点
	Shrink();

}











