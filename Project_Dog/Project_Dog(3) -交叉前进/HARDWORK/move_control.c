#include "main.h"

/*************************************************数据定义区*************************************************/
//腿数据结构体
Leg_Data_Typedef Left_Front_Leg_Struct;
Leg_Data_Typedef Left_Back_Leg_Struct;
Leg_Data_Typedef Right_Front_Leg_Struct;
Leg_Data_Typedef Right_Back_Leg_Struct;


 
  //步态控制标志位
  char step_back_flag=0;//1表示进的是左前，此时，左侧退到FOOT_X(-150)+0.5步长=0，右侧退到FOOT_X(0)-0.5步长=-150
												//0表示进的是右前，此时，左侧腿退到FOOT_x=-150,右侧腿退到FOOT_x=0

  char gait_mode_turn=LEFT_BACK_LEG;

  char fornt_back_flag=1;   //1表示前进

 int   TIM_t=0;

/*************************************************函数定义区**************************************************/
/**
  * @brief  移动控制任务函数
  * @param  void
  * @retval void
  * @notes  根据遥控器的模式选择不同的方式
  */

int right_front_1=160;
int right_front_2=140;
int right_front_3=140;

double foot_x=0;
 double foot_y=140;

void Move_Control_Task(void *pvParameters)
{
	
	
	IMU_Init();//陀螺仪初始化，开启陀螺仪中断
	vTaskDelay(500);
  Steering_Init();//舵机初始化，头舵机设置为90°
	Foot_Direction_Init();
	
	
	Beep_ON ();
	delay_ms(500);
	Beep_OFF();
	
	while(1)
	{

		
		TIM_t++;
		
		
		if(TIM_t%300<=50)
		{
		
			 Foot_Control(&Left_Front_Leg_Struct,STEP_FORWARD);	
			 Foot_Control(&Right_Back_Leg_Struct,STEP_FORWARD);	
		}
		else  if(TIM_t%300<=150)
		{
		
			 Foot_Control(&Left_Front_Leg_Struct,QUIESECENCE);
			Foot_Control(&Right_Back_Leg_Struct,QUIESECENCE);
		
		}
		else  if (TIM_t%300<=200)
		{
		
			 Foot_Control(&Right_Front_Leg_Struct,STEP_FORWARD);	
			 Foot_Control(&Left_Back_Leg_Struct,STEP_FORWARD);	
		}
		else  
		{
		
			 Foot_Control(&Right_Front_Leg_Struct,QUIESECENCE);
			 Foot_Control(&Left_Back_Leg_Struct,QUIESECENCE);
		
		} 
		
		
		
		
	
			
		 
		
	   vTaskDelay(10);
	}

}

/********************************************滑行***************************************************/
/**
  * @brief  轮式滑行控制函数
  * @param  无                    
  * @retval void
  * @notes  用遥控器控制机械狗前进，转向
  */
/********************************************步行***************************************************/
/**
  * @brief  坐标反解函数
  * @param  Leg_Data：腿部数据结构体指针                    
  * @retval void
  * @notes  使用反解公式将X,Y坐标反解为舵机的输出控制值，
            将解算的值存在腿数据结构体中的成员。
  */
double  seigamar=0;
double  gama=0;
	
void Coordinate_Inverse(Leg_Data_Typedef* Leg_Data)
{
	//角度定义
	
	
	
	
  //坐标反解公式   

	gama    =asin((Leg_Data->Leg_X*Leg_Data->Leg_X+Leg_Data->Leg_Y*Leg_Data->Leg_Y+SMALL_LENGTH_2*SMALL_LENGTH_2-GIG_LENGTH_1*GIG_LENGTH_1)/2/SMALL_LENGTH_2/sqrt(Leg_Data->Leg_X*Leg_Data->Leg_X+Leg_Data->Leg_Y*Leg_Data->Leg_Y))-atan(Leg_Data->Leg_X/Leg_Data->Leg_Y);
	
	
	seigamar=asin((Leg_Data->Leg_X*Leg_Data->Leg_X+Leg_Data->Leg_Y*Leg_Data->Leg_Y+GIG_LENGTH_1*GIG_LENGTH_1-SMALL_LENGTH_2*SMALL_LENGTH_2)/2/GIG_LENGTH_1/sqrt(Leg_Data->Leg_X*Leg_Data->Leg_X+Leg_Data->Leg_Y*Leg_Data->Leg_Y))+atan(Leg_Data->Leg_X/Leg_Data->Leg_Y);
	
	
	
 
  
	
	
	//舵机输出控制值解算
	if(Leg_Data->Leg_Type <=LEFT_BACK_LEG)//左侧腿
	{
		//外侧舵机
		Leg_Data->Steerint_OUT =320-180*(gama+seigamar)/pi;
		//内侧舵机
		Leg_Data->Steerint_IN  =230-180*seigamar/pi;
	}
	else//右侧腿
	{
		//外侧舵机
	   Leg_Data->Steerint_OUT =180*(gama+seigamar)/pi-40;
		//内侧舵机
	   Leg_Data->Steerint_IN =50+180*seigamar/pi;
	}
}

/**
  * @brief  腿部PWM输出函数
  * @param  Leg_Data：腿部数据结构体指针                    
  * @retval void
  * @notes  根据腿的类型将相对应的控制PWM输出到对应的舵机
  */
void Steering_PWM_Out(Leg_Data_Typedef* Leg_Data)
{
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
			Right_Back_OUT(Leg_Data->Steerint_OUT+10 )
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
	//坐标反解
	Coordinate_Inverse(Leg_Data);
	//PWM输出
	Steering_PWM_Out(Leg_Data);

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
void Foot_Control(Leg_Data_Typedef* Leg_Data,char trail )
{
	//
	double fai=0;
 
 //迈步前进
  if(trail==STEP_FORWARD)
	{ 
		 //控时参数转换,每次运行的时候进行转换
			 Leg_Data->step_forward_time +=1*2*pi/STEP_FORWARD_TIME;	 
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
			 Leg_Data->Leg_Y = Leg_Data->FOOT_y+Leg_Data->Y_Add;
		 }
		
	}
	
	
	
	//静退
  else if(trail==QUIESECENCE)
	{
		Leg_Data->Leg_X  -=1*STEP_LENTH/QUIESECENCE_TIME;
		
		if(Leg_Data->Leg_X<FOOT_X) 
		 {
			 Leg_Data->Leg_X=FOOT_X ;
			 
			 Leg_Data->step_forward_time=0;
			 
		 }
		 	
		//设置Y
		  Leg_Data->Leg_Y=Leg_Data->FOOT_y+Leg_Data->Y_Add;
	}


	//坐标反解输出控制
	Coordinate_Inverse(Leg_Data);
	
	Steering_PWM_Out(Leg_Data);

}
	



/**
  * @brief  腿部收缩函数
  * @param  void
  * @retval void
  * @notes  四腿标志位归零，计数参数和轮转参数复位，迈步时间周期归零，足端坐标初始化，足端回位。
  */
void Shrink(void)
{

	 Left_Front_Speed(100)    //W   
	 Left_Back_Speed(100)	    //X
   Right_Front_Speed(180)   //Y
	 Right_Back_Speed(180)		//A
	
	
	//左前
	Left_Front_Leg_Struct.step_forward_time=0;
	
	Left_Front_Leg_Struct.Leg_X =FOOT_X;
	Left_Front_Leg_Struct.Leg_Y =FOOT_Y;
	Left_Front_Leg_Struct.FOOT_x =FOOT_X;
	Left_Front_Leg_Struct.FOOT_y =FOOT_Y;
	Left_Front_Leg_Struct.Y_Add =0;
	Coordinate_Inverse(&Left_Front_Leg_Struct);
	Steering_PWM_Out(&Left_Front_Leg_Struct);
	//左后
	Left_Back_Leg_Struct.step_forward_time=0;
	Left_Back_Leg_Struct.Leg_X =FOOT_X;
	Left_Back_Leg_Struct.Leg_Y =FOOT_Y;
	Left_Back_Leg_Struct.FOOT_x =FOOT_X;
	Left_Back_Leg_Struct.FOOT_y =FOOT_Y;
	Left_Back_Leg_Struct.Y_Add =0;
	Coordinate_Inverse(&Left_Back_Leg_Struct);
	Steering_PWM_Out(&Left_Back_Leg_Struct);
	
	
	
	//右前
  Right_Front_Leg_Struct.step_forward_time=0;
	Right_Front_Leg_Struct.Leg_X =FOOT_X;
	Right_Front_Leg_Struct.Leg_Y =FOOT_Y;
	Right_Front_Leg_Struct.FOOT_x =FOOT_X;
	Right_Front_Leg_Struct.FOOT_y =FOOT_Y;
	Right_Front_Leg_Struct.Y_Add =0;
	Coordinate_Inverse(&Right_Front_Leg_Struct);
	Steering_PWM_Out(&Right_Front_Leg_Struct);
	//右后
	Right_Back_Leg_Struct.step_forward_time=0;
  Right_Back_Leg_Struct.Leg_X =FOOT_X;
	Right_Back_Leg_Struct.Leg_Y =FOOT_Y;
	Right_Back_Leg_Struct.FOOT_x =FOOT_X;
	Right_Back_Leg_Struct.FOOT_y =FOOT_Y;
	Right_Back_Leg_Struct.Y_Add =0;
	Coordinate_Inverse(&Right_Back_Leg_Struct);
	Steering_PWM_Out(&Right_Back_Leg_Struct);
	

	
}

/**
  * @brief  足端位置初始化函数
  * @param  void
  * @retval void
  * @notes  腿类型标定，足端回位，轮电机激活以及速度设置为停止
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

}











