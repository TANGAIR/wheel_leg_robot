#include "main.h"

/*************************************************数据定义区*************************************************/
  //腿数据结构体,
  Leg_Data_Typedef Left_Front_Leg_Struct;
  Leg_Data_Typedef Left_Back_Leg_Struct;
  Leg_Data_Typedef Right_Back_Leg_Struct;
  Leg_Data_Typedef Right_Front_Leg_Struct;


  //步态控制标志位
  char step_back_flag=0;//1表示进的是左前，此时，左侧退到FOOT_X(-150)+0.5步长=0，右侧退到FOOT_X(0)-0.5步长=-150
												//0表示进的是右前，此时，左侧腿退到FOOT_x=-150,右侧腿退到FOOT_x=0

  char gait_mode_turn=LEFT_BACK_LEG;

  char fornt_back_flag=1;   //1表示前进
	
	
	

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
	float RC_Motor_Ra=7;
	
      Wheel_LF.Target_Speed =(DBUS.RC.ch1+DBUS.RC.ch0)*RC_Motor_Ra;
			Wheel_LB.Target_Speed =(DBUS.RC.ch1+DBUS.RC.ch0)*RC_Motor_Ra;
			Wheel_RB.Target_Speed =(DBUS.RC.ch1-DBUS.RC.ch0)*RC_Motor_Ra;
			Wheel_RF.Target_Speed =(DBUS.RC.ch1-DBUS.RC.ch0)*RC_Motor_Ra;
}


/*









*******************************************步行***************************************************/
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
	
	
	
	//X限位防止出错，X工作空间在-150~150
		if(Leg_Data->Leg_X<=-150)Leg_Data->Leg_X=-150;
		if(Leg_Data->Leg_X>=150)Leg_Data->Leg_X=150;
	//Y限位防止出错，Y工作空间在150~250
	   if(Leg_Data->Leg_Y>=250)Leg_Data->Leg_Y=260;
		 if(Leg_Data->Leg_Y<=120)Leg_Data->Leg_Y=120;
	
  //坐标反解公式
	seigamar=asin(sqrt(Leg_Data->Leg_X*Leg_Data->Leg_X+Leg_Data->Leg_Y*Leg_Data->Leg_Y)/2/CLUB_LENGTH)+atan(Leg_Data->Leg_X/Leg_Data->Leg_Y);//长，上，外
  gama    =asin(sqrt(Leg_Data->Leg_X*Leg_Data->Leg_X+Leg_Data->Leg_Y*Leg_Data->Leg_Y)/2/CLUB_LENGTH)-atan(Leg_Data->Leg_X/Leg_Data->Leg_Y);//短，下，内
  
	 //限位防止超出控制
	 if(seigamar>2) seigamar=2;
	 if(seigamar<0) seigamar=0;
	 if(gama>2) seigamar=2;
	 if(gama<0) seigamar=0;


	//串口输出变量赋值，根据腿的类型将角度数据输出给不同的关节电机
	switch(Leg_Data->Leg_Type)
	{
		//左腿
		case LEFT_FRONT_LEG :
		{
			Joint_LF_U.send.send_Position =-seigamar;
			Joint_LF_D.send .send_Position =-gama; 
			break;
		}
	  case LEFT_BACK_LEG :
		{
			Joint_LB_U.send.send_Position =-seigamar;
			Joint_LB_D.send .send_Position =-gama;  
			break;
		}

		//右腿
	 case  RIGHT_BACK_LEG:
		{
			Joint_RB_U.send.send_Position =seigamar;
			Joint_RB_D.send .send_Position =gama;
			break;
		}
		case RIGHT_FRONT_LEG :
		{
			Joint_RF_U.send.send_Position =seigamar;
			Joint_RF_D.send .send_Position =gama;
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



/*



*******************************************静态步态***************************************************/



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
		 	 
		 //轨迹坐标计算，凡是腿前进都是从-150开始
		 Leg_Data->Leg_X =STEP_LENTH/2/pi*(fai-sin(fai))+FOOT_X;
		 Leg_Data->Leg_Y =(STEP_HIGHT/2*(1-cos(fai)))*(-1)+Leg_Data->FOOT_y+Leg_Data->Y_Add;
		 
		 //轨迹衔接,防止计算出错
		 if(fai==2*pi) 
		 {
			 Leg_Data->Leg_Mode =0;    //0表示进入静退阶段
			 Leg_Data->Leg_X =(FOOT_X+STEP_LENTH);
			 Leg_Data->Leg_Y = Leg_Data->FOOT_y+Leg_Data->Y_Add;
		 }
		 if(fai==0)    
		 {
			 
			 Leg_Data->Leg_X = FOOT_X;
			 Leg_Data->Leg_Y = Leg_Data->FOOT_y+Leg_Data->Y_Add;
		 }
	}
	//静退
  else if(trail==QUIESECENCE)
	{
		 //控时参数转换//轨迹坐标计算
	   if(direction==DIRECTION_FORWARD)
			 Leg_Data->Leg_X  -=1*STEP_LENTH/QUIESECENCE_TIME/2;//只退半步
	   else if(direction==DIRECTION_BACK)
			 Leg_Data->Leg_X  +=1*STEP_LENTH/QUIESECENCE_TIME/2;
		
		 if(step_back_flag==1)//左侧腿在前
		 {
			   //对于左侧腿FOOT_x=-150,从150到0
			   if(Leg_Data->Leg_Type <33)
				 { //150->0
					 if(Leg_Data->Leg_X > (Leg_Data->FOOT_x+STEP_LENTH))Leg_Data->Leg_X = (Leg_Data->FOOT_x+STEP_LENTH);
				   if(Leg_Data->Leg_X < (Leg_Data->FOOT_x+STEP_LENTH/2))Leg_Data->Leg_X = Leg_Data->FOOT_x+STEP_LENTH/2;
				 
						//轨迹衔接
						if(Leg_Data->Leg_X == (Leg_Data->FOOT_x+STEP_LENTH))
							 Leg_Data->step_forward_time=2*pi;
						if(Leg_Data->Leg_X <= Leg_Data->FOOT_x+STEP_LENTH/2)
						{
							 Leg_Data->step_forward_time=0;
							 Leg_Data->Leg_Mode =1;   //1表示前进
						} 
				 }
				  //对于右侧腿FOOT_x=0，从0到-150
			   else  if(Leg_Data->Leg_Type >33)
				 { //0->-150
					 if(Leg_Data->Leg_X > (Leg_Data->FOOT_x))Leg_Data->Leg_X = Leg_Data->FOOT_x;
				   if(Leg_Data->Leg_X < (Leg_Data->FOOT_x-STEP_LENTH/2))Leg_Data->Leg_X = Leg_Data->FOOT_x-STEP_LENTH/2;
				 
						//轨迹衔接 0->-150
						if(Leg_Data->Leg_X == (Leg_Data->FOOT_x))
							 Leg_Data->step_forward_time=2*pi;
						if(Leg_Data->Leg_X <= Leg_Data->FOOT_x-STEP_LENTH/2)
						{
							 Leg_Data->step_forward_time=0;
							 Leg_Data->Leg_Mode =1;   //1表示前进
						} 
				 }
		 }
		 else //右侧腿在前
		 {
			    //左：0->-150,右侧150->0
				   if(Leg_Data->Leg_X > (Leg_Data->FOOT_x+STEP_LENTH/2))Leg_Data->Leg_X = (Leg_Data->FOOT_x+STEP_LENTH/2);
				   if(Leg_Data->Leg_X < (Leg_Data->FOOT_x))Leg_Data->Leg_X = Leg_Data->FOOT_x;
				 
						//轨迹衔接
						if(Leg_Data->Leg_X == (Leg_Data->FOOT_x+STEP_LENTH/2))
							 Leg_Data->step_forward_time=2*pi;
						if(Leg_Data->Leg_X <= Leg_Data->FOOT_x)
						{
							 Leg_Data->step_forward_time=0;
							 Leg_Data->Leg_Mode =1;   //1表示前进
						} 
		 }
		 	
		//设置Y
		  Leg_Data->Leg_Y=Leg_Data->FOOT_y+Leg_Data->Y_Add;
	}


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
	 

	 
   if(Leg_Data->Leg_Mode ==1)
	 {

	   Foot_Control(Leg_Data,STEP_FORWARD,DIRECTION_FORWARD);

	 }
	 else 
	 {    
		 Foot_Control(Leg_Data,QUIESECENCE,DIRECTION_FORWARD); 
	 }
 
 }


/**
  * @brief  四足步态控制函数
  * @param  gait：步态，前进，后退，收缩
  * @retval void
  * @notes  采用单侧交替步态，每次单侧腿迈出一大步300，当一侧的两条腿走完之后，退一小步150，然后迈另外一侧的腿，
            每次前腿迈步的时候必须后退150
  */
 int   TIM_t=0;
 char leg_turn_mix_flag=0x1e;
void Gait_Control(char gait)
{
	
	//如果步态为前进或者后退
	if(gait!=SHRINK)
	{
		
		//使用控制位置进行限制
		 leg_turn_mix_flag=(Left_Back_Leg_Struct.Leg_Mode <<4)|(Left_Front_Leg_Struct.Leg_Mode <<3)|(Right_Back_Leg_Struct.Leg_Mode <<2)|(Right_Front_Leg_Struct.Leg_Mode<<1 )|(step_back_flag);
		
		 switch(leg_turn_mix_flag)
		 {
			 //迈左后腿
			 case  0x1e://11110
			 {
				 gait_mode_turn= LEFT_BACK_LEG;
				 break;
			 }
			 //迈左前腿
			  case  0x0e://01110
			 {
				 gait_mode_turn= LEFT_FRONT_LEG;
				 break;
			 }
			 //左侧在前时，退
			 case  0x06://00110
			 {
				 gait_mode_turn= LEFT_FRONT_BACKFORWARD;
				 step_back_flag=1;
				 break;
			 }
		 
			 //右后腿迈步
			 case 0x1f: //11111
			 {
				 gait_mode_turn= RIGHT_BACK_LEG;

				  break;
			 }
				 //右前腿迈步
			  case 0x1b://11011
			 {
				 gait_mode_turn= RIGHT_FRONT_LEG;
				  break;
			 }
				 //右侧靠前时，退
			  case 0x19://11001
			 {
				 gait_mode_turn= RIGHT_FRONT_BACKFORWARD;
				 step_back_flag=0;
				  break;
			 }
			 
				default:
					break;
		 }
		

		
		switch(gait_mode_turn)
		{
			
			//左后腿迈步 11
			case LEFT_BACK_LEG :
			{	
				//左后腿前进 300
				Foot_Control(&Left_Back_Leg_Struct,STEP_FORWARD,gait);	
			}	
			break;
			//左前腿迈步 22
			case LEFT_FRONT_LEG :
			{						
				//左前腿前进300
				Foot_Control(&Left_Front_Leg_Struct,STEP_FORWARD,gait);
			}
			break;
			
			
			//左侧腿靠前时整体后退  33
			case LEFT_FRONT_BACKFORWARD :
			{					
				 
				//左侧腿退到0，右侧腿退到-150	
				Foot_Control(&Left_Front_Leg_Struct,QUIESECENCE,gait);
				Foot_Control(&Left_Back_Leg_Struct,QUIESECENCE,gait);
			  Foot_Control(&Right_Front_Leg_Struct,QUIESECENCE,gait);
				Foot_Control(&Right_Back_Leg_Struct,QUIESECENCE,gait);		
			}
			break;
			
			
			//右后腿前进300   ,44
			case RIGHT_BACK_LEG :
			{
				//右后腿前进 300
				Foot_Control(&Right_Back_Leg_Struct,STEP_FORWARD,gait);
			}	
			break;
			//右前腿前进300  55
			case RIGHT_FRONT_LEG :
			{
				//右前腿前进300
				Foot_Control(&Right_Front_Leg_Struct,STEP_FORWARD,gait);
					
			}
			break;
			
			//右侧腿靠前时，整体后退150  66
			case RIGHT_FRONT_BACKFORWARD :
			{	
				
				//左侧腿退到-150，右侧腿退到0
				Foot_Control(&Right_Front_Leg_Struct,QUIESECENCE,gait);
				Foot_Control(&Left_Front_Leg_Struct,QUIESECENCE,gait);
				Foot_Control(&Right_Back_Leg_Struct,QUIESECENCE,gait);
				Foot_Control(&Left_Back_Leg_Struct,QUIESECENCE,gait);
				 				
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







 /*


*******************************************动态步态**************************************************




*/
/**
  * @brief  动态交替足端轨迹控制函数
  * @param  Leg_Data：腿部数据结构体指针
  * @retval void
  * @notes  使用抬腿静推相结合的方式，实现灵活的动态步态，使用步长来控制单腿迈步长度以及前进或者后退，
	          初始步长为10，只有每个迈步周期结束时才会更新步长，使用遥控器控制步长以控制前进后退
						两条腿，一个主一个从，两腿交替同步运行
  */
 void Foot_Dynamic_Control(Leg_Data_Typedef* Leg_Data,Leg_Data_Typedef* Leg_Data_Follow )
{
	 double fai=0;
	 double fai_follow=0;
	static float  step_lenght=10;
	static float  step_lenght_follow=10;
	
	if(Leg_Data->step_lenght >=150) Leg_Data->step_lenght=150;
	if(Leg_Data->step_lenght <=-150) Leg_Data->step_lenght=-150;
	
	if(Leg_Data_Follow->step_lenght >=150) Leg_Data_Follow->step_lenght=150;
	if(Leg_Data_Follow->step_lenght <=-150) Leg_Data_Follow->step_lenght=-150;
	
	
	//抬腿前进部分
	 if( Leg_Data->Leg_Mode==1)
	 {
		  //主腿
		 {
		   //控时参数转换,每次运行的时候进行转换
			 Leg_Data->step_forward_time +=1*2*pi/STEP_FORWARD_TIME;// (t-last_t)		
			 //数据类型转换		 
			 fai=Leg_Data->step_forward_time;
		
			 //限位
			 if(fai>2*pi)fai=2*pi;
			 if(fai<0)fai=0;
				 
			 //轨迹坐标计算
			 Leg_Data->Leg_X =step_lenght/2/pi*(fai-sin(fai))+Leg_Data->FOOT_x;
			 Leg_Data->Leg_Y =(STEP_HIGHT/2*(1-cos(fai)))*(-1)+Leg_Data->FOOT_y+Leg_Data->Y_Add;
			 
			 //轨迹衔接,防止计算出错
			 if(fai==2*pi) 
			 {
				 Leg_Data->Leg_Mode =0;    //0表示进入静退阶段
				 Leg_Data->Leg_X =(Leg_Data->FOOT_x +step_lenght);
				 Leg_Data->Leg_Y = Leg_Data->FOOT_y+Leg_Data->Y_Add;
				 Leg_Data_Follow->step_forward_time=0;
				 Leg_Data_Follow->Leg_Mode =1;   //1表示前进
				 //更新步长
				 step_lenght_follow=Leg_Data_Follow->step_lenght ;
			 }
		 }

		 //从腿，时间以主腿为准
		 {
			  //直线轨迹坐标计算
		    Leg_Data_Follow->Leg_X  -=1*step_lenght_follow/STEP_FORWARD_TIME;
				 //当步长为正时，限制
				if(step_lenght_follow>=0)
				{
					if(Leg_Data_Follow->Leg_X > (Leg_Data_Follow->FOOT_x+step_lenght_follow))Leg_Data_Follow->Leg_X = (Leg_Data_Follow->FOOT_x+step_lenght_follow);
					//此时一个循环周期结束，可以更新步长数据
					if(Leg_Data_Follow->Leg_X < Leg_Data_Follow->FOOT_x)
					{
						Leg_Data_Follow->Leg_X = Leg_Data_Follow->FOOT_x;
					}	
				}
				//当步长为负时，限制
				else	if(step_lenght_follow<0)
				{
					if(Leg_Data_Follow->Leg_X < (Leg_Data_Follow->FOOT_x+step_lenght_follow))Leg_Data_Follow->Leg_X = (Leg_Data_Follow->FOOT_x+step_lenght_follow);
					if(Leg_Data_Follow->Leg_X > Leg_Data_Follow->FOOT_x)
					{
						Leg_Data_Follow->Leg_X = Leg_Data_Follow->FOOT_x;
					}
		    }
				//设置Y
				Leg_Data_Follow->Leg_Y=Leg_Data_Follow->FOOT_y+Leg_Data_Follow->Y_Add;
		 }  
	 }
    	 
	 //足端落地静摩擦部分	 
	 else if( Leg_Data->Leg_Mode==0)
	 {
		 {
			 //直线轨迹坐标计算
			 Leg_Data->Leg_X  -=1*step_lenght/QUIESECENCE_TIME;
			
			 //当步长为正时，限制
			if(step_lenght>=0)
			{
				if(Leg_Data->Leg_X > (Leg_Data->FOOT_x+step_lenght))Leg_Data->Leg_X = (Leg_Data->FOOT_x+step_lenght);
				//此时一个循环周期结束，可以更新步长数据
				if(Leg_Data->Leg_X < Leg_Data->FOOT_x)
				{
					Leg_Data->Leg_X = Leg_Data->FOOT_x;
					Leg_Data->step_forward_time=0;
					Leg_Data->Leg_Mode =1;   //1表示前进
					//更新步长
					step_lenght=Leg_Data->step_lenght ;
				}	
			}
			//当步长为负时，限制
			else	if(step_lenght<0)
			{
				if(Leg_Data->Leg_X < (Leg_Data->FOOT_x+step_lenght))Leg_Data->Leg_X = (Leg_Data->FOOT_x+step_lenght);
				if(Leg_Data->Leg_X > Leg_Data->FOOT_x)
				{
					Leg_Data->Leg_X = Leg_Data->FOOT_x;
					Leg_Data->step_forward_time=0;
					Leg_Data->Leg_Mode =1;   //1表示前进
					//更新步长
					step_lenght=Leg_Data->step_lenght ;
				}
			}	
			//设置Y
			Leg_Data->Leg_Y=Leg_Data->FOOT_y+Leg_Data->Y_Add;
		 }
		 
		//主腿后退，从腿前进
		{
		 //控时参数转换,每次运行的时候进行转换
		 Leg_Data_Follow->step_forward_time +=1*2*pi/QUIESECENCE_TIME;// (t-last_t)		
     //数据类型转换		 
		 fai_follow=Leg_Data_Follow->step_forward_time;
	
		 //限位
		 if(fai_follow>2*pi)fai_follow=2*pi;
		 if(fai_follow<0)fai_follow=0;
		 	 
		 //轨迹坐标计算
		 Leg_Data_Follow->Leg_X =step_lenght_follow/2/pi*(fai_follow-sin(fai_follow))+Leg_Data_Follow->FOOT_x ;
		 Leg_Data_Follow->Leg_Y =(STEP_HIGHT/2*(1-cos(fai_follow)))*(-1)+Leg_Data_Follow->FOOT_y+Leg_Data_Follow->Y_Add;
		 
		 //轨迹衔接,防止计算出错
		 if(fai_follow==2*pi) 
		 {
			 Leg_Data_Follow->Leg_Mode =0;    //0表示进入静退阶段
			 Leg_Data_Follow->Leg_X =(Leg_Data_Follow->FOOT_x +step_lenght);
			 Leg_Data_Follow->Leg_Y = Leg_Data_Follow->FOOT_y+Leg_Data_Follow->Y_Add;
		 }
		}
	 }
	//坐标反解输出控制
	Coordinate_Inverse(Leg_Data);
	Coordinate_Inverse(Leg_Data_Follow);
}



/**
  * @brief  动态步态控制函数
  * @param  
  * @retval void
  * @notes  使用典型的机器狗交叉动态步态
  */
/********************************************轮腿切换与初始化***************************************************/
/**
  * @brief  腿部收缩函数
  * @param  void
  * @retval void
  * @notes  四腿标志位归零，计数参数和轮转参数复位，迈步时间周期归零，足端坐标初始化，足端回位。
  */
void Shrink(void)
{

	 step_back_flag=0;
	 gait_mode_turn=LEFT_BACK_LEG;
	
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
	
	switch(Leg_Struct_Point->Leg_Type)
	{
		    //左前
		case LEFT_FRONT_LEG:
		{
			 Leg_Struct_Point->FOOT_x = FOOT_X;
			 Leg_Struct_Point->FOOT_y = FOOT_Y+15;
				break;
		}
				//左后
		case LEFT_BACK_LEG:
		{
			 Leg_Struct_Point->FOOT_x = FOOT_X;
			 Leg_Struct_Point->FOOT_y = FOOT_Y-5;
				break;
		}
				//右后
		case RIGHT_BACK_LEG:
		{
			Leg_Struct_Point->FOOT_x = FOOT_X;
			 Leg_Struct_Point->FOOT_y = FOOT_Y;
				break;
		}
	
				//右前
		case RIGHT_FRONT_LEG:
		{
			Leg_Struct_Point->FOOT_x = FOOT_X;
			 Leg_Struct_Point->FOOT_y = FOOT_Y+20;
				break;
		}
		
		
		default:
		break;
	
	
	}
		Leg_Struct_Point->step_lenght=STEP_LENTH;
	  Leg_Struct_Point->Leg_Mode =1;
		Leg_Struct_Point->Leg_X =  Leg_Struct_Point->FOOT_x;
		Leg_Struct_Point->Leg_Y =  Leg_Struct_Point->FOOT_y+Leg_Struct_Point->Y_Add;

	Coordinate_Inverse(Leg_Struct_Point);

}




