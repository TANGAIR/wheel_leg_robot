#include "main.h"

/*************************************************���ݶ�����*************************************************/
//�����ݽṹ��
Leg_Data_Typedef Left_Front_Leg_Struct;
Leg_Data_Typedef Left_Back_Leg_Struct;
Leg_Data_Typedef Right_Front_Leg_Struct;
Leg_Data_Typedef Right_Back_Leg_Struct;

//��̬���Ʊ�־λ
 char Left_Front_Start_Flag=0;
 char Left_Back_Start_Flag=0;
 char Right_Back_Start_Flag=0;
 char Right_Front_Start_Flag=0;	
	
 char count_num=RIGHT_FRONT_LEG;
 char wheel_turn=LEFT_FRONT_LEG;



/*************************************************����������**************************************************/
/**
  * @brief  �ƶ�����������
  * @param  void
  * @retval void
  * @notes  ����ң������ģʽѡ��ͬ�ķ�ʽ
  */

int right_front_1=160;
int right_front_2=140;
int right_front_3=140;

double foot_x=0;
 double foot_y=140;

void Move_Control_Task(void *pvParameters)
{
	
	
	IMU_Init();//�����ǳ�ʼ���������������ж�
	vTaskDelay(500);
  Steering_Init();//�����ʼ����ͷ�������Ϊ90��
	Foot_Direction_Init();
	
	
	Beep_ON ();
	delay_ms(500);
	Beep_OFF();
	
				 
	 

	Left_Front_IN(140)          //��ǰ��-S-PA0-TIM2C1 Left_Front_OUT(x) 
		 Left_Front_OUT(140)
		
		
		 Right_Front_IN(140)   //sigema
		 Right_Front_OUT(140)  //gama
		
		
		
		
		 Left_Back_In(140)
		 Left_Back_OUT(140) 
	while(1)
	{

		
		
		 
		
		
		 
 
		 Leg_Diretion_Control(&Right_Back_Leg_Struct,foot_x,foot_y);
		
			//Right_Back_In(right_front_2)
			//Right_Back_OUT(right_front_3 )
		
	   vTaskDelay(100);
	}

}

/********************************************����***************************************************/
/**
  * @brief  ��ʽ���п��ƺ���
  * @param  ��                    
  * @retval void
  * @notes  ��ң�������ƻ�е��ǰ����ת��
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
		
		Leg_Diretion_Control(&Right_Front_Leg_Struct,FOOT_X,(FOOT_Y+20));
			
		Leg_Diretion_Control(&Right_Back_Leg_Struct,(FOOT_X+15),(FOOT_Y-STEP_HIGHT));	
			
		turn_flag=1;
	
	}
  else if(DBUS.RC.ch0<0)
	{
	  speed=abs(DBUS.RC.ch0) *1/3+1000;
	  Left_Front_Speed(WHEEL_STOP-70)
	  Left_Back_Speed(WHEEL_STOP)
		Right_Front_Speed(speed)
	  Right_Back_Speed(speed)
	
		Leg_Diretion_Control(&Left_Front_Leg_Struct,FOOT_X,(FOOT_Y+20));
			
		Leg_Diretion_Control(&Left_Back_Leg_Struct,(FOOT_X+15),(FOOT_Y-STEP_HIGHT));	
			
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


/********************************************����***************************************************/
/**
  * @brief  ���귴�⺯��
  * @param  Leg_Data���Ȳ����ݽṹ��ָ��                    
  * @retval void
  * @notes  ʹ�÷��⹫ʽ��X,Y���귴��Ϊ������������ֵ��
            �������ֵ���������ݽṹ���еĳ�Ա��
  */
void Coordinate_Inverse(Leg_Data_Typedef* Leg_Data)
{
	//�Ƕȶ���
	double  seigamar=0;
	double  gama=0;
	
	
	
	
  //���귴�⹫ʽ   

	gama    =asin((Leg_Data->Leg_X*Leg_Data->Leg_X+Leg_Data->Leg_Y*Leg_Data->Leg_Y+SMALL_LENGTH_2*SMALL_LENGTH_2-GIG_LENGTH_1*GIG_LENGTH_1)/2/SMALL_LENGTH_2/sqrt(Leg_Data->Leg_X*Leg_Data->Leg_X+Leg_Data->Leg_Y*Leg_Data->Leg_Y))-atan(Leg_Data->Leg_X/Leg_Data->Leg_Y);
	
	
	seigamar=asin((Leg_Data->Leg_X*Leg_Data->Leg_X+Leg_Data->Leg_Y*Leg_Data->Leg_Y+GIG_LENGTH_1*GIG_LENGTH_1-SMALL_LENGTH_2*SMALL_LENGTH_2)/2/GIG_LENGTH_1/sqrt(Leg_Data->Leg_X*Leg_Data->Leg_X+Leg_Data->Leg_Y*Leg_Data->Leg_Y))+atan(Leg_Data->Leg_X/Leg_Data->Leg_Y);
	
	
	
 
  
	
	
	//����������ֵ����
	if(Leg_Data->Leg_Type <=LEFT_BACK_LEG)//�����
	{
		//�����
		Leg_Data->Steerint_OUT =230-180*seigamar/pi;
		//�ڲ���
		Leg_Data->Steerint_IN  =230-180*gama/pi;
	}
	else//�Ҳ���
	{
		//�����
	   Leg_Data->Steerint_OUT =50+180*seigamar/pi;
		//�ڲ���
	   Leg_Data->Steerint_IN =50+180*gama/pi;
	}
}

/**
  * @brief  �Ȳ�PWM�������
  * @param  Leg_Data���Ȳ����ݽṹ��ָ��                    
  * @retval void
  * @notes  �����ȵ����ͽ����Ӧ�Ŀ���PWM�������Ӧ�Ķ��
  */
void Steering_PWM_Out(Leg_Data_Typedef* Leg_Data)
{
//����
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
  * @brief  �Ȳ�������ƺ���
  * @param  void
  * @retval void
  * @notes  ʹ������������λ��,X����ǰΪ����Y������Ϊ��
  */
void Leg_Diretion_Control(Leg_Data_Typedef* Leg_Data,double foot_x,double foot_y)
{
	switch (Leg_Data->Leg_Type)
	{		
		 //��ǰ������
			case LEFT_FRONT_LEG :
			{					
					Left_Front_Leg_Struct.Leg_X =foot_x;
					Left_Front_Leg_Struct.Leg_Y =foot_y;
					
			}
			break;
			//���������
			case LEFT_BACK_LEG :
			{	
				Left_Back_Leg_Struct.Leg_X =foot_x;
				Left_Back_Leg_Struct.Leg_Y =foot_y;
				
			}	
			break;
			//�Һ���
			case RIGHT_BACK_LEG :
			{			
				Right_Back_Leg_Struct.Leg_X =foot_x;
				Right_Back_Leg_Struct.Leg_Y =foot_y;
				
			}	
			break;
			//��ǰ��
			case RIGHT_FRONT_LEG :
			{
				Right_Front_Leg_Struct.Leg_X =foot_x;
				Right_Front_Leg_Struct.Leg_Y =foot_y;
				
			}
			break;
			
			default :
			break;	
	}
	//���귴��
	Coordinate_Inverse(Leg_Data);
	//PWM���
	Steering_PWM_Out(Leg_Data);

}

/**
  * @brief  ������˹켣���ƺ���
  * @param  Leg_Data���Ȳ����ݽṹ��ָ��
            trail���켣������ѡ���������ߺ�ֱ��
            direction���������ƶ�����ǰ���
  * @retval void
  * @notes  �����ڽ���ת��������ʱ����Ϊ������˹켣�Ĳ�������һ���켣����˵�ʱ��λ��ͬʱ������һ���켣������
            Ȼ����п������ݽ��㣬�ٸ����ȵ����ͽ��п���
  */
void Foot_Control(Leg_Data_Typedef* Leg_Data,char trail,char direction )
{
	//
	double fai=0;
	
	
	//����ǰ��
	if(trail==STEP_FORWARD)
	{ 
		 //��ʱ����ת��
	   if(direction==DIRECTION_FORWARD) 
			 Leg_Data->step_forward_time +=1*2*pi/STEP_FORWARD_TIME;// (t-last_t)		
		 else if(direction==DIRECTION_BACK)
			 Leg_Data->step_forward_time -=1*2*pi/STEP_FORWARD_TIME; //(t-last_t)
		 
     //��������ת��		 
		 fai=Leg_Data->step_forward_time;
	
		 //��λ
		 if(fai>2*pi)fai=2*pi;
		 if(fai<0)fai=0;
		 	 
		 //�켣�������
		 Leg_Data->Leg_X =STEP_LENTH/2/pi*(fai-sin(fai));
		 Leg_Data->Leg_Y =(STEP_HIGHT/2*(1-cos(fai)))*(-1)+FOOT_Y;
		 
		 //�켣�ν�,��ֹ�������
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
	
	
	
	//����
  else if(trail==QUIESECENCE)
	{
		 //��ʱ����ת��//�켣�������
	   if(direction==DIRECTION_FORWARD)
			 Leg_Data->Leg_X  -=1*STEP_LENTH/QUIESECENCE_TIME;
	   else if(direction==DIRECTION_BACK)
			 Leg_Data->Leg_X  +=1*STEP_LENTH/QUIESECENCE_TIME;
		 
		 if(Leg_Data->Leg_X > (FOOT_X+STEP_LENTH))Leg_Data->Leg_X = (FOOT_X+STEP_LENTH);
		 if(Leg_Data->Leg_X < (FOOT_X))Leg_Data->Leg_X = FOOT_X;

		 Leg_Data->Leg_Y=FOOT_Y;
		 
		//�켣�ν�
		if(Leg_Data->Leg_X == (FOOT_X+STEP_LENTH))
		  Leg_Data->step_forward_time=2*pi;
	  if(Leg_Data->Leg_X == FOOT_X)
			Leg_Data->step_forward_time=0;
	
	}
	
	
	//�ȶ�����
	 Leg_Data->Leg_Y+= Leg_Data->Y_Add ;
	
	
	
	//���귴��
	Coordinate_Inverse(Leg_Data);
	
	
	//PWM���
	Steering_PWM_Out(Leg_Data);
	
}

/**
  * @brief  ���㲽̬���ƺ���
  * @param  gait����̬��ǰ�������ˣ�����
  * @retval void
  * @notes  ������ѭ����̬��̬����������������ʱ�������������һ��������ʱ�������������ջ�
            ʹ����ת�������������ȵ����ͣ�ʹ�üƴβ������Ƶ���һ��������ʱ�����������ջ�
            ����ʱҲ����Ӧ
  */
int TIM_t=0;
void Gait_Control(char gait)
{
	
	//�����̬Ϊǰ�����ߺ���
	if(gait!=SHRINK)
	{
		//��ʱ��������
	  TIM_t++;
		//�������Ȱ�������
	  if(TIM_t%(STEP_FORWARD_TIME+100)==0)
		{
			//�������ȶ�����ʱ����ձ�־λ��������ǰ�ȱ�־���ü����������ģ�
			if((Left_Front_Start_Flag==1)&&(Left_Back_Start_Flag==1)&&(Right_Back_Start_Flag==1)&&(Right_Front_Start_Flag==1))
			{
				Left_Front_Start_Flag=0;
	      Left_Back_Start_Flag=0;
	      Right_Back_Start_Flag=0;
        Right_Front_Start_Flag=0;	
				//���õ�ǰ�������ȵı�־Ϊ1
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
				//����������
			  count_num-=11;
			  //ѭ���ν�
				if(count_num<11)count_num=44;
			}
			//������ת����
		  wheel_turn+=11;	
		  //ѭ���ν�
		  if(wheel_turn>44)wheel_turn=11;
		}
	  
		
		switch(wheel_turn)
		{
			//��ǰ������
			case LEFT_FRONT_LEG :
			{					
				//���±�־λ
				Left_Front_Start_Flag=1;
				
		
				
				//��ǰ��ǰ��
				Foot_Control(&Left_Front_Leg_Struct,STEP_FORWARD,gait);
			  //�������Ϊ��ǰ�ȣ��������Ⱥ���
				if(count_num==LEFT_FRONT_LEG)
				{
					Foot_Control(&Left_Back_Leg_Struct,QUIESECENCE,gait);
				  Foot_Control(&Right_Front_Leg_Struct,QUIESECENCE,gait);
				  Foot_Control(&Right_Back_Leg_Struct,QUIESECENCE,gait);
				}		
			}
			break;
			//���������
			case LEFT_BACK_LEG :
			{
				//��������ȿ�ʼ��־
				Left_Back_Start_Flag=1;
			
				
				//�����ǰ��
				Foot_Control(&Left_Back_Leg_Struct,STEP_FORWARD,gait);
				//�������Ϊ��ǰ�ȣ��������Ⱥ���
				if(count_num==LEFT_BACK_LEG)
				{
					Foot_Control(&Left_Front_Leg_Struct,QUIESECENCE,gait);
				  Foot_Control(&Right_Front_Leg_Struct,QUIESECENCE,gait);
				  Foot_Control(&Right_Back_Leg_Struct,QUIESECENCE,gait);
				}		
			}	
			break;
			//�Һ���
			case RIGHT_BACK_LEG :
			{
			  //�����Һ��ȱ�־
				Right_Back_Start_Flag=1;
			
				
				//�Һ���ǰ��
				Foot_Control(&Right_Back_Leg_Struct,STEP_FORWARD,gait);
				//�������Ϊ��ǰ�ȣ��������Ⱥ���
				if(count_num==RIGHT_BACK_LEG)
				{
					Foot_Control(&Left_Front_Leg_Struct,QUIESECENCE,gait);
				  Foot_Control(&Right_Front_Leg_Struct,QUIESECENCE,gait);
				  Foot_Control(&Left_Back_Leg_Struct,QUIESECENCE,gait);
				}		
			}	
			break;
			//��ǰ��
			case RIGHT_FRONT_LEG :
			{
				//������ǰ�ȱ�־
				Right_Front_Start_Flag=1;
				
			
				
				//��ǰ��ǰ��
				Foot_Control(&Right_Front_Leg_Struct,STEP_FORWARD,gait);
				//�������Ϊ��ǰ�ȣ��������Ⱥ���
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
	//����̬Ϊ����	
  else if(gait==SHRINK)
	{
		//��������ʼ����
	  Shrink();
	}
}


/**
  * @brief  �Ȳ���������
  * @param  void
  * @retval void
  * @notes  ���ȱ�־λ���㣬������������ת������λ������ʱ�����ڹ��㣬��������ʼ������˻�λ��
  */
void Shrink(void)
{

	 Left_Front_Start_Flag=0;
	 Left_Back_Start_Flag=0;
	 Right_Back_Start_Flag=0;
	 Right_Front_Start_Flag=0;	
		
	 count_num=RIGHT_FRONT_LEG;
	 wheel_turn=LEFT_FRONT_LEG;
	
	//��ǰ
	Left_Front_Leg_Struct.step_forward_time=0;
	Left_Front_Leg_Struct.Leg_X =FOOT_X;
	Left_Front_Leg_Struct.Leg_Y =FOOT_Y;
	Left_Front_Leg_Struct.Y_Add =0;
	Coordinate_Inverse(&Left_Front_Leg_Struct);
	Steering_PWM_Out(&Left_Front_Leg_Struct);
	//��ǰ
  Right_Front_Leg_Struct.step_forward_time=0;
	Right_Front_Leg_Struct.Leg_X =FOOT_X;
	Right_Front_Leg_Struct.Leg_Y =FOOT_Y;
	Right_Front_Leg_Struct.Y_Add =0;
	Coordinate_Inverse(&Right_Front_Leg_Struct);
	Steering_PWM_Out(&Right_Front_Leg_Struct);
	//�Һ�
	Right_Back_Leg_Struct.step_forward_time=0;
  Right_Back_Leg_Struct.Leg_X =FOOT_X;
	Right_Back_Leg_Struct.Leg_Y =FOOT_Y;
	Right_Back_Leg_Struct.Y_Add =0;
	Coordinate_Inverse(&Right_Back_Leg_Struct);
	Steering_PWM_Out(&Right_Back_Leg_Struct);
	//���
	Left_Back_Leg_Struct.step_forward_time=0;
	Left_Back_Leg_Struct.Leg_X =FOOT_X;
	Left_Back_Leg_Struct.Leg_Y =FOOT_Y;
	Left_Back_Leg_Struct.Y_Add =0;
	Coordinate_Inverse(&Left_Back_Leg_Struct);
	Steering_PWM_Out(&Left_Back_Leg_Struct);

	
}

/**
  * @brief  ���λ�ó�ʼ������
  * @param  void
  * @retval void
  * @notes  �����ͱ궨����˻�λ���ֵ�������Լ��ٶ�����Ϊֹͣ
  */
void Foot_Direction_Init(void)
{
	//�Ȳ����ͱ궨
	Left_Front_Leg_Struct.Leg_Type  = LEFT_FRONT_LEG;
	Left_Back_Leg_Struct.Leg_Type   = LEFT_BACK_LEG;
	Right_Front_Leg_Struct.Leg_Type = RIGHT_FRONT_LEG;
  Right_Back_Leg_Struct.Leg_Type  = RIGHT_BACK_LEG;
	
  //�Ȳ���ԭ��
	//Shrink();
	

	
//	vTaskDelay(1500);	
	
 //�ĸ����������Ķ��������ֱ��
 	 Left_Front_Speed(100)       //��ǰ��-W-PI5-TIM8C1
	 Right_Front_Speed(160)
	 Left_Back_Speed(100)	
	 Right_Back_Speed(180)			
	

}











