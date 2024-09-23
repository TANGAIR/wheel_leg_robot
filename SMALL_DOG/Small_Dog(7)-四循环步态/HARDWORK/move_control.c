#include "main.h"

/*************************************************���ݶ�����*************************************************/
//�����ݽṹ��
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

//��ʱ����
int TIM_t=0;


/*************************************************����������**************************************************/
/**
  * @brief  �ƶ�����������
  * @param  void
  * @retval void
  * @notes  ����ң������ģʽѡ��ͬ�ķ�ʽ
  */
void Move_Control_Task(void *pvParameters)
{
	
	//�����ǳ�ʼ���������������ж�
	IMU_Init();
	vTaskDelay(500);
	//�����ʼ����ͷ�������Ϊ90��
  Steering_Init();
	//���λ�ó�ʼ����վ��
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
	
	
	//�Ȳ��������,�Һ���
	if(Leg_Data->Leg_Type ==RIGHT_BACK_LEG)
	{
	  Leg_Data->Leg_Y+=30;
	}
	
	
  //���귴�⹫ʽ
	seigamar=asin(sqrt(Leg_Data->Leg_X*Leg_Data->Leg_X+Leg_Data->Leg_Y*Leg_Data->Leg_Y)/2/CLUB_LENGTH)+atan(Leg_Data->Leg_X/Leg_Data->Leg_Y);
  gama    =asin(sqrt(Leg_Data->Leg_X*Leg_Data->Leg_X+Leg_Data->Leg_Y*Leg_Data->Leg_Y)/2/CLUB_LENGTH)-atan(Leg_Data->Leg_X/Leg_Data->Leg_Y);
  
	
	
	//����������ֵ����
	if(Leg_Data->Leg_Type <=LEFT_BACK_LEG)//�����
	{
		Leg_Data->Steerint_OUT =100*seigamar/pi+100;
		Leg_Data->Steerint_IN  =100*gama/pi+100;
	}
	else//�Ҳ���
	{
	   Leg_Data->Steerint_OUT =200-100*seigamar/pi;
	   Leg_Data->Steerint_IN =200-100*gama/pi;
	}
}

/**
  * @brief  �Ȳ�������ƺ���
  * @param  void
  * @retval void
  * @notes  ʹ������������λ��
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
					Coordinate_Inverse(&Left_Front_Leg_Struct);
					Left_Front_IN(Left_Front_Leg_Struct.Steerint_IN )
					Left_Front_OUT(Left_Front_Leg_Struct.Steerint_OUT )
			}
			break;
			//���������
			case LEFT_BACK_LEG :
			{	
				Left_Back_Leg_Struct.Leg_X =foot_x;
				Left_Back_Leg_Struct.Leg_Y =foot_y;
				Coordinate_Inverse(&Left_Back_Leg_Struct);
				Left_Back_In(Left_Back_Leg_Struct.Steerint_IN )
				Left_Back_OUT(Left_Back_Leg_Struct.Steerint_OUT )
			}	
			break;
			//�Һ���
			case RIGHT_BACK_LEG :
			{			
				Right_Back_Leg_Struct.Leg_X =foot_x;
				Right_Back_Leg_Struct.Leg_Y =foot_y;
				Coordinate_Inverse(&Right_Back_Leg_Struct);
				Right_Back_In(Right_Back_Leg_Struct.Steerint_IN )
				Right_Back_OUT(Right_Back_Leg_Struct.Steerint_OUT )
			}	
			break;
			//��ǰ��
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
  * @brief  ��˹켣���ƺ���
  * @param  Leg_Data���Ȳ����ݽṹ��ָ��
            trail���켣������ѡ���������ߺ�ֱ��
            direction���������ƶ�����ǰ���
            t��ʱ��������100ms����1,ֻ�ܵ���
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
	
	
	//����
  Coordinate_Inverse(Leg_Data);
	
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
  * @brief  ��̬���ƺ���
  * @param  gait����̬��ǰ�������ˣ�����
  * @retval void
  * @notes  ������ѭ����̬��̬����������������ʱ�������������һ��������ʱ�������������ջ�
            ʹ����ת�������������ȵ����ͣ�ʹ�üƴβ������Ƶ���һ��������ʱ�����������ջ�
            ����ʱҲ����Ӧ
  */
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
  * @brief  ���λ�ó�ʼ������
  * @param  void
  * @retval void
  * @notes  ��˻�λ������ٶȹ��㣬
  */
void Foot_Direction_Init(void)
{
	//�Ȳ����ͱ궨
	Left_Front_Leg_Struct.Leg_Type  = LEFT_FRONT_LEG;
	Left_Back_Leg_Struct.Leg_Type   = LEFT_BACK_LEG;
	Right_Front_Leg_Struct.Leg_Type = RIGHT_FRONT_LEG;
  Right_Back_Leg_Struct.Leg_Type  = RIGHT_BACK_LEG;
	
  //�Ȳ���ԭ��
	Shrink();
	
	
	//�ֵ������	
	Left_Front_Speed(400)
	Left_Back_Speed(400)
  Right_Back_Speed(400)
  Right_Front_Speed(400)
	
	vTaskDelay(1500);	
  //�ֵ���ٶȹ�0,��ǰ��Ҫ��50
  Left_Front_Speed(WHEEL_STOP-70)
	Left_Back_Speed(WHEEL_STOP)
	Right_Front_Speed(WHEEL_STOP)
	Right_Back_Speed(WHEEL_STOP+30)
	

}











