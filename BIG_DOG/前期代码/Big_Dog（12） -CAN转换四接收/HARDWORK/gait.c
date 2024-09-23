#include "main.h"

/*************************************************���ݶ�����*************************************************/
//�����ݽṹ��,
Leg_Data_Typedef Left_Front_Leg_Struct;
Leg_Data_Typedef Left_Back_Leg_Struct;
Leg_Data_Typedef Right_Back_Leg_Struct;
Leg_Data_Typedef Right_Front_Leg_Struct;


//��̬���Ʊ�־λ
 char Left_Front_Start_Flag=0;
 char Left_Back_Start_Flag=0;
 char Right_Back_Start_Flag=0;
 char Right_Front_Start_Flag=0;

 char count_num=RIGHT_FRONT_LEG;
 char wheel_turn=LEFT_FRONT_LEG;


 //��ʱ����
 int TIM_t=0;

/*************************************************����������**************************************************/


/********************************************����***************************************************/
/**
  * @brief  ��ʽ���п��ƺ���
  * @param  ��                    
  * @retval void
  * @notes  ��ң�������ƻ�е��ǰ����ת��
  */
void Slide(void)
{
	float RC_Motor_Ra=10;
	
      Wheel_LF.Target_Speed =(DBUS.RC.ch1+DBUS.RC.ch0)*RC_Motor_Ra;
			Wheel_LB.Target_Speed =(DBUS.RC.ch1+DBUS.RC.ch0)*RC_Motor_Ra;
			Wheel_RB.Target_Speed =(DBUS.RC.ch1-DBUS.RC.ch0)*RC_Motor_Ra;
			Wheel_RF.Target_Speed =(DBUS.RC.ch1-DBUS.RC.ch0)*RC_Motor_Ra;
}


/********************************************����***************************************************/
/**
  * @brief  ���귴��,���Ϳ��ƺ���
  * @param  Leg_Data���Ȳ����ݽṹ��ָ��                    
  * @retval void
  * @notes  ʹ�÷��⹫ʽ��X,Y���귴��Ϊ�������λ��ֵ���������
                                            
  */
void Coordinate_Inverse(Leg_Data_Typedef* Leg_Data)
{
	//�Ƕȶ���,��λ����
	double  seigamar=0;
	double  gama=0;
  
  
  //���귴�⹫ʽ
	seigamar=asin(sqrt(Leg_Data->Leg_X*Leg_Data->Leg_X+Leg_Data->Leg_Y*Leg_Data->Leg_Y)/2/CLUB_LENGTH)+atan(Leg_Data->Leg_X/Leg_Data->Leg_Y);//������,��
  gama    =asin(sqrt(Leg_Data->Leg_X*Leg_Data->Leg_X+Leg_Data->Leg_Y*Leg_Data->Leg_Y)/2/CLUB_LENGTH)-atan(Leg_Data->Leg_X/Leg_Data->Leg_Y);//�̣��£���
  
	
	//CAN1�������
	
	switch(Leg_Data->Leg_Type)
	{
		//����
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
		
		
		//����
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

	    CAN1_TX_Jiont(&Joint_RF_U);//���Ϳ���7��
     
			CAN1_TX_Jiont(&Joint_RF_D);//���Ϳ���8��
     
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
            ���ڶԵ��Ƚ��е���
  */
void Leg_Diretion_Control(Leg_Data_Typedef* Leg_Data,float foot_x,float foot_y)
{
	
	
//	//��λ
//	if(foot_x>180)foot_x=180;
//	if(foot_x<-180)foot_x=-180;
//	
//	//��λ
//	if(foot_y>=360)foot_y=360;
//	if(foot_y<200)foot_y=200;
//	
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
	//���귴�⣬����
	Coordinate_Inverse(Leg_Data);
	
}



/**
  * @brief  ������˹켣���ƺ���
  * @param  Leg_Data���Ȳ����ݽṹ��ָ��
            trail���켣������ѡ���������ߺ�ֱ��
            direction���������ƶ�����ǰ���
  * @retval void
  * @notes  ʹ�ú�������������Ϊ�����켣�����ۼ����ڣ�ʹ�ø�̬�����㷨��
             �Լ���Ϊ��Χ��ֻ�е�����ƫ��֮�ڵ���ܼ����ۼ�
  */
void Foot_Control(Leg_Data_Typedef* Leg_Data,char trail,char direction )
{
	//
	double fai=0;
  double deta_distance_2=0;//��ʵ���������Ŀ������ľ����ƽ��

  
  //�ɵ���ķ����Ƕȼ����������
   Leg_Data->real_x =CLUB_LENGTH*(cos(Leg_Data->Jiont_Motor_D->recieve.recieve_Position)-cos(Leg_Data->Jiont_Motor_U->recieve.recieve_Position));
   Leg_Data->real_y =CLUB_LENGTH*(sin(Leg_Data->Jiont_Motor_D->recieve.recieve_Position)+sin(Leg_Data->Jiont_Motor_U->recieve.recieve_Position));
  //�������ƫ���ƽ��,���뵥λmm��
   deta_distance_2 = (Leg_Data->real_x-Leg_Data->Leg_X)*(Leg_Data->real_x-Leg_Data->Leg_X)+(Leg_Data->real_y-Leg_Data->Leg_Y)*(Leg_Data->real_y-Leg_Data->Leg_Y);
   

	//��������ƫ������һֱ���֮ǰ����ֵ����ֹ�켣����������������������������������������������������������������������������������������������������
// if(deta_distance_2<=(DISTANCE_IGNORE*DISTANCE_IGNORE))
// {
 //����ǰ��
  if(trail==STEP_FORWARD)
	{ 
		 //��ʱ����ת��,ÿ�����е�ʱ�����ת��
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
		 Leg_Data->Leg_X =STEP_LENTH/2/pi*(fai-sin(fai))+FOOT_X;
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
		
  //��λ��X
		 if(Leg_Data->Leg_X > (FOOT_X+STEP_LENTH))Leg_Data->Leg_X = (FOOT_X+STEP_LENTH);
		 if(Leg_Data->Leg_X < (FOOT_X))Leg_Data->Leg_X = FOOT_X;
 //����Y
		 Leg_Data->Leg_Y=FOOT_Y;
		 
		//�켣�ν�
		if(Leg_Data->Leg_X == (FOOT_X+STEP_LENTH))
		  Leg_Data->step_forward_time=2*pi;
	  if(Leg_Data->Leg_X <= FOOT_X+10)
			Leg_Data->step_forward_time=0;
	
	}
// }

	//���귴���������
	Coordinate_Inverse(Leg_Data);

}


/**
  * @brief  ����ѭ��ǰ������
  * @param  Leg_Data���Ȳ����ݽṹ��ָ��
            
  * @retval void
  * @notes  ����Foot_Control����ʵ�ֵ���ѭ��
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
  * @brief  ���㲽̬���ƺ���
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
	  if(TIM_t%(STEP_FORWARD_TIME)==0)
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




/********************************************�����л����ʼ��***************************************************/
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
   Leg_Deinit(&Left_Front_Leg_Struct);
	 //��ǰ
	 Leg_Deinit(&Right_Front_Leg_Struct);
	 //�Һ�
   Leg_Deinit(&Right_Back_Leg_Struct);
   //���
   Leg_Deinit(&Left_Back_Leg_Struct);
}




/**
  * @brief  �Ȳ�����Ӧ��ʼ��
  * @param  void
  * @retval void
  * @notes  ���ȱ�־λ���㣬������������ת������λ������ʱ�����ڹ��㣬��������ʼ������˻�λ��
  */
void Leg_Deinit(Leg_Data_Typedef *Leg_Struct_Point)
{
  Leg_Struct_Point->step_forward_time=0;
	Leg_Struct_Point->Leg_X =FOOT_X;
	Leg_Struct_Point->Leg_Y =FOOT_Y+Leg_Struct_Point->Y_Add ;

	Coordinate_Inverse(Leg_Struct_Point);

}






























