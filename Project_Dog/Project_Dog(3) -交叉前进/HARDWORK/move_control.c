#include "main.h"

/*************************************************���ݶ�����*************************************************/
//�����ݽṹ��
Leg_Data_Typedef Left_Front_Leg_Struct;
Leg_Data_Typedef Left_Back_Leg_Struct;
Leg_Data_Typedef Right_Front_Leg_Struct;
Leg_Data_Typedef Right_Back_Leg_Struct;


 
  //��̬���Ʊ�־λ
  char step_back_flag=0;//1��ʾ��������ǰ����ʱ������˵�FOOT_X(-150)+0.5����=0���Ҳ��˵�FOOT_X(0)-0.5����=-150
												//0��ʾ��������ǰ����ʱ��������˵�FOOT_x=-150,�Ҳ����˵�FOOT_x=0

  char gait_mode_turn=LEFT_BACK_LEG;

  char fornt_back_flag=1;   //1��ʾǰ��

 int   TIM_t=0;

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

/********************************************����***************************************************/
/**
  * @brief  ��ʽ���п��ƺ���
  * @param  ��                    
  * @retval void
  * @notes  ��ң�������ƻ�е��ǰ����ת��
  */
/********************************************����***************************************************/
/**
  * @brief  ���귴�⺯��
  * @param  Leg_Data���Ȳ����ݽṹ��ָ��                    
  * @retval void
  * @notes  ʹ�÷��⹫ʽ��X,Y���귴��Ϊ������������ֵ��
            �������ֵ���������ݽṹ���еĳ�Ա��
  */
double  seigamar=0;
double  gama=0;
	
void Coordinate_Inverse(Leg_Data_Typedef* Leg_Data)
{
	//�Ƕȶ���
	
	
	
	
  //���귴�⹫ʽ   

	gama    =asin((Leg_Data->Leg_X*Leg_Data->Leg_X+Leg_Data->Leg_Y*Leg_Data->Leg_Y+SMALL_LENGTH_2*SMALL_LENGTH_2-GIG_LENGTH_1*GIG_LENGTH_1)/2/SMALL_LENGTH_2/sqrt(Leg_Data->Leg_X*Leg_Data->Leg_X+Leg_Data->Leg_Y*Leg_Data->Leg_Y))-atan(Leg_Data->Leg_X/Leg_Data->Leg_Y);
	
	
	seigamar=asin((Leg_Data->Leg_X*Leg_Data->Leg_X+Leg_Data->Leg_Y*Leg_Data->Leg_Y+GIG_LENGTH_1*GIG_LENGTH_1-SMALL_LENGTH_2*SMALL_LENGTH_2)/2/GIG_LENGTH_1/sqrt(Leg_Data->Leg_X*Leg_Data->Leg_X+Leg_Data->Leg_Y*Leg_Data->Leg_Y))+atan(Leg_Data->Leg_X/Leg_Data->Leg_Y);
	
	
	
 
  
	
	
	//����������ֵ����
	if(Leg_Data->Leg_Type <=LEFT_BACK_LEG)//�����
	{
		//�����
		Leg_Data->Steerint_OUT =320-180*(gama+seigamar)/pi;
		//�ڲ���
		Leg_Data->Steerint_IN  =230-180*seigamar/pi;
	}
	else//�Ҳ���
	{
		//�����
	   Leg_Data->Steerint_OUT =180*(gama+seigamar)/pi-40;
		//�ڲ���
	   Leg_Data->Steerint_IN =50+180*seigamar/pi;
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
			Right_Back_OUT(Leg_Data->Steerint_OUT+10 )
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
  * @notes  ʹ�ú�������������Ϊ�����켣�����ۼ����ڣ�ʹ�ø�̬�����㷨��
             �Լ���Ϊ��Χ��ֻ�е�����ƫ��֮�ڵ���ܼ����ۼ�
  */
void Foot_Control(Leg_Data_Typedef* Leg_Data,char trail )
{
	//
	double fai=0;
 
 //����ǰ��
  if(trail==STEP_FORWARD)
	{ 
		 //��ʱ����ת��,ÿ�����е�ʱ�����ת��
			 Leg_Data->step_forward_time +=1*2*pi/STEP_FORWARD_TIME;	 
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
			 Leg_Data->Leg_Y = Leg_Data->FOOT_y+Leg_Data->Y_Add;
		 }
		
	}
	
	
	
	//����
  else if(trail==QUIESECENCE)
	{
		Leg_Data->Leg_X  -=1*STEP_LENTH/QUIESECENCE_TIME;
		
		if(Leg_Data->Leg_X<FOOT_X) 
		 {
			 Leg_Data->Leg_X=FOOT_X ;
			 
			 Leg_Data->step_forward_time=0;
			 
		 }
		 	
		//����Y
		  Leg_Data->Leg_Y=Leg_Data->FOOT_y+Leg_Data->Y_Add;
	}


	//���귴���������
	Coordinate_Inverse(Leg_Data);
	
	Steering_PWM_Out(Leg_Data);

}
	



/**
  * @brief  �Ȳ���������
  * @param  void
  * @retval void
  * @notes  ���ȱ�־λ���㣬������������ת������λ������ʱ�����ڹ��㣬��������ʼ������˻�λ��
  */
void Shrink(void)
{

	 Left_Front_Speed(100)    //W   
	 Left_Back_Speed(100)	    //X
   Right_Front_Speed(180)   //Y
	 Right_Back_Speed(180)		//A
	
	
	//��ǰ
	Left_Front_Leg_Struct.step_forward_time=0;
	
	Left_Front_Leg_Struct.Leg_X =FOOT_X;
	Left_Front_Leg_Struct.Leg_Y =FOOT_Y;
	Left_Front_Leg_Struct.FOOT_x =FOOT_X;
	Left_Front_Leg_Struct.FOOT_y =FOOT_Y;
	Left_Front_Leg_Struct.Y_Add =0;
	Coordinate_Inverse(&Left_Front_Leg_Struct);
	Steering_PWM_Out(&Left_Front_Leg_Struct);
	//���
	Left_Back_Leg_Struct.step_forward_time=0;
	Left_Back_Leg_Struct.Leg_X =FOOT_X;
	Left_Back_Leg_Struct.Leg_Y =FOOT_Y;
	Left_Back_Leg_Struct.FOOT_x =FOOT_X;
	Left_Back_Leg_Struct.FOOT_y =FOOT_Y;
	Left_Back_Leg_Struct.Y_Add =0;
	Coordinate_Inverse(&Left_Back_Leg_Struct);
	Steering_PWM_Out(&Left_Back_Leg_Struct);
	
	
	
	//��ǰ
  Right_Front_Leg_Struct.step_forward_time=0;
	Right_Front_Leg_Struct.Leg_X =FOOT_X;
	Right_Front_Leg_Struct.Leg_Y =FOOT_Y;
	Right_Front_Leg_Struct.FOOT_x =FOOT_X;
	Right_Front_Leg_Struct.FOOT_y =FOOT_Y;
	Right_Front_Leg_Struct.Y_Add =0;
	Coordinate_Inverse(&Right_Front_Leg_Struct);
	Steering_PWM_Out(&Right_Front_Leg_Struct);
	//�Һ�
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
	Shrink();

}











