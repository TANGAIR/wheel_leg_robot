#include "main.h"

/*************************************************���ݶ�����*************************************************/
  //�����ݽṹ��,
  Leg_Data_Typedef Left_Front_Leg_Struct;
  Leg_Data_Typedef Left_Back_Leg_Struct;
  Leg_Data_Typedef Right_Back_Leg_Struct;
  Leg_Data_Typedef Right_Front_Leg_Struct;


  //��̬���Ʊ�־λ
  char step_back_flag=0;//1��ʾ��������ǰ����ʱ������˵�FOOT_X(-150)+0.5����=0���Ҳ��˵�FOOT_X(0)-0.5����=-150
												//0��ʾ��������ǰ����ʱ��������˵�FOOT_x=-150,�Ҳ����˵�FOOT_x=0

  char gait_mode_turn=LEFT_BACK_LEG;

  char fornt_back_flag=1;   //1��ʾǰ��
	
	
	

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
	float RC_Motor_Ra=7;
	
      Wheel_LF.Target_Speed =(DBUS.RC.ch1+DBUS.RC.ch0)*RC_Motor_Ra;
			Wheel_LB.Target_Speed =(DBUS.RC.ch1+DBUS.RC.ch0)*RC_Motor_Ra;
			Wheel_RB.Target_Speed =(DBUS.RC.ch1-DBUS.RC.ch0)*RC_Motor_Ra;
			Wheel_RF.Target_Speed =(DBUS.RC.ch1-DBUS.RC.ch0)*RC_Motor_Ra;
}


/*









*******************************************����***************************************************/
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
	
	
	
	//X��λ��ֹ����X�����ռ���-150~150
		if(Leg_Data->Leg_X<=-150)Leg_Data->Leg_X=-150;
		if(Leg_Data->Leg_X>=150)Leg_Data->Leg_X=150;
	//Y��λ��ֹ����Y�����ռ���150~250
	   if(Leg_Data->Leg_Y>=250)Leg_Data->Leg_Y=260;
		 if(Leg_Data->Leg_Y<=120)Leg_Data->Leg_Y=120;
	
  //���귴�⹫ʽ
	seigamar=asin(sqrt(Leg_Data->Leg_X*Leg_Data->Leg_X+Leg_Data->Leg_Y*Leg_Data->Leg_Y)/2/CLUB_LENGTH)+atan(Leg_Data->Leg_X/Leg_Data->Leg_Y);//�����ϣ���
  gama    =asin(sqrt(Leg_Data->Leg_X*Leg_Data->Leg_X+Leg_Data->Leg_Y*Leg_Data->Leg_Y)/2/CLUB_LENGTH)-atan(Leg_Data->Leg_X/Leg_Data->Leg_Y);//�̣��£���
  
	 //��λ��ֹ��������
	 if(seigamar>2) seigamar=2;
	 if(seigamar<0) seigamar=0;
	 if(gama>2) seigamar=2;
	 if(gama<0) seigamar=0;


	//�������������ֵ�������ȵ����ͽ��Ƕ������������ͬ�Ĺؽڵ��
	switch(Leg_Data->Leg_Type)
	{
		//����
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

		//����
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
  * @brief  �Ȳ�������ƺ���
  * @param  void
  * @retval void
  * @notes  ʹ������������λ��,X����ǰΪ����Y������Ϊ��
            ���ڶԵ��Ƚ��е���
  */
void Leg_Diretion_Control(Leg_Data_Typedef* Leg_Data,float foot_x,float foot_y)
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
	//���귴�⣬����
	Coordinate_Inverse(Leg_Data);

}



/*



*******************************************��̬��̬***************************************************/



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
		 	 
		 //�켣������㣬������ǰ�����Ǵ�-150��ʼ
		 Leg_Data->Leg_X =STEP_LENTH/2/pi*(fai-sin(fai))+FOOT_X;
		 Leg_Data->Leg_Y =(STEP_HIGHT/2*(1-cos(fai)))*(-1)+Leg_Data->FOOT_y+Leg_Data->Y_Add;
		 
		 //�켣�ν�,��ֹ�������
		 if(fai==2*pi) 
		 {
			 Leg_Data->Leg_Mode =0;    //0��ʾ���뾲�˽׶�
			 Leg_Data->Leg_X =(FOOT_X+STEP_LENTH);
			 Leg_Data->Leg_Y = Leg_Data->FOOT_y+Leg_Data->Y_Add;
		 }
		 if(fai==0)    
		 {
			 
			 Leg_Data->Leg_X = FOOT_X;
			 Leg_Data->Leg_Y = Leg_Data->FOOT_y+Leg_Data->Y_Add;
		 }
	}
	//����
  else if(trail==QUIESECENCE)
	{
		 //��ʱ����ת��//�켣�������
	   if(direction==DIRECTION_FORWARD)
			 Leg_Data->Leg_X  -=1*STEP_LENTH/QUIESECENCE_TIME/2;//ֻ�˰벽
	   else if(direction==DIRECTION_BACK)
			 Leg_Data->Leg_X  +=1*STEP_LENTH/QUIESECENCE_TIME/2;
		
		 if(step_back_flag==1)//�������ǰ
		 {
			   //���������FOOT_x=-150,��150��0
			   if(Leg_Data->Leg_Type <33)
				 { //150->0
					 if(Leg_Data->Leg_X > (Leg_Data->FOOT_x+STEP_LENTH))Leg_Data->Leg_X = (Leg_Data->FOOT_x+STEP_LENTH);
				   if(Leg_Data->Leg_X < (Leg_Data->FOOT_x+STEP_LENTH/2))Leg_Data->Leg_X = Leg_Data->FOOT_x+STEP_LENTH/2;
				 
						//�켣�ν�
						if(Leg_Data->Leg_X == (Leg_Data->FOOT_x+STEP_LENTH))
							 Leg_Data->step_forward_time=2*pi;
						if(Leg_Data->Leg_X <= Leg_Data->FOOT_x+STEP_LENTH/2)
						{
							 Leg_Data->step_forward_time=0;
							 Leg_Data->Leg_Mode =1;   //1��ʾǰ��
						} 
				 }
				  //�����Ҳ���FOOT_x=0����0��-150
			   else  if(Leg_Data->Leg_Type >33)
				 { //0->-150
					 if(Leg_Data->Leg_X > (Leg_Data->FOOT_x))Leg_Data->Leg_X = Leg_Data->FOOT_x;
				   if(Leg_Data->Leg_X < (Leg_Data->FOOT_x-STEP_LENTH/2))Leg_Data->Leg_X = Leg_Data->FOOT_x-STEP_LENTH/2;
				 
						//�켣�ν� 0->-150
						if(Leg_Data->Leg_X == (Leg_Data->FOOT_x))
							 Leg_Data->step_forward_time=2*pi;
						if(Leg_Data->Leg_X <= Leg_Data->FOOT_x-STEP_LENTH/2)
						{
							 Leg_Data->step_forward_time=0;
							 Leg_Data->Leg_Mode =1;   //1��ʾǰ��
						} 
				 }
		 }
		 else //�Ҳ�����ǰ
		 {
			    //��0->-150,�Ҳ�150->0
				   if(Leg_Data->Leg_X > (Leg_Data->FOOT_x+STEP_LENTH/2))Leg_Data->Leg_X = (Leg_Data->FOOT_x+STEP_LENTH/2);
				   if(Leg_Data->Leg_X < (Leg_Data->FOOT_x))Leg_Data->Leg_X = Leg_Data->FOOT_x;
				 
						//�켣�ν�
						if(Leg_Data->Leg_X == (Leg_Data->FOOT_x+STEP_LENTH/2))
							 Leg_Data->step_forward_time=2*pi;
						if(Leg_Data->Leg_X <= Leg_Data->FOOT_x)
						{
							 Leg_Data->step_forward_time=0;
							 Leg_Data->Leg_Mode =1;   //1��ʾǰ��
						} 
		 }
		 	
		//����Y
		  Leg_Data->Leg_Y=Leg_Data->FOOT_y+Leg_Data->Y_Add;
	}


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
  * @brief  ���㲽̬���ƺ���
  * @param  gait����̬��ǰ�������ˣ�����
  * @retval void
  * @notes  ���õ��ཻ�沽̬��ÿ�ε���������һ��300����һ�������������֮����һС��150��Ȼ��������һ����ȣ�
            ÿ��ǰ��������ʱ��������150
  */
 int   TIM_t=0;
 char leg_turn_mix_flag=0x1e;
void Gait_Control(char gait)
{
	
	//�����̬Ϊǰ�����ߺ���
	if(gait!=SHRINK)
	{
		
		//ʹ�ÿ���λ�ý�������
		 leg_turn_mix_flag=(Left_Back_Leg_Struct.Leg_Mode <<4)|(Left_Front_Leg_Struct.Leg_Mode <<3)|(Right_Back_Leg_Struct.Leg_Mode <<2)|(Right_Front_Leg_Struct.Leg_Mode<<1 )|(step_back_flag);
		
		 switch(leg_turn_mix_flag)
		 {
			 //�������
			 case  0x1e://11110
			 {
				 gait_mode_turn= LEFT_BACK_LEG;
				 break;
			 }
			 //����ǰ��
			  case  0x0e://01110
			 {
				 gait_mode_turn= LEFT_FRONT_LEG;
				 break;
			 }
			 //�����ǰʱ����
			 case  0x06://00110
			 {
				 gait_mode_turn= LEFT_FRONT_BACKFORWARD;
				 step_back_flag=1;
				 break;
			 }
		 
			 //�Һ�������
			 case 0x1f: //11111
			 {
				 gait_mode_turn= RIGHT_BACK_LEG;

				  break;
			 }
				 //��ǰ������
			  case 0x1b://11011
			 {
				 gait_mode_turn= RIGHT_FRONT_LEG;
				  break;
			 }
				 //�Ҳ࿿ǰʱ����
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
			
			//��������� 11
			case LEFT_BACK_LEG :
			{	
				//�����ǰ�� 300
				Foot_Control(&Left_Back_Leg_Struct,STEP_FORWARD,gait);	
			}	
			break;
			//��ǰ������ 22
			case LEFT_FRONT_LEG :
			{						
				//��ǰ��ǰ��300
				Foot_Control(&Left_Front_Leg_Struct,STEP_FORWARD,gait);
			}
			break;
			
			
			//����ȿ�ǰʱ�������  33
			case LEFT_FRONT_BACKFORWARD :
			{					
				 
				//������˵�0���Ҳ����˵�-150	
				Foot_Control(&Left_Front_Leg_Struct,QUIESECENCE,gait);
				Foot_Control(&Left_Back_Leg_Struct,QUIESECENCE,gait);
			  Foot_Control(&Right_Front_Leg_Struct,QUIESECENCE,gait);
				Foot_Control(&Right_Back_Leg_Struct,QUIESECENCE,gait);		
			}
			break;
			
			
			//�Һ���ǰ��300   ,44
			case RIGHT_BACK_LEG :
			{
				//�Һ���ǰ�� 300
				Foot_Control(&Right_Back_Leg_Struct,STEP_FORWARD,gait);
			}	
			break;
			//��ǰ��ǰ��300  55
			case RIGHT_FRONT_LEG :
			{
				//��ǰ��ǰ��300
				Foot_Control(&Right_Front_Leg_Struct,STEP_FORWARD,gait);
					
			}
			break;
			
			//�Ҳ��ȿ�ǰʱ���������150  66
			case RIGHT_FRONT_BACKFORWARD :
			{	
				
				//������˵�-150���Ҳ����˵�0
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
	//����̬Ϊ����	
  else if(gait==SHRINK)
	{
		//��������ʼ����
	  Shrink();
	}
}







 /*


*******************************************��̬��̬**************************************************




*/
/**
  * @brief  ��̬������˹켣���ƺ���
  * @param  Leg_Data���Ȳ����ݽṹ��ָ��
  * @retval void
  * @notes  ʹ��̧�Ⱦ������ϵķ�ʽ��ʵ�����Ķ�̬��̬��ʹ�ò��������Ƶ������������Լ�ǰ�����ߺ��ˣ�
	          ��ʼ����Ϊ10��ֻ��ÿ���������ڽ���ʱ�Ż���²�����ʹ��ң�������Ʋ����Կ���ǰ������
						�����ȣ�һ����һ���ӣ����Ƚ���ͬ������
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
	
	
	//̧��ǰ������
	 if( Leg_Data->Leg_Mode==1)
	 {
		  //����
		 {
		   //��ʱ����ת��,ÿ�����е�ʱ�����ת��
			 Leg_Data->step_forward_time +=1*2*pi/STEP_FORWARD_TIME;// (t-last_t)		
			 //��������ת��		 
			 fai=Leg_Data->step_forward_time;
		
			 //��λ
			 if(fai>2*pi)fai=2*pi;
			 if(fai<0)fai=0;
				 
			 //�켣�������
			 Leg_Data->Leg_X =step_lenght/2/pi*(fai-sin(fai))+Leg_Data->FOOT_x;
			 Leg_Data->Leg_Y =(STEP_HIGHT/2*(1-cos(fai)))*(-1)+Leg_Data->FOOT_y+Leg_Data->Y_Add;
			 
			 //�켣�ν�,��ֹ�������
			 if(fai==2*pi) 
			 {
				 Leg_Data->Leg_Mode =0;    //0��ʾ���뾲�˽׶�
				 Leg_Data->Leg_X =(Leg_Data->FOOT_x +step_lenght);
				 Leg_Data->Leg_Y = Leg_Data->FOOT_y+Leg_Data->Y_Add;
				 Leg_Data_Follow->step_forward_time=0;
				 Leg_Data_Follow->Leg_Mode =1;   //1��ʾǰ��
				 //���²���
				 step_lenght_follow=Leg_Data_Follow->step_lenght ;
			 }
		 }

		 //���ȣ�ʱ��������Ϊ׼
		 {
			  //ֱ�߹켣�������
		    Leg_Data_Follow->Leg_X  -=1*step_lenght_follow/STEP_FORWARD_TIME;
				 //������Ϊ��ʱ������
				if(step_lenght_follow>=0)
				{
					if(Leg_Data_Follow->Leg_X > (Leg_Data_Follow->FOOT_x+step_lenght_follow))Leg_Data_Follow->Leg_X = (Leg_Data_Follow->FOOT_x+step_lenght_follow);
					//��ʱһ��ѭ�����ڽ��������Ը��²�������
					if(Leg_Data_Follow->Leg_X < Leg_Data_Follow->FOOT_x)
					{
						Leg_Data_Follow->Leg_X = Leg_Data_Follow->FOOT_x;
					}	
				}
				//������Ϊ��ʱ������
				else	if(step_lenght_follow<0)
				{
					if(Leg_Data_Follow->Leg_X < (Leg_Data_Follow->FOOT_x+step_lenght_follow))Leg_Data_Follow->Leg_X = (Leg_Data_Follow->FOOT_x+step_lenght_follow);
					if(Leg_Data_Follow->Leg_X > Leg_Data_Follow->FOOT_x)
					{
						Leg_Data_Follow->Leg_X = Leg_Data_Follow->FOOT_x;
					}
		    }
				//����Y
				Leg_Data_Follow->Leg_Y=Leg_Data_Follow->FOOT_y+Leg_Data_Follow->Y_Add;
		 }  
	 }
    	 
	 //�����ؾ�Ħ������	 
	 else if( Leg_Data->Leg_Mode==0)
	 {
		 {
			 //ֱ�߹켣�������
			 Leg_Data->Leg_X  -=1*step_lenght/QUIESECENCE_TIME;
			
			 //������Ϊ��ʱ������
			if(step_lenght>=0)
			{
				if(Leg_Data->Leg_X > (Leg_Data->FOOT_x+step_lenght))Leg_Data->Leg_X = (Leg_Data->FOOT_x+step_lenght);
				//��ʱһ��ѭ�����ڽ��������Ը��²�������
				if(Leg_Data->Leg_X < Leg_Data->FOOT_x)
				{
					Leg_Data->Leg_X = Leg_Data->FOOT_x;
					Leg_Data->step_forward_time=0;
					Leg_Data->Leg_Mode =1;   //1��ʾǰ��
					//���²���
					step_lenght=Leg_Data->step_lenght ;
				}	
			}
			//������Ϊ��ʱ������
			else	if(step_lenght<0)
			{
				if(Leg_Data->Leg_X < (Leg_Data->FOOT_x+step_lenght))Leg_Data->Leg_X = (Leg_Data->FOOT_x+step_lenght);
				if(Leg_Data->Leg_X > Leg_Data->FOOT_x)
				{
					Leg_Data->Leg_X = Leg_Data->FOOT_x;
					Leg_Data->step_forward_time=0;
					Leg_Data->Leg_Mode =1;   //1��ʾǰ��
					//���²���
					step_lenght=Leg_Data->step_lenght ;
				}
			}	
			//����Y
			Leg_Data->Leg_Y=Leg_Data->FOOT_y+Leg_Data->Y_Add;
		 }
		 
		//���Ⱥ��ˣ�����ǰ��
		{
		 //��ʱ����ת��,ÿ�����е�ʱ�����ת��
		 Leg_Data_Follow->step_forward_time +=1*2*pi/QUIESECENCE_TIME;// (t-last_t)		
     //��������ת��		 
		 fai_follow=Leg_Data_Follow->step_forward_time;
	
		 //��λ
		 if(fai_follow>2*pi)fai_follow=2*pi;
		 if(fai_follow<0)fai_follow=0;
		 	 
		 //�켣�������
		 Leg_Data_Follow->Leg_X =step_lenght_follow/2/pi*(fai_follow-sin(fai_follow))+Leg_Data_Follow->FOOT_x ;
		 Leg_Data_Follow->Leg_Y =(STEP_HIGHT/2*(1-cos(fai_follow)))*(-1)+Leg_Data_Follow->FOOT_y+Leg_Data_Follow->Y_Add;
		 
		 //�켣�ν�,��ֹ�������
		 if(fai_follow==2*pi) 
		 {
			 Leg_Data_Follow->Leg_Mode =0;    //0��ʾ���뾲�˽׶�
			 Leg_Data_Follow->Leg_X =(Leg_Data_Follow->FOOT_x +step_lenght);
			 Leg_Data_Follow->Leg_Y = Leg_Data_Follow->FOOT_y+Leg_Data_Follow->Y_Add;
		 }
		}
	 }
	//���귴���������
	Coordinate_Inverse(Leg_Data);
	Coordinate_Inverse(Leg_Data_Follow);
}



/**
  * @brief  ��̬��̬���ƺ���
  * @param  
  * @retval void
  * @notes  ʹ�õ��͵Ļ��������涯̬��̬
  */
/********************************************�����л����ʼ��***************************************************/
/**
  * @brief  �Ȳ���������
  * @param  void
  * @retval void
  * @notes  ���ȱ�־λ���㣬������������ת������λ������ʱ�����ڹ��㣬��������ʼ������˻�λ��
  */
void Shrink(void)
{

	 step_back_flag=0;
	 gait_mode_turn=LEFT_BACK_LEG;
	
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
	
	switch(Leg_Struct_Point->Leg_Type)
	{
		    //��ǰ
		case LEFT_FRONT_LEG:
		{
			 Leg_Struct_Point->FOOT_x = FOOT_X;
			 Leg_Struct_Point->FOOT_y = FOOT_Y+15;
				break;
		}
				//���
		case LEFT_BACK_LEG:
		{
			 Leg_Struct_Point->FOOT_x = FOOT_X;
			 Leg_Struct_Point->FOOT_y = FOOT_Y-5;
				break;
		}
				//�Һ�
		case RIGHT_BACK_LEG:
		{
			Leg_Struct_Point->FOOT_x = FOOT_X;
			 Leg_Struct_Point->FOOT_y = FOOT_Y;
				break;
		}
	
				//��ǰ
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




