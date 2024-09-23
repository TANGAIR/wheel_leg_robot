#include "main.h"
#include "move_control.h"
/*************************************************���ݶ�����*************************************************/
 int mode=1;
 char Contron_ID=8; 
 char Contron_ID_2=6;
 char Contron_ID_3=7;
char Contron_ID_4=5;
 /*************************************************����������**************************************************/
/**
  * @brief  �ƶ�����������
  * @param  void
  * @retval void
  * @notes  ����ң������ģʽѡ��ͬ�ķ�ʽ
  */
		 int delay_time=15;  //��15msΪ�������ڣ��Ϳ��Զ��յ� ������0.12s�İ�����ָ������
void Move_Control_Task(void *pvParameters)
{
	
	BMI088_init();            //�����ǳ�ʼ���������������ж�
	
	USART1_Init();						//����1��˫��ͨѶ��ʼ��
	CAN1_Init();              //�ؽڵ��Can��ʼ��
	CAN2_Init();              //�ֵ����ʼ��
	vTaskDelay(100);

	Beep_ON ();
	vTaskDelay(500);
	Beep_OFF();
  Foot_Direction_Init();    //�Ȳ�λ�ó�ʼ��������״̬
	vTaskDelay(1000);
	
	

	while(1)
	{	
		
		
		
		
		
		
	 Contron_TX_Jiont();
   vTaskDelay(delay_time);
			
	}	
	
	
	
	//�������
		//Motor_Debug();	
//    //����
//	  if(	DBUS.RC.Switch_Left==RC_SW_UP)
//		{
//      //��
//		  signal_leg(&Right_Front_Leg_Struct);
//			//��
//			Slide();

//		}
//		//��ʽǰ��
//		else if(DBUS.RC.Switch_Left==RC_SW_DOWN)
//		{
//			//��
//		  Shrink();
//			//��
//		  Slide();
//		}
//    //�м��Լ�����
//		else 
//		{
//		  //�Ȳ���ԭ��
//	    Shrink();
//			//���ٶ���Ϊ0
//			Wheel_LF.Target_Speed =0;
//			Wheel_LB.Target_Speed =0;
//			Wheel_RB.Target_Speed =0;
//			Wheel_RF.Target_Speed =0;
//			
//		}
//		//��ͨѶ���
//		CAN2_TX();
			

	
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
  Right_Back_Leg_Struct.Leg_Type  = RIGHT_BACK_LEG;
	Right_Front_Leg_Struct.Leg_Type = RIGHT_FRONT_LEG;
  

  //�������ݽṹ�������ؽڵ���ṹ���Ӧ
  Left_Front_Leg_Struct.Jiont_Motor_U=&Joint_LF_U;//��ǰ�ϽǶȵ��1
  Left_Front_Leg_Struct.Jiont_Motor_D=&Joint_LF_D;//��ǰ�½Ƕȵ��2
  Left_Back_Leg_Struct.Jiont_Motor_U=&Joint_LB_U;//����ϽǶȵ��3
  Left_Back_Leg_Struct.Jiont_Motor_D=&Joint_LB_D;//����½Ƕȵ��4
  
  Right_Back_Leg_Struct.Jiont_Motor_U=&Joint_RB_U;//�Һ��ϽǶȵ��5
  Right_Back_Leg_Struct.Jiont_Motor_D=&Joint_RB_D;//�Һ��½Ƕȵ��6
  Right_Front_Leg_Struct.Jiont_Motor_U=&Joint_RF_U;//��ǰ�ϽǶȵ��7
  Right_Front_Leg_Struct.Jiont_Motor_D=&Joint_RF_D;//��ǰ�½Ƕȵ��8


	
	//�ؽڵ��ǿ��PID��ʼ��
  Jiont_Motor_PID_Init();
	//�ֵ��PID��ʼ��
	Wheel_Motor_PID_Init();
	
	
	//�ؽڵ��������ʼ��,�������е����������������Ҫ0.5����ȴ
	Jiont_Motor_FOC_Init(); 
	vTaskDelay(500);//������ϣ�������
	
	
  //�Ȳ���ԭ��
  //Shrink();

  vTaskDelay(100);//������ϣ�������

}


/**
  * @brief  ������Ժ���
  * @param  void
  * @retval void
  * @notes  �������õ��ID�Լ���Ƕ�λ��
  */
 void Motor_Debug(void)
 {
 
   	//�������ģʽ
		//�������
		if(mode==1)
		{
			  CAN1_Send_Cmd(CAN_DATA_CMD_ON,Contron_ID);
			
//				CAN1_Send_Cmd(CAN_DATA_CMD_ON,Contron_ID_2);
//			
//			  CAN1_Send_Cmd(CAN_DATA_CMD_ON,Contron_ID_3);
//			
//		  	CAN1_Send_Cmd(CAN_DATA_CMD_ON,Contron_ID_4);
			
		}
		
		//FOC����
		else if (mode==2)
		{
			switch(Contron_ID)
			{
				 case CAN1_JOINT_ID_LF_U:                                                                                
					{
						CAN1_TX_Jiont(&Joint_LF_U);
			
					}break ;
					
					case CAN1_JOINT_ID_LF_D:                                                                               
					{
						CAN1_TX_Jiont(&Joint_LF_D);
					}break ;
					
				  case CAN1_JOINT_ID_LB_U:                                                                                
					{
						CAN1_TX_Jiont(&Joint_LB_U);
			
					}break ;
					
					case CAN1_JOINT_ID_LB_D:                                                                               
					{
						CAN1_TX_Jiont(&Joint_LB_D);
					}break ;
					
					case CAN1_JOINT_ID_RB_U:                                                                                
					{
						 CAN1_TX_Jiont(&Joint_RB_U);
			
					}break ;
					
					case CAN1_JOINT_ID_RB_D:                                                                               
					{
						CAN1_TX_Jiont(&Joint_RB_D);
						
					}break ;
					
					case CAN1_JOINT_ID_RF_U:                                                                                
					{
						CAN1_TX_Jiont(&Joint_RF_U);
					}break ;
					
					case CAN1_JOINT_ID_RF_D:                                                                               
					{
            CAN1_TX_Jiont(&Joint_RF_D);
						
					}break ;
					

					default:
					{}
					break ;	   

			}
			
		}		
		//��ID
		else if(mode==9)
		{
			 CAN1_Send_Cmd(CAN_DATA_CMD_Change_ID,Contron_ID);
		}
		//��0
		else if(mode==10)
		{
			 CAN1_Send_Cmd(CAN_DATA_CMD_SET_0,Contron_ID);
		}
		else
		{
	  	 CAN1_Send_Cmd(CAN_DATA_CMD_OFF,Contron_ID);
			
//			  CAN1_Send_Cmd(CAN_DATA_CMD_OFF,Contron_ID_4);
//			
//				 CAN1_Send_Cmd(CAN_DATA_CMD_OFF,Contron_ID_2);
//			
//			  CAN1_Send_Cmd(CAN_DATA_CMD_OFF,Contron_ID_3);
			
			
		}
		
 
 
 
 
 }
 

 
 

 
 
 //
 









