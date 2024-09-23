#include "main.h"
#include "move_control.h"
/*************************************************���ݶ�����*************************************************/
 int mode=0;
 char Contron_ID=2; 

 /*************************************************����������**************************************************/
/**
  * @brief  �ƶ�����������
  * @param  void
  * @retval void
  * @notes  ����ң������ģʽѡ��ͬ�ķ�ʽ
  */
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

	   //��ದ�˲���
	  if(	DBUS.RC.Switch_Left==RC_SW_UP)
		{
      //��
			 Shrink();
//		 signal_leg(&Right_Back_Leg_Struct);
//		 signal_leg(&Left_Back_Leg_Struct);
			//��
			Slide();
		}
		//��ʽǰ��
		else if(DBUS.RC.Switch_Left==RC_SW_DOWN)
		{
			//��
		  Shrink();
			//��
		  Slide();
		}
    //�м��Լ����� ��ͣ
		else 
		{
		  //�Ȳ���ԭ��
	    Shrink();
			//���ٶ���Ϊ0
			Wheel_LF.Target_Speed =0;
			Wheel_LB.Target_Speed =0;
			Wheel_RB.Target_Speed =0;
			Wheel_RF.Target_Speed =0;
			
		} 
		
	//	Motor_Debug();
		//��ͨѶ���
		CAN2_TX();
		//�ؽ����
	  
		
	 Contron_TX_Jiont();
    vTaskDelay(10);
			
	}	

	
	//�������
			

			

	
}


/**
  * @brief  ������Ժ���
  * @param  void
  * @retval void
  * @notes  �������õ��ID�Լ���Ƕ�λ��
  */
char Set_0=0;
 void Motor_Debug(void)
 {
   	//�������ģʽ
		//�������
		if(mode==1)
		{
			  CAN1_Send_Cmd(CAN_DATA_CMD_ON,Contron_ID);
		}
		
		//FOC����
		else if (mode==2)
		{
			switch(Contron_ID)
			{
				 //1
				 case CAN1_JOINT_ID_LF_U:                                                                                
					{
						CAN1_TX_Jiont(&Joint_LF_U);
			
					}break ;
					//2
					case CAN1_JOINT_ID_LF_D:                                                                               
					{
						CAN1_TX_Jiont(&Joint_LF_D);
					}break ;
					//3
				  case CAN1_JOINT_ID_LB_U:                                                                                
					{
						CAN1_TX_Jiont(&Joint_LB_U);
			
					}break ;
					//4
					case CAN1_JOINT_ID_LB_D:                                                                               
					{
						CAN1_TX_Jiont(&Joint_LB_D);
					}break ;
					 //5
					case CAN1_JOINT_ID_RB_U:                                                                                
					{
						 CAN1_TX_Jiont(&Joint_RB_U);
			
					}break ;
					//6
					case CAN1_JOINT_ID_RB_D:                                                                               
					{
						CAN1_TX_Jiont(&Joint_RB_D);
						
					}break ;
					 //7
					case CAN1_JOINT_ID_RF_U:                                                                                
					{
						CAN1_TX_Jiont(&Joint_RF_U);
					}break ;
					 //8
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
			if(Set_0==0)
			{
				 CAN1_Send_Cmd(CAN_DATA_CMD_SET_0,Contron_ID);
				 Set_0=1;
			}
			 
		}
		else
		{
	  	 CAN1_Send_Cmd(CAN_DATA_CMD_OFF,Contron_ID);		
		}

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
  
	Left_Front_Leg_Struct.Leg_Mode   = 1;
	Left_Back_Leg_Struct.Leg_Mode    = 1;
  Right_Back_Leg_Struct.Leg_Mode   = 1;
	Right_Front_Leg_Struct.Leg_Mode  = 1;
	
	
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
 // Shrink();

  vTaskDelay(100);//������ϣ�������

}



 

 










