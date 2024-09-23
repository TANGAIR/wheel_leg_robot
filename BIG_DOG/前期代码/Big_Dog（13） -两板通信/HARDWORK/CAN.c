#include "main.h"


/****************************************���ݶ�����*********************************/
static int  CAN1_Tx_Flag;                         //CAN1������ɱ�־λ
static int  CAN2_Tx_Flag;                         //CAN1������ɱ�־λ
//�ؽڵ��CAN���ݽṹ
CAN1_Data_TypeDef  Joint_LF_U,//��ǰ�ϽǶȵ��1
                   Joint_LF_D,//��ǰ�½Ƕȵ��2
									 Joint_LB_U,//����ϽǶȵ��3
                   Joint_LB_D,//����½Ƕȵ��4
									 Joint_RB_U,//�Һ��ϽǶȵ��5
                   Joint_RB_D,//�Һ��½Ƕȵ��6
									 Joint_RF_U,//��ǰ�ϽǶȵ��7
                   Joint_RF_D;//��ǰ�½Ƕȵ��8

//�ֵ��
CAN2_Data_TypeDef  Wheel_LF,//��ǰ��
                   Wheel_LB,//�����
									 Wheel_RB,//�Һ���
									 Wheel_RF;//��ǰ��
		
//�ؽڵ������
 char CAN_DATA_CMD_ON[8]    =       {0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC};
 char CAN_DATA_CMD_OFF[8]   =       {0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD};
 char CAN_DATA_CMD_SET_0[8] =       {0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0X00};
 char CAN_DATA_CMD_Change_ID[8] =   {0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF0,0X06}; //���һλΪID��


/***************************************************************����������*************************************************************/
/*****************************************CAN1********************************/
 /**
  * @brief  CAN1ͨ�ų�ʼ������
  * @param  void
  * @retval void
  * @notes  �����ж����ȼ�Ϊ8�������ж����ȼ�Ϊ9
	  *CAN1 GPIO Configuration    
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX 
    *CAN2 GPIO Configuration    
    PB5     ------> CAN2_RX
    PB6     ------> CAN2_TX 
		
  */
void CAN1_Init(void)
{
	GPIO_InitTypeDef      GPIO_InitStruct;
	NVIC_InitTypeDef      NVIC_InitStruct;
	CAN_InitTypeDef       CAN_InitStruct;
	CAN_FilterInitTypeDef CAN_FilterInitStruct;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_CAN1);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_CAN1);
	
	GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(GPIOD,&GPIO_InitStruct);
	
	NVIC_InitStruct.NVIC_IRQChannel                    = CAN1_RX0_IRQn;             //ע�����ж�����������  refer to stm32f4xx.h file
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority  = 5;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority         = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd                 = ENABLE ;
	NVIC_Init(&NVIC_InitStruct);
	
   NVIC_InitStruct.NVIC_IRQChannel                    = CAN1_TX_IRQn;              //ע�����ж�����������  refer to stm32f4xx.h file
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority  = 5;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority         = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd                 = ENABLE ;
	NVIC_Init(&NVIC_InitStruct);
  
   CAN_DeInit(CAN1);
   CAN_StructInit(&CAN_InitStruct);
    
	CAN_InitStruct.CAN_Prescaler = 3;                                               //CAN BaudRate 42/(1+9+4)/3=1Mbps  //��Ƶϵ��(Fdiv)Ϊ3+1
	CAN_InitStruct.CAN_Mode      = CAN_Mode_Normal;                                 //ģʽ���� 
	CAN_InitStruct.CAN_SJW       = CAN_SJW_1tq;                                     //����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ CAN_SJW_1tq~CAN_SJW_4tq
	CAN_InitStruct.CAN_BS1       = CAN_BS1_9tq;                                     //Tbs1��ΧCAN_BS1_1tq ~CAN_BS1_16tq
	CAN_InitStruct.CAN_BS2       = CAN_BS2_4tq;                                     //Tbs2��ΧCAN_BS2_1tq ~	CAN_BS2_8tq
	CAN_InitStruct.CAN_TTCM      = DISABLE;                                         //��ʱ�䴥��ͨ��ģʽ
	CAN_InitStruct.CAN_ABOM      = DISABLE;                                         //����Զ����߹���	  
	CAN_InitStruct.CAN_AWUM      = DISABLE;                                         //˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
	CAN_InitStruct.CAN_NART      = DISABLE;	                                        //��ֹ�����Զ�����
	CAN_InitStruct.CAN_RFLM      = DISABLE;                                         //���Ĳ�����,�µĸ��Ǿɵ�
	CAN_InitStruct.CAN_TXFP      = DISABLE;	                                        //���ȼ��ɱ��ı�ʶ������
	CAN_Init(CAN1,&CAN_InitStruct);
	


	
	CAN_FilterInitStruct.CAN_FilterIdHigh         = 0x0000;                         //32λID
	CAN_FilterInitStruct.CAN_FilterIdLow          = 0x0000;                         //32λID
	CAN_FilterInitStruct.CAN_FilterMaskIdHigh     = 0x0000;
	CAN_FilterInitStruct.CAN_FilterMaskIdLow      = 0x0000;
	CAN_FilterInitStruct.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;               //������0������FIFO0;
	CAN_FilterInitStruct.CAN_FilterNumber         = 2;                              //������0,��Χ0---13
	CAN_FilterInitStruct.CAN_FilterMode           = CAN_FilterMode_IdMask;
	CAN_FilterInitStruct.CAN_FilterScale          = CAN_FilterScale_32bit;
	CAN_FilterInitStruct.CAN_FilterActivation     = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStruct);
	
   CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);                                          //FIFO0��Ϣ�Һ��ж������򿪽����ж�
	 CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE);                                           //�򿪷����ж�		
	
}

/**
  * @brief  CAN1�����жϺ���
  * @param  void
  * @retval void
  * @notes  void
  */
void CAN1_TX_IRQHandler(void)
{
  if (CAN_GetITStatus (CAN1,CAN_IT_TME)!=RESET)                                 //if transmit mailbox is empty     
	{
		CAN1_Tx_Flag=1;
		CAN_ClearITPendingBit(CAN1,CAN_IT_TME);    
	}
}

/**
  * @brief  �ؽڵ���������
  * @param  void
  * @retval void
  * @notes  
  */
void CAN1_Send_Cmd (char CAN_DATA_CMD[8],char ID)
{
	CanTxMsg CAN1_Tx_Message;

	CAN1_Tx_Message.IDE = CAN_ID_STD;                                               //��ʶ�����ͣ���׼֡
	CAN1_Tx_Message.RTR = CAN_RTR_DATA;                                             //�ܹ����ͣ�����֡
	CAN1_Tx_Message.DLC = 0x08;                                                     //֡����Ϊ8
	CAN1_Tx_Message.StdId = ID ;                                                    //֡IDΪ������Ϣ�ĵ����ID
  CAN1_Tx_Message.ExtId =0;                                                       //��չID

	CAN1_Tx_Message.Data[0] = CAN_DATA_CMD[0];             
	CAN1_Tx_Message.Data[1] = CAN_DATA_CMD[1];                
	CAN1_Tx_Message.Data[2] = CAN_DATA_CMD[2];           
	CAN1_Tx_Message.Data[3] = CAN_DATA_CMD[3];                
	CAN1_Tx_Message.Data[4] = CAN_DATA_CMD[4];               //203���յ�����8λ
	CAN1_Tx_Message.Data[5] = CAN_DATA_CMD[5];                  //203���յ�����8λ
	CAN1_Tx_Message.Data[6] = CAN_DATA_CMD[6];               //204���յ�����8λ
	CAN1_Tx_Message.Data[7] = CAN_DATA_CMD[7];                  //204���յ�����8λ
	
	CAN1_Tx_Flag = 0; 
	CAN_Transmit(CAN1,&CAN1_Tx_Message);

	while(CAN1_Tx_Flag == 0); 
}


/**
  * @brief  �ؽڵ�����Ʒ��ͺ���
  * @param  void
  * @retval void
  * @notes  
  */

void CAN1_TX_Jiont (CAN1_Data_TypeDef* motor)
{
	CanTxMsg CAN1_Tx_Message;
	CAN1_Tx_Message.IDE = CAN_ID_STD;                                               //��׼֡
	CAN1_Tx_Message.RTR = CAN_RTR_DATA;                                             //����֡
	CAN1_Tx_Message.DLC = 0x08;                                                     //֡����Ϊ8
	CAN1_Tx_Message.StdId = motor->Motor_ID;                                        //֡IDΪ���ID
	
	
	
	//����λ�ò��pid,kp��ǿ����
	motor->send.send_kp=BASE_JIONT_KP;//+fabs(Pid_Calc(motor->Motor_PID,motor->recieve .recieve_Position ,motor->send .send_Position));

	//KP����
	if(motor->send.send_kp  >450)motor->send.send_kp=450;
	if(motor->send.send_kp  <=0)motor->send.send_kp=0;

	//λ����λ
	if(motor->send.send_Position >=1.57f)motor->send.send_Position=1.57f;
	if(motor->send.send_Position <=0)motor->send.send_Position=0;

	CAN1_Tx_Message.Data[0] = (float_to_uint(motor->send.send_Position, P_MIN, P_MAX, 16)>>8)&0xff;             
	CAN1_Tx_Message.Data[1] = (float_to_uint(motor->send.send_Position, P_MIN, P_MAX, 16))&0xff;                
	CAN1_Tx_Message.Data[2] = (float_to_uint(motor->send.send_Velocity, V_MIN, V_MAX, 12)>>4)&0xff;           
	CAN1_Tx_Message.Data[3] = (float_to_uint(motor->send.send_Velocity, V_MIN, V_MAX, 12)<<4)|((float_to_uint(motor->send.send_kp, KP_MIN, KP_MAX, 12)>>8)&0x0f);                
	CAN1_Tx_Message.Data[4] = (float_to_uint(motor->send.send_kp, KP_MIN, KP_MAX, 12))&0xff;                     //203���յ�����8λ
	CAN1_Tx_Message.Data[5] = (float_to_uint(motor->send.send_kd, KD_MIN, KD_MAX, 12)>>4)&0xff;                  //203���յ�����8λ
	CAN1_Tx_Message.Data[6] = (float_to_uint(motor->send.send_kd, KD_MIN, KD_MAX, 12)<<4)|((float_to_uint(motor->send.send_torque, T_MIN, T_MAX, 12)>>8)&0x0f);               //204���յ�����8λ
	CAN1_Tx_Message.Data[7] = (float_to_uint(motor->send.send_torque, T_MIN, T_MAX, 12) )&0xff;                  //204���յ�����8λ
	
	CAN1_Tx_Flag = 0; 
	CAN_Transmit(CAN1,&CAN1_Tx_Message);

	while(CAN1_Tx_Flag == 0); 
}









/**
  * @brief  CAN1�����жϺ���
  * @param  void
  * @retval void
  * @notes  ���յ�������ĽǶȡ��ٶȡ�����
  */
	CAN1_Data_TypeDef  Jiont_Motor_Unpack_CAN1;
  CanRxMsg CAN1_Rx_Message;
void CAN1_RX0_IRQHandler(void)                                          
{   

	if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET)
	{
		  CAN_Receive(CAN1,CAN_FIFO0,&CAN1_Rx_Message);
		  
			if ((CAN1_Rx_Message.IDE == CAN_Id_Standard) && (CAN1_Rx_Message.RTR == CAN_RTR_Data) && (CAN1_Rx_Message.DLC == 6) )//��׼֡������֡�����ݳ���Ϊ6�ֽ�
			{	
				//���ղ���
				Jiont_Motor_Unpack_CAN1.Motor_ID                  = CAN1_Rx_Message.Data[0];				
				Jiont_Motor_Unpack_CAN1.recieve.recieve_Position	 = uint_to_float((CAN1_Rx_Message.Data[1]<<8)|(CAN1_Rx_Message.Data[2]), P_MIN, P_MAX, 16);
				Jiont_Motor_Unpack_CAN1.recieve.recieve_Velocity  = uint_to_float((CAN1_Rx_Message.Data[3]<<4)|( CAN1_Rx_Message.Data[4]>>4), V_MIN, V_MAX, 12);
				Jiont_Motor_Unpack_CAN1.recieve.recieve_Current   = uint_to_float(((CAN1_Rx_Message.Data[4]&0x0f)<<4)|( CAN1_Rx_Message.Data[5]), -T_MAX, T_MAX, 12);
       
				switch (Jiont_Motor_Unpack_CAN1.Motor_ID )
				{			
	
					//�ؽ�,��ǰ
          case CAN1_JOINT_ID_LF_U:                                                                                
					{
						Joint_LF_U.recieve   =  Jiont_Motor_Unpack_CAN1.recieve ; 			//�϶�Ӧseigamar
					}break ; 
					
					case CAN1_JOINT_ID_LF_D:                                                                               
					{
						Joint_LF_D.recieve    =  Jiont_Motor_Unpack_CAN1.recieve ; 	 //�¶�Ӧgama
					}break ;
					
				  case CAN1_JOINT_ID_LB_U:                                                                                
					{
						Joint_LB_U.recieve   =  Jiont_Motor_Unpack_CAN1.recieve ; 			
					}break ;
					
					case CAN1_JOINT_ID_LB_D:                                                                               
					{
						Joint_LB_D.recieve    =  Jiont_Motor_Unpack_CAN1.recieve ; 		 
					}break ;
					
					case CAN1_JOINT_ID_RB_U:                                                                                
					{
						Joint_RB_U.recieve   =  Jiont_Motor_Unpack_CAN1.recieve ; 			
					}break ;
					
					case CAN1_JOINT_ID_RB_D:                                                                               
					{
						Joint_RB_D.recieve    =  Jiont_Motor_Unpack_CAN1.recieve ; 		 	
					}break ;
					
					
					case CAN1_JOINT_ID_RF_U:                                                                                
					{
						Joint_RF_U.recieve   =  Jiont_Motor_Unpack_CAN1.recieve ; 			
					}break ;
					
					case CAN1_JOINT_ID_RF_D:                                                                               
					{
						Joint_RF_D.recieve    =  Jiont_Motor_Unpack_CAN1.recieve ; 		
					}break ;
					

					default:
					{}
					break ;	   
				}




			}
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);		
	}
}

/*****************************************CAN2********************************/
/**
  * @brief  CAN2ͨ�ų�ʼ������
  * @param  void
  * @retval void
  * @notes  �����ж����ȼ�Ϊ8�������ж����ȼ�Ϊ
    *CAN2 GPIO Configuration    
    PB5     ------> CAN2_RX
    PB6     ------> CAN2_TX 
		
  */
void CAN2_Init(void)
{
	GPIO_InitTypeDef      GPIO_InitStruct;
	NVIC_InitTypeDef      NVIC_InitStruct;
	CAN_InitTypeDef       CAN_InitStruct;
	CAN_FilterInitTypeDef CAN_FilterInitStruct;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE); //��CAN1 ��CAN2
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_CAN2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_CAN2);
	
	GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(GPIOB,&GPIO_InitStruct);
	
	NVIC_InitStruct.NVIC_IRQChannel                    = CAN2_RX0_IRQn;             //ע�����ж�����������  refer to stm32f4xx.h file
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority  = 7;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority         = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd                 = ENABLE ;
	NVIC_Init(&NVIC_InitStruct);
	
  NVIC_InitStruct.NVIC_IRQChannel                    = CAN2_TX_IRQn;              //ע�����ж�����������  refer to stm32f4xx.h file
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority  = 8;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority         = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd                 = ENABLE ;
	NVIC_Init(&NVIC_InitStruct);
  
  CAN_DeInit(CAN2);
  CAN_StructInit(&CAN_InitStruct);
    
	CAN_InitStruct.CAN_Prescaler = 3;                                               //CAN BaudRate 42/(1+9+4)/3=1Mbps  //��Ƶϵ��(Fdiv)Ϊ3+1
	CAN_InitStruct.CAN_Mode      = CAN_Mode_Normal;                                 //ģʽ���� 
	CAN_InitStruct.CAN_SJW       = CAN_SJW_1tq;                                     //����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ CAN_SJW_1tq~CAN_SJW_4tq
	CAN_InitStruct.CAN_BS1       = CAN_BS1_9tq;                                     //Tbs1��ΧCAN_BS1_1tq ~CAN_BS1_16tq
	CAN_InitStruct.CAN_BS2       = CAN_BS2_4tq;                                     //Tbs2��ΧCAN_BS2_1tq ~	CAN_BS2_8tq
	CAN_InitStruct.CAN_TTCM      = DISABLE;                                         //��ʱ�䴥��ͨ��ģʽ
	CAN_InitStruct.CAN_ABOM      = DISABLE;                                         //����Զ����߹���	  
	CAN_InitStruct.CAN_AWUM      = DISABLE;                                         //˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
	CAN_InitStruct.CAN_NART      = DISABLE;	                                        //��ֹ�����Զ�����
	CAN_InitStruct.CAN_RFLM      = DISABLE;                                         //���Ĳ�����,�µĸ��Ǿɵ�
	CAN_InitStruct.CAN_TXFP      = ENABLE;	                                        //���ȼ��ɱ��ı�ʶ������
	CAN_Init(CAN2,&CAN_InitStruct);
	
	CAN_FilterInitStruct.CAN_FilterIdHigh         = 0x0000;                         //32λID
	CAN_FilterInitStruct.CAN_FilterIdLow          = 0x0000;                         //32λID
	CAN_FilterInitStruct.CAN_FilterMaskIdHigh     = 0x0000;
	CAN_FilterInitStruct.CAN_FilterMaskIdLow      = 0x0000;
	CAN_FilterInitStruct.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;               //������0������FIFO0;
	CAN_FilterInitStruct.CAN_FilterNumber         = 14;                             //CAN2�Ĺ����������޸�Ϊ14����������˽����ж�
	CAN_FilterInitStruct.CAN_FilterMode           = CAN_FilterMode_IdMask;
	CAN_FilterInitStruct.CAN_FilterScale          = CAN_FilterScale_32bit;
	CAN_FilterInitStruct.CAN_FilterActivation     = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStruct);
	
  CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);                                          //FIFO0��Ϣ�Һ��ж������򿪽����ж�
	CAN_ITConfig(CAN2,CAN_IT_TME,ENABLE);                                           //�򿪷����ж�



}


/**
 * @Function : CAN2 send interrupt
 * @Input    : void
 * @Output   : void
 * @Notes    : void
 * @Copyright: Aword
 **/
void CAN2_TX_IRQHandler(void)
{
  if (CAN_GetITStatus (CAN2,CAN_IT_TME)!=RESET)                                 //if transmit mailbox is empty     
	{
		CAN2_Tx_Flag=1;
		CAN_ClearITPendingBit(CAN2,CAN_IT_TME);    
	}
}

/**
 * @Function : CAN2 Shoot send
 * @Input    : void
 * @Output   : void
 * @Notes    : ID 0x200
 * @Copyright: Aword
 **/
void CAN2_TX(void)
{
	
	//pid����
	Wheel_LF.Send_Current=Pid_Calc(&Wheel_Motor_PID_201,Wheel_LF.Real_Speed,-Wheel_LF.Target_Speed);
	Wheel_LB.Send_Current=Pid_Calc(&Wheel_Motor_PID_202,Wheel_LB.Real_Speed,-Wheel_LB.Target_Speed);
  Wheel_RB.Send_Current=Pid_Calc(&Wheel_Motor_PID_203,Wheel_RB.Real_Speed, Wheel_RB.Target_Speed);
	Wheel_RF.Send_Current=Pid_Calc(&Wheel_Motor_PID_204,Wheel_RF.Real_Speed, Wheel_RF.Target_Speed);
	
	
	CanTxMsg CAN2_Tx_Message;

	CAN2_Tx_Message.IDE = CAN_ID_STD;                                               //��׼֡
	CAN2_Tx_Message.RTR = CAN_RTR_DATA;                                             //����֡
	CAN2_Tx_Message.DLC = 0x08;                                                     //֡����Ϊ8
	CAN2_Tx_Message.StdId = CAN2_TRANSMIT_ID;                                //֡IDΪ���������CAN_ID
	
	CAN2_Tx_Message.Data[0] = (Wheel_LF.Send_Current >>8)&0xff;              //201���յ�����8λ
	CAN2_Tx_Message.Data[1] =  Wheel_LF.Send_Current&0xff;                   //201���յ�����8λ
	CAN2_Tx_Message.Data[2] = (Wheel_LB.Send_Current>>8)&0xff;               //202���յ�����8λ
	CAN2_Tx_Message.Data[3] =  Wheel_LB.Send_Current&0xff;                   //202���յ�����8λ
	CAN2_Tx_Message.Data[4] = (Wheel_RB.Send_Current>>8)&0xff;               //203���յ�����8λ
	CAN2_Tx_Message.Data[5] =  Wheel_RB.Send_Current&0xff;                   //203���յ�����8λ
	CAN2_Tx_Message.Data[6] = (Wheel_RF.Send_Current>>8)&0xff;               //204���յ�����8λ
	CAN2_Tx_Message.Data[7] =  Wheel_RF.Send_Current&0xff;                   //204���յ�����8λ
 
	CAN2_Tx_Flag = 0;
	CAN_Transmit(CAN2,&CAN2_Tx_Message);

	while(CAN2_Tx_Flag == 0); 
}



/**
 * @Function : CAN2 receive interrupt
 * @Input    : void
 * @Output   : void
 * @Notes    : void
 * @Copyright: Aword
 **/

	
void CAN2_RX0_IRQHandler(void)            //�ж�������ο���startup_stm32f40_41xxx.s��
{   
  CanRxMsg CAN2_Rx_Message;
	if (CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET)
	{
		CAN_Receive(CAN2,CAN_FIFO0,&CAN2_Rx_Message);
		if ( (CAN2_Rx_Message.IDE == CAN_Id_Standard) && (CAN2_Rx_Message.RTR == CAN_RTR_Data) && (CAN2_Rx_Message.DLC == 8) )//��׼֡������֡�����ݳ���Ϊ8�ֽ�
		{
			switch (CAN2_Rx_Message.StdId)
			{
				case CAN2_LF:     //�ֵ��--��ǰ-->0x201                  
				{
					
					Wheel_LF.Real_Speed=            (CAN2_Rx_Message.Data[2]<<8)|(CAN2_Rx_Message.Data[3]);   //ת��	
				
				}break ;
				
				
				case CAN2_LB:     //�ֵ��--���-->0x202   
				{
					
					Wheel_LB.Real_Speed=          (CAN2_Rx_Message.Data[2]<<8)|(CAN2_Rx_Message.Data[3]);		//ת��
				
				}break ; 
				
				
				case CAN2_RB:      //�ֵ��--�Һ�-->0x203                                                                  
				{
					
					Wheel_RB.Real_Speed          =(CAN2_Rx_Message.Data[2]<<8)|(CAN2_Rx_Message.Data[3]);    //ת��
				
				}break ;
				
				
				case CAN2_RF:      //�ֵ��--��ǰ-->0x204                                                                  
				{
					
					Wheel_RF.Real_Speed          =(CAN2_Rx_Message.Data[2]<<8)|(CAN2_Rx_Message.Data[3]);   //ת��
				
				}break ;
				
				default:
        {}
        break ;
					
			}
	
		}
		CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);		
	}
}




















/*******************************************���ݸ�ʽת������**************************************************/
int float_to_uint(float x, float x_min, float x_max, int bits)
	{
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
    }
    
    
float uint_to_float(int x_int, float x_min, float x_max, int bits)
	{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
    }



