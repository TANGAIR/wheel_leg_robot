#include "main.h"


/****************************************���ݶ�����*********************************/
CAN1_Data_TypeDef Joint_LF_U,
                  Joint_LF_D;


									
static int  CAN1_Tx_Flag;                         //CAN������ɱ�־λ




 #define P_MIN -12.5f
 #define P_MAX 12.5f
 #define V_MIN -45.0f
 #define V_MAX 45.0f
 #define KP_MIN 0.0f
 #define KP_MAX 500.0f
 #define KD_MIN 0.0f
 #define KD_MAX 5.0f
 #define T_MIN -18.0f
 #define T_MAX 18.0f
 
 
 

/*****************************************����������********************************/
/**
  * @brief  CANͨ�ų�ʼ������
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
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority  = 9;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority         = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd                 = ENABLE ;
	NVIC_Init(&NVIC_InitStruct);
	
   NVIC_InitStruct.NVIC_IRQChannel                    = CAN1_TX_IRQn;              //ע�����ж�����������  refer to stm32f4xx.h file
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority  = 8;
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
	CAN_InitStruct.CAN_TXFP      = ENABLE;	                                        //���ȼ��ɱ��ı�ʶ������
	CAN_Init(CAN1,&CAN_InitStruct);
	
	CAN_FilterInitStruct.CAN_FilterIdHigh         = 0x0000;                         //32λID
	CAN_FilterInitStruct.CAN_FilterIdLow          = 0x0000;                         //32λID
	CAN_FilterInitStruct.CAN_FilterMaskIdHigh     = 0x0000;
	CAN_FilterInitStruct.CAN_FilterMaskIdLow      = 0x0000;
	CAN_FilterInitStruct.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;               //������0������FIFO0;
	CAN_FilterInitStruct.CAN_FilterNumber         = 0;                              //������0,��Χ0---13
	CAN_FilterInitStruct.CAN_FilterMode           = CAN_FilterMode_IdMask;
	CAN_FilterInitStruct.CAN_FilterScale          = CAN_FilterScale_32bit;
	CAN_FilterInitStruct.CAN_FilterActivation     = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStruct);
	
   CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);                                          //FIFO0��Ϣ�Һ��ж������򿪽����ж�
	CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE);                                           //�򿪷����ж�	
	
	
	//���
	
	
	
	
	
	
	
	
	
	
}












/**
  * @brief  Ħ����CAN1���ͺ���
  * @param  void
  * @retval void
  * @notes  �������ݷ��ͣ��㲥ID 0x200����������������ֵ��Χ��-16384~+16384
  */
CanTxMsg CAN1_Tx_Message;
void CAN1_TX_Jiont (CAN1_Data_TypeDef* motor)
{
	

	CAN1_Tx_Message.IDE = CAN_ID_STD;                                               //��׼֡
	CAN1_Tx_Message.RTR = CAN_RTR_DATA;                                             //����֡
	CAN1_Tx_Message.DLC = 0x08;                                                     //֡����Ϊ8
	CAN1_Tx_Message.StdId = motor->Motor_ID;                                   //֡IDΪ���������CAN_ID


	CAN1_Tx_Message.Data[0] = (float_to_uint(motor->send_Position, P_MIN, P_MAX, 16)>>8)&0xff;             
	CAN1_Tx_Message.Data[1] = (float_to_uint(motor->send_Position, P_MIN, P_MAX, 16))&0xff;                
	CAN1_Tx_Message.Data[2] = (float_to_uint(motor->send_Velocity, V_MIN, V_MAX, 12)>>4)&0xff;           
	CAN1_Tx_Message.Data[3] = (float_to_uint(motor->send_Velocity, V_MIN, V_MAX, 12)<<4)|((float_to_uint(motor->send_kp, KP_MIN, KP_MAX, 12)>>8)&0x0f);                
	CAN1_Tx_Message.Data[4] = (float_to_uint(motor->send_kp, KP_MIN, KP_MAX, 12))&0xff;                     //203���յ�����8λ
	CAN1_Tx_Message.Data[5] = (float_to_uint(motor->send_kd, KD_MIN, KD_MAX, 12)>>4)&0xff;                  //203���յ�����8λ
	CAN1_Tx_Message.Data[6] = (float_to_uint(motor->send_kd, KD_MIN, KD_MAX, 12)<<4)|((float_to_uint(motor->send_torque, T_MIN, T_MAX, 12)>>8)&0x0f);               //204���յ�����8λ
	CAN1_Tx_Message.Data[7] = (float_to_uint(motor->send_torque, T_MIN, T_MAX, 12) )&0xff;                  //204���յ�����8λ
	
	CAN1_Tx_Flag = 0; 
	CAN_Transmit(CAN1,&CAN1_Tx_Message);

	while(CAN1_Tx_Flag == 0); 
}

void CAN1_Enlab ()
{
	CanTxMsg CAN1_Tx_Message;

	CAN1_Tx_Message.IDE = CAN_ID_STD;                                               //��׼֡
	CAN1_Tx_Message.RTR = CAN_RTR_DATA;                                             //����֡
	CAN1_Tx_Message.DLC = 0x08;                                                     //֡����Ϊ8
	CAN1_Tx_Message.StdId = 1;                                                     //֡IDΪ���������CAN_ID


	CAN1_Tx_Message.Data[0] = 0xff;             
	CAN1_Tx_Message.Data[1] = 0xff;                
	CAN1_Tx_Message.Data[2] = 0xff;           
	CAN1_Tx_Message.Data[3] = 0xff;                
	CAN1_Tx_Message.Data[4] = 0xff;               //203���յ�����8λ
	CAN1_Tx_Message.Data[5] = 0xff;                  //203���յ�����8λ
	CAN1_Tx_Message.Data[6] = 0xff;               //204���յ�����8λ
	CAN1_Tx_Message.Data[7] = 0xfc;                  //204���յ�����8λ
	
	CAN1_Tx_Flag = 0; 
	CAN_Transmit(CAN1,&CAN1_Tx_Message);

	while(CAN1_Tx_Flag == 0); 
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

	CanRxMsg CAN1_Rx_Message;

	CAN1_Data_TypeDef  Jiont_Motor_Unpack;
/**
  * @brief  CAN1�����жϺ���
  * @param  void
  * @retval void
  * @notes  ÿ1ms����һ�Σ�������֮��ֱ����̨����PITCH�Ƕȴ���Ϊ�˷�Խ�����䣩��
            ��YAW����д�����Ҫ���ж���̨�Ƿ�������״̬
  */
void CAN1_RX0_IRQHandler(void)                                          
{   

	
	if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET)
	{
		  CAN_Receive(CAN1,CAN_FIFO0,&CAN1_Rx_Message);
		  
			if ( (CAN1_Rx_Message.IDE == CAN_Id_Standard) && (CAN1_Rx_Message.RTR == CAN_RTR_Data) && (CAN1_Rx_Message.DLC == 8) )//��׼֡������֡�����ݳ���Ϊ8�ֽ�
			{	
			  Jiont_Motor_Unpack.Motor_ID = CAN1_Rx_Message.Data[0];
		    Jiont_Motor_Unpack.recieve_Position = (CAN1_Rx_Message.Data[1]>>8)|(CAN1_Rx_Message.Data[2]);
		    Jiont_Motor_Unpack.recieve_Velocity = (CAN1_Rx_Message.Data[3]<<4)|( CAN1_Rx_Message.Data[4]>>4);
		    Jiont_Motor_Unpack.recieve_Current = ((CAN1_Rx_Message.Data[4]&0x0f)<<4)|( CAN1_Rx_Message.Data[5]);
				switch (Jiont_Motor_Unpack.Motor_ID )
				{			
	
					//�ؽ�
          case CAN1_JOINT_ID_LF_U:                                                                                
					{
						Joint_LF_U   =  Jiont_Motor_Unpack; 			//ת��
					}break ;
					
					case CAN1_JOINT_ID_LF_D:                                                                               
					{
						Joint_LF_U   =  Jiont_Motor_Unpack; 		 //ת��	
					}break ;
					

					default:
					{}
					break ;	   
				}
			}
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);		
	}
}







/*********************************************************************************************/
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



