 #include "main.h"  
 
 
uint8_t USART3CAN_start[7]={0x41, 0x54,0x2B,0X41,0X54,0X0D,0X0A};


USART2CAN_Typedef Tx_massage_u3;
USART2CAN_Typedef Rx_massage_u3;

uint8_t   USART3_CAN_Tx[17];
uint8_t   USART3_CAN_Rx[15];
 
 /**
  * @brief  ��ʼ��USART3
  * @param  void
  * @retval void
  * @notes     USART3_TX-PD8      USART3_RX -----PD9
  */
DMA_InitTypeDef    DMA_InitStruct_U3;
void USART3_Init(void)
{
	GPIO_InitTypeDef   GPIO_InitStruct;
	USART_InitTypeDef  USART_InitStruct;
	
	NVIC_InitTypeDef   NVIC_InitStruct;
	
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_DMA1,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_USART3);
	
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_8;
	GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;                                      //�������
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF;                                        //����ģʽ
	GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_NOPULL;                                    //������ 
	GPIO_InitStruct.GPIO_Speed=GPIO_High_Speed;                                    //100MHZ
	GPIO_Init(GPIOD,&GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_9;
	GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;                                      //�������
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF;                                        //����ģʽ
	GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_NOPULL;                                    //������ 
	GPIO_InitStruct.GPIO_Speed=GPIO_High_Speed;                                    //100MHZ
	GPIO_Init(GPIOD,&GPIO_InitStruct);
	
	USART_InitStruct.USART_BaudRate=1000000;
	USART_InitStruct.USART_HardwareFlowControl=USART_HardwareFlowControl_None;     //Ӳ����������
	USART_InitStruct.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;                       //�շ�ģʽ
	USART_InitStruct.USART_Parity=USART_Parity_No;                                 //����żУ��λ
	USART_InitStruct.USART_StopBits=USART_StopBits_1;                              //�ֽ�
	USART_InitStruct.USART_WordLength=USART_WordLength_8b;                         //���ݳ���
	USART_Init(USART3,&USART_InitStruct);
	USART_Cmd(USART3,ENABLE);
	
	
	USART_ITConfig(USART3,USART_IT_IDLE,ENABLE);

/* -------------- Configure NVIC ---------------------------------------*/
//�����ж�
	NVIC_InitStruct.NVIC_IRQChannel                   =USART3_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority =3;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority        =0;
	NVIC_InitStruct.NVIC_IRQChannelCmd                =ENABLE ;
	NVIC_Init(&NVIC_InitStruct);

//���ͺ��ж�
	NVIC_InitStruct.NVIC_IRQChannel                   =DMA1_Stream3_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority =3;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority        =0;
	NVIC_InitStruct.NVIC_IRQChannelCmd                =ENABLE ;
	NVIC_Init(&NVIC_InitStruct);
	
/* ------------------------------------- Configure DMA  ����---------------------------------------*/
  USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);
	
	DMA_Cmd(DMA1_Stream1,DISABLE);
	DMA_DeInit(DMA1_Stream1);
	
	DMA_InitStruct_U3.DMA_Channel=DMA_Channel_4;                                      //ͨ�����ã�ͨ��5��
	DMA_InitStruct_U3.DMA_BufferSize=2;                                              //����������Ŀ                
	DMA_InitStruct_U3.DMA_DIR=DMA_DIR_PeripheralToMemory;                             //���ݴ��䷽�����衪���洢����                   
	DMA_InitStruct_U3.DMA_FIFOMode=ENABLE;                                            //ʹ��FIFOģʽ
	DMA_InitStruct_U3.DMA_FIFOThreshold=DMA_FIFOThreshold_1QuarterFull;               //��ֵ1/4      
	DMA_InitStruct_U3.DMA_Memory0BaseAddr=(uint32_t)USART3_CAN_Rx;                    //�洢�����ݵ�ַ
	DMA_InitStruct_U3.DMA_MemoryBurst=DMA_MemoryBurst_Single;                         //�洢��ͻ������
	DMA_InitStruct_U3.DMA_MemoryDataSize=DMA_MemoryDataSize_Byte;                     //�洢�����ݸ�ʽ���ֽڣ�
	DMA_InitStruct_U3.DMA_MemoryInc=DMA_MemoryInc_Enable;                             //�ڴ��ַ����
	DMA_InitStruct_U3.DMA_Mode=DMA_Mode_Circular;                                     //�Ƿ�ѭ�����ͣ�ѭ������ͨģʽֻ�ܽ���һ�Σ�
	DMA_InitStruct_U3.DMA_PeripheralBaseAddr=(uint32_t)&(USART3->DR);                 //�����ַ 
	DMA_InitStruct_U3.DMA_PeripheralBurst=DMA_PeripheralBurst_Single;                 //����ͻ������         
	DMA_InitStruct_U3.DMA_PeripheralDataSize=DMA_PeripheralDataSize_Byte;             //�������ݸ�ʽ
	DMA_InitStruct_U3.DMA_PeripheralInc=DMA_PeripheralInc_Disable;                    //�����ַ������
	DMA_InitStruct_U3.DMA_Priority=DMA_Priority_VeryHigh;                             //���ȼ��ǳ���      
	DMA_Init(DMA1_Stream1,&DMA_InitStruct_U3);
	DMA_Cmd(DMA1_Stream1,ENABLE);

///*********************************************DMA����************************************/
	
	USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);
	
	DMA_Cmd(DMA1_Stream3,DISABLE);
  DMA_DeInit(DMA1_Stream3); 

//ΪDMA����ͨ��
	DMA_InitStruct_U3.DMA_Channel           =DMA_Channel_4;
	DMA_InitStruct_U3.DMA_PeripheralBaseAddr=(uint32_t)&(USART3->DR);          //��ʼ��ַ
	DMA_InitStruct_U3.DMA_Memory0BaseAddr   =(uint32_t)USART3CAN_start;                 //�洢����
	DMA_InitStruct_U3.DMA_DIR               =DMA_DIR_MemoryToPeripheral;         //���䷽��
	DMA_InitStruct_U3.DMA_BufferSize        =7; //����������
	DMA_InitStruct_U3.DMA_PeripheralInc     =DMA_PeripheralInc_Disable;          //�������ģʽ
	DMA_InitStruct_U3.DMA_MemoryInc         =DMA_MemoryInc_Enable;               //�ڴ����ģʽ
	DMA_InitStruct_U3.DMA_PeripheralDataSize=DMA_PeripheralDataSize_Byte;        //DMA����ʱÿ�β��������ݳ���
	DMA_InitStruct_U3.DMA_MemoryDataSize    =DMA_PeripheralDataSize_Byte;        //DMA����ʱÿ�β��������ݳ���
	DMA_InitStruct_U3.DMA_Mode              =DMA_Mode_Normal;                    //����ģʽ����������
	DMA_InitStruct_U3.DMA_Priority          =DMA_Priority_VeryHigh;              //���ȼ���
  DMA_InitStruct_U3.DMA_FIFOMode			     =DMA_FIFOMode_Enable;
	DMA_InitStruct_U3.DMA_FIFOThreshold     =DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStruct_U3.DMA_MemoryBurst       =DMA_MemoryBurst_Single;
	DMA_InitStruct_U3.DMA_PeripheralBurst   =DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream3,&DMA_InitStruct_U3);
	
	DMA_ITConfig(DMA1_Stream3,DMA_IT_TC,ENABLE);

	delay_ms(3);
	//����CANת��оƬ
	DMA_Cmd(DMA1_Stream3,ENABLE);
}



/**
  * @brief  CAN DMA�����������жϷ�����
  * @param  void
  * @retval void
  * @notes  
            
  */
char DMA_change_flag_U3=0;
void DMA1_Stream3_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_Stream3,DMA_IT_TCIF3)==SET)
	{
	 //�ر�dma������
	 DMA_Cmd(DMA1_Stream3,DISABLE);
		//��Ϊ��һ�η��͵��ǽ���ATģʽָ�����֮����Ҫ�޸ķ������ݵĳ��Ⱥ͵�ַ
		if(DMA_change_flag_U3==0)
		{
	   DMA_InitStruct_U3.DMA_Memory0BaseAddr   =(uint32_t)USART3_CAN_Tx;                 //�洢����
	   DMA_Init(DMA1_Stream3,&DMA_InitStruct_U3);
	   DMA_change_flag_U3 =1;
		}		
	
	 //��������dma���ݳ���
	 DMA_SetCurrDataCounter(DMA1_Stream3,17);
		
		
		
	}
	DMA_ClearITPendingBit(DMA1_Stream3,DMA_IT_TCIF3);	
	
}
/**
  * @brief  ����1�����ж� ������
  * @param  void
  * @retval void
  * @notes  ��ʱû��
  */

void USART3_IRQHandler(void)
{  
	CAN1_Data_TypeDef  Jiont_Motor_Unpack;
	 char num=0;
	if(USART_GetITStatus(USART3,USART_IT_IDLE)==SET)
	{
		 num = USART3->SR;
     num = USART3->DR; //������ڿ����жϣ������ֲ�712ҳ��
		 num++;            //û�����壬ֻ����������
 		
		 DMA_Cmd(DMA1_Stream1,DISABLE); 
		 DMA_SetCurrDataCounter(DMA1_Stream1,15);      //�������ý������ݳ���
		 DMA_Cmd(DMA1_Stream1,ENABLE);                 //���䣬������1���ݼĴ��������ݴ��䵽USART3_CAN_Rx
		
		if((USART3_CAN_Rx[0]==0x41)&&(USART3_CAN_Rx[1]==0x54)&&(USART3_CAN_Rx[13]==0x0d)&&(USART3_CAN_Rx[14]==0x0a))
		{
			 //���ݽ���
			 Rx_massage_u3.Can_ID = bit8TObit32(&USART3_CAN_Rx[2]);
			 Rx_massage_u3.Data_Lenth =  USART3_CAN_Rx[6];
			
			 Rx_massage_u3.Data[0]= USART3_CAN_Rx[7] ;
			 Rx_massage_u3.Data[1]= USART3_CAN_Rx[8] ;
			 Rx_massage_u3.Data[2]= USART3_CAN_Rx[9] ;
			 Rx_massage_u3.Data[3]= USART3_CAN_Rx[10] ;
			 Rx_massage_u3.Data[4]= USART3_CAN_Rx[11] ;
			 Rx_massage_u3.Data[5]= USART3_CAN_Rx[12] ;
			
			
			Jiont_Motor_Unpack.Motor_ID                  = Rx_massage_u3.Data[0];				
			Jiont_Motor_Unpack.recieve.recieve_Position	 = uint_to_float((Rx_massage_u3.Data[1]<<8)|(Rx_massage_u3.Data[2]), P_MIN, P_MAX, 16);
			Jiont_Motor_Unpack.recieve.recieve_Velocity  = uint_to_float((Rx_massage_u3.Data[3]<<4)|( Rx_massage_u3.Data[4]>>4), V_MIN, V_MAX, 12);
			Jiont_Motor_Unpack.recieve.recieve_Current   = uint_to_float(((Rx_massage_u3.Data[4]&0x0f)<<4)|( Rx_massage_u3.Data[5]), -T_MAX, T_MAX, 12);
		 
				switch (Jiont_Motor_Unpack.Motor_ID )
				{			
	
					//�ؽ�,��ǰ
          case CAN1_JOINT_ID_LF_U: //0x01                                                                               
					{
						Joint_LF_U.recieve   =  Jiont_Motor_Unpack.recieve ; 			//�϶�Ӧseigamar
					}break ; 
					
					case CAN1_JOINT_ID_LF_D: //0x02                                                                              
					{
						Joint_LF_D.recieve    =  Jiont_Motor_Unpack.recieve ; 	 //�¶�Ӧgama
					}break ;
					
				  case CAN1_JOINT_ID_LB_U: //0x03                                                                               
					{
						Joint_LB_U.recieve   =  Jiont_Motor_Unpack.recieve ; 			
					}break ;
					
					case CAN1_JOINT_ID_LB_D: //0x04                                                                              
					{
						Joint_LB_D.recieve    =  Jiont_Motor_Unpack.recieve ; 		 
					}break ;
					
					case CAN1_JOINT_ID_RB_U:  //0x05                                                                              
					{
						Joint_RB_U.recieve   =  Jiont_Motor_Unpack.recieve ; 			
					}break ;
					
					case CAN1_JOINT_ID_RB_D:  //0x06                                                                             
					{
						Joint_RB_D.recieve    =  Jiont_Motor_Unpack.recieve ; 		 	
					}break ;
					
					
					case CAN1_JOINT_ID_RF_U:  //0x07                                                                              
					{
						Joint_RF_U.recieve   =  Jiont_Motor_Unpack.recieve ; 			
					}break ;
					
					case CAN1_JOINT_ID_RF_D:  //0x08                                                                             
					{
						Joint_RF_D.recieve    =  Jiont_Motor_Unpack.recieve ; 		
					}break ;
					

					default:
					{}
					break ;	   
				}
		}		
	}	

  ///USART_ClearFlag ( USART3,USART_IT_IDLE  );
}




/**
  * @brief  �ؽڵ���������
  * @param  void
  * @retval void
  * @notes  
  */
void USART32CAN_TX_Send_Cmd (char CAN_DATA_CMD[8],char ID)
{
	                                      
	Tx_massage_u3.Data_Lenth  = 0x08;                                                     //֡����Ϊ8
	Tx_massage_u3.Can_ID  = (ID<<21)&0xfff00000 ;                                                      //֡IDΪ������Ϣ�ĵ����ID
  

	Tx_massage_u3.Data[0] = CAN_DATA_CMD[0];             
	Tx_massage_u3.Data[1] = CAN_DATA_CMD[1];                
	Tx_massage_u3.Data[2] = CAN_DATA_CMD[2];           
	Tx_massage_u3.Data[3] = CAN_DATA_CMD[3];                
	Tx_massage_u3.Data[4] = CAN_DATA_CMD[4];               //203���յ�����8λ
	Tx_massage_u3.Data[5] = CAN_DATA_CMD[5];                  //203���յ�����8λ
	Tx_massage_u3.Data[6] = CAN_DATA_CMD[6];               //204���յ�����8λ
	Tx_massage_u3.Data[7] = CAN_DATA_CMD[7];                  //204���յ�����8λ
	
	 USART3_CAN_Tx[0]=0x41;
	 USART3_CAN_Tx[1]=0x54;
	 USART3_CAN_Tx[2]=bit32TObit8(3,Tx_massage_u3.Can_ID);
	 USART3_CAN_Tx[3]=bit32TObit8(2,Tx_massage_u3.Can_ID);
	 USART3_CAN_Tx[4]=bit32TObit8(1,Tx_massage_u3.Can_ID);
	 USART3_CAN_Tx[5]=bit32TObit8(0,Tx_massage_u3.Can_ID);
	 USART3_CAN_Tx[6]=Tx_massage_u3.Data_Lenth ;
	 USART3_CAN_Tx[7]=  Tx_massage_u3.Data[0]  ;
	 USART3_CAN_Tx[8]=  Tx_massage_u3.Data[1]  ;
	 USART3_CAN_Tx[9]=  Tx_massage_u3.Data[2]  ;
	 USART3_CAN_Tx[10]= Tx_massage_u3.Data[3]  ;
	 USART3_CAN_Tx[11]= Tx_massage_u3.Data[4]  ;
	 USART3_CAN_Tx[12]= Tx_massage_u3.Data[5]  ;
	 USART3_CAN_Tx[13]= Tx_massage_u3.Data[6]  ;
	 USART3_CAN_Tx[14]= Tx_massage_u3.Data[7]  ;
	 USART3_CAN_Tx[15]= 0x0d ;
	 USART3_CAN_Tx[16]= 0x0a ;

	
	
  DMA_Cmd(DMA1_Stream3,ENABLE);
}

/**
  * @brief  �ؽڵ�����Ʒ��ͺ���
  * @param  void
  * @retval void
  * @notes  
  */

void USART32CAN_TX_Jiont (CAN1_Data_TypeDef* motor)
{

	Tx_massage_u3.Data_Lenth = 0x08;                                                                      //֡����Ϊ8
	Tx_massage_u3.Can_ID  = (motor->Motor_ID<<21)&0xfff00000;                                             //֡IDΪ���ID
	
	
	
	//����λ�ò��pid,kp��ǿ����
	motor->send.send_kp=BASE_JIONT_KP;//+fabs(Pid_Calc(motor->Motor_PID,motor->recieve .recieve_Position ,motor->send .send_Position));

	//KP����
	if(motor->send.send_kp  >450)motor->send.send_kp=450;
	if(motor->send.send_kp  <=0)motor->send.send_kp=0;

	//λ����λ
	if(motor->send.send_Position >=1.57f)motor->send.send_Position=1.57f;
	if(motor->send.send_Position <=0)motor->send.send_Position=0;

	Tx_massage_u3.Data[0] = (float_to_uint(motor->send.send_Position, P_MIN, P_MAX, 16)>>8)&0xff;             
	Tx_massage_u3.Data[1] = (float_to_uint(motor->send.send_Position, P_MIN, P_MAX, 16))&0xff;                
	Tx_massage_u3.Data[2] = (float_to_uint(motor->send.send_Velocity, V_MIN, V_MAX, 12)>>4)&0xff;           
	Tx_massage_u3.Data[3] = (float_to_uint(motor->send.send_Velocity, V_MIN, V_MAX, 12)<<4)|((float_to_uint(motor->send.send_kp, KP_MIN, KP_MAX, 12)>>8)&0x0f);                
	Tx_massage_u3.Data[4] = (float_to_uint(motor->send.send_kp, KP_MIN, KP_MAX, 12))&0xff;                     //203���յ�����8λ
	Tx_massage_u3.Data[5] = (float_to_uint(motor->send.send_kd, KD_MIN, KD_MAX, 12)>>4)&0xff;                  //203���յ�����8λ
	Tx_massage_u3.Data[6] = (float_to_uint(motor->send.send_kd, KD_MIN, KD_MAX, 12)<<4)|((float_to_uint(motor->send.send_torque, T_MIN, T_MAX, 12)>>8)&0x0f);               //204���յ�����8λ
	Tx_massage_u3.Data[7] = (float_to_uint(motor->send.send_torque, T_MIN, T_MAX, 12) )&0xff;                  //204���յ�����8λ
	
	 USART3_CAN_Tx[0]=0x41;
	 USART3_CAN_Tx[1]=0x54;
	 USART3_CAN_Tx[2]=bit32TObit8(3,Tx_massage_u3.Can_ID);
	 USART3_CAN_Tx[3]=bit32TObit8(2,Tx_massage_u3.Can_ID);
	 USART3_CAN_Tx[4]=bit32TObit8(1,Tx_massage_u3.Can_ID);
	 USART3_CAN_Tx[5]=bit32TObit8(0,Tx_massage_u3.Can_ID);
	 USART3_CAN_Tx[6]=Tx_massage_u3.Data_Lenth;
	 USART3_CAN_Tx[7]=  Tx_massage_u3.Data[0]  ;
	 USART3_CAN_Tx[8]=  Tx_massage_u3.Data[1]  ;
	 USART3_CAN_Tx[9]=  Tx_massage_u3.Data[2]  ;
	 USART3_CAN_Tx[10]= Tx_massage_u3.Data[3]  ;
	 USART3_CAN_Tx[11]= Tx_massage_u3.Data[4]  ;
	 USART3_CAN_Tx[12]= Tx_massage_u3.Data[5]  ;
	 USART3_CAN_Tx[13]= Tx_massage_u3.Data[6]  ;
	 USART3_CAN_Tx[14]= Tx_massage_u3.Data[7]  ;
	 USART3_CAN_Tx[15]= 0x0d ;
	 USART3_CAN_Tx[16]= 0x0a ;

	
	
  DMA_Cmd(DMA1_Stream3,ENABLE);
}








 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 

