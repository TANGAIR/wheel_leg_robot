 #include "main.h"  
 
 
uint8_t UART4CAN_start[7]={0x41, 0x54,0x2B,0X41,0X54,0X0D,0X0A};


USART2CAN_Typedef Tx_massage_u4;
USART2CAN_Typedef Rx_massage_u4;

uint8_t   UART4_CAN_Tx[17];
uint8_t   UART4_CAN_Rx[15];
 
 /**
  * @brief  ��ʼ��UART4
  * @param  void
  * @retval void
  * @notes      UART4_TX-PC10      UART4_RX -----PC11
  */
DMA_InitTypeDef    DMA_InitStruct_U4;
void USART4_Init(void)
{
	GPIO_InitTypeDef   GPIO_InitStruct;
	USART_InitTypeDef  USART_InitStruct;
	
	NVIC_InitTypeDef   NVIC_InitStruct;
	
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_DMA1,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4);
	
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_10;
	GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;                                      //�������
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF;                                        //����ģʽ
	GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_NOPULL;                                    //������ 
	GPIO_InitStruct.GPIO_Speed=GPIO_High_Speed;                                    //100MHZ
	GPIO_Init(GPIOC,&GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_11;
	GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;                                      //�������
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF;                                        //����ģʽ
	GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_NOPULL;                                    //������ 
	GPIO_InitStruct.GPIO_Speed=GPIO_High_Speed;                                    //100MHZ
	GPIO_Init(GPIOC,&GPIO_InitStruct);
	
	USART_InitStruct.USART_BaudRate=921600;
	USART_InitStruct.USART_HardwareFlowControl=USART_HardwareFlowControl_None;     //Ӳ����������
	USART_InitStruct.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;                       //�շ�ģʽ
	USART_InitStruct.USART_Parity=USART_Parity_No;                                 //����żУ��λ
	USART_InitStruct.USART_StopBits=USART_StopBits_1;                              //�ֽ�
	USART_InitStruct.USART_WordLength=USART_WordLength_8b;                         //���ݳ���
	USART_Init(UART4,&USART_InitStruct);
	USART_Cmd(UART4,ENABLE);
	
	
	USART_ITConfig(UART4,USART_IT_IDLE,ENABLE);

/* -------------- Configure NVIC ---------------------------------------*/
//�����ж�
	NVIC_InitStruct.NVIC_IRQChannel                   =UART4_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority =3;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority        =0;
	NVIC_InitStruct.NVIC_IRQChannelCmd                =ENABLE ;
	NVIC_Init(&NVIC_InitStruct);

//���ͺ��ж�
	NVIC_InitStruct.NVIC_IRQChannel                   =DMA1_Stream4_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority =3;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority        =0;
	NVIC_InitStruct.NVIC_IRQChannelCmd                =ENABLE ;
	NVIC_Init(&NVIC_InitStruct);
	
/* ------------------------------------- Configure DMA  ����---------------------------------------*/
  USART_DMACmd(UART4,USART_DMAReq_Rx,ENABLE);
	
	DMA_Cmd(DMA1_Stream2,DISABLE);
	DMA_DeInit(DMA1_Stream2);
	
	DMA_InitStruct_U4.DMA_Channel=DMA_Channel_4;                                      //ͨ�����ã�ͨ��5��
	DMA_InitStruct_U4.DMA_BufferSize=2;                                              //����������Ŀ                
	DMA_InitStruct_U4.DMA_DIR=DMA_DIR_PeripheralToMemory;                             //���ݴ��䷽�����衪���洢����                   
	DMA_InitStruct_U4.DMA_FIFOMode=ENABLE;                                            //ʹ��FIFOģʽ
	DMA_InitStruct_U4.DMA_FIFOThreshold=DMA_FIFOThreshold_1QuarterFull;               //��ֵ1/4      
	DMA_InitStruct_U4.DMA_Memory0BaseAddr=(uint32_t)UART4_CAN_Rx;                    //�洢�����ݵ�ַ
	DMA_InitStruct_U4.DMA_MemoryBurst=DMA_MemoryBurst_Single;                         //�洢��ͻ������
	DMA_InitStruct_U4.DMA_MemoryDataSize=DMA_MemoryDataSize_Byte;                     //�洢�����ݸ�ʽ���ֽڣ�
	DMA_InitStruct_U4.DMA_MemoryInc=DMA_MemoryInc_Enable;                             //�ڴ��ַ����
	DMA_InitStruct_U4.DMA_Mode=DMA_Mode_Circular;                                     //�Ƿ�ѭ�����ͣ�ѭ������ͨģʽֻ�ܽ���һ�Σ�
	DMA_InitStruct_U4.DMA_PeripheralBaseAddr=(uint32_t)&(UART4->DR);                 //�����ַ 
	DMA_InitStruct_U4.DMA_PeripheralBurst=DMA_PeripheralBurst_Single;                 //����ͻ������         
	DMA_InitStruct_U4.DMA_PeripheralDataSize=DMA_PeripheralDataSize_Byte;             //�������ݸ�ʽ
	DMA_InitStruct_U4.DMA_PeripheralInc=DMA_PeripheralInc_Disable;                    //�����ַ������
	DMA_InitStruct_U4.DMA_Priority=DMA_Priority_VeryHigh;                             //���ȼ��ǳ���      
	DMA_Init(DMA1_Stream2,&DMA_InitStruct_U4);
	DMA_Cmd(DMA1_Stream2,ENABLE);

///*********************************************DMA����************************************/
	
	USART_DMACmd(UART4,USART_DMAReq_Tx,ENABLE);
	
	DMA_Cmd(DMA1_Stream4,DISABLE);
  DMA_DeInit(DMA1_Stream4); 

//ΪDMA����ͨ��
	DMA_InitStruct_U4.DMA_Channel           =DMA_Channel_4;
	DMA_InitStruct_U4.DMA_PeripheralBaseAddr=(uint32_t)&(	UART4->DR);          //��ʼ��ַ
	DMA_InitStruct_U4.DMA_Memory0BaseAddr   =(uint32_t)UART4CAN_start;                 //�洢����
	DMA_InitStruct_U4.DMA_DIR               =DMA_DIR_MemoryToPeripheral;         //���䷽��
	DMA_InitStruct_U4.DMA_BufferSize        =7; //����������
	DMA_InitStruct_U4.DMA_PeripheralInc     =DMA_PeripheralInc_Disable;          //�������ģʽ
	DMA_InitStruct_U4.DMA_MemoryInc         =DMA_MemoryInc_Enable;               //�ڴ����ģʽ
	DMA_InitStruct_U4.DMA_PeripheralDataSize=DMA_PeripheralDataSize_Byte;        //DMA����ʱÿ�β��������ݳ���
	DMA_InitStruct_U4.DMA_MemoryDataSize    =DMA_PeripheralDataSize_Byte;        //DMA����ʱÿ�β��������ݳ���
	DMA_InitStruct_U4.DMA_Mode              =DMA_Mode_Normal;                    //����ģʽ����������
	DMA_InitStruct_U4.DMA_Priority          =DMA_Priority_VeryHigh;              //���ȼ���
  DMA_InitStruct_U4.DMA_FIFOMode			     =DMA_FIFOMode_Enable;
	DMA_InitStruct_U4.DMA_FIFOThreshold     =DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStruct_U4.DMA_MemoryBurst       =DMA_MemoryBurst_Single;
	DMA_InitStruct_U4.DMA_PeripheralBurst   =DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream4,&DMA_InitStruct_U4);
	
	DMA_ITConfig(DMA1_Stream4,DMA_IT_TC,ENABLE);

	delay_ms(3);
	//����CANת��оƬ
	DMA_Cmd(DMA1_Stream4,ENABLE);
}



/**
  * @brief  CAN DMA�����������жϷ�����
  * @param  void
  * @retval void
  * @notes  
            
  */
char DMA_change_flag_U4=0;
void DMA1_Stream4_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_Stream4,DMA_IT_TCIF4)==SET)
	{
	 //�ر�dma������
	 DMA_Cmd(DMA1_Stream4,DISABLE);
		//��Ϊ��һ�η��͵��ǽ���ATģʽָ�����֮����Ҫ�޸ķ������ݵĳ��Ⱥ͵�ַ
		if(DMA_change_flag_U4==0)
		{
	   DMA_InitStruct_U4.DMA_Memory0BaseAddr   =(uint32_t)UART4_CAN_Tx;                 //�洢����
	   DMA_Init(DMA1_Stream4,&DMA_InitStruct_U4);
	   DMA_change_flag_U4 =1;
		}		
	
	 //��������dma���ݳ���
	 DMA_SetCurrDataCounter(DMA1_Stream4,17);
		
		
		
	}
	DMA_ClearITPendingBit(DMA1_Stream4,DMA_IT_TCIF4);	
	
}
/**
  * @brief  ����1�����ж� ������
  * @param  void
  * @retval void
  * @notes  ��ʱû��
  */

void UART4_IRQHandler(void)
{  
	CAN1_Data_TypeDef  Jiont_Motor_Unpack;
	 char num=0;
	if(USART_GetITStatus(UART4,USART_IT_IDLE)==SET)
	{
		 num = UART4->SR;
     num = UART4->DR; //������ڿ����жϣ������ֲ�712ҳ��
		 num++;            //û�����壬ֻ����������
 		
		 DMA_Cmd(DMA1_Stream2,DISABLE); 
		 DMA_SetCurrDataCounter(DMA1_Stream2,15);      //�������ý������ݳ���
		 DMA_Cmd(DMA1_Stream2,ENABLE);                 //���䣬������1���ݼĴ��������ݴ��䵽UART4_CAN_Rx
		
		if((UART4_CAN_Rx[0]==0x41)&&(UART4_CAN_Rx[1]==0x54)&&(UART4_CAN_Rx[13]==0x0d)&&(UART4_CAN_Rx[14]==0x0a))
		{
			 //���ݽ���
			 Rx_massage_u4.Can_ID = bit8TObit32(&UART4_CAN_Rx[2]);
			 Rx_massage_u4.Data_Lenth =  UART4_CAN_Rx[6];
			
			 Rx_massage_u4.Data[0]= UART4_CAN_Rx[7] ;
			 Rx_massage_u4.Data[1]= UART4_CAN_Rx[8] ;
			 Rx_massage_u4.Data[2]= UART4_CAN_Rx[9] ;
			 Rx_massage_u4.Data[3]= UART4_CAN_Rx[10] ;
			 Rx_massage_u4.Data[4]= UART4_CAN_Rx[11] ;
			 Rx_massage_u4.Data[5]= UART4_CAN_Rx[12] ;
			
			
			Jiont_Motor_Unpack.Motor_ID                  = Rx_massage_u4.Data[0];				
			Jiont_Motor_Unpack.recieve.recieve_Position	 = uint_to_float((Rx_massage_u4.Data[1]<<8)|(Rx_massage_u4.Data[2]), P_MIN, P_MAX, 16);
			Jiont_Motor_Unpack.recieve.recieve_Velocity  = uint_to_float((Rx_massage_u4.Data[3]<<4)|( Rx_massage_u4.Data[4]>>4), V_MIN, V_MAX, 12);
			Jiont_Motor_Unpack.recieve.recieve_Current   = uint_to_float(((Rx_massage_u4.Data[4]&0x0f)<<4)|( Rx_massage_u4.Data[5]), -T_MAX, T_MAX, 12);
		 
				switch (Jiont_Motor_Unpack.Motor_ID )
				{			
	
					//�ؽ�,��ǰ
          case CAN1_JOINT_ID_LF_U: //0x01                                                                               
					{
						Joint_LF_U.recieve   =  Jiont_Motor_Unpack.recieve ; 			//�϶�Ӧseigamar
						
						LF_U_Last_Position=Joint_LF_U.recieve.recieve_Position;
						
					}break ; 
					
					case CAN1_JOINT_ID_LF_D: //0x02                                                                              
					{
						Joint_LF_D.recieve    =  Jiont_Motor_Unpack.recieve ; 	 //�¶�Ӧgama
						
						LF_D_Last_Position=Joint_LF_D.recieve.recieve_Position;
					}break ;
					
				  case CAN1_JOINT_ID_LB_U: //0x03                                                                               
					{
						Joint_LB_U.recieve   =  Jiont_Motor_Unpack.recieve ; 	

						LB_U_Last_Position=Joint_LB_U.recieve.recieve_Position;
					}break ;
					
					case CAN1_JOINT_ID_LB_D: //0x04                                                                              
					{
						Joint_LB_D.recieve    =  Jiont_Motor_Unpack.recieve ; 

            LB_D_Last_Position=Joint_LB_D.recieve.recieve_Position;						
					}break ;
					
					case CAN1_JOINT_ID_RB_U:  //0x05                                                                              
					{
						Joint_RB_U.recieve   =  Jiont_Motor_Unpack.recieve ; 
            RB_U_Last_Position=Joint_RB_U.recieve.recieve_Position;						
					}break ;
					
					case CAN1_JOINT_ID_RB_D:  //0x06                                                                             
					{
						Joint_RB_D.recieve    =  Jiont_Motor_Unpack.recieve ; 

						 RB_D_Last_Position=Joint_RB_D.recieve.recieve_Position;
					}break ;
					
					
					case CAN1_JOINT_ID_RF_U:  //0x07                                                                              
					{
						Joint_RF_U.recieve   =  Jiont_Motor_Unpack.recieve ; 
						 RF_U_Last_Position=Joint_RF_U.recieve.recieve_Position;
						
					}break ;
					
					case CAN1_JOINT_ID_RF_D:  //0x08                                                                             
					{
						Joint_RF_D.recieve    =  Jiont_Motor_Unpack.recieve ; 
						 RF_D_Last_Position=Joint_RF_D.recieve.recieve_Position;
						
					}break ;
					

					default:
					{}
					break ;	   
				}
		}		
	}	

  ///USART_ClearFlag ( UART4,USART_IT_IDLE  );
}




/**
  * @brief  �ؽڵ���������
  * @param  void
  * @retval void
  * @notes  
  */
void USART42CAN_TX_Send_Cmd (char CAN_DATA_CMD[8],char ID)
{
	                                      
	Tx_massage_u4.Data_Lenth  = 0x08;                                                     //֡����Ϊ8
	Tx_massage_u4.Can_ID  = (ID<<21)&0xfff00000 ;                                                      //֡IDΪ������Ϣ�ĵ����ID
  

	Tx_massage_u4.Data[0] = CAN_DATA_CMD[0];             
	Tx_massage_u4.Data[1] = CAN_DATA_CMD[1];                
	Tx_massage_u4.Data[2] = CAN_DATA_CMD[2];           
	Tx_massage_u4.Data[3] = CAN_DATA_CMD[3];                
	Tx_massage_u4.Data[4] = CAN_DATA_CMD[4];               //203���յ�����8λ
	Tx_massage_u4.Data[5] = CAN_DATA_CMD[5];                  //203���յ�����8λ
	Tx_massage_u4.Data[6] = CAN_DATA_CMD[6];               //204���յ�����8λ
	Tx_massage_u4.Data[7] = CAN_DATA_CMD[7];                  //204���յ�����8λ
	
	 UART4_CAN_Tx[0]=0x41;
	 UART4_CAN_Tx[1]=0x54;
	 UART4_CAN_Tx[2]=bit32TObit8(3,Tx_massage_u4.Can_ID);
	 UART4_CAN_Tx[3]=bit32TObit8(2,Tx_massage_u4.Can_ID);
	 UART4_CAN_Tx[4]=bit32TObit8(1,Tx_massage_u4.Can_ID);
	 UART4_CAN_Tx[5]=bit32TObit8(0,Tx_massage_u4.Can_ID);
	 UART4_CAN_Tx[6]=Tx_massage_u4.Data_Lenth ;
	 UART4_CAN_Tx[7]=  Tx_massage_u4.Data[0]  ;
	 UART4_CAN_Tx[8]=  Tx_massage_u4.Data[1]  ;
	 UART4_CAN_Tx[9]=  Tx_massage_u4.Data[2]  ;
	 UART4_CAN_Tx[10]= Tx_massage_u4.Data[3]  ;
	 UART4_CAN_Tx[11]= Tx_massage_u4.Data[4]  ;
	 UART4_CAN_Tx[12]= Tx_massage_u4.Data[5]  ;
	 UART4_CAN_Tx[13]= Tx_massage_u4.Data[6]  ;
	 UART4_CAN_Tx[14]= Tx_massage_u4.Data[7]  ;
	 UART4_CAN_Tx[15]= 0x0d ;
	 UART4_CAN_Tx[16]= 0x0a ;

	

  DMA_Cmd(DMA1_Stream4,ENABLE);
	
}

/**
  * @brief  �ؽڵ�����Ʒ��ͺ���
  * @param  void
  * @retval void
  * @notes  
  */

void USART42CAN_TX_Jiont (CAN1_Data_TypeDef* motor)
{

	Tx_massage_u4.Data_Lenth = 0x08;                                                                      //֡����Ϊ8
	Tx_massage_u4.Can_ID  = (motor->Motor_ID<<21)&0xfff00000;                                             //֡IDΪ���ID
	
	
	
	//����λ�ò��pid,kp��ǿ����
	motor->send.send_kp=BASE_JIONT_KP;//+fabs(Pid_Calc(motor->Motor_PID,motor->recieve .recieve_Position ,motor->send .send_Position));

	//KP����
	if(motor->send.send_kp  >450)motor->send.send_kp=450;
	if(motor->send.send_kp  <=0)motor->send.send_kp=0;

	//λ����λ
	if(motor->send.send_Position >=1.57f)motor->send.send_Position=1.57f;
	if(motor->send.send_Position <=-1.57f)motor->send.send_Position=-1.57f;

	Tx_massage_u4.Data[0] = (float_to_uint(motor->send.send_Position, P_MIN, P_MAX, 16)>>8)&0xff;             
	Tx_massage_u4.Data[1] = (float_to_uint(motor->send.send_Position, P_MIN, P_MAX, 16))&0xff;                
	Tx_massage_u4.Data[2] = (float_to_uint(motor->send.send_Velocity, V_MIN, V_MAX, 12)>>4)&0xff;           
	Tx_massage_u4.Data[3] = (float_to_uint(motor->send.send_Velocity, V_MIN, V_MAX, 12)<<4)|((float_to_uint(motor->send.send_kp, KP_MIN, KP_MAX, 12)>>8)&0x0f);                
	Tx_massage_u4.Data[4] = (float_to_uint(motor->send.send_kp, KP_MIN, KP_MAX, 12))&0xff;                     //203���յ�����8λ
	Tx_massage_u4.Data[5] = (float_to_uint(motor->send.send_kd, KD_MIN, KD_MAX, 12)>>4)&0xff;                  //203���յ�����8λ
	Tx_massage_u4.Data[6] = (float_to_uint(motor->send.send_kd, KD_MIN, KD_MAX, 12)<<4)|((float_to_uint(motor->send.send_torque, T_MIN, T_MAX, 12)>>8)&0x0f);               //204���յ�����8λ
	Tx_massage_u4.Data[7] = (float_to_uint(motor->send.send_torque, T_MIN, T_MAX, 12) )&0xff;                  //204���յ�����8λ
	
	

	
	 UART4_CAN_Tx[0]=0x41;
	 UART4_CAN_Tx[1]=0x54;
	 UART4_CAN_Tx[2]=bit32TObit8(3,Tx_massage_u4.Can_ID);
	 UART4_CAN_Tx[3]=bit32TObit8(2,Tx_massage_u4.Can_ID);
	 UART4_CAN_Tx[4]=bit32TObit8(1,Tx_massage_u4.Can_ID);
	 UART4_CAN_Tx[5]=bit32TObit8(0,Tx_massage_u4.Can_ID);
	 UART4_CAN_Tx[6]=Tx_massage_u4.Data_Lenth;
	 UART4_CAN_Tx[7]=  Tx_massage_u4.Data[0]  ;
	 UART4_CAN_Tx[8]=  Tx_massage_u4.Data[1]  ;
	 UART4_CAN_Tx[9]=  Tx_massage_u4.Data[2]  ;
	 UART4_CAN_Tx[10]= Tx_massage_u4.Data[3]  ;
	 UART4_CAN_Tx[11]= Tx_massage_u4.Data[4]  ;
	 UART4_CAN_Tx[12]= Tx_massage_u4.Data[5]  ;
	 UART4_CAN_Tx[13]= Tx_massage_u4.Data[6]  ;
	 UART4_CAN_Tx[14]= Tx_massage_u4.Data[7]  ;
	 UART4_CAN_Tx[15]= 0x0d ;
	 UART4_CAN_Tx[16]= 0x0a ;

	
	
  DMA_Cmd(DMA1_Stream4,ENABLE);
}








 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 

