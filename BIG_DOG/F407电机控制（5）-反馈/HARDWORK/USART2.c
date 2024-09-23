 #include "main.h"  
 
 
uint8_t USART2CAN_start[7]={0x41, 0x54,0x2B,0X41,0X54,0X0D,0X0A};


USART2CAN_Typedef Tx_massage_u2;
USART2CAN_Typedef Rx_massage_u2;

uint8_t   USART2_CAN_Tx[17];
uint8_t   USART2_CAN_Rx[15];
 
 /**
  * @brief  ��ʼ��USART2
  * @param  void
  * @retval void
  * @notes      USART2_TX-PA2      USART2_RX -----PA3
  */
DMA_InitTypeDef    DMA_InitStruct;
void USART2_Init(void)
{
	GPIO_InitTypeDef   GPIO_InitStruct;
	USART_InitTypeDef  USART_InitStruct;
	
	NVIC_InitTypeDef   NVIC_InitStruct;
	
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_DMA1,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);
	
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_2;
	GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;                                      //�������
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF;                                        //����ģʽ
	GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_NOPULL;                                    //������ 
	GPIO_InitStruct.GPIO_Speed=GPIO_High_Speed;                                    //100MHZ
	GPIO_Init(GPIOA,&GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_3;
	GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;                                      //�������
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF;                                        //����ģʽ
	GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_NOPULL;                                    //������ 
	GPIO_InitStruct.GPIO_Speed=GPIO_High_Speed;                                    //100MHZ
	GPIO_Init(GPIOA,&GPIO_InitStruct);
	
	USART_InitStruct.USART_BaudRate=921600;
	USART_InitStruct.USART_HardwareFlowControl=USART_HardwareFlowControl_None;     //Ӳ����������
	USART_InitStruct.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;                       //�շ�ģʽ
	USART_InitStruct.USART_Parity=USART_Parity_No;                                 //����żУ��λ
	USART_InitStruct.USART_StopBits=USART_StopBits_1;                              //�ֽ�
	USART_InitStruct.USART_WordLength=USART_WordLength_8b;                         //���ݳ���
	USART_Init(USART2,&USART_InitStruct);
	USART_Cmd(USART2,ENABLE);
	
	
	USART_ITConfig(USART2,USART_IT_IDLE,ENABLE);

/* -------------- Configure NVIC ---------------------------------------*/
//�����ж�
	NVIC_InitStruct.NVIC_IRQChannel                   =USART2_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority =3;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority        =0;
	NVIC_InitStruct.NVIC_IRQChannelCmd                =ENABLE ;
	NVIC_Init(&NVIC_InitStruct);

//���ͺ��ж�
	NVIC_InitStruct.NVIC_IRQChannel                   =DMA1_Stream6_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority =3;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority        =0;
	NVIC_InitStruct.NVIC_IRQChannelCmd                =ENABLE ;
	NVIC_Init(&NVIC_InitStruct);
	
/* ------------------------------------- Configure DMA  ����---------------------------------------*/
  USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);
	
	DMA_Cmd(DMA1_Stream5,DISABLE);
	DMA_DeInit(DMA1_Stream5);
	
	DMA_InitStruct.DMA_Channel=DMA_Channel_4;                                      //ͨ�����ã�ͨ��5��
	DMA_InitStruct.DMA_BufferSize=2;                                              //����������Ŀ                
	DMA_InitStruct.DMA_DIR=DMA_DIR_PeripheralToMemory;                             //���ݴ��䷽�����衪���洢����                   
	DMA_InitStruct.DMA_FIFOMode=ENABLE;                                            //ʹ��FIFOģʽ
	DMA_InitStruct.DMA_FIFOThreshold=DMA_FIFOThreshold_1QuarterFull;               //��ֵ1/4      
	DMA_InitStruct.DMA_Memory0BaseAddr=(uint32_t)USART2_CAN_Rx;                    //�洢�����ݵ�ַ
	DMA_InitStruct.DMA_MemoryBurst=DMA_MemoryBurst_Single;                         //�洢��ͻ������
	DMA_InitStruct.DMA_MemoryDataSize=DMA_MemoryDataSize_Byte;                     //�洢�����ݸ�ʽ���ֽڣ�
	DMA_InitStruct.DMA_MemoryInc=DMA_MemoryInc_Enable;                             //�ڴ��ַ����
	DMA_InitStruct.DMA_Mode=DMA_Mode_Circular;                                     //�Ƿ�ѭ�����ͣ�ѭ������ͨģʽֻ�ܽ���һ�Σ�
	DMA_InitStruct.DMA_PeripheralBaseAddr=(uint32_t)&(USART2->DR);                 //�����ַ 
	DMA_InitStruct.DMA_PeripheralBurst=DMA_PeripheralBurst_Single;                 //����ͻ������         
	DMA_InitStruct.DMA_PeripheralDataSize=DMA_PeripheralDataSize_Byte;             //�������ݸ�ʽ
	DMA_InitStruct.DMA_PeripheralInc=DMA_PeripheralInc_Disable;                    //�����ַ������
	DMA_InitStruct.DMA_Priority=DMA_Priority_VeryHigh;                             //���ȼ��ǳ���      
	DMA_Init(DMA1_Stream5,&DMA_InitStruct);
	DMA_Cmd(DMA1_Stream5,ENABLE);

///*********************************************DMA����************************************/
	
	USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);
	
	DMA_Cmd(DMA1_Stream6,DISABLE);
  DMA_DeInit(DMA1_Stream6); 

//ΪDMA����ͨ��
	DMA_InitStruct.DMA_Channel           =DMA_Channel_4;
	DMA_InitStruct.DMA_PeripheralBaseAddr=(uint32_t)&(	USART2->DR);          //��ʼ��ַ
	DMA_InitStruct.DMA_Memory0BaseAddr   =(uint32_t)USART2CAN_start;                 //�洢����
	DMA_InitStruct.DMA_DIR               =DMA_DIR_MemoryToPeripheral;         //���䷽��
	DMA_InitStruct.DMA_BufferSize        =7; //����������
	DMA_InitStruct.DMA_PeripheralInc     =DMA_PeripheralInc_Disable;          //�������ģʽ
	DMA_InitStruct.DMA_MemoryInc         =DMA_MemoryInc_Enable;               //�ڴ����ģʽ
	DMA_InitStruct.DMA_PeripheralDataSize=DMA_PeripheralDataSize_Byte;        //DMA����ʱÿ�β��������ݳ���
	DMA_InitStruct.DMA_MemoryDataSize    =DMA_PeripheralDataSize_Byte;        //DMA����ʱÿ�β��������ݳ���
	DMA_InitStruct.DMA_Mode              =DMA_Mode_Normal;                    //����ģʽ����������
	DMA_InitStruct.DMA_Priority          =DMA_Priority_VeryHigh;              //���ȼ���
  DMA_InitStruct.DMA_FIFOMode			     =DMA_FIFOMode_Enable;
	DMA_InitStruct.DMA_FIFOThreshold     =DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStruct.DMA_MemoryBurst       =DMA_MemoryBurst_Single;
	DMA_InitStruct.DMA_PeripheralBurst   =DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream6,&DMA_InitStruct);
	
	DMA_ITConfig(DMA1_Stream6,DMA_IT_TC,ENABLE);

	delay_ms(3);
	//����CANת��оƬ
	DMA_Cmd(DMA1_Stream6,ENABLE);
}



/**
  * @brief  CAN DMA�����������жϷ�����
  * @param  void
  * @retval void
  * @notes  
            
  */
char DMA_change_flag=0;
void DMA1_Stream6_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_Stream6,DMA_IT_TCIF6)==SET)
	{
	 //�ر�dma������
	 DMA_Cmd(DMA1_Stream6,DISABLE);
		//��Ϊ��һ�η��͵��ǽ���ATģʽָ�����֮����Ҫ�޸ķ������ݵĳ��Ⱥ͵�ַ
		if(DMA_change_flag==0)
		{
	   DMA_InitStruct.DMA_Memory0BaseAddr   =(uint32_t)USART2_CAN_Tx;                 //�洢����
	   DMA_Init(DMA1_Stream6,&DMA_InitStruct);
	   DMA_change_flag =1;
		}		
	
	 //��������dma���ݳ���
	 DMA_SetCurrDataCounter(DMA1_Stream6,17);
		
		
		
	}
	DMA_ClearITPendingBit(DMA1_Stream6,DMA_IT_TCIF6);	
	
}
/**
  * @brief  ����1�����ж� ������
  * @param  void
  * @retval void
  * @notes  ��ʱû��
  */

void USART2_IRQHandler(void)
{  
	CAN1_Data_TypeDef  Jiont_Motor_Unpack;
	 char num=0;
	if(USART_GetITStatus(USART2,USART_IT_IDLE)==SET)
	{
		 num = USART2->SR;
     num = USART2->DR; //������ڿ����жϣ������ֲ�712ҳ��
		 num++;            //û�����壬ֻ����������
 		
		 DMA_Cmd(DMA1_Stream5,DISABLE); 
		 DMA_SetCurrDataCounter(DMA1_Stream5,15);      //�������ý������ݳ���
		 DMA_Cmd(DMA1_Stream5,ENABLE);                 //���䣬������1���ݼĴ��������ݴ��䵽USART2_CAN_Rx
		
		if((USART2_CAN_Rx[0]==0x41)&&(USART2_CAN_Rx[1]==0x54)&&(USART2_CAN_Rx[13]==0x0d)&&(USART2_CAN_Rx[14]==0x0a))
		{
			 //���ݽ���
			 Rx_massage_u2.Can_ID = bit8TObit32(&USART2_CAN_Rx[2]);
			 Rx_massage_u2.Data_Lenth =  USART2_CAN_Rx[6];
			
			 Rx_massage_u2.Data[0]= USART2_CAN_Rx[7] ;
			 Rx_massage_u2.Data[1]= USART2_CAN_Rx[8] ;
			 Rx_massage_u2.Data[2]= USART2_CAN_Rx[9] ;
			 Rx_massage_u2.Data[3]= USART2_CAN_Rx[10] ;
			 Rx_massage_u2.Data[4]= USART2_CAN_Rx[11] ;
			 Rx_massage_u2.Data[5]= USART2_CAN_Rx[12] ;
			
			
			Jiont_Motor_Unpack.Motor_ID                  = Rx_massage_u2.Data[0];				
			Jiont_Motor_Unpack.recieve.recieve_Position	 = uint_to_float((Rx_massage_u2.Data[1]<<8)|(Rx_massage_u2.Data[2]), P_MIN, P_MAX, 16);
			Jiont_Motor_Unpack.recieve.recieve_Velocity  = uint_to_float((Rx_massage_u2.Data[3]<<4)|( Rx_massage_u2.Data[4]>>4), V_MIN, V_MAX, 12);
			Jiont_Motor_Unpack.recieve.recieve_Current   = uint_to_float(((Rx_massage_u2.Data[4]&0x0f)<<4)|( Rx_massage_u2.Data[5]), -T_MAX, T_MAX, 12);
		 
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

  ///USART_ClearFlag ( USART2,USART_IT_IDLE  );
}




/**
  * @brief  �ؽڵ���������
  * @param  void
  * @retval void
  * @notes  
  */
void USART22CAN_TX_Send_Cmd (char CAN_DATA_CMD[8],char ID)
{
	                                      
	Tx_massage_u2.Data_Lenth  = 0x08;                                                     //֡����Ϊ8
	Tx_massage_u2.Can_ID  = (ID<<21)&0xfff00000 ;                                                      //֡IDΪ������Ϣ�ĵ����ID
  

	Tx_massage_u2.Data[0] = CAN_DATA_CMD[0];             
	Tx_massage_u2.Data[1] = CAN_DATA_CMD[1];                
	Tx_massage_u2.Data[2] = CAN_DATA_CMD[2];           
	Tx_massage_u2.Data[3] = CAN_DATA_CMD[3];                
	Tx_massage_u2.Data[4] = CAN_DATA_CMD[4];               //203���յ�����8λ
	Tx_massage_u2.Data[5] = CAN_DATA_CMD[5];                  //203���յ�����8λ
	Tx_massage_u2.Data[6] = CAN_DATA_CMD[6];               //204���յ�����8λ
	Tx_massage_u2.Data[7] = CAN_DATA_CMD[7];                  //204���յ�����8λ
	
	 USART2_CAN_Tx[0]=0x41;
	 USART2_CAN_Tx[1]=0x54;
	 USART2_CAN_Tx[2]=bit32TObit8(3,Tx_massage_u2.Can_ID);
	 USART2_CAN_Tx[3]=bit32TObit8(2,Tx_massage_u2.Can_ID);
	 USART2_CAN_Tx[4]=bit32TObit8(1,Tx_massage_u2.Can_ID);
	 USART2_CAN_Tx[5]=bit32TObit8(0,Tx_massage_u2.Can_ID);
	 USART2_CAN_Tx[6]=Tx_massage_u2.Data_Lenth ;
	 USART2_CAN_Tx[7]=  Tx_massage_u2.Data[0]  ;
	 USART2_CAN_Tx[8]=  Tx_massage_u2.Data[1]  ;
	 USART2_CAN_Tx[9]=  Tx_massage_u2.Data[2]  ;
	 USART2_CAN_Tx[10]= Tx_massage_u2.Data[3]  ;
	 USART2_CAN_Tx[11]= Tx_massage_u2.Data[4]  ;
	 USART2_CAN_Tx[12]= Tx_massage_u2.Data[5]  ;
	 USART2_CAN_Tx[13]= Tx_massage_u2.Data[6]  ;
	 USART2_CAN_Tx[14]= Tx_massage_u2.Data[7]  ;
	 USART2_CAN_Tx[15]= 0x0d ;
	 USART2_CAN_Tx[16]= 0x0a ;

	
	
  DMA_Cmd(DMA1_Stream6,ENABLE);
}

/**
  * @brief  �ؽڵ�����Ʒ��ͺ���
  * @param  void
  * @retval void
  * @notes  
  */

void USART22CAN_TX_Jiont (CAN1_Data_TypeDef* motor)
{

	Tx_massage_u2.Data_Lenth = 0x08;                                                                      //֡����Ϊ8
	Tx_massage_u2.Can_ID  = (motor->Motor_ID<<21)&0xfff00000;                                             //֡IDΪ���ID
	
	
	
	//����λ�ò��pid,kp��ǿ����
	motor->send.send_kp=BASE_JIONT_KP;//+fabs(Pid_Calc(motor->Motor_PID,motor->recieve .recieve_Position ,motor->send .send_Position));

	//KP����
	if(motor->send.send_kp  >450)motor->send.send_kp=450;
	if(motor->send.send_kp  <=0)motor->send.send_kp=0;

	//λ����λ
	if(motor->send.send_Position >=1.57f)motor->send.send_Position=1.57f;
	if(motor->send.send_Position <=-1.57f)motor->send.send_Position=-1.57f;

	Tx_massage_u2.Data[0] = (float_to_uint(motor->send.send_Position, P_MIN, P_MAX, 16)>>8)&0xff;             
	Tx_massage_u2.Data[1] = (float_to_uint(motor->send.send_Position, P_MIN, P_MAX, 16))&0xff;                
	Tx_massage_u2.Data[2] = (float_to_uint(motor->send.send_Velocity, V_MIN, V_MAX, 12)>>4)&0xff;           
	Tx_massage_u2.Data[3] = (float_to_uint(motor->send.send_Velocity, V_MIN, V_MAX, 12)<<4)|((float_to_uint(motor->send.send_kp, KP_MIN, KP_MAX, 12)>>8)&0x0f);                
	Tx_massage_u2.Data[4] = (float_to_uint(motor->send.send_kp, KP_MIN, KP_MAX, 12))&0xff;                     //203���յ�����8λ
	Tx_massage_u2.Data[5] = (float_to_uint(motor->send.send_kd, KD_MIN, KD_MAX, 12)>>4)&0xff;                  //203���յ�����8λ
	Tx_massage_u2.Data[6] = (float_to_uint(motor->send.send_kd, KD_MIN, KD_MAX, 12)<<4)|((float_to_uint(motor->send.send_torque, T_MIN, T_MAX, 12)>>8)&0x0f);               //204���յ�����8λ
	Tx_massage_u2.Data[7] = (float_to_uint(motor->send.send_torque, T_MIN, T_MAX, 12) )&0xff;                  //204���յ�����8λ
	
	

	
	 USART2_CAN_Tx[0]=0x41;
	 USART2_CAN_Tx[1]=0x54;
	 USART2_CAN_Tx[2]=bit32TObit8(3,Tx_massage_u2.Can_ID);
	 USART2_CAN_Tx[3]=bit32TObit8(2,Tx_massage_u2.Can_ID);
	 USART2_CAN_Tx[4]=bit32TObit8(1,Tx_massage_u2.Can_ID);
	 USART2_CAN_Tx[5]=bit32TObit8(0,Tx_massage_u2.Can_ID);
	 USART2_CAN_Tx[6]=Tx_massage_u2.Data_Lenth;
	 USART2_CAN_Tx[7]=  Tx_massage_u2.Data[0]  ;
	 USART2_CAN_Tx[8]=  Tx_massage_u2.Data[1]  ;
	 USART2_CAN_Tx[9]=  Tx_massage_u2.Data[2]  ;
	 USART2_CAN_Tx[10]= Tx_massage_u2.Data[3]  ;
	 USART2_CAN_Tx[11]= Tx_massage_u2.Data[4]  ;
	 USART2_CAN_Tx[12]= Tx_massage_u2.Data[5]  ;
	 USART2_CAN_Tx[13]= Tx_massage_u2.Data[6]  ;
	 USART2_CAN_Tx[14]= Tx_massage_u2.Data[7]  ;
	 USART2_CAN_Tx[15]= 0x0d ;
	 USART2_CAN_Tx[16]= 0x0a ;

	

  DMA_Cmd(DMA1_Stream6,ENABLE);
	
	
}








 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 

