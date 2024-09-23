#include "main.h"

/**********************************���ݶ�����*****************************/
Send_TX2_Data_Typedef    Send_TX2_Data_TypeStruct;
Receive_TX2_Data_Typedef Receive_TX2_Data_TypeStruct;

char   Usart3Rx_Info[10];//��������������
char        Send_TX2[10];//��������������

uint8_t USART2CAN_start[7]={0x41, 0x54,0x2B,0X41,0X54,0X0D,0X0A};


USART2CAN_Typedef Tx_massage;
USART2CAN_Typedef Rx_massage;

uint8_t   USART_CAN_Tx[17];
uint8_t   USART_CAN_Rx[15];
/**********************************����6��ӡ***************************/
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 
FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
//���ڴ�ӡ����
int fputc(int ch, FILE *f)
{
	while (USART_GetFlagStatus(USART1,USART_FLAG_TC) == RESET);//�ȴ�֮ǰ���ַ��������
	USART_SendData(USART1, (uint8_t)ch);
	return (ch);
}
#endif
/**
  * @brief  ��ʼ��USART1
  * @param  void
  * @retval void
  * @notes  ����6��ӡ��ʼ��    USART1_TX-PA9      USART1_RX -----PB7
  */
		DMA_InitTypeDef    DMA_InitStruct;
void USART1_Init(void)
{
	GPIO_InitTypeDef   GPIO_InitStruct;
	USART_InitTypeDef  USART_InitStruct;
	
	NVIC_InitTypeDef   NVIC_InitStruct;
	
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_DMA2,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_USART1);
	
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_9;
	GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;                                      //�������
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF;                                        //����ģʽ
	GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_NOPULL;                                    //������ 
	GPIO_InitStruct.GPIO_Speed=GPIO_High_Speed;                                    //100MHZ
	GPIO_Init(GPIOA,&GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_7;
	GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;                                      //�������
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF;                                        //����ģʽ
	GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_NOPULL;                                    //������ 
	GPIO_InitStruct.GPIO_Speed=GPIO_High_Speed;                                    //100MHZ
	GPIO_Init(GPIOB,&GPIO_InitStruct);
	
	USART_InitStruct.USART_BaudRate=9600;
	USART_InitStruct.USART_HardwareFlowControl=USART_HardwareFlowControl_None;     //Ӳ����������
	USART_InitStruct.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;                       //�շ�ģʽ
	USART_InitStruct.USART_Parity=USART_Parity_No;                                 //����żУ��λ
	USART_InitStruct.USART_StopBits=USART_StopBits_1;                              //�ֽ�
	USART_InitStruct.USART_WordLength=USART_WordLength_8b;                         //���ݳ���
	USART_Init(USART1,&USART_InitStruct);
	USART_Cmd(USART1,ENABLE);
	
	
	USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);

/* -------------- Configure NVIC ---------------------------------------*/
//�����ж�
	NVIC_InitStruct.NVIC_IRQChannel                   =USART1_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority =5;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority        =0;
	NVIC_InitStruct.NVIC_IRQChannelCmd                =ENABLE ;
	NVIC_Init(&NVIC_InitStruct);

//���ͺ��ж�
	NVIC_InitStruct.NVIC_IRQChannel                   =DMA2_Stream7_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority =5;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority        =0;
	NVIC_InitStruct.NVIC_IRQChannelCmd                =ENABLE ;
	NVIC_Init(&NVIC_InitStruct);
	
/* ------------------------------------- Configure DMA  ����---------------------------------------*/
  USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
	
	DMA_Cmd(DMA2_Stream5,DISABLE);
	DMA_DeInit(DMA2_Stream5);
	
	DMA_InitStruct.DMA_Channel=DMA_Channel_4;                                      //ͨ�����ã�ͨ��5��
	DMA_InitStruct.DMA_BufferSize=2;                                              //����������Ŀ                
	DMA_InitStruct.DMA_DIR=DMA_DIR_PeripheralToMemory;                             //���ݴ��䷽�����衪���洢����                   
	DMA_InitStruct.DMA_FIFOMode=ENABLE;                                            //ʹ��FIFOģʽ
	DMA_InitStruct.DMA_FIFOThreshold=DMA_FIFOThreshold_1QuarterFull;               //��ֵ1/4      
	DMA_InitStruct.DMA_Memory0BaseAddr=(uint32_t)USART_CAN_Rx;                    //�洢�����ݵ�ַ
	DMA_InitStruct.DMA_MemoryBurst=DMA_MemoryBurst_Single;                         //�洢��ͻ������
	DMA_InitStruct.DMA_MemoryDataSize=DMA_MemoryDataSize_Byte;                     //�洢�����ݸ�ʽ���ֽڣ�
	DMA_InitStruct.DMA_MemoryInc=DMA_MemoryInc_Enable;                             //�ڴ��ַ����
	DMA_InitStruct.DMA_Mode=DMA_Mode_Circular;                                     //�Ƿ�ѭ�����ͣ�ѭ������ͨģʽֻ�ܽ���һ�Σ�
	DMA_InitStruct.DMA_PeripheralBaseAddr=(uint32_t)&(USART1->DR);                 //�����ַ 
	DMA_InitStruct.DMA_PeripheralBurst=DMA_PeripheralBurst_Single;                 //����ͻ������         
	DMA_InitStruct.DMA_PeripheralDataSize=DMA_PeripheralDataSize_Byte;             //�������ݸ�ʽ
	DMA_InitStruct.DMA_PeripheralInc=DMA_PeripheralInc_Disable;                    //�����ַ������
	DMA_InitStruct.DMA_Priority=DMA_Priority_VeryHigh;                             //���ȼ��ǳ���      
	DMA_Init(DMA2_Stream5,&DMA_InitStruct);
	DMA_Cmd(DMA2_Stream5,ENABLE);

///*********************************************DMA����************************************/
	
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);
	
	DMA_Cmd(DMA2_Stream7,DISABLE);
  DMA_DeInit(DMA2_Stream7); 

//ΪDMA����ͨ��
	DMA_InitStruct.DMA_Channel           =DMA_Channel_4;
	DMA_InitStruct.DMA_PeripheralBaseAddr=(uint32_t)&(	USART1->DR);          //��ʼ��ַ
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
	DMA_Init(DMA2_Stream7,&DMA_InitStruct);
	
	DMA_ITConfig(DMA2_Stream7,DMA_IT_TC,ENABLE);
	

 
	delay_ms(3);
	//����CANת��оƬ

}



/**
  * @brief  CAN DMA�����������жϷ�����
  * @param  void
  * @retval void
  * @notes  
            
  */
char DMA_change_flag=0;
void DMA2_Stream7_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA2_Stream7,DMA_IT_TCIF7)==SET)
	{
	 //�ر�dma������
	 DMA_Cmd(DMA2_Stream7,DISABLE);
		//��Ϊ��һ�η��͵��ǽ���ATģʽָ�����֮����Ҫ�޸ķ������ݵĳ��Ⱥ͵�ַ
		if(DMA_change_flag==0)
		{
	   DMA_InitStruct.DMA_Memory0BaseAddr   =(uint32_t)USART_CAN_Tx;                 //�洢����
	   DMA_Init(DMA2_Stream7,&DMA_InitStruct);
	   DMA_change_flag =1;
		}		
	
	 //��������dma���ݳ���
	 DMA_SetCurrDataCounter(DMA2_Stream7,17);
		
		
		
	}
	DMA_ClearITPendingBit(DMA2_Stream7,DMA_IT_TCIF7);	
	
}
/**
  * @brief  ����1�����ж� ������
  * @param  void
  * @retval void
  * @notes  ��ʱû��
  */
CAN1_Data_TypeDef  Jiont_Motor_Unpack;
void USART1_IRQHandler(void)
{
	 char num=0;
	if(USART_GetITStatus(USART1,USART_IT_IDLE)==SET)
	{
		 num = USART1->SR;
     num = USART1->DR; //������ڿ����жϣ������ֲ�712ҳ��
		 num++;            //û�����壬ֻ����������
 		
		 DMA_Cmd(DMA2_Stream5,DISABLE); 
		 DMA_SetCurrDataCounter(DMA2_Stream5,15);      //�������ý������ݳ���
		 DMA_Cmd(DMA2_Stream5,ENABLE);                 //���䣬������1���ݼĴ��������ݴ��䵽USART_CAN_Rx
		
		if((USART_CAN_Rx[0]==0x41)&&(USART_CAN_Rx[1]==0x54)&&(USART_CAN_Rx[13]==0x0d)&&(USART_CAN_Rx[14]==0x0a))
		{
			 //���ݽ���
			 Rx_massage.Can_ID = bit8TObit32(&USART_CAN_Rx[2]);
			 Rx_massage.Data_Lenth =  USART_CAN_Rx[6];
			
			 Rx_massage.Data[0]= USART_CAN_Rx[7] ;
			 Rx_massage.Data[1]= USART_CAN_Rx[8] ;
			 Rx_massage.Data[2]= USART_CAN_Rx[9] ;
			 Rx_massage.Data[3]= USART_CAN_Rx[10] ;
			 Rx_massage.Data[4]= USART_CAN_Rx[11] ;
			 Rx_massage.Data[5]= USART_CAN_Rx[12] ;
			
			
			Jiont_Motor_Unpack.Motor_ID                  = Rx_massage.Data[0];				
			Jiont_Motor_Unpack.recieve.recieve_Position	 = uint_to_float((Rx_massage.Data[1]<<8)|(Rx_massage.Data[2]), P_MIN, P_MAX, 16);
			Jiont_Motor_Unpack.recieve.recieve_Velocity  = uint_to_float((Rx_massage.Data[3]<<4)|( Rx_massage.Data[4]>>4), V_MIN, V_MAX, 12);
			Jiont_Motor_Unpack.recieve.recieve_Current   = uint_to_float(((Rx_massage.Data[4]&0x0f)<<4)|( Rx_massage.Data[5]), -T_MAX, T_MAX, 12);
		 
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

  ///USART_ClearFlag ( USART1,USART_IT_IDLE  );
}




/**
  * @brief  �ؽڵ���������
  * @param  void
  * @retval void
  * @notes  
  */
void USART12CAN_TX_Send_Cmd (char CAN_DATA_CMD[8],char ID)
{
	                                      
	Tx_massage.Data_Lenth  = 0x08;                                                     //֡����Ϊ8
	Tx_massage.Can_ID  = (ID<<21)&0xfff00000 ;                                                      //֡IDΪ������Ϣ�ĵ����ID
  

	Tx_massage.Data[0] = CAN_DATA_CMD[0];             
	Tx_massage.Data[1] = CAN_DATA_CMD[1];                
	Tx_massage.Data[2] = CAN_DATA_CMD[2];           
	Tx_massage.Data[3] = CAN_DATA_CMD[3];                
	Tx_massage.Data[4] = CAN_DATA_CMD[4];               //203���յ�����8λ
	Tx_massage.Data[5] = CAN_DATA_CMD[5];                  //203���յ�����8λ
	Tx_massage.Data[6] = CAN_DATA_CMD[6];               //204���յ�����8λ
	Tx_massage.Data[7] = CAN_DATA_CMD[7];                  //204���յ�����8λ
	
	 USART_CAN_Tx[0]=0x41;
	 USART_CAN_Tx[1]=0x54;
	 USART_CAN_Tx[2]=bit32TObit8(3,Tx_massage.Can_ID);
	 USART_CAN_Tx[3]=bit32TObit8(2,Tx_massage.Can_ID);
	 USART_CAN_Tx[4]=bit32TObit8(1,Tx_massage.Can_ID);
	 USART_CAN_Tx[5]=bit32TObit8(0,Tx_massage.Can_ID);
	 USART_CAN_Tx[6]=Tx_massage.Data_Lenth ;
	 USART_CAN_Tx[7]=  Tx_massage.Data[0]  ;
	 USART_CAN_Tx[8]=  Tx_massage.Data[1]  ;
	 USART_CAN_Tx[9]=  Tx_massage.Data[2]  ;
	 USART_CAN_Tx[10]= Tx_massage.Data[3]  ;
	 USART_CAN_Tx[11]= Tx_massage.Data[4]  ;
	 USART_CAN_Tx[12]= Tx_massage.Data[5]  ;
	 USART_CAN_Tx[13]= Tx_massage.Data[6]  ;
	 USART_CAN_Tx[14]= Tx_massage.Data[7]  ;
	 USART_CAN_Tx[15]= 0x0d ;
	 USART_CAN_Tx[16]= 0x0a ;

	
	
  DMA_Cmd(DMA2_Stream7,ENABLE);
}

/**
  * @brief  �ؽڵ�����Ʒ��ͺ���
  * @param  void
  * @retval void
  * @notes  
  */

void USART12CAN_TX_Jiont (CAN1_Data_TypeDef* motor)
{

	Tx_massage.Data_Lenth = 0x08;                                                                      //֡����Ϊ8
	Tx_massage.Can_ID  = (motor->Motor_ID<<21)&0xfff00000;                                             //֡IDΪ���ID
	
	
	
	//����λ�ò��pid,kp��ǿ����
	motor->send.send_kp=BASE_JIONT_KP;//+fabs(Pid_Calc(motor->Motor_PID,motor->recieve .recieve_Position ,motor->send .send_Position));

	//KP����
	if(motor->send.send_kp  >450)motor->send.send_kp=450;
	if(motor->send.send_kp  <=0)motor->send.send_kp=0;

	//λ����λ
	if(motor->send.send_Position >=1.57f)motor->send.send_Position=1.57f;
	if(motor->send.send_Position <=0)motor->send.send_Position=0;

	Tx_massage.Data[0] = (float_to_uint(motor->send.send_Position, P_MIN, P_MAX, 16)>>8)&0xff;             
	Tx_massage.Data[1] = (float_to_uint(motor->send.send_Position, P_MIN, P_MAX, 16))&0xff;                
	Tx_massage.Data[2] = (float_to_uint(motor->send.send_Velocity, V_MIN, V_MAX, 12)>>4)&0xff;           
	Tx_massage.Data[3] = (float_to_uint(motor->send.send_Velocity, V_MIN, V_MAX, 12)<<4)|((float_to_uint(motor->send.send_kp, KP_MIN, KP_MAX, 12)>>8)&0x0f);                
	Tx_massage.Data[4] = (float_to_uint(motor->send.send_kp, KP_MIN, KP_MAX, 12))&0xff;                     //203���յ�����8λ
	Tx_massage.Data[5] = (float_to_uint(motor->send.send_kd, KD_MIN, KD_MAX, 12)>>4)&0xff;                  //203���յ�����8λ
	Tx_massage.Data[6] = (float_to_uint(motor->send.send_kd, KD_MIN, KD_MAX, 12)<<4)|((float_to_uint(motor->send.send_torque, T_MIN, T_MAX, 12)>>8)&0x0f);               //204���յ�����8λ
	Tx_massage.Data[7] = (float_to_uint(motor->send.send_torque, T_MIN, T_MAX, 12) )&0xff;                  //204���յ�����8λ
	
	

	
	 USART_CAN_Tx[0]=0x41;
	 USART_CAN_Tx[1]=0x54;
	 USART_CAN_Tx[2]=bit32TObit8(3,Tx_massage.Can_ID);
	 USART_CAN_Tx[3]=bit32TObit8(2,Tx_massage.Can_ID);
	 USART_CAN_Tx[4]=bit32TObit8(1,Tx_massage.Can_ID);
	 USART_CAN_Tx[5]=bit32TObit8(0,Tx_massage.Can_ID);
	 USART_CAN_Tx[6]=Tx_massage.Data_Lenth;
	 USART_CAN_Tx[7]=  Tx_massage.Data[0]  ;
	 USART_CAN_Tx[8]=  Tx_massage.Data[1]  ;
	 USART_CAN_Tx[9]=  Tx_massage.Data[2]  ;
	 USART_CAN_Tx[10]= Tx_massage.Data[3]  ;
	 USART_CAN_Tx[11]= Tx_massage.Data[4]  ;
	 USART_CAN_Tx[12]= Tx_massage.Data[5]  ;
	 USART_CAN_Tx[13]= Tx_massage.Data[6]  ;
	 USART_CAN_Tx[14]= Tx_massage.Data[7]  ;
	 USART_CAN_Tx[15]= 0x0d ;
	 USART_CAN_Tx[16]= 0x0a ;

	
	
  DMA_Cmd(DMA2_Stream7,ENABLE);
}







/*********************************************������TX2����ͨ��**********************************************************************/
/**
  * @brief  USART3-TX2ͨ�ų�ʼ��
  * @param  void
  * @retval void
  * @notes  USART3_RX -----PD9    USART3_TX-----PD8     
            ������DMA1_Stream3��DMA_Channel_4��ʹ��DMA1_Stream3_IRQHandler
            ������DMA1_Stream1��DMA_Channel_4��ʹ��USART3_IRQHandler
  */
void TX2_Init(void)
{
  GPIO_InitTypeDef   GPIO_InitStruct;
	USART_InitTypeDef  USART_InitStruct;
	DMA_InitTypeDef    DMA_InitStruct;
	NVIC_InitTypeDef   NVIC_InitStruct;
	
	/*enabe clocks*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD| RCC_AHB1Periph_DMA1,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	
	/*open the alternative function*/
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_USART3);
	
	/*Configure PB10,PB11 as GPIO_InitStruct1 input*/
	GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_9|GPIO_Pin_8;
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD,&GPIO_InitStruct);
	
	USART_InitStruct.USART_BaudRate            = 115200;
	USART_InitStruct.USART_WordLength          = USART_WordLength_8b;
	USART_InitStruct.USART_StopBits            = USART_StopBits_1;
	USART_InitStruct.USART_Parity              = USART_Parity_No;
	USART_InitStruct.USART_Mode                = USART_Mode_Rx|USART_Mode_Tx;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART3,&USART_InitStruct);
	
	USART_ITConfig(USART3,USART_IT_IDLE,ENABLE);
	
	USART_Cmd(USART3,ENABLE);		
	USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);
	
/* -------------- Configure NVIC ---------------------------------------*/
//�����ж�
	NVIC_InitStruct.NVIC_IRQChannel                   =USART3_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority =6;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority        =0;
	NVIC_InitStruct.NVIC_IRQChannelCmd                =ENABLE ;
	NVIC_Init(&NVIC_InitStruct);
//���ͺ��ж�
	NVIC_InitStruct.NVIC_IRQChannel                   =DMA1_Stream3_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority =6;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority        =0;
	NVIC_InitStruct.NVIC_IRQChannelCmd                =ENABLE ;
	NVIC_Init(&NVIC_InitStruct);
	
/* ------------------------------------- Configure DMA  ����---------------------------------------*/

	USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);
	
	DMA_Cmd(DMA1_Stream1,DISABLE);
	DMA_DeInit(DMA1_Stream1);
	
	DMA_InitStruct.DMA_Channel=DMA_Channel_4;                                      //ͨ�����ã�ͨ��5��
	DMA_InitStruct.DMA_BufferSize=15;                                              //����������Ŀ                
	DMA_InitStruct.DMA_DIR=DMA_DIR_PeripheralToMemory;                             //���ݴ��䷽�����衪���洢����                   
	DMA_InitStruct.DMA_FIFOMode=ENABLE;                                            //ʹ��FIFOģʽ
	DMA_InitStruct.DMA_FIFOThreshold=DMA_FIFOThreshold_1QuarterFull;               //��ֵ1/4      
	DMA_InitStruct.DMA_Memory0BaseAddr=(uint32_t)Usart3Rx_Info;                    //�洢�����ݵ�ַ
	DMA_InitStruct.DMA_MemoryBurst=DMA_MemoryBurst_Single;                         //�洢��ͻ������
	DMA_InitStruct.DMA_MemoryDataSize=DMA_MemoryDataSize_Byte;                     //�洢�����ݸ�ʽ���ֽڣ�
	DMA_InitStruct.DMA_MemoryInc=DMA_MemoryInc_Enable;                             //�ڴ��ַ����
	DMA_InitStruct.DMA_Mode=DMA_Mode_Circular;                                     //�Ƿ�ѭ�����ͣ�ѭ������ͨģʽֻ�ܽ���һ�Σ�
	DMA_InitStruct.DMA_PeripheralBaseAddr=(uint32_t)&(USART3->DR);                 //�����ַ 
	DMA_InitStruct.DMA_PeripheralBurst=DMA_PeripheralBurst_Single;                 //����ͻ������         
	DMA_InitStruct.DMA_PeripheralDataSize=DMA_PeripheralDataSize_Byte;             //�������ݸ�ʽ
	DMA_InitStruct.DMA_PeripheralInc=DMA_PeripheralInc_Disable;                    //�����ַ������
	DMA_InitStruct.DMA_Priority=DMA_Priority_VeryHigh;                             //���ȼ��ǳ���      
	DMA_Init(DMA1_Stream1,&DMA_InitStruct);
	DMA_Cmd(DMA1_Stream1,ENABLE);

///*********************************************DMA����************************************/
	
	USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);
	
	DMA_Cmd(DMA1_Stream3,DISABLE);
  DMA_DeInit(DMA1_Stream3);                                                 //ΪDMA����ͨ��
	DMA_InitStruct.DMA_Channel           =DMA_Channel_4;
	DMA_InitStruct.DMA_PeripheralBaseAddr=(uint32_t)&(	USART3->DR);          //��ʼ��ַ
	DMA_InitStruct.DMA_Memory0BaseAddr   =(uint32_t)Send_TX2;                 //�洢����
	DMA_InitStruct.DMA_DIR               =DMA_DIR_MemoryToPeripheral;         //���䷽��
	DMA_InitStruct.DMA_BufferSize        =15; //����������
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
	DMA_Init(DMA1_Stream3,&DMA_InitStruct);
	
	DMA_ITConfig(DMA1_Stream3,DMA_IT_TC,ENABLE);
	
 
	delay_ms(3);
}
/**
  * @brief  TX2 DMA�����������жϷ�����
  * @param  void
  * @retval void
  * @notes  ÿ����������������֮����������жϷ����������ͺ����������ǻ�ȡ�����б����ã�ÿ1msִ��һ��
            �ж��н�DMA�������رղ�����������DNA���ݳ���
            
  */
void DMA1_Stream3_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_Stream3,DMA_IT_TCIF3)==SET)
	{
	 //�ر�dma������
	 DMA_Cmd(DMA1_Stream3,DISABLE);
	 //��������dma���ݳ���
	 DMA_SetCurrDataCounter(DMA1_Stream3,10);
	}
	DMA_ClearITPendingBit(DMA1_Stream3,DMA_IT_TCIF3);	
}
/**
  * @brief  TX2 ����3���ݽ����жϷ�����
  * @param  void
  * @retval void
  * @notes  ���������յ�TX2����������֮���Ƚ�dma�������ر������������ݳ���֮���ٴ�������
            Ȼ��ת��TX2���ݴ�������DMA����USART3->DR��Usart3Rx_Info
  */
void USART3_IRQHandler(void)
{
	
	BaseType_t pxHigherPriorityTaskWoken;
	char num=0;
	if(USART_GetITStatus(USART3,USART_IT_IDLE)==SET)
	{
		 num = USART3->SR;
     num = USART3->DR; //������ڿ����жϣ������ֲ�712ҳ��
		 num++;            //û�����壬ֻ����������
		
		 DMA_Cmd(DMA1_Stream1,DISABLE); 
		 DMA_SetCurrDataCounter(DMA1_Stream1,10);      //�������ý������ݳ���
		 DMA_Cmd(DMA1_Stream1,ENABLE);                 //���䣬������3���ݼĴ��������ݴ��䵽Usart3Rx_Info
		
		//��TX2���ݽ���������֪ͨ���������������б�
		 vTaskNotifyGiveFromISR(TX2_Decode_Task_Handler,&pxHigherPriorityTaskWoken);
		//���������л�
		 portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
	}	
}
/******************************���嵽TX2ͨ�����ݷ���****************************
	
  char  Left_Front_Leg_X;  //��ǰ������ڹؽ�����ϵX������,��λCM���Թؽڶ������Ϊԭ�㣬������ǰ������ΪX��������
	char  Left_Front_Leg_Y;  //��ǰ������ڹؽ�����ϵY������,��λCM���Թؽڶ������Ϊԭ�㣬����ΪY��������
	char  Right_Front_Leg_X; //��ǰ������ڹؽ�����ϵX�����꣬��λCM���Թؽڶ������Ϊԭ�㣬������ǰ������ΪX��������
	char  Right_Front_Leg_Y; //��ǰ������ڹؽ�����ϵY�����꣬��λCM���Թؽڶ������Ϊԭ�㣬����ΪY��������
	short IMU_PITCH;         //�����Ǹ�����PITCH����λΪ�㣬G_Y������Ϊ��������Ϊ��
	short IMU_YAW;           //�����Ǻ����YAW����λΪ�㣬G_Z����������Ϊ��������Ϊ��
	char  Left_Front_Wheel_Speed;//��ǰ�ֵ���ٶȣ���λCM/S
	
*************************************************************************************/
/**
  * @brief  ���������ݷ��ͺ���
  * @param  void
  * @retval void
  * @notes  ����̨yaw���pitch������ݾ�������֮�󴢴�Send_TX2������
            Ȼ������dma�����������ݷ��ͳ�ȥ�������������ݽ��뺯���б�����
            ÿ1ms����һ��Send_TX2��Send_TX2���ٷ���
  */
void USART3_Send_TX2(void)
{
	
	Send_TX2_Data_TypeStruct.IMU_PITCH=(short)IMU_Real_Data.Pitch ;
	Send_TX2_Data_TypeStruct.IMU_YAW=(short)IMU_Real_Data.YAW ;
	
	Send_TX2[0]=Send_TX2_Data_TypeStruct.Left_Front_Leg_X ;
	Send_TX2[1]=Send_TX2_Data_TypeStruct.Left_Front_Leg_Y ;
	Send_TX2[2]=Send_TX2_Data_TypeStruct.Right_Front_Leg_X ;
	Send_TX2[3]=Send_TX2_Data_TypeStruct.Right_Front_Leg_Y ;
	Send_TX2[4]=shortTou8(1,Send_TX2_Data_TypeStruct.IMU_PITCH );
	Send_TX2[5]=shortTou8(0,Send_TX2_Data_TypeStruct.IMU_PITCH );
	Send_TX2[6]=shortTou8(1,Send_TX2_Data_TypeStruct.IMU_YAW );
	Send_TX2[7]=shortTou8(0,Send_TX2_Data_TypeStruct.IMU_YAW );
	Send_TX2[8]=Send_TX2_Data_TypeStruct.Left_Front_Wheel_Speed;
	Send_TX2[9]=(Send_TX2[1]+	Send_TX2[3]+Send_TX2[5])%255;
	//��Send_TX2��Send_TX2
	DMA_Cmd(DMA1_Stream3,ENABLE);
}




/*******************************TX2������ͨ�����ݷ���*****************************
	char  Dog_Mode;      //������ģʽ,1����ʽ����ģʽ��2����¥��ģʽ��3����¥��ģʽ��0��ֹͣ
	char  Climb_Status;  //��¥״̬,1��ǰ�����к��Ȼ��У�2��ǰ���ȶ����У�3��ǰ�Ȼ��к�������
	char  Front_Leg_Distance; //��¥ʱ����ǰ�������¥���е��ˮƽ����,��λCM����ǰ������Ϊ�������е�Ϊ0
	char  Stair_Higth;        //¥�ݸ߶ȣ���λCM
	char  Stair_Width;        //¥�ݿ�ȣ���λCM
	char  Slide_Speed;        //ƽ�ػ����ٶȣ���λCM/S����¥ʱΪ0�����ڿ��ƻ���������ģʽʱ�Ļ����ٶ�
	char  Turn_Angle;         //��������ת��,��λ�㣬���ڿ��ƻ�����ת��
	char  Head_Pitch_Angle;   //ͷ���PITCH�Ƕ�,��λ�㣬���ڿ��ƻ�����̧ͷ��ͷ
	char  Head_Yaw_Angle;     //ͷ���YAW�Ƕ�,��λ�㣬���ڿ��ƻ���������תͷ
**********************************************************************************/
/**
  * @brief  TX2���ݽ�������
  * @param  void
  * @retval void
  * @notes  ÿ�ν�����TX2������������ʱִ��һ��
            TX2��ʼ������ʼ������3������TX2��ͨѶ������TX2�������񣬵����ݾ���У��֮��˵��TX2�Ѿ��������ˣ����ʱ��TX2��ʧʱ�����0��
            ��TX2�������������е�װ�װ����Ͳ�Ϊ0��ʱ��˵���Ѿ�ʶ���˵��ˣ���ʱ����̨ģʽ�л�Ϊʶ�𵽵��ˣ�
            �����������װ�װ�������0�Ļ���˵��û�м�⵽װ�װ壬���ʱ����̨����ģʽ�л�Ϊû��ʶ�𵽵���            
  */
void TX2_Decode_Task(void *pvParameters)
{
	uint32_t err;

	TX2_Init();
	vTaskDelay(200);
	while(1)
	{
		//�ȴ�����֪ͨ
		err=ulTaskNotifyTake(pdTRUE,portMAX_DELAY);

		if(err==1)
		{//����У��
			if( ( Usart3Rx_Info[1] + Usart3Rx_Info[3] + Usart3Rx_Info[5]) % 255 == Usart3Rx_Info[9] ) 
			{
				Receive_TX2_Data_TypeStruct.Dog_Mode            = Usart3Rx_Info[0];
				Receive_TX2_Data_TypeStruct.Climb_Status        = Usart3Rx_Info[1];
				Receive_TX2_Data_TypeStruct.Front_Leg_Distance  = Usart3Rx_Info[2];
				Receive_TX2_Data_TypeStruct.Stair_Higth         = Usart3Rx_Info[3];
				Receive_TX2_Data_TypeStruct.Stair_Width         = Usart3Rx_Info[4];
				Receive_TX2_Data_TypeStruct.Slide_Speed         = Usart3Rx_Info[5];
				Receive_TX2_Data_TypeStruct.Turn_Angle          = Usart3Rx_Info[6];
				Receive_TX2_Data_TypeStruct.Head_Pitch_Angle    = Usart3Rx_Info[7];
				Receive_TX2_Data_TypeStruct.Head_Yaw_Angle      = Usart3Rx_Info[8];
					
			}
		}
	}
}

























/*********************************************��������ת������**********************************************************************/

 float bit8TObit32(uint8_t *change_info)
{
	union
	{
    float f;
		char  byte[4];
	}u32val;
    u32val.byte[0]=change_info[0];
    u32val.byte[1]=change_info[1];
    u32val.byte[2]=change_info[2];
    u32val.byte[3]=change_info[3];
	return u32val.f;
}

u8 bit32TObit8(int index_need,int bit32)
{
	union
	{
    int  f;
		u8  byte[4];
	}u32val;
   u32val.f = bit32;
	return u32val.byte[index_need];
}
short u8ToShort(char a[],char b)
{
	union
	{
		short i;
		char byte[2];
	}u16val;
	u16val.byte[0]=a[b];
	u16val.byte[1]=a[b+1];
	return u16val.i;
}

char shortTou8(char bit,short data)
{
	union
	{
		short i;
		char byte[2];
	}u16val;
	u16val.i=data;
	return u16val.byte[bit];
}

int8_t shortToint8(char bit,short data)
{
	union
	{
		short i;
		int8_t byte[2];
	}u16val;
	u16val.i=data;
	return u16val.byte[bit];
}
short int8ToShort(int8_t a[],char b)
{
	union
	{
		short i;
		int8_t byte[2];
	}u16val;
	u16val.byte[0]=a[b];
	u16val.byte[1]=a[b+1];
	return u16val.i;
}

char floatTou8(char bit,float data)
{
	union
	{
		float i;
		char byte[4];
	}u16val;
	u16val.i=data;
	return u16val.byte[bit];
}



