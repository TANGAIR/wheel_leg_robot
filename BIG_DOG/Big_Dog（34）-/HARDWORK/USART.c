#include "main.h"

/**********************************���ݶ�����*****************************/
Send_TX2_Data_Typedef    Send_TX2_Data_TypeStruct;
Receive_TX2_Data_Typedef Receive_TX2_Data_TypeStruct;

char   Usart6Rx_Info[10];//����6��������
char        Send_TX2[10];//����6��������



uint8_t   C_To_F407_Tx[18];
uint8_t   F407_To_C_Rx[18];


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
  * @notes  ��ʼ��    USART1_TX-PA9      USART1_RX -----PB7
  */
	
void USART1_Init(void)
{
	GPIO_InitTypeDef   GPIO_InitStruct;
	USART_InitTypeDef  USART_InitStruct;
		DMA_InitTypeDef    DMA_InitStruct;
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
	
	USART_InitStruct.USART_BaudRate=1000000;
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
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority =2;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority        =0;
	NVIC_InitStruct.NVIC_IRQChannelCmd                =ENABLE ;
	NVIC_Init(&NVIC_InitStruct);
	
/* ------------------------------------- Configure DMA  ����---------------------------------------*/
  USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
	
	DMA_Cmd(DMA2_Stream5,DISABLE);
	DMA_DeInit(DMA2_Stream5);
	
	DMA_InitStruct.DMA_Channel=DMA_Channel_4;                                      //ͨ�����ã�ͨ��5��
	DMA_InitStruct.DMA_BufferSize=18;                                              //����������Ŀ                
	DMA_InitStruct.DMA_DIR=DMA_DIR_PeripheralToMemory;                             //���ݴ��䷽�����衪���洢����                   
	DMA_InitStruct.DMA_FIFOMode=ENABLE;                                            //ʹ��FIFOģʽ
	DMA_InitStruct.DMA_FIFOThreshold=DMA_FIFOThreshold_1QuarterFull;               //��ֵ1/4      
	DMA_InitStruct.DMA_Memory0BaseAddr=(uint32_t)F407_To_C_Rx;                    //�洢�����ݵ�ַ
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
	DMA_InitStruct.DMA_Memory0BaseAddr   =(uint32_t)C_To_F407_Tx;                 //�洢����
	DMA_InitStruct.DMA_DIR               =DMA_DIR_MemoryToPeripheral;         //���䷽��
	DMA_InitStruct.DMA_BufferSize        =18; //����������
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

void DMA2_Stream7_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA2_Stream7,DMA_IT_TCIF7)==SET)
	{
	 //�ر�dma������
	 DMA_Cmd(DMA2_Stream7,DISABLE);
	 //��������dma���ݳ���
	 DMA_SetCurrDataCounter(DMA2_Stream7,18);
	}
	DMA_ClearITPendingBit(DMA2_Stream7,DMA_IT_TCIF7);	
	
}
/**
  * @brief  ����1�����ж� ������
  * @param  void
  * @retval void
  * @notes  ��ʱû��
  */
void USART1_IRQHandler(void)
{
	 char num=0;
	if(USART_GetITStatus(USART1,USART_IT_IDLE)==SET)
	{
		 num = USART1->SR;
     num = USART1->DR; //������ڿ����жϣ������ֲ�712ҳ��
		 num++;            //û�����壬ֻ����������
 		
		 DMA_Cmd(DMA2_Stream5,DISABLE); 
		 DMA_SetCurrDataCounter(DMA2_Stream5,18);      //�������ý������ݳ���
		 DMA_Cmd(DMA2_Stream5,ENABLE);                 //���䣬������1���ݼĴ��������ݴ��䵽USART_CAN_Rx
		
		if((F407_To_C_Rx[0]==0x0A)&&(F407_To_C_Rx[17]==0x0B))
		{
			
			Joint_LF_U.recieve.recieve_Position  =  uint_to_float((F407_To_C_Rx[1]<<8)|(F407_To_C_Rx[2]), P_MIN, P_MAX, 16);  			//�϶�Ӧseigamar
	
			Joint_LF_D.recieve.recieve_Position  =  uint_to_float((F407_To_C_Rx[3]<<8)|(F407_To_C_Rx[4]), P_MIN, P_MAX, 16) ; 	    //�¶�Ӧgama
		
			Joint_LB_U.recieve.recieve_Position  =  uint_to_float((F407_To_C_Rx[5]<<8)|(F407_To_C_Rx[6]), P_MIN, P_MAX, 16) ; 			
		
			Joint_LB_D.recieve.recieve_Position  =  uint_to_float((F407_To_C_Rx[7]<<8)|(F407_To_C_Rx[8]), P_MIN, P_MAX, 16) ; 		 
		
			Joint_RB_U.recieve.recieve_Position  =  uint_to_float((F407_To_C_Rx[9]<<8)|(F407_To_C_Rx[10]), P_MIN, P_MAX, 16) ; 			
		
			Joint_RB_D.recieve.recieve_Position  =  uint_to_float((F407_To_C_Rx[11]<<8)|(F407_To_C_Rx[12]), P_MIN, P_MAX, 16)  ; 		 	
		
			Joint_RF_U.recieve.recieve_Position  =  uint_to_float((F407_To_C_Rx[13]<<8)|(F407_To_C_Rx[14]), P_MIN, P_MAX, 16) ; 			
		
			Joint_RF_D.recieve.recieve_Position  =  uint_to_float((F407_To_C_Rx[15]<<8)|(F407_To_C_Rx[16]), P_MIN, P_MAX, 16) ; 		
			 		
		}		
	}	
}




void Positing_change_Slow(CAN1_Data_TypeDef* Joint_Motor_Data)
{
	 //���յ���������
	if(Joint_Motor_Data->recieve.recieve_Position!=0)
	{
			//���ƽǶȱ仯�ٶ� 
		if(Joint_Motor_Data->send.send_Position >(Joint_Motor_Data->recieve.recieve_Position +SINGLE_POSITION_CHANGE) )
			 Joint_Motor_Data->send.send_Position=  Joint_Motor_Data->recieve.recieve_Position +SINGLE_POSITION_CHANGE;
		
		
			else if (Joint_Motor_Data->send.send_Position <(Joint_Motor_Data->recieve.recieve_Position -SINGLE_POSITION_CHANGE) )
			 Joint_Motor_Data->send.send_Position=  Joint_Motor_Data->recieve.recieve_Position -SINGLE_POSITION_CHANGE;

	}
		

		  //��λ��ֹ��������
	 if(Joint_Motor_Data->send.send_Position>1.5) Joint_Motor_Data->send.send_Position=1.5;
	 if(Joint_Motor_Data->send.send_Position<-1.5f) Joint_Motor_Data->send.send_Position=-1.5;

}




/**
  * @brief  �ؽڵ�����Ʒ��ͺ���
  * @param  void
  * @retval void
  * @notes  
  */

void Contron_TX_Jiont (void)
{
	
	
//	
	Positing_change_Slow(&Joint_LF_U);
	Positing_change_Slow(&Joint_LF_D);
	Positing_change_Slow(&Joint_LB_U);
	Positing_change_Slow(&Joint_LB_D);
//	Positing_change_Slow(&Joint_RB_U);
//	Positing_change_Slow(&Joint_RB_D);
	Positing_change_Slow(&Joint_RF_U);
	Positing_change_Slow(&Joint_RF_D);
//	
	
	
	
	C_To_F407_Tx[0]=0x0A;                                                                                       //����˫��.��������  
	C_To_F407_Tx[1]  = (float_to_uint(Joint_LF_U.send.send_Position, P_MIN, P_MAX, 16)>>8)&0xff;               //1����ǰ������ 
	C_To_F407_Tx[2]  = (float_to_uint(Joint_LF_U.send.send_Position, P_MIN, P_MAX, 16))&0xff;         
	C_To_F407_Tx[3]  = (float_to_uint(Joint_LF_D.send.send_Position, P_MIN, P_MAX, 16)>>8)&0xff;               //2����ǰ���ڵ��
	C_To_F407_Tx[4]  = (float_to_uint(Joint_LF_D.send.send_Position, P_MIN, P_MAX, 16))&0xff;  
	C_To_F407_Tx[5]  = (float_to_uint(Joint_LB_U.send.send_Position, P_MIN, P_MAX, 16)>>8)&0xff;               //3�����������
	C_To_F407_Tx[6]  = (float_to_uint(Joint_LB_U.send.send_Position, P_MIN, P_MAX, 16))&0xff;  
	C_To_F407_Tx[7]  = (float_to_uint(Joint_LB_D.send.send_Position, P_MIN, P_MAX, 16)>>8)&0xff;               //4��������ڵ��
	C_To_F407_Tx[8]  = (float_to_uint(Joint_LB_D.send.send_Position, P_MIN, P_MAX, 16))&0xff;  
	C_To_F407_Tx[9]  = (float_to_uint(Joint_RB_U.send.send_Position, P_MIN, P_MAX, 16)>>8)&0xff;               //5���Һ�������
	C_To_F407_Tx[10] = (float_to_uint(Joint_RB_U.send.send_Position, P_MIN, P_MAX, 16))&0xff;  
	C_To_F407_Tx[11] = (float_to_uint(Joint_RB_D.send.send_Position, P_MIN, P_MAX, 16)>>8)&0xff;               //6���Һ����ڵ��
	C_To_F407_Tx[12] = (float_to_uint(Joint_RB_D.send.send_Position, P_MIN, P_MAX, 16))&0xff;  
	C_To_F407_Tx[13] = (float_to_uint(Joint_RF_U.send.send_Position, P_MIN, P_MAX, 16)>>8)&0xff;               //7����ǰ������
	C_To_F407_Tx[14] = (float_to_uint(Joint_RF_U.send.send_Position, P_MIN, P_MAX, 16))&0xff;                   
	C_To_F407_Tx[15] = (float_to_uint(Joint_RF_D.send.send_Position, P_MIN, P_MAX, 16)>>8)&0xff;               //8����ǰ���ڵ��
	C_To_F407_Tx[16] = (float_to_uint(Joint_RF_D.send.send_Position, P_MIN, P_MAX, 16))&0xff;                 
	C_To_F407_Tx[17] = 0x0B;             

 //����ģʽ���Ҳ�Ϊ��
	if(	DBUS.RC.Switch_Right==RC_SW_DOWN)
	{
		 C_To_F407_Tx[0]=0xCC;
		 C_To_F407_Tx[17] = 0xDD; 
	}
	
  DMA_Cmd(DMA2_Stream7,ENABLE);
}







/*********************************************������TX2����ͨ��**********************************************************************/
/**
  * @brief  USART3-TX2ͨ�ų�ʼ��
  * @param  void
  * @retval void
  * @notes  USART6_RX -----PG9    USART6_TX-----PG14     
            ������DMA2_Stream6��DMA_Channel_5��ʹ��DMA2_Stream6_IRQHandler
            ������DMA2_Stream1��DMA_Channel_5��ʹ��USART6_IRQHandler
  */
void TX2_Init(void)
{
  GPIO_InitTypeDef   GPIO_InitStruct;
	USART_InitTypeDef  USART_InitStruct;
	DMA_InitTypeDef    DMA_InitStruct;
	NVIC_InitTypeDef   NVIC_InitStruct;
	
	/*enabe clocks*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG| RCC_AHB1Periph_DMA2,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);
	
	/*open the alternative function*/
	GPIO_PinAFConfig(GPIOG,GPIO_PinSource14,GPIO_AF_USART6);
	GPIO_PinAFConfig(GPIOG,GPIO_PinSource9,GPIO_AF_USART6);
	
	/*Configure PB10,PB11 as GPIO_InitStruct1 input*/
	GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_9|GPIO_Pin_14;
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOG,&GPIO_InitStruct);
	
	USART_InitStruct.USART_BaudRate            = 115200;
	USART_InitStruct.USART_WordLength          = USART_WordLength_8b;
	USART_InitStruct.USART_StopBits            = USART_StopBits_1;
	USART_InitStruct.USART_Parity              = USART_Parity_No;
	USART_InitStruct.USART_Mode                = USART_Mode_Rx|USART_Mode_Tx;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART6,&USART_InitStruct);
	
	USART_ITConfig(USART6,USART_IT_IDLE,ENABLE);
	
	USART_Cmd(USART6,ENABLE);		
	USART_DMACmd(USART6,USART_DMAReq_Rx,ENABLE);
	
/* -------------- Configure NVIC ---------------------------------------*/
//�����ж�
	NVIC_InitStruct.NVIC_IRQChannel                   =USART6_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority =6;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority        =0;
	NVIC_InitStruct.NVIC_IRQChannelCmd                =ENABLE ;
	NVIC_Init(&NVIC_InitStruct);
//���ͺ��ж�
	NVIC_InitStruct.NVIC_IRQChannel                   =DMA2_Stream6_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority =6;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority        =0;
	NVIC_InitStruct.NVIC_IRQChannelCmd                =ENABLE ;
	NVIC_Init(&NVIC_InitStruct);
	
/* ------------------------------------- Configure DMA  ����---------------------------------------*/

	USART_DMACmd(USART6,USART_DMAReq_Rx,ENABLE);
	
	DMA_Cmd(DMA2_Stream1,DISABLE);
	DMA_DeInit(DMA2_Stream1);
	
	DMA_InitStruct.DMA_Channel=DMA_Channel_5;                                      //ͨ�����ã�ͨ��5��
	DMA_InitStruct.DMA_BufferSize=10;                                              //����������Ŀ                
	DMA_InitStruct.DMA_DIR=DMA_DIR_PeripheralToMemory;                             //���ݴ��䷽�����衪���洢����                   
	DMA_InitStruct.DMA_FIFOMode=ENABLE;                                            //ʹ��FIFOģʽ
	DMA_InitStruct.DMA_FIFOThreshold=DMA_FIFOThreshold_1QuarterFull;               //��ֵ1/4      
	DMA_InitStruct.DMA_Memory0BaseAddr=(uint32_t)Usart6Rx_Info;                    //�洢�����ݵ�ַ
	DMA_InitStruct.DMA_MemoryBurst=DMA_MemoryBurst_Single;                         //�洢��ͻ������
	DMA_InitStruct.DMA_MemoryDataSize=DMA_MemoryDataSize_Byte;                     //�洢�����ݸ�ʽ���ֽڣ�
	DMA_InitStruct.DMA_MemoryInc=DMA_MemoryInc_Enable;                             //�ڴ��ַ����
	DMA_InitStruct.DMA_Mode=DMA_Mode_Circular;                                     //�Ƿ�ѭ�����ͣ�ѭ������ͨģʽֻ�ܽ���һ�Σ�
	DMA_InitStruct.DMA_PeripheralBaseAddr=(uint32_t)&(USART6->DR);                 //�����ַ 
	DMA_InitStruct.DMA_PeripheralBurst=DMA_PeripheralBurst_Single;                 //����ͻ������         
	DMA_InitStruct.DMA_PeripheralDataSize=DMA_PeripheralDataSize_Byte;             //�������ݸ�ʽ
	DMA_InitStruct.DMA_PeripheralInc=DMA_PeripheralInc_Disable;                    //�����ַ������
	DMA_InitStruct.DMA_Priority=DMA_Priority_VeryHigh;                             //���ȼ��ǳ���      
	DMA_Init(DMA2_Stream1,&DMA_InitStruct);
	DMA_Cmd(DMA2_Stream1,ENABLE);

///*********************************************DMA����************************************/
	
	USART_DMACmd(USART6,USART_DMAReq_Tx,ENABLE);
	
	DMA_Cmd(DMA2_Stream6,DISABLE);
  DMA_DeInit(DMA2_Stream6);                                                 //ΪDMA����ͨ��
	DMA_InitStruct.DMA_Channel           =DMA_Channel_5;
	DMA_InitStruct.DMA_PeripheralBaseAddr=(uint32_t)&(	USART6->DR);          //��ʼ��ַ
	DMA_InitStruct.DMA_Memory0BaseAddr   =(uint32_t)Send_TX2;                 //�洢����
	DMA_InitStruct.DMA_DIR               =DMA_DIR_MemoryToPeripheral;         //���䷽��
	DMA_InitStruct.DMA_BufferSize        =10; //����������
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
	DMA_Init(DMA2_Stream6,&DMA_InitStruct);
	
	DMA_ITConfig(DMA2_Stream6,DMA_IT_TC,ENABLE);
	
 
	delay_ms(3);
}
/**
  * @brief  TX2 DMA�����������жϷ�����
  * @param  void
  * @retval void
  * @notes  ÿ����������������֮����������жϷ����������ͺ����������ǻ�ȡ�����б����ã�ÿ1msִ��һ��
            �ж��н�DMA�������رղ�����������DNA���ݳ���
            
  */
void DMA2_Stream6_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA2_Stream6,DMA_IT_TCIF6)==SET)
	{
	 //�ر�dma������
	 DMA_Cmd(DMA2_Stream6,DISABLE);
	 //��������dma���ݳ���
	 DMA_SetCurrDataCounter(DMA2_Stream6,10);
	}
	DMA_ClearITPendingBit(DMA2_Stream6,DMA_IT_TCIF6);	
}
/**
  * @brief  TX2 ����6���ݽ����жϷ�����
  * @param  void
  * @retval void
  * @notes  ���������յ�TX2����������֮���Ƚ�dma�������ر������������ݳ���֮���ٴ�������
            Ȼ��ת��TX2���ݴ�������DMA����USART6->DR��Usart6Rx_Info
  */
void USART6_IRQHandler(void)
{
	
	BaseType_t pxHigherPriorityTaskWoken;
	char num=0;
	if(USART_GetITStatus(USART6,USART_IT_IDLE)==SET)
	{
		 num = USART6->SR;
     num = USART6->DR; //������ڿ����жϣ������ֲ�712ҳ��
		 num++;            //û�����壬ֻ����������
		
		 DMA_Cmd(DMA2_Stream1,DISABLE); 
		 DMA_SetCurrDataCounter(DMA2_Stream1,10);      //�������ý������ݳ���
		 DMA_Cmd(DMA2_Stream1,ENABLE);                 //���䣬������3���ݼĴ��������ݴ��䵽Usart6Rx_Info
		
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
//	DMA_Cmd(DMA2_Stream6,ENABLE);
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
			
			  Usart6Rx_Info[0]            = Usart6Rx_Info[0]&0x0f;
			
			
			
			
//			if( ( Usart6Rx_Info[1] + Usart6Rx_Info[3] + Usart6Rx_Info[5]) % 255 == Usart6Rx_Info[9] ) 
//			{
				Receive_TX2_Data_TypeStruct.Dog_Mode            = Usart6Rx_Info[0];
				Receive_TX2_Data_TypeStruct.Climb_Status        = Usart6Rx_Info[1];
				Receive_TX2_Data_TypeStruct.Front_Leg_Distance  = Usart6Rx_Info[2];
				Receive_TX2_Data_TypeStruct.Stair_Higth         = Usart6Rx_Info[3];
				Receive_TX2_Data_TypeStruct.Stair_Width         = Usart6Rx_Info[4];
				Receive_TX2_Data_TypeStruct.Slide_Speed         = Usart6Rx_Info[5];
				Receive_TX2_Data_TypeStruct.Turn_Angle          = Usart6Rx_Info[6];
				Receive_TX2_Data_TypeStruct.Head_Pitch_Angle    = Usart6Rx_Info[7];
				Receive_TX2_Data_TypeStruct.Head_Yaw_Angle      = Usart6Rx_Info[8];
					
		//	}
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



