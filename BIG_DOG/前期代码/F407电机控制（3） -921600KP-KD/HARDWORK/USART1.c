 #include "main.h"  
 
 
 //�ؽڵ��CAN���ݽṹ
CAN1_Data_TypeDef  Joint_LF_U,//��ǰ�ϽǶȵ��1
                   Joint_LF_D,//��ǰ�½Ƕȵ��2
									 Joint_LB_U,//����ϽǶȵ��3
                   Joint_LB_D,//����½Ƕȵ��4
									 Joint_RB_U,//�Һ��ϽǶȵ��5
                   Joint_RB_D,//�Һ��½Ƕȵ��6
									 Joint_RF_U,//��ǰ�ϽǶȵ��7
                   Joint_RF_D;//��ǰ�½Ƕȵ��8


//�ؽڵ������
 char CAN_DATA_CMD_ON[8]    =       {0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC};
 char CAN_DATA_CMD_OFF[8]   =       {0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD};
 char CAN_DATA_CMD_SET_0[8] =       {0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0X00};
 char CAN_DATA_CMD_Change_ID[8] =   {0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF0,0X06}; //���һλΪID��

 
uint8_t   C_To_F407_Rx[18];
uint8_t   F407_To_C_Tx[18]; 
 

/**********************************����1��ʼ��***************************/ 
/**
  * @brief  ��ʼ��USART1�ķ��͡�����6�Ľ���
  * @param  void
  * @retval void
  * @notes  ����6��ӡ��ʼ��    USART1_TX-PA9      USART6_RX -----PG9
  */
	
void USART1_Init(void)
{
	GPIO_InitTypeDef   GPIO_InitStruct;
	USART_InitTypeDef  USART_InitStruct;
	DMA_InitTypeDef    DMA_InitStruct;
	NVIC_InitTypeDef   NVIC_InitStruct;
	
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_DMA2,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOG,GPIO_PinSource9,GPIO_AF_USART6);
	
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_9;
	GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;                                      //�������
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF;                                        //����ģʽ
	GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_NOPULL;                                    //������ 
	GPIO_InitStruct.GPIO_Speed=GPIO_High_Speed;                                    //100MHZ
	GPIO_Init(GPIOA,&GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_9;
	GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;                                      //�������
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF;                                        //����ģʽ
	GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_NOPULL;                                    //������ 
	GPIO_InitStruct.GPIO_Speed=GPIO_High_Speed;                                    //100MHZ
	GPIO_Init(GPIOG,&GPIO_InitStruct);
	
	USART_InitStruct.USART_BaudRate=1000000;
	USART_InitStruct.USART_HardwareFlowControl=USART_HardwareFlowControl_None;     //Ӳ����������
	USART_InitStruct.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;                       //�շ�ģʽ
	USART_InitStruct.USART_Parity=USART_Parity_No;                                 //����żУ��λ
	USART_InitStruct.USART_StopBits=USART_StopBits_1;                              //�ֽ�
	USART_InitStruct.USART_WordLength=USART_WordLength_8b;                         //���ݳ���
	USART_Init(USART1,&USART_InitStruct);
	USART_Cmd(USART1,ENABLE);
	USART_Init(USART6,&USART_InitStruct);
	USART_Cmd(USART6,ENABLE);
	
	USART_ITConfig(USART6,USART_IT_IDLE,ENABLE);

/* -------------- Configure NVIC ---------------------------------------*/
//�����ж�
	NVIC_InitStruct.NVIC_IRQChannel                   =USART6_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority =4;
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
  USART_DMACmd(USART6,USART_DMAReq_Rx,ENABLE);
	
	DMA_Cmd(DMA2_Stream1,DISABLE);
	DMA_DeInit(DMA2_Stream1);
	
	DMA_InitStruct.DMA_Channel=DMA_Channel_5;                                      //ͨ�����ã�ͨ��5��
	DMA_InitStruct.DMA_BufferSize=18;                                              //����������Ŀ                
	DMA_InitStruct.DMA_DIR=DMA_DIR_PeripheralToMemory;                             //���ݴ��䷽�����衪���洢����                   
	DMA_InitStruct.DMA_FIFOMode=ENABLE;                                            //ʹ��FIFOģʽ
	DMA_InitStruct.DMA_FIFOThreshold=DMA_FIFOThreshold_1QuarterFull;               //��ֵ1/4      
	DMA_InitStruct.DMA_Memory0BaseAddr=(uint32_t)C_To_F407_Rx;                    //�洢�����ݵ�ַ
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
	
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);
	
	DMA_Cmd(DMA2_Stream7,DISABLE);
  DMA_DeInit(DMA2_Stream7); 

//ΪDMA����ͨ��
	DMA_InitStruct.DMA_Channel           =DMA_Channel_4;
	DMA_InitStruct.DMA_PeripheralBaseAddr=(uint32_t)&(	USART1->DR);          //��ʼ��ַ
	DMA_InitStruct.DMA_Memory0BaseAddr   =(uint32_t)F407_To_C_Tx;                 //�洢����
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
  * @brief  ����6�����ж� ������
  * @param  void
  * @retval void
  * @notes  ����C�巢���Ŀ�����Ϣ������Ϊ15ms
  */
char   turn_flag=1;
void USART6_IRQHandler(void)
{
	 char num=0;
	if(USART_GetITStatus(USART6,USART_IT_IDLE)==SET)
	{
		 num = USART6->SR;
     num = USART6->DR; //������ڿ����жϣ������ֲ�712ҳ��
		 num++;            //û�����壬ֻ����������
 		
		 DMA_Cmd(DMA2_Stream1,DISABLE); 
		 DMA_SetCurrDataCounter(DMA2_Stream1,18);      //�������ý������ݳ���
		 DMA_Cmd(DMA2_Stream1,ENABLE);                 //���䣬������1���ݼĴ��������ݴ��䵽USART_CAN_Rx
		
		if((C_To_F407_Rx[0]==0x0A)&&(C_To_F407_Rx[17]==0x0B))
		{
			
			 turn_flag=-turn_flag;
			
			Joint_LF_U.send.send_Position  =  uint_to_float((C_To_F407_Rx[1]<<8)|(C_To_F407_Rx[2]), P_MIN, P_MAX, 16);  			//�϶�Ӧseigamar
	
			Joint_LF_D.send.send_Position  =  uint_to_float((C_To_F407_Rx[3]<<8)|(C_To_F407_Rx[4]), P_MIN, P_MAX, 16) ; 	    //�¶�Ӧgama
		
			Joint_LB_U.send.send_Position  =  uint_to_float((C_To_F407_Rx[5]<<8)|(C_To_F407_Rx[6]), P_MIN, P_MAX, 16) ; 			
		
			Joint_LB_D.send.send_Position  =  uint_to_float((C_To_F407_Rx[7]<<8)|(C_To_F407_Rx[8]), P_MIN, P_MAX, 16) ; 		 
		
			Joint_RB_U.send.send_Position  =  uint_to_float((C_To_F407_Rx[9]<<8)|(C_To_F407_Rx[10]), P_MIN, P_MAX, 16) ; 			
		
			Joint_RB_D.send.send_Position  =  uint_to_float((C_To_F407_Rx[11]<<8)|(C_To_F407_Rx[12]), P_MIN, P_MAX, 16)  ; 		 	
		
			Joint_RF_U.send.send_Position  =  uint_to_float((C_To_F407_Rx[13]<<8)|(C_To_F407_Rx[14]), P_MIN, P_MAX, 16) ; 			
		
			Joint_RF_D.send.send_Position  =  uint_to_float((C_To_F407_Rx[15]<<8)|(C_To_F407_Rx[16]), P_MIN, P_MAX, 16) ; 

			 	
      if((Joint_LF_U.send.send_Position!=0)&&(Joint_RF_D.send.send_Position!=0))//��ֹȫ��0	
			{ 
				//ͨ�����ڷ�����Ϣ�����Ƹ���	
				if(turn_flag==1)
				 {
					USART22CAN_TX_Jiont(&Joint_LF_U);   //1
				 
				  USART32CAN_TX_Jiont(&Joint_LB_D);  //4
					 
					USART42CAN_TX_Jiont(&Joint_RB_U);   //5
				 
					USART52CAN_TX_Jiont(&Joint_RF_U);   //7
				
				 }
			   else 
				 { 
					 USART22CAN_TX_Jiont(&Joint_LF_D);  //2
					 
					 USART32CAN_TX_Jiont(&Joint_LB_U);   //3
					 
					 USART42CAN_TX_Jiont(&Joint_RB_D);  //6
					 
					 USART52CAN_TX_Jiont(&Joint_RF_D);  //8
					
				 }	
			}				
				 
			 		
		}


		
	}	
}


/**
  * @brief  �����ʵλ�÷�������
  * @param  void
  * @retval void
  * @notes ���ڽ������������Ϣ���ظ��������� 
  */

void F407_to_C_Send (void)
{
	
	
	F407_To_C_Tx[0]=0x0A;
	F407_To_C_Tx[1]  = (float_to_uint(Joint_LF_U.recieve.recieve_Position, P_MIN, P_MAX, 16)>>8)&0xff;             
	F407_To_C_Tx[2]  = (float_to_uint(Joint_LF_U.recieve.recieve_Position, P_MIN, P_MAX, 16))&0xff;         
	F407_To_C_Tx[3]  = (float_to_uint(Joint_LF_D.recieve.recieve_Position, P_MIN, P_MAX, 16)>>8)&0xff;             
	F407_To_C_Tx[4]  = (float_to_uint(Joint_LF_D.recieve.recieve_Position, P_MIN, P_MAX, 16))&0xff;  
	F407_To_C_Tx[5]  = (float_to_uint(Joint_LB_U.recieve.recieve_Position, P_MIN, P_MAX, 16)>>8)&0xff;             
	F407_To_C_Tx[6]  = (float_to_uint(Joint_LB_U.recieve.recieve_Position, P_MIN, P_MAX, 16))&0xff;  
	F407_To_C_Tx[7]  = (float_to_uint(Joint_LB_D.recieve.recieve_Position, P_MIN, P_MAX, 16)>>8)&0xff;             
	F407_To_C_Tx[8]  = (float_to_uint(Joint_LB_D.recieve.recieve_Position, P_MIN, P_MAX, 16))&0xff;  
	F407_To_C_Tx[9]  = (float_to_uint(Joint_RB_U.recieve.recieve_Position, P_MIN, P_MAX, 16)>>8)&0xff;             
	F407_To_C_Tx[10] = (float_to_uint(Joint_RB_U.recieve.recieve_Position, P_MIN, P_MAX, 16))&0xff;  
	F407_To_C_Tx[11] = (float_to_uint(Joint_RB_D.recieve.recieve_Position, P_MIN, P_MAX, 16)>>8)&0xff;             
	F407_To_C_Tx[12] = (float_to_uint(Joint_RB_D.recieve.recieve_Position, P_MIN, P_MAX, 16))&0xff;  
	F407_To_C_Tx[13] = (float_to_uint(Joint_RF_U.recieve.recieve_Position, P_MIN, P_MAX, 16)>>8)&0xff;             
	F407_To_C_Tx[14] = (float_to_uint(Joint_RF_U.recieve.recieve_Position, P_MIN, P_MAX, 16))&0xff;  
	F407_To_C_Tx[15] = (float_to_uint(Joint_RF_D.recieve.recieve_Position, P_MIN, P_MAX, 16)>>8)&0xff;             
	F407_To_C_Tx[16] = (float_to_uint(Joint_RF_D.recieve.recieve_Position, P_MIN, P_MAX, 16))&0xff;  
	F407_To_C_Tx[17] = 0x0B;             
	
	
	
	
  DMA_Cmd(DMA2_Stream7,ENABLE);
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

 
 
 
 
 
 


		