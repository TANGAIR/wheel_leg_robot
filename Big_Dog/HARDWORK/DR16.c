#include "main.h"

/***************************************���ݶ�����***************************************/
uint8_t DBUS_buffer[DBUS_DATE_LENGTH+DBUS_SAFTY_LENGTH];
DBUS_DecodingDate_TypeDef DBUS;

/***************************************����������***************************************/
/**
  * @brief  ң������ʼ��
  * @param  void
  * @retval void
  * @notes  USART3_RX-->PC11    DMA����
	          ��������1���չ��ܣ���������1DMA,DMA1_Stream1_Channel_4,����DMA�жϣ������괥��һ��
  */
void DR16_InitConfig(void)
{
  GPIO_InitTypeDef   GPIO_InitStructure;
	USART_InitTypeDef  USART_InitStructure;
	DMA_InitTypeDef    DMA_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	
	/* -------------- Enable Module Clock Source ----------------------------*/
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_DMA1, ENABLE);
	 RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
   
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_USART3);
	
	/* -------------- Configure GPIO ---------------------------------------*/
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
	/* -------------- Configure USART ---------------------------------------*/
	USART_DeInit(USART3);
	USART_InitStructure.USART_BaudRate           =100000;//100Kbps   ң��������Э��V1.4.pdf
	USART_InitStructure.USART_WordLength         =USART_WordLength_8b;
	USART_InitStructure.USART_StopBits           =USART_StopBits_1;
	USART_InitStructure.USART_Parity             =USART_Parity_Even;
	USART_InitStructure.USART_Mode               =USART_Mode_Rx;
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART_Init(USART3,&USART_InitStructure);
	
	USART_Cmd(USART3,ENABLE);
	USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);
	
	/* -------------- Configure NVIC ---------------------------------------*/
	NVIC_InitStructure.NVIC_IRQChannel                   =DMA1_Stream1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =7;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority        =0;
	NVIC_InitStructure.NVIC_IRQChannelCmd                =ENABLE ;
	NVIC_Init(&NVIC_InitStructure);
	
	/* -------------- Configure DMA ---------------------------------------*/
  DMA_DeInit(DMA1_Stream1);                //ΪDMA����ͨ��
	DMA_InitStructure.DMA_Channel           =DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr=(uint32_t)&(	USART3->DR);           //��ʼ��ַ
	DMA_InitStructure.DMA_Memory0BaseAddr   =(uint32_t)DBUS_buffer;              //�洢����
	DMA_InitStructure.DMA_DIR               =DMA_DIR_PeripheralToMemory;         //���䷽��
	DMA_InitStructure.DMA_BufferSize        =DBUS_DATE_LENGTH+DBUS_SAFTY_LENGTH; //����������
	DMA_InitStructure.DMA_PeripheralInc     =DMA_PeripheralInc_Disable;          //�������ģʽ
	DMA_InitStructure.DMA_MemoryInc         =DMA_MemoryInc_Enable;               //�ڴ����ģʽ
	DMA_InitStructure.DMA_PeripheralDataSize=DMA_PeripheralDataSize_Byte;        //DMA����ʱÿ�β��������ݳ���
	DMA_InitStructure.DMA_MemoryDataSize    =DMA_PeripheralDataSize_Byte;        //DMA����ʱÿ�β��������ݳ���
	DMA_InitStructure.DMA_Mode              =DMA_Mode_Circular;                  //����ģʽ����������
	DMA_InitStructure.DMA_Priority          =DMA_Priority_VeryHigh;              //���ȼ���
  DMA_InitStructure.DMA_FIFOMode          =DMA_FIFOMode_Enable;
	DMA_InitStructure.DMA_FIFOThreshold     =DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_MemoryBurst       =DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst   =DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream1,&DMA_InitStructure);
	
	DMA_ITConfig(DMA1_Stream1,DMA_IT_TC,ENABLE);
	DMA_Cmd(DMA1_Stream1,ENABLE);	
	
	DBUS.RC.ch1          = 0;
	DBUS.RC.ch2          = 0;
	DBUS.RC.ch3          = 0;
	DBUS.RC.Switch_Left  = 0;
	DBUS.RC.Switch_Right = 0;      
}

/**
  * @brief  ң�������ݽ���DMA�ж�
  * @param  void
  * @retval void
  * @notes  ÿ��14msͨ�� DBUS ����һ֡���ݣ�18 �ֽڣ�
            ��ÿ�ν�����1���ݼĴ��������ݴ�����֮�󣬱����һ���жϡ�
            ����жϱ�־λ����ң�������ݽ���������֪ͨ������һ�������л�
            DBUS_Receive_Data_Task�����ȼ�Ϊ5���ж����ȼ�Ϊ7
  */
void DMA1_Stream1_IRQHandler(void)//ÿ�� 14ms ͨ�� DBUS ����һ֡���ݣ�18 �ֽڣ�
{
	BaseType_t pxHigherPriorityTaskWoken;
	if(DMA_GetITStatus(DMA1_Stream1,DMA_IT_TCIF1))
	{	
		DMA_ClearITPendingBit(DMA1_Stream1,DMA_IT_TCIF1);
		DMA_ClearFlag(DMA1_Stream1,DMA_IT_TCIF1);
		vTaskNotifyGiveFromISR(DBUS_Receive_Data_Task_Handler,&pxHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);	
	}
}

/**
  * @brief  ң�������ݽ�������
  * @param  void
  * @retval void
  * @notes  ch0��ch2����Ϊ��,ch1��ch3����Ϊ��,���X������Ϊ��,Y������Ϊ����
            ÿ��ң�������ջ���USART3������Ϣ֮��DMA������ɣ����ж�����
            ������֪ͨ������ң�������ݽ�������
  */
void DBUS_Receive_Data_Task(void *pvParameters)
{
	uint32_t err;
	DR16_InitConfig();
	while(1)
	{
		err=ulTaskNotifyTake(pdTRUE,  portMAX_DELAY);
		if( err==1 )
		{
			//3 2 1 0 ���ϵ��£������ң�����Ϊ��
			DBUS.RC.ch0          = ((( DBUS_buffer[0] | (DBUS_buffer [1]<<8) ) & 0x07FF)-1024);                        //channel-0
			DBUS.RC.ch1          = ((( DBUS_buffer[1]>>3 | DBUS_buffer[2]<<5 ) & 0x07FF)-1024);
			DBUS.RC.ch2          = ((( DBUS_buffer[2]>>6 | DBUS_buffer[3]<<2 | DBUS_buffer[4]<<10 ) & 0x07FF)-1024);  //channel-2
			DBUS.RC.ch3          = ((( DBUS_buffer[4]>>1 | DBUS_buffer[5]<<7 ) & 0x07FF)-1024);                          //channel-3	
			DBUS.RC.Switch_Left  = (( DBUS_buffer[5]>>4 ) & 0x00C )>>2;                                                //Switch_Left
			DBUS.RC.Switch_Right = ( DBUS_buffer[5]>>4 ) & 0x003;                                                      //Switch_Right   

			if(abs(DBUS.RC.ch0)<20)  DBUS.RC.ch0 = 0;
			if(abs(DBUS.RC.ch1)<20)  DBUS.RC.ch1 = 0;
			if(abs(DBUS.RC.ch2)<20)  DBUS.RC.ch2 = 0;
			if(abs(DBUS.RC.ch3)<20)  DBUS.RC.ch3 = 0;	//Switch_Right
		}
	}
}
