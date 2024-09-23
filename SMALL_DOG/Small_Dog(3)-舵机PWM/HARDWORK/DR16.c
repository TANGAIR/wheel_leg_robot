#include "main.h"

/***************************************���ݶ�����***************************************/
uint8_t DBUS_buffer[DBUS_DATE_LENGTH+DBUS_SAFTY_LENGTH];
DBUS_DecodingDate_TypeDef DBUS;

/***************************************����������***************************************/
/**
  * @brief  ң������ʼ��
  * @param  void
  * @retval void
  * @notes  USART1_RX-->PB7    DMA����
	          ��������1���չ��ܣ���������1DMA,DMA2_Stream5_Channel_4,����DMA�жϣ������괥��һ��
  */
void DR16_InitConfig(void)
{
  GPIO_InitTypeDef   GPIO_InitStructure;
	USART_InitTypeDef  USART_InitStructure;
	DMA_InitTypeDef    DMA_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	
	/* -------------- Enable Module Clock Source ----------------------------*/
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_DMA2, ENABLE);
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
   
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_USART1);
	
	/* -------------- Configure GPIO ---------------------------------------*/
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	/* -------------- Configure USART ---------------------------------------*/
	USART_DeInit(USART1);
	USART_InitStructure.USART_BaudRate           =100000;//100Kbps   ң��������Э��V1.4.pdf
	USART_InitStructure.USART_WordLength         =USART_WordLength_8b;
	USART_InitStructure.USART_StopBits           =USART_StopBits_1;
	USART_InitStructure.USART_Parity             =USART_Parity_Even;
	USART_InitStructure.USART_Mode               =USART_Mode_Rx;
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART_Init(USART1,&USART_InitStructure);
	
	USART_Cmd(USART1,ENABLE);
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
	
	/* -------------- Configure NVIC ---------------------------------------*/
	NVIC_InitStructure.NVIC_IRQChannel                   =DMA2_Stream5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =7;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority        =0;
	NVIC_InitStructure.NVIC_IRQChannelCmd                =ENABLE ;
	NVIC_Init(&NVIC_InitStructure);
	
	/* -------------- Configure DMA ---------------------------------------*/
  DMA_DeInit(DMA2_Stream5);                //ΪDMA����ͨ��
	DMA_InitStructure.DMA_Channel           =DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr=(uint32_t)&(	USART1->DR);           //��ʼ��ַ
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
	DMA_Init(DMA2_Stream5,&DMA_InitStructure);
	
	DMA_ITConfig(DMA2_Stream5,DMA_IT_TC,ENABLE);
	DMA_Cmd(DMA2_Stream5,ENABLE);	
	
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
void DMA2_Stream5_IRQHandler(void)//ÿ�� 14ms ͨ�� DBUS ����һ֡���ݣ�18 �ֽڣ�
{
	BaseType_t pxHigherPriorityTaskWoken;
	if(DMA_GetITStatus(DMA2_Stream5,DMA_IT_TCIF5))
	{	
		DMA_ClearITPendingBit(DMA2_Stream5,DMA_IT_TCIF5);
		DMA_ClearFlag(DMA2_Stream5,DMA_IT_TCIF5);
		vTaskNotifyGiveFromISR(DBUS_Receive_Data_Task_Handler,&pxHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);	
	}
}

/**
  * @brief  ң�������ݽ�������
  * @param  void
  * @retval void
  * @notes  ch0��ch2����Ϊ��,ch1��ch3����Ϊ��,���X������Ϊ��,Y������Ϊ����
            ÿ��ң�������ջ���USART1������Ϣ֮��DMA������ɣ����ж�����
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
			DBUS.RC.ch0          = ((( DBUS_buffer[0] | (DBUS_buffer [1]<<8) ) & 0x07FF)-1024);                        //channel-0
			DBUS.RC.ch2          = -((( DBUS_buffer[2]>>6 | DBUS_buffer[3]<<2 | DBUS_buffer[4]<<10 ) & 0x07FF)-1024);  //channel-2
			DBUS.RC.ch3          = (( DBUS_buffer[4]>>1 | DBUS_buffer[5]<<7 ) & 0x07FF)-1024;                          //channel-3	
			DBUS.RC.Switch_Left  = (( DBUS_buffer[5]>>4 ) & 0x00C )>>2;                                                //Switch_Left
			DBUS.RC.Switch_Right = ( DBUS_buffer[5]>>4 ) & 0x003;                                                      //Switch_Right   

			if(abs(DBUS.RC.ch0)<20)  DBUS.RC.ch0 = 0;
			if(abs(DBUS.RC.ch2)<20)  DBUS.RC.ch2 = 0;
			if(abs(DBUS.RC.ch3)<20)  DBUS.RC.ch3 = 0;	//Switch_Right
		}
	}
}
