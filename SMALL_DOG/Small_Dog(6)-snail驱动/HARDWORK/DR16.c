#include "main.h"

/***************************************数据定义区***************************************/
uint8_t DBUS_buffer[DBUS_DATE_LENGTH+DBUS_SAFTY_LENGTH];
DBUS_DecodingDate_TypeDef DBUS;

/***************************************函数处理区***************************************/
/**
  * @brief  遥控器初始化
  * @param  void
  * @retval void
  * @notes  USART1_RX-->PB7    DMA接收
	          开启串口1接收功能，开启串口1DMA,DMA2_Stream5_Channel_4,配置DMA中断，传输完触发一次
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
	USART_InitStructure.USART_BaudRate           =100000;//100Kbps   遥控器控制协议V1.4.pdf
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
  DMA_DeInit(DMA2_Stream5);                //为DMA配置通道
	DMA_InitStructure.DMA_Channel           =DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr=(uint32_t)&(	USART1->DR);           //起始地址
	DMA_InitStructure.DMA_Memory0BaseAddr   =(uint32_t)DBUS_buffer;              //存储变量
	DMA_InitStructure.DMA_DIR               =DMA_DIR_PeripheralToMemory;         //传输方向
	DMA_InitStructure.DMA_BufferSize        =DBUS_DATE_LENGTH+DBUS_SAFTY_LENGTH; //缓冲区长度
	DMA_InitStructure.DMA_PeripheralInc     =DMA_PeripheralInc_Disable;          //外设递增模式
	DMA_InitStructure.DMA_MemoryInc         =DMA_MemoryInc_Enable;               //内存递增模式
	DMA_InitStructure.DMA_PeripheralDataSize=DMA_PeripheralDataSize_Byte;        //DMA访问时每次操作的数据长度
	DMA_InitStructure.DMA_MemoryDataSize    =DMA_PeripheralDataSize_Byte;        //DMA访问时每次操作的数据长度
	DMA_InitStructure.DMA_Mode              =DMA_Mode_Circular;                  //传输模式：连续不断
	DMA_InitStructure.DMA_Priority          =DMA_Priority_VeryHigh;              //优先级别
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
  * @brief  遥控器数据接收DMA中断
  * @param  void
  * @retval void
  * @notes  每隔14ms通过 DBUS 接收一帧数据（18 字节）
            当每次将串口1数据寄存器的数据传输完之后，便产生一次中断。
            清除中断标志位，向遥控器数据解码任务发送通知，进行一次任务切换
            DBUS_Receive_Data_Task的优先级为5，中断优先级为7
  */
void DMA2_Stream5_IRQHandler(void)//每隔 14ms 通过 DBUS 接收一帧数据（18 字节）
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
  * @brief  遥控器数据解码任务
  * @param  void
  * @retval void
  * @notes  ch0、ch2向右为正,ch1、ch3向上为正,鼠标X轴向右为正,Y轴向下为正，
            每次遥控器接收机向USART1发送消息之后，DMA传输完成，在中断中向
            任务发送通知，启动遥控器数据解码任务。
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
