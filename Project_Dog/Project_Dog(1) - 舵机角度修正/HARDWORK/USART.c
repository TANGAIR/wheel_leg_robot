#include "main.h"

/**********************************数据定义区*****************************/
Send_TX2_Data_Typedef    Send_TX2_Data_TypeStruct;
Receive_TX2_Data_Typedef Receive_TX2_Data_TypeStruct;

char   Usart3Rx_Info[10];//串口三接收数组
char        Send_TX2[10];//串口三发送数组


/**********************************串口6打印***************************/
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 
FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
//用于打印数据
int fputc(int ch, FILE *f)
{
	while (USART_GetFlagStatus(USART6,USART_FLAG_TC) == RESET);//等待之前的字符发送完成
	USART_SendData(USART6, (uint8_t)ch);
	return (ch);
}
#endif
/**
  * @brief  初始化USART6
  * @param  void
  * @retval void
  * @notes  串口6打印初始化    USART6_TX-PG14      USART6_RX -----PG9
  */
void USART6_Init(void)
{
	GPIO_InitTypeDef       GPIO_InitSturct;
	USART_InitTypeDef      USART_InitStruct;
	NVIC_InitTypeDef       NVIC_InitStruct;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);
	
	GPIO_PinAFConfig(GPIOG,GPIO_PinSource9,GPIO_AF_USART6);
	GPIO_PinAFConfig(GPIOG,GPIO_PinSource14,GPIO_AF_USART6);
	
	GPIO_InitSturct.GPIO_Pin=GPIO_Pin_9|GPIO_Pin_14;
	GPIO_InitSturct.GPIO_OType=GPIO_OType_PP;                                      //推挽输出
	GPIO_InitSturct.GPIO_Mode=GPIO_Mode_AF;                                        //复用模式
	GPIO_InitSturct.GPIO_PuPd=GPIO_PuPd_NOPULL;                                    //不上拉 
	GPIO_InitSturct.GPIO_Speed=GPIO_High_Speed;                                    //100MHZ
	GPIO_Init(GPIOG,&GPIO_InitSturct);
	
	USART_InitStruct.USART_BaudRate=115200;
	USART_InitStruct.USART_HardwareFlowControl=USART_HardwareFlowControl_None;     //硬件流控制无
	USART_InitStruct.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;                       //收发模式
	USART_InitStruct.USART_Parity=USART_Parity_No;                                 //无奇偶校验位
	USART_InitStruct.USART_StopBits=USART_StopBits_1;                              //字节
	USART_InitStruct.USART_WordLength=USART_WordLength_8b;                         //数据长度
	USART_Init(USART6,&USART_InitStruct);
	USART_Cmd(USART6,ENABLE);
	USART_ITConfig(USART6,USART_IT_RXNE,ENABLE);
	
	NVIC_InitStruct.NVIC_IRQChannel=USART6_IRQn;                                   //中断通道
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;                                     //使能
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=15;                           //抢占优先级
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=0;                                  //子优先级
	NVIC_Init(&NVIC_InitStruct);	
}


/**
  * @brief  串口6打印中断
  * @param  void
  * @retval void
  * @notes  暂时没用
  */
void USART6_IRQHandler(void)
{
  if ( USART_GetITStatus( USART6, USART_IT_RXNE ) != RESET )
  { //串口六接收到的数据个数
  }
  if ( USART_GetITStatus( USART6, USART_IT_ORE_RX ) != RESET )
  { 
		
  }
  USART_ClearFlag ( USART6,USART_IT_RXNE | USART_IT_ORE_RX );
}



/*********************************************主板与TX2数据通信**********************************************************************/
/**
  * @brief  USART3-TX2通信初始化
  * @param  void
  * @retval void
  * @notes  USART3_RX -----PD9    USART3_TX-----PD8     
            发送用DMA1_Stream3，DMA_Channel_4，使用DMA1_Stream3_IRQHandler
            接收用DMA1_Stream1，DMA_Channel_4，使用USART3_IRQHandler
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
//接收中断
	NVIC_InitStruct.NVIC_IRQChannel                   =USART3_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority =6;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority        =0;
	NVIC_InitStruct.NVIC_IRQChannelCmd                =ENABLE ;
	NVIC_Init(&NVIC_InitStruct);
//发送后中断
	NVIC_InitStruct.NVIC_IRQChannel                   =DMA1_Stream3_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority =6;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority        =0;
	NVIC_InitStruct.NVIC_IRQChannelCmd                =ENABLE ;
	NVIC_Init(&NVIC_InitStruct);
	
/* ------------------------------------- Configure DMA  接收---------------------------------------*/

	USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);
	
	DMA_Cmd(DMA1_Stream1,DISABLE);
	DMA_DeInit(DMA1_Stream1);
	
	DMA_InitStruct.DMA_Channel=DMA_Channel_4;                                      //通道配置（通道5）
	DMA_InitStruct.DMA_BufferSize=15;                                              //传输数据数目                
	DMA_InitStruct.DMA_DIR=DMA_DIR_PeripheralToMemory;                             //数据传输方向（外设——存储器）                   
	DMA_InitStruct.DMA_FIFOMode=ENABLE;                                            //使能FIFO模式
	DMA_InitStruct.DMA_FIFOThreshold=DMA_FIFOThreshold_1QuarterFull;               //阈值1/4      
	DMA_InitStruct.DMA_Memory0BaseAddr=(uint32_t)Usart3Rx_Info;                    //存储器数据地址
	DMA_InitStruct.DMA_MemoryBurst=DMA_MemoryBurst_Single;                         //存储器突发配置
	DMA_InitStruct.DMA_MemoryDataSize=DMA_MemoryDataSize_Byte;                     //存储器数据格式（字节）
	DMA_InitStruct.DMA_MemoryInc=DMA_MemoryInc_Enable;                             //内存地址递增
	DMA_InitStruct.DMA_Mode=DMA_Mode_Circular;                                     //是否循环发送（循环，普通模式只能接受一次）
	DMA_InitStruct.DMA_PeripheralBaseAddr=(uint32_t)&(USART3->DR);                 //外设地址 
	DMA_InitStruct.DMA_PeripheralBurst=DMA_PeripheralBurst_Single;                 //外设突发配置         
	DMA_InitStruct.DMA_PeripheralDataSize=DMA_PeripheralDataSize_Byte;             //外设数据格式
	DMA_InitStruct.DMA_PeripheralInc=DMA_PeripheralInc_Disable;                    //外设地址不递增
	DMA_InitStruct.DMA_Priority=DMA_Priority_VeryHigh;                             //优先级非常高      
	DMA_Init(DMA1_Stream1,&DMA_InitStruct);
	DMA_Cmd(DMA1_Stream1,ENABLE);

///*********************************************DMA发送************************************/
	
	USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);
	
	DMA_Cmd(DMA1_Stream3,DISABLE);
  DMA_DeInit(DMA1_Stream3);                                                 //为DMA配置通道
	DMA_InitStruct.DMA_Channel           =DMA_Channel_4;
	DMA_InitStruct.DMA_PeripheralBaseAddr=(uint32_t)&(	USART3->DR);          //起始地址
	DMA_InitStruct.DMA_Memory0BaseAddr   =(uint32_t)Send_TX2;                 //存储变量
	DMA_InitStruct.DMA_DIR               =DMA_DIR_MemoryToPeripheral;         //传输方向
	DMA_InitStruct.DMA_BufferSize        =15; //缓冲区长度
	DMA_InitStruct.DMA_PeripheralInc     =DMA_PeripheralInc_Disable;          //外设递增模式
	DMA_InitStruct.DMA_MemoryInc         =DMA_MemoryInc_Enable;               //内存递增模式
	DMA_InitStruct.DMA_PeripheralDataSize=DMA_PeripheralDataSize_Byte;        //DMA访问时每次操作的数据长度
	DMA_InitStruct.DMA_MemoryDataSize    =DMA_PeripheralDataSize_Byte;        //DMA访问时每次操作的数据长度
	DMA_InitStruct.DMA_Mode              =DMA_Mode_Normal;                    //传输模式：连续不断
	DMA_InitStruct.DMA_Priority          =DMA_Priority_VeryHigh;              //优先级别
  DMA_InitStruct.DMA_FIFOMode			     =DMA_FIFOMode_Enable;
	DMA_InitStruct.DMA_FIFOThreshold     =DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStruct.DMA_MemoryBurst       =DMA_MemoryBurst_Single;
	DMA_InitStruct.DMA_PeripheralBurst   =DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream3,&DMA_InitStruct);
	
	DMA_ITConfig(DMA1_Stream3,DMA_IT_TC,ENABLE);
	
 
	delay_ms(3);
}
/**
  * @brief  TX2 DMA数据流发送中断服务函数
  * @param  void
  * @retval void
  * @notes  每次启动发送任务函数之后启动这个中断服务函数，发送函数在陀螺仪获取任务中被调用，每1ms执行一次
            中断中将DMA数据流关闭并且重新设置DNA数据长度
            
  */
void DMA1_Stream3_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_Stream3,DMA_IT_TCIF3)==SET)
	{
	 //关闭dma数据流
	 DMA_Cmd(DMA1_Stream3,DISABLE);
	 //重新设置dma数据长度
	 DMA_SetCurrDataCounter(DMA1_Stream3,10);
	}
	DMA_ClearITPendingBit(DMA1_Stream3,DMA_IT_TCIF3);	
}
/**
  * @brief  TX2 串口3数据接收中断服务函数
  * @param  void
  * @retval void
  * @notes  串口三接收到TX2发来的数据之后先将dma数据流关闭重新设置数据长度之后再打开数据流
            然后转到TX2数据处理任务，DMA：从USART3->DR到Usart3Rx_Info
  */
void USART3_IRQHandler(void)
{
	
	BaseType_t pxHigherPriorityTaskWoken;
	char num=0;
	if(USART_GetITStatus(USART3,USART_IT_IDLE)==SET)
	{
		 num = USART3->SR;
     num = USART3->DR; //清除串口空闲中断（中文手册712页）
		 num++;            //没有意义，只是消除警告
		
		 DMA_Cmd(DMA1_Stream1,DISABLE); 
		 DMA_SetCurrDataCounter(DMA1_Stream1,10);      //重新设置接收数据长度
		 DMA_Cmd(DMA1_Stream1,ENABLE);                 //传输，将串口3数据寄存器的数据传输到Usart3Rx_Info
		
		//向TX2数据解码任务发送通知，将其移至就绪列表
		 vTaskNotifyGiveFromISR(TX2_Decode_Task_Handler,&pxHigherPriorityTaskWoken);
		//进行任务切换
		 portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
	}	
}
/******************************主板到TX2通信数据分析****************************
	
  char  Left_Front_Leg_X;  //左前腿足端在关节坐标系X的坐标,单位CM，以关节舵机轴心为原点，机器狗前进方向为X轴正方向
	char  Left_Front_Leg_Y;  //左前腿足端在关节坐标系Y的坐标,单位CM，以关节舵机轴心为原点，向下为Y轴正方向
	char  Right_Front_Leg_X; //右前腿足端在关节坐标系X的坐标，单位CM，以关节舵机轴心为原点，机器狗前进方向为X轴正方向
	char  Right_Front_Leg_Y; //右前腿足端在关节坐标系Y的坐标，单位CM，以关节舵机轴心为原点，向下为Y轴正方向
	short IMU_PITCH;         //陀螺仪俯仰角PITCH，单位为°，G_Y，向上为正，向下为负
	short IMU_YAW;           //陀螺仪航向角YAW，单位为°，G_Z，俯视向左为正，向右为负
	char  Left_Front_Wheel_Speed;//右前轮电机速度，单位CM/S
	
*************************************************************************************/
/**
  * @brief  串口三数据发送函数
  * @param  void
  * @retval void
  * @notes  将云台yaw轴和pitch轴的数据经过处理之后储存Send_TX2数组中
            然后启动dma数据流将数据发送出去，在陀螺仪数据解码函数中被调用
            每1ms发送一次Send_TX2到Send_TX2，再发出
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
	//从Send_TX2到Send_TX2
	DMA_Cmd(DMA1_Stream3,ENABLE);
}




/*******************************TX2到主板通信数据分析*****************************
	char  Dog_Mode;      //机器狗模式,1，轮式跟随模式；2，上楼梯模式；3，下楼梯模式；0，停止
	char  Climb_Status;  //爬楼状态,1，前腿爬行后腿滑行；2，前后腿都爬行；3，前腿滑行后腿爬行
	char  Front_Leg_Distance; //爬楼时，左前腿落点与楼梯中点的水平距离,单位CM，以前进方向为正，以中点为0
	char  Stair_Higth;        //楼梯高度，单位CM
	char  Stair_Width;        //楼梯宽度，单位CM
	char  Slide_Speed;        //平地滑行速度，单位CM/S，爬楼时为0，用于控制机器狗跟随模式时的滑行速度
	char  Turn_Angle;         //机器狗的转角,单位°，用于控制机器狗转向
	char  Head_Pitch_Angle;   //头舵机PITCH角度,单位°，用于控制机器狗抬头低头
	char  Head_Yaw_Angle;     //头舵机YAW角度,单位°，用于控制机器狗左右转头
**********************************************************************************/
/**
  * @brief  TX2数据解码任务
  * @param  void
  * @retval void
  * @notes  每次接收完TX2发过来的数据时执行一次
            TX2初始化：初始化串口3进行与TX2的通讯，进入TX2解码任务，当数据经过校验之后说明TX2已经连接上了，这个时候TX2丢失时间等于0；
            当TX2发过来的数据中的装甲板类型不为0的时候说明已经识别到了敌人，此时将云台模式切换为识别到敌人，
            如果发过来的装甲板类型是0的话就说明没有检测到装甲板，这个时候将云台控制模式切换为没有识别到敌人            
  */
void TX2_Decode_Task(void *pvParameters)
{
	uint32_t err;

	TX2_Init();
	vTaskDelay(200);
	while(1)
	{
		//等待任务通知
		err=ulTaskNotifyTake(pdTRUE,portMAX_DELAY);

		if(err==1)
		{//数据校验
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

























/*********************************************数据类型转换函数**********************************************************************/


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



