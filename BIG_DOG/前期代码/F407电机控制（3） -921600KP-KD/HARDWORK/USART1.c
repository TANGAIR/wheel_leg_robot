 #include "main.h"  
 
 
 //关节电机CAN数据结构
CAN1_Data_TypeDef  Joint_LF_U,//左前上角度电机1
                   Joint_LF_D,//左前下角度电机2
									 Joint_LB_U,//左后上角度电机3
                   Joint_LB_D,//左后下角度电机4
									 Joint_RB_U,//右后上角度电机5
                   Joint_RB_D,//右后下角度电机6
									 Joint_RF_U,//右前上角度电机7
                   Joint_RF_D;//右前下角度电机8


//关节电机命令
 char CAN_DATA_CMD_ON[8]    =       {0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC};
 char CAN_DATA_CMD_OFF[8]   =       {0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD};
 char CAN_DATA_CMD_SET_0[8] =       {0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0X00};
 char CAN_DATA_CMD_Change_ID[8] =   {0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF0,0X06}; //最后一位为ID号

 
uint8_t   C_To_F407_Rx[18];
uint8_t   F407_To_C_Tx[18]; 
 

/**********************************串口1初始化***************************/ 
/**
  * @brief  初始化USART1的发送、串口6的接收
  * @param  void
  * @retval void
  * @notes  串口6打印初始化    USART1_TX-PA9      USART6_RX -----PG9
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
	GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;                                      //推挽输出
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF;                                        //复用模式
	GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_NOPULL;                                    //不上拉 
	GPIO_InitStruct.GPIO_Speed=GPIO_High_Speed;                                    //100MHZ
	GPIO_Init(GPIOA,&GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_9;
	GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;                                      //推挽输出
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF;                                        //复用模式
	GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_NOPULL;                                    //不上拉 
	GPIO_InitStruct.GPIO_Speed=GPIO_High_Speed;                                    //100MHZ
	GPIO_Init(GPIOG,&GPIO_InitStruct);
	
	USART_InitStruct.USART_BaudRate=1000000;
	USART_InitStruct.USART_HardwareFlowControl=USART_HardwareFlowControl_None;     //硬件流控制无
	USART_InitStruct.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;                       //收发模式
	USART_InitStruct.USART_Parity=USART_Parity_No;                                 //无奇偶校验位
	USART_InitStruct.USART_StopBits=USART_StopBits_1;                              //字节
	USART_InitStruct.USART_WordLength=USART_WordLength_8b;                         //数据长度
	USART_Init(USART1,&USART_InitStruct);
	USART_Cmd(USART1,ENABLE);
	USART_Init(USART6,&USART_InitStruct);
	USART_Cmd(USART6,ENABLE);
	
	USART_ITConfig(USART6,USART_IT_IDLE,ENABLE);

/* -------------- Configure NVIC ---------------------------------------*/
//接收中断
	NVIC_InitStruct.NVIC_IRQChannel                   =USART6_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority =4;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority        =0;
	NVIC_InitStruct.NVIC_IRQChannelCmd                =ENABLE ;
	NVIC_Init(&NVIC_InitStruct);

//发送后中断
	NVIC_InitStruct.NVIC_IRQChannel                   =DMA2_Stream7_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority =5;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority        =0;
	NVIC_InitStruct.NVIC_IRQChannelCmd                =ENABLE ;
	NVIC_Init(&NVIC_InitStruct);
	
/* ------------------------------------- Configure DMA  接收---------------------------------------*/
  USART_DMACmd(USART6,USART_DMAReq_Rx,ENABLE);
	
	DMA_Cmd(DMA2_Stream1,DISABLE);
	DMA_DeInit(DMA2_Stream1);
	
	DMA_InitStruct.DMA_Channel=DMA_Channel_5;                                      //通道配置（通道5）
	DMA_InitStruct.DMA_BufferSize=18;                                              //传输数据数目                
	DMA_InitStruct.DMA_DIR=DMA_DIR_PeripheralToMemory;                             //数据传输方向（外设——存储器）                   
	DMA_InitStruct.DMA_FIFOMode=ENABLE;                                            //使能FIFO模式
	DMA_InitStruct.DMA_FIFOThreshold=DMA_FIFOThreshold_1QuarterFull;               //阈值1/4      
	DMA_InitStruct.DMA_Memory0BaseAddr=(uint32_t)C_To_F407_Rx;                    //存储器数据地址
	DMA_InitStruct.DMA_MemoryBurst=DMA_MemoryBurst_Single;                         //存储器突发配置
	DMA_InitStruct.DMA_MemoryDataSize=DMA_MemoryDataSize_Byte;                     //存储器数据格式（字节）
	DMA_InitStruct.DMA_MemoryInc=DMA_MemoryInc_Enable;                             //内存地址递增
	DMA_InitStruct.DMA_Mode=DMA_Mode_Circular;                                     //是否循环发送（循环，普通模式只能接受一次）
	DMA_InitStruct.DMA_PeripheralBaseAddr=(uint32_t)&(USART6->DR);                 //外设地址 
	DMA_InitStruct.DMA_PeripheralBurst=DMA_PeripheralBurst_Single;                 //外设突发配置         
	DMA_InitStruct.DMA_PeripheralDataSize=DMA_PeripheralDataSize_Byte;             //外设数据格式
	DMA_InitStruct.DMA_PeripheralInc=DMA_PeripheralInc_Disable;                    //外设地址不递增
	DMA_InitStruct.DMA_Priority=DMA_Priority_VeryHigh;                             //优先级非常高      
	DMA_Init(DMA2_Stream1,&DMA_InitStruct);
	DMA_Cmd(DMA2_Stream1,ENABLE);

///*********************************************DMA发送************************************/
	
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);
	
	DMA_Cmd(DMA2_Stream7,DISABLE);
  DMA_DeInit(DMA2_Stream7); 

//为DMA配置通道
	DMA_InitStruct.DMA_Channel           =DMA_Channel_4;
	DMA_InitStruct.DMA_PeripheralBaseAddr=(uint32_t)&(	USART1->DR);          //起始地址
	DMA_InitStruct.DMA_Memory0BaseAddr   =(uint32_t)F407_To_C_Tx;                 //存储变量
	DMA_InitStruct.DMA_DIR               =DMA_DIR_MemoryToPeripheral;         //传输方向
	DMA_InitStruct.DMA_BufferSize        =18; //缓冲区长度
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
	DMA_Init(DMA2_Stream7,&DMA_InitStruct);
	
	DMA_ITConfig(DMA2_Stream7,DMA_IT_TC,ENABLE);

	delay_ms(3);


}



/**
  * @brief  CAN DMA数据流发送中断服务函数
  * @param  void
  * @retval void
  * @notes          
  */

void DMA2_Stream7_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA2_Stream7,DMA_IT_TCIF7)==SET)
	{
	 //关闭dma数据流
	 DMA_Cmd(DMA2_Stream7,DISABLE);
	 //重新设置dma数据长度
	 DMA_SetCurrDataCounter(DMA2_Stream7,18);
	}
	DMA_ClearITPendingBit(DMA2_Stream7,DMA_IT_TCIF7);	
	
}
/**
  * @brief  串口6接收中断 服务函数
  * @param  void
  * @retval void
  * @notes  接收C板发来的控制信息，周期为15ms
  */
char   turn_flag=1;
void USART6_IRQHandler(void)
{
	 char num=0;
	if(USART_GetITStatus(USART6,USART_IT_IDLE)==SET)
	{
		 num = USART6->SR;
     num = USART6->DR; //清除串口空闲中断（中文手册712页）
		 num++;            //没有意义，只是消除警告
 		
		 DMA_Cmd(DMA2_Stream1,DISABLE); 
		 DMA_SetCurrDataCounter(DMA2_Stream1,18);      //重新设置接收数据长度
		 DMA_Cmd(DMA2_Stream1,ENABLE);                 //传输，将串口1数据寄存器的数据传输到USART_CAN_Rx
		
		if((C_To_F407_Rx[0]==0x0A)&&(C_To_F407_Rx[17]==0x0B))
		{
			
			 turn_flag=-turn_flag;
			
			Joint_LF_U.send.send_Position  =  uint_to_float((C_To_F407_Rx[1]<<8)|(C_To_F407_Rx[2]), P_MIN, P_MAX, 16);  			//上对应seigamar
	
			Joint_LF_D.send.send_Position  =  uint_to_float((C_To_F407_Rx[3]<<8)|(C_To_F407_Rx[4]), P_MIN, P_MAX, 16) ; 	    //下对应gama
		
			Joint_LB_U.send.send_Position  =  uint_to_float((C_To_F407_Rx[5]<<8)|(C_To_F407_Rx[6]), P_MIN, P_MAX, 16) ; 			
		
			Joint_LB_D.send.send_Position  =  uint_to_float((C_To_F407_Rx[7]<<8)|(C_To_F407_Rx[8]), P_MIN, P_MAX, 16) ; 		 
		
			Joint_RB_U.send.send_Position  =  uint_to_float((C_To_F407_Rx[9]<<8)|(C_To_F407_Rx[10]), P_MIN, P_MAX, 16) ; 			
		
			Joint_RB_D.send.send_Position  =  uint_to_float((C_To_F407_Rx[11]<<8)|(C_To_F407_Rx[12]), P_MIN, P_MAX, 16)  ; 		 	
		
			Joint_RF_U.send.send_Position  =  uint_to_float((C_To_F407_Rx[13]<<8)|(C_To_F407_Rx[14]), P_MIN, P_MAX, 16) ; 			
		
			Joint_RF_D.send.send_Position  =  uint_to_float((C_To_F407_Rx[15]<<8)|(C_To_F407_Rx[16]), P_MIN, P_MAX, 16) ; 

			 	
      if((Joint_LF_U.send.send_Position!=0)&&(Joint_RF_D.send.send_Position!=0))//防止全发0	
			{ 
				//通过串口发送信息，控制各腿	
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
  * @brief  电机真实位置反馈函数
  * @param  void
  * @retval void
  * @notes 定期将电机反馈的消息返回给核心主板 
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




 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
/*********************************************数据类型转换函数**********************************************************************/

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

 
 
 
 
 
 


		