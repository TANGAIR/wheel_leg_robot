 #include "main.h"  
 
 
uint8_t USART3CAN_start[7]={0x41, 0x54,0x2B,0X41,0X54,0X0D,0X0A};


USART2CAN_Typedef Tx_massage_u3;
USART2CAN_Typedef Rx_massage_u3;

uint8_t   USART3_CAN_Tx[17];
uint8_t   USART3_CAN_Rx[15];
 
 /**
  * @brief  初始化USART3
  * @param  void
  * @retval void
  * @notes     USART3_TX-PD8      USART3_RX -----PD9
  */
DMA_InitTypeDef    DMA_InitStruct_U3;
void USART3_Init(void)
{
	GPIO_InitTypeDef   GPIO_InitStruct;
	USART_InitTypeDef  USART_InitStruct;
	
	NVIC_InitTypeDef   NVIC_InitStruct;
	
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_DMA1,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_USART3);
	
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_8;
	GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;                                      //推挽输出
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF;                                        //复用模式
	GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_NOPULL;                                    //不上拉 
	GPIO_InitStruct.GPIO_Speed=GPIO_High_Speed;                                    //100MHZ
	GPIO_Init(GPIOD,&GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_9;
	GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;                                      //推挽输出
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF;                                        //复用模式
	GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_NOPULL;                                    //不上拉 
	GPIO_InitStruct.GPIO_Speed=GPIO_High_Speed;                                    //100MHZ
	GPIO_Init(GPIOD,&GPIO_InitStruct);
	
	USART_InitStruct.USART_BaudRate=1000000;
	USART_InitStruct.USART_HardwareFlowControl=USART_HardwareFlowControl_None;     //硬件流控制无
	USART_InitStruct.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;                       //收发模式
	USART_InitStruct.USART_Parity=USART_Parity_No;                                 //无奇偶校验位
	USART_InitStruct.USART_StopBits=USART_StopBits_1;                              //字节
	USART_InitStruct.USART_WordLength=USART_WordLength_8b;                         //数据长度
	USART_Init(USART3,&USART_InitStruct);
	USART_Cmd(USART3,ENABLE);
	
	
	USART_ITConfig(USART3,USART_IT_IDLE,ENABLE);

/* -------------- Configure NVIC ---------------------------------------*/
//接收中断
	NVIC_InitStruct.NVIC_IRQChannel                   =USART3_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority =3;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority        =0;
	NVIC_InitStruct.NVIC_IRQChannelCmd                =ENABLE ;
	NVIC_Init(&NVIC_InitStruct);

//发送后中断
	NVIC_InitStruct.NVIC_IRQChannel                   =DMA1_Stream3_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority =3;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority        =0;
	NVIC_InitStruct.NVIC_IRQChannelCmd                =ENABLE ;
	NVIC_Init(&NVIC_InitStruct);
	
/* ------------------------------------- Configure DMA  接收---------------------------------------*/
  USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);
	
	DMA_Cmd(DMA1_Stream1,DISABLE);
	DMA_DeInit(DMA1_Stream1);
	
	DMA_InitStruct_U3.DMA_Channel=DMA_Channel_4;                                      //通道配置（通道5）
	DMA_InitStruct_U3.DMA_BufferSize=2;                                              //传输数据数目                
	DMA_InitStruct_U3.DMA_DIR=DMA_DIR_PeripheralToMemory;                             //数据传输方向（外设——存储器）                   
	DMA_InitStruct_U3.DMA_FIFOMode=ENABLE;                                            //使能FIFO模式
	DMA_InitStruct_U3.DMA_FIFOThreshold=DMA_FIFOThreshold_1QuarterFull;               //阈值1/4      
	DMA_InitStruct_U3.DMA_Memory0BaseAddr=(uint32_t)USART3_CAN_Rx;                    //存储器数据地址
	DMA_InitStruct_U3.DMA_MemoryBurst=DMA_MemoryBurst_Single;                         //存储器突发配置
	DMA_InitStruct_U3.DMA_MemoryDataSize=DMA_MemoryDataSize_Byte;                     //存储器数据格式（字节）
	DMA_InitStruct_U3.DMA_MemoryInc=DMA_MemoryInc_Enable;                             //内存地址递增
	DMA_InitStruct_U3.DMA_Mode=DMA_Mode_Circular;                                     //是否循环发送（循环，普通模式只能接受一次）
	DMA_InitStruct_U3.DMA_PeripheralBaseAddr=(uint32_t)&(USART3->DR);                 //外设地址 
	DMA_InitStruct_U3.DMA_PeripheralBurst=DMA_PeripheralBurst_Single;                 //外设突发配置         
	DMA_InitStruct_U3.DMA_PeripheralDataSize=DMA_PeripheralDataSize_Byte;             //外设数据格式
	DMA_InitStruct_U3.DMA_PeripheralInc=DMA_PeripheralInc_Disable;                    //外设地址不递增
	DMA_InitStruct_U3.DMA_Priority=DMA_Priority_VeryHigh;                             //优先级非常高      
	DMA_Init(DMA1_Stream1,&DMA_InitStruct_U3);
	DMA_Cmd(DMA1_Stream1,ENABLE);

///*********************************************DMA发送************************************/
	
	USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);
	
	DMA_Cmd(DMA1_Stream3,DISABLE);
  DMA_DeInit(DMA1_Stream3); 

//为DMA配置通道
	DMA_InitStruct_U3.DMA_Channel           =DMA_Channel_4;
	DMA_InitStruct_U3.DMA_PeripheralBaseAddr=(uint32_t)&(USART3->DR);          //起始地址
	DMA_InitStruct_U3.DMA_Memory0BaseAddr   =(uint32_t)USART3CAN_start;                 //存储变量
	DMA_InitStruct_U3.DMA_DIR               =DMA_DIR_MemoryToPeripheral;         //传输方向
	DMA_InitStruct_U3.DMA_BufferSize        =7; //缓冲区长度
	DMA_InitStruct_U3.DMA_PeripheralInc     =DMA_PeripheralInc_Disable;          //外设递增模式
	DMA_InitStruct_U3.DMA_MemoryInc         =DMA_MemoryInc_Enable;               //内存递增模式
	DMA_InitStruct_U3.DMA_PeripheralDataSize=DMA_PeripheralDataSize_Byte;        //DMA访问时每次操作的数据长度
	DMA_InitStruct_U3.DMA_MemoryDataSize    =DMA_PeripheralDataSize_Byte;        //DMA访问时每次操作的数据长度
	DMA_InitStruct_U3.DMA_Mode              =DMA_Mode_Normal;                    //传输模式：连续不断
	DMA_InitStruct_U3.DMA_Priority          =DMA_Priority_VeryHigh;              //优先级别
  DMA_InitStruct_U3.DMA_FIFOMode			     =DMA_FIFOMode_Enable;
	DMA_InitStruct_U3.DMA_FIFOThreshold     =DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStruct_U3.DMA_MemoryBurst       =DMA_MemoryBurst_Single;
	DMA_InitStruct_U3.DMA_PeripheralBurst   =DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream3,&DMA_InitStruct_U3);
	
	DMA_ITConfig(DMA1_Stream3,DMA_IT_TC,ENABLE);

	delay_ms(3);
	//启动CAN转换芯片
	DMA_Cmd(DMA1_Stream3,ENABLE);
}



/**
  * @brief  CAN DMA数据流发送中断服务函数
  * @param  void
  * @retval void
  * @notes  
            
  */
char DMA_change_flag_U3=0;
void DMA1_Stream3_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_Stream3,DMA_IT_TCIF3)==SET)
	{
	 //关闭dma数据流
	 DMA_Cmd(DMA1_Stream3,DISABLE);
		//因为第一次发送的是进入AT模式指令，所以之后需要修改发送数据的长度和地址
		if(DMA_change_flag_U3==0)
		{
	   DMA_InitStruct_U3.DMA_Memory0BaseAddr   =(uint32_t)USART3_CAN_Tx;                 //存储变量
	   DMA_Init(DMA1_Stream3,&DMA_InitStruct_U3);
	   DMA_change_flag_U3 =1;
		}		
	
	 //重新设置dma数据长度
	 DMA_SetCurrDataCounter(DMA1_Stream3,17);
		
		
		
	}
	DMA_ClearITPendingBit(DMA1_Stream3,DMA_IT_TCIF3);	
	
}
/**
  * @brief  串口1接收中断 服务函数
  * @param  void
  * @retval void
  * @notes  暂时没用
  */

void USART3_IRQHandler(void)
{  
	CAN1_Data_TypeDef  Jiont_Motor_Unpack;
	 char num=0;
	if(USART_GetITStatus(USART3,USART_IT_IDLE)==SET)
	{
		 num = USART3->SR;
     num = USART3->DR; //清除串口空闲中断（中文手册712页）
		 num++;            //没有意义，只是消除警告
 		
		 DMA_Cmd(DMA1_Stream1,DISABLE); 
		 DMA_SetCurrDataCounter(DMA1_Stream1,15);      //重新设置接收数据长度
		 DMA_Cmd(DMA1_Stream1,ENABLE);                 //传输，将串口1数据寄存器的数据传输到USART3_CAN_Rx
		
		if((USART3_CAN_Rx[0]==0x41)&&(USART3_CAN_Rx[1]==0x54)&&(USART3_CAN_Rx[13]==0x0d)&&(USART3_CAN_Rx[14]==0x0a))
		{
			 //数据解码
			 Rx_massage_u3.Can_ID = bit8TObit32(&USART3_CAN_Rx[2]);
			 Rx_massage_u3.Data_Lenth =  USART3_CAN_Rx[6];
			
			 Rx_massage_u3.Data[0]= USART3_CAN_Rx[7] ;
			 Rx_massage_u3.Data[1]= USART3_CAN_Rx[8] ;
			 Rx_massage_u3.Data[2]= USART3_CAN_Rx[9] ;
			 Rx_massage_u3.Data[3]= USART3_CAN_Rx[10] ;
			 Rx_massage_u3.Data[4]= USART3_CAN_Rx[11] ;
			 Rx_massage_u3.Data[5]= USART3_CAN_Rx[12] ;
			
			
			Jiont_Motor_Unpack.Motor_ID                  = Rx_massage_u3.Data[0];				
			Jiont_Motor_Unpack.recieve.recieve_Position	 = uint_to_float((Rx_massage_u3.Data[1]<<8)|(Rx_massage_u3.Data[2]), P_MIN, P_MAX, 16);
			Jiont_Motor_Unpack.recieve.recieve_Velocity  = uint_to_float((Rx_massage_u3.Data[3]<<4)|( Rx_massage_u3.Data[4]>>4), V_MIN, V_MAX, 12);
			Jiont_Motor_Unpack.recieve.recieve_Current   = uint_to_float(((Rx_massage_u3.Data[4]&0x0f)<<4)|( Rx_massage_u3.Data[5]), -T_MAX, T_MAX, 12);
		 
				switch (Jiont_Motor_Unpack.Motor_ID )
				{			
	
					//关节,左前
          case CAN1_JOINT_ID_LF_U: //0x01                                                                               
					{
						Joint_LF_U.recieve   =  Jiont_Motor_Unpack.recieve ; 			//上对应seigamar
					}break ; 
					
					case CAN1_JOINT_ID_LF_D: //0x02                                                                              
					{
						Joint_LF_D.recieve    =  Jiont_Motor_Unpack.recieve ; 	 //下对应gama
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

  ///USART_ClearFlag ( USART3,USART_IT_IDLE  );
}




/**
  * @brief  关节电机命令发函数
  * @param  void
  * @retval void
  * @notes  
  */
void USART32CAN_TX_Send_Cmd (char CAN_DATA_CMD[8],char ID)
{
	                                      
	Tx_massage_u3.Data_Lenth  = 0x08;                                                     //帧长度为8
	Tx_massage_u3.Can_ID  = (ID<<21)&0xfff00000 ;                                                      //帧ID为接收信息的电机的ID
  

	Tx_massage_u3.Data[0] = CAN_DATA_CMD[0];             
	Tx_massage_u3.Data[1] = CAN_DATA_CMD[1];                
	Tx_massage_u3.Data[2] = CAN_DATA_CMD[2];           
	Tx_massage_u3.Data[3] = CAN_DATA_CMD[3];                
	Tx_massage_u3.Data[4] = CAN_DATA_CMD[4];               //203接收电流高8位
	Tx_massage_u3.Data[5] = CAN_DATA_CMD[5];                  //203接收电流低8位
	Tx_massage_u3.Data[6] = CAN_DATA_CMD[6];               //204接收电流高8位
	Tx_massage_u3.Data[7] = CAN_DATA_CMD[7];                  //204接收电流低8位
	
	 USART3_CAN_Tx[0]=0x41;
	 USART3_CAN_Tx[1]=0x54;
	 USART3_CAN_Tx[2]=bit32TObit8(3,Tx_massage_u3.Can_ID);
	 USART3_CAN_Tx[3]=bit32TObit8(2,Tx_massage_u3.Can_ID);
	 USART3_CAN_Tx[4]=bit32TObit8(1,Tx_massage_u3.Can_ID);
	 USART3_CAN_Tx[5]=bit32TObit8(0,Tx_massage_u3.Can_ID);
	 USART3_CAN_Tx[6]=Tx_massage_u3.Data_Lenth ;
	 USART3_CAN_Tx[7]=  Tx_massage_u3.Data[0]  ;
	 USART3_CAN_Tx[8]=  Tx_massage_u3.Data[1]  ;
	 USART3_CAN_Tx[9]=  Tx_massage_u3.Data[2]  ;
	 USART3_CAN_Tx[10]= Tx_massage_u3.Data[3]  ;
	 USART3_CAN_Tx[11]= Tx_massage_u3.Data[4]  ;
	 USART3_CAN_Tx[12]= Tx_massage_u3.Data[5]  ;
	 USART3_CAN_Tx[13]= Tx_massage_u3.Data[6]  ;
	 USART3_CAN_Tx[14]= Tx_massage_u3.Data[7]  ;
	 USART3_CAN_Tx[15]= 0x0d ;
	 USART3_CAN_Tx[16]= 0x0a ;

	
	
  DMA_Cmd(DMA1_Stream3,ENABLE);
}

/**
  * @brief  关节电机控制发送函数
  * @param  void
  * @retval void
  * @notes  
  */

void USART32CAN_TX_Jiont (CAN1_Data_TypeDef* motor)
{

	Tx_massage_u3.Data_Lenth = 0x08;                                                                      //帧长度为8
	Tx_massage_u3.Can_ID  = (motor->Motor_ID<<21)&0xfff00000;                                             //帧ID为电机ID
	
	
	
	//基于位置差的pid,kp增强控制
	motor->send.send_kp=BASE_JIONT_KP;//+fabs(Pid_Calc(motor->Motor_PID,motor->recieve .recieve_Position ,motor->send .send_Position));

	//KP限制
	if(motor->send.send_kp  >450)motor->send.send_kp=450;
	if(motor->send.send_kp  <=0)motor->send.send_kp=0;

	//位置限位
	if(motor->send.send_Position >=1.57f)motor->send.send_Position=1.57f;
	if(motor->send.send_Position <=0)motor->send.send_Position=0;

	Tx_massage_u3.Data[0] = (float_to_uint(motor->send.send_Position, P_MIN, P_MAX, 16)>>8)&0xff;             
	Tx_massage_u3.Data[1] = (float_to_uint(motor->send.send_Position, P_MIN, P_MAX, 16))&0xff;                
	Tx_massage_u3.Data[2] = (float_to_uint(motor->send.send_Velocity, V_MIN, V_MAX, 12)>>4)&0xff;           
	Tx_massage_u3.Data[3] = (float_to_uint(motor->send.send_Velocity, V_MIN, V_MAX, 12)<<4)|((float_to_uint(motor->send.send_kp, KP_MIN, KP_MAX, 12)>>8)&0x0f);                
	Tx_massage_u3.Data[4] = (float_to_uint(motor->send.send_kp, KP_MIN, KP_MAX, 12))&0xff;                     //203接收电流高8位
	Tx_massage_u3.Data[5] = (float_to_uint(motor->send.send_kd, KD_MIN, KD_MAX, 12)>>4)&0xff;                  //203接收电流低8位
	Tx_massage_u3.Data[6] = (float_to_uint(motor->send.send_kd, KD_MIN, KD_MAX, 12)<<4)|((float_to_uint(motor->send.send_torque, T_MIN, T_MAX, 12)>>8)&0x0f);               //204接收电流高8位
	Tx_massage_u3.Data[7] = (float_to_uint(motor->send.send_torque, T_MIN, T_MAX, 12) )&0xff;                  //204接收电流低8位
	
	 USART3_CAN_Tx[0]=0x41;
	 USART3_CAN_Tx[1]=0x54;
	 USART3_CAN_Tx[2]=bit32TObit8(3,Tx_massage_u3.Can_ID);
	 USART3_CAN_Tx[3]=bit32TObit8(2,Tx_massage_u3.Can_ID);
	 USART3_CAN_Tx[4]=bit32TObit8(1,Tx_massage_u3.Can_ID);
	 USART3_CAN_Tx[5]=bit32TObit8(0,Tx_massage_u3.Can_ID);
	 USART3_CAN_Tx[6]=Tx_massage_u3.Data_Lenth;
	 USART3_CAN_Tx[7]=  Tx_massage_u3.Data[0]  ;
	 USART3_CAN_Tx[8]=  Tx_massage_u3.Data[1]  ;
	 USART3_CAN_Tx[9]=  Tx_massage_u3.Data[2]  ;
	 USART3_CAN_Tx[10]= Tx_massage_u3.Data[3]  ;
	 USART3_CAN_Tx[11]= Tx_massage_u3.Data[4]  ;
	 USART3_CAN_Tx[12]= Tx_massage_u3.Data[5]  ;
	 USART3_CAN_Tx[13]= Tx_massage_u3.Data[6]  ;
	 USART3_CAN_Tx[14]= Tx_massage_u3.Data[7]  ;
	 USART3_CAN_Tx[15]= 0x0d ;
	 USART3_CAN_Tx[16]= 0x0a ;

	
	
  DMA_Cmd(DMA1_Stream3,ENABLE);
}








 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 

