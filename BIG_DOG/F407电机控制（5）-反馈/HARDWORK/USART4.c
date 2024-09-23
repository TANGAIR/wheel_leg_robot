 #include "main.h"  
 
 
uint8_t UART4CAN_start[7]={0x41, 0x54,0x2B,0X41,0X54,0X0D,0X0A};


USART2CAN_Typedef Tx_massage_u4;
USART2CAN_Typedef Rx_massage_u4;

uint8_t   UART4_CAN_Tx[17];
uint8_t   UART4_CAN_Rx[15];
 
 /**
  * @brief  初始化UART4
  * @param  void
  * @retval void
  * @notes      UART4_TX-PC10      UART4_RX -----PC11
  */
DMA_InitTypeDef    DMA_InitStruct_U4;
void USART4_Init(void)
{
	GPIO_InitTypeDef   GPIO_InitStruct;
	USART_InitTypeDef  USART_InitStruct;
	
	NVIC_InitTypeDef   NVIC_InitStruct;
	
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_DMA1,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4);
	
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_10;
	GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;                                      //推挽输出
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF;                                        //复用模式
	GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_NOPULL;                                    //不上拉 
	GPIO_InitStruct.GPIO_Speed=GPIO_High_Speed;                                    //100MHZ
	GPIO_Init(GPIOC,&GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_11;
	GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;                                      //推挽输出
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF;                                        //复用模式
	GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_NOPULL;                                    //不上拉 
	GPIO_InitStruct.GPIO_Speed=GPIO_High_Speed;                                    //100MHZ
	GPIO_Init(GPIOC,&GPIO_InitStruct);
	
	USART_InitStruct.USART_BaudRate=921600;
	USART_InitStruct.USART_HardwareFlowControl=USART_HardwareFlowControl_None;     //硬件流控制无
	USART_InitStruct.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;                       //收发模式
	USART_InitStruct.USART_Parity=USART_Parity_No;                                 //无奇偶校验位
	USART_InitStruct.USART_StopBits=USART_StopBits_1;                              //字节
	USART_InitStruct.USART_WordLength=USART_WordLength_8b;                         //数据长度
	USART_Init(UART4,&USART_InitStruct);
	USART_Cmd(UART4,ENABLE);
	
	
	USART_ITConfig(UART4,USART_IT_IDLE,ENABLE);

/* -------------- Configure NVIC ---------------------------------------*/
//接收中断
	NVIC_InitStruct.NVIC_IRQChannel                   =UART4_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority =3;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority        =0;
	NVIC_InitStruct.NVIC_IRQChannelCmd                =ENABLE ;
	NVIC_Init(&NVIC_InitStruct);

//发送后中断
	NVIC_InitStruct.NVIC_IRQChannel                   =DMA1_Stream4_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority =3;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority        =0;
	NVIC_InitStruct.NVIC_IRQChannelCmd                =ENABLE ;
	NVIC_Init(&NVIC_InitStruct);
	
/* ------------------------------------- Configure DMA  接收---------------------------------------*/
  USART_DMACmd(UART4,USART_DMAReq_Rx,ENABLE);
	
	DMA_Cmd(DMA1_Stream2,DISABLE);
	DMA_DeInit(DMA1_Stream2);
	
	DMA_InitStruct_U4.DMA_Channel=DMA_Channel_4;                                      //通道配置（通道5）
	DMA_InitStruct_U4.DMA_BufferSize=2;                                              //传输数据数目                
	DMA_InitStruct_U4.DMA_DIR=DMA_DIR_PeripheralToMemory;                             //数据传输方向（外设——存储器）                   
	DMA_InitStruct_U4.DMA_FIFOMode=ENABLE;                                            //使能FIFO模式
	DMA_InitStruct_U4.DMA_FIFOThreshold=DMA_FIFOThreshold_1QuarterFull;               //阈值1/4      
	DMA_InitStruct_U4.DMA_Memory0BaseAddr=(uint32_t)UART4_CAN_Rx;                    //存储器数据地址
	DMA_InitStruct_U4.DMA_MemoryBurst=DMA_MemoryBurst_Single;                         //存储器突发配置
	DMA_InitStruct_U4.DMA_MemoryDataSize=DMA_MemoryDataSize_Byte;                     //存储器数据格式（字节）
	DMA_InitStruct_U4.DMA_MemoryInc=DMA_MemoryInc_Enable;                             //内存地址递增
	DMA_InitStruct_U4.DMA_Mode=DMA_Mode_Circular;                                     //是否循环发送（循环，普通模式只能接受一次）
	DMA_InitStruct_U4.DMA_PeripheralBaseAddr=(uint32_t)&(UART4->DR);                 //外设地址 
	DMA_InitStruct_U4.DMA_PeripheralBurst=DMA_PeripheralBurst_Single;                 //外设突发配置         
	DMA_InitStruct_U4.DMA_PeripheralDataSize=DMA_PeripheralDataSize_Byte;             //外设数据格式
	DMA_InitStruct_U4.DMA_PeripheralInc=DMA_PeripheralInc_Disable;                    //外设地址不递增
	DMA_InitStruct_U4.DMA_Priority=DMA_Priority_VeryHigh;                             //优先级非常高      
	DMA_Init(DMA1_Stream2,&DMA_InitStruct_U4);
	DMA_Cmd(DMA1_Stream2,ENABLE);

///*********************************************DMA发送************************************/
	
	USART_DMACmd(UART4,USART_DMAReq_Tx,ENABLE);
	
	DMA_Cmd(DMA1_Stream4,DISABLE);
  DMA_DeInit(DMA1_Stream4); 

//为DMA配置通道
	DMA_InitStruct_U4.DMA_Channel           =DMA_Channel_4;
	DMA_InitStruct_U4.DMA_PeripheralBaseAddr=(uint32_t)&(	UART4->DR);          //起始地址
	DMA_InitStruct_U4.DMA_Memory0BaseAddr   =(uint32_t)UART4CAN_start;                 //存储变量
	DMA_InitStruct_U4.DMA_DIR               =DMA_DIR_MemoryToPeripheral;         //传输方向
	DMA_InitStruct_U4.DMA_BufferSize        =7; //缓冲区长度
	DMA_InitStruct_U4.DMA_PeripheralInc     =DMA_PeripheralInc_Disable;          //外设递增模式
	DMA_InitStruct_U4.DMA_MemoryInc         =DMA_MemoryInc_Enable;               //内存递增模式
	DMA_InitStruct_U4.DMA_PeripheralDataSize=DMA_PeripheralDataSize_Byte;        //DMA访问时每次操作的数据长度
	DMA_InitStruct_U4.DMA_MemoryDataSize    =DMA_PeripheralDataSize_Byte;        //DMA访问时每次操作的数据长度
	DMA_InitStruct_U4.DMA_Mode              =DMA_Mode_Normal;                    //传输模式：连续不断
	DMA_InitStruct_U4.DMA_Priority          =DMA_Priority_VeryHigh;              //优先级别
  DMA_InitStruct_U4.DMA_FIFOMode			     =DMA_FIFOMode_Enable;
	DMA_InitStruct_U4.DMA_FIFOThreshold     =DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStruct_U4.DMA_MemoryBurst       =DMA_MemoryBurst_Single;
	DMA_InitStruct_U4.DMA_PeripheralBurst   =DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream4,&DMA_InitStruct_U4);
	
	DMA_ITConfig(DMA1_Stream4,DMA_IT_TC,ENABLE);

	delay_ms(3);
	//启动CAN转换芯片
	DMA_Cmd(DMA1_Stream4,ENABLE);
}



/**
  * @brief  CAN DMA数据流发送中断服务函数
  * @param  void
  * @retval void
  * @notes  
            
  */
char DMA_change_flag_U4=0;
void DMA1_Stream4_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_Stream4,DMA_IT_TCIF4)==SET)
	{
	 //关闭dma数据流
	 DMA_Cmd(DMA1_Stream4,DISABLE);
		//因为第一次发送的是进入AT模式指令，所以之后需要修改发送数据的长度和地址
		if(DMA_change_flag_U4==0)
		{
	   DMA_InitStruct_U4.DMA_Memory0BaseAddr   =(uint32_t)UART4_CAN_Tx;                 //存储变量
	   DMA_Init(DMA1_Stream4,&DMA_InitStruct_U4);
	   DMA_change_flag_U4 =1;
		}		
	
	 //重新设置dma数据长度
	 DMA_SetCurrDataCounter(DMA1_Stream4,17);
		
		
		
	}
	DMA_ClearITPendingBit(DMA1_Stream4,DMA_IT_TCIF4);	
	
}
/**
  * @brief  串口1接收中断 服务函数
  * @param  void
  * @retval void
  * @notes  暂时没用
  */

void UART4_IRQHandler(void)
{  
	CAN1_Data_TypeDef  Jiont_Motor_Unpack;
	 char num=0;
	if(USART_GetITStatus(UART4,USART_IT_IDLE)==SET)
	{
		 num = UART4->SR;
     num = UART4->DR; //清除串口空闲中断（中文手册712页）
		 num++;            //没有意义，只是消除警告
 		
		 DMA_Cmd(DMA1_Stream2,DISABLE); 
		 DMA_SetCurrDataCounter(DMA1_Stream2,15);      //重新设置接收数据长度
		 DMA_Cmd(DMA1_Stream2,ENABLE);                 //传输，将串口1数据寄存器的数据传输到UART4_CAN_Rx
		
		if((UART4_CAN_Rx[0]==0x41)&&(UART4_CAN_Rx[1]==0x54)&&(UART4_CAN_Rx[13]==0x0d)&&(UART4_CAN_Rx[14]==0x0a))
		{
			 //数据解码
			 Rx_massage_u4.Can_ID = bit8TObit32(&UART4_CAN_Rx[2]);
			 Rx_massage_u4.Data_Lenth =  UART4_CAN_Rx[6];
			
			 Rx_massage_u4.Data[0]= UART4_CAN_Rx[7] ;
			 Rx_massage_u4.Data[1]= UART4_CAN_Rx[8] ;
			 Rx_massage_u4.Data[2]= UART4_CAN_Rx[9] ;
			 Rx_massage_u4.Data[3]= UART4_CAN_Rx[10] ;
			 Rx_massage_u4.Data[4]= UART4_CAN_Rx[11] ;
			 Rx_massage_u4.Data[5]= UART4_CAN_Rx[12] ;
			
			
			Jiont_Motor_Unpack.Motor_ID                  = Rx_massage_u4.Data[0];				
			Jiont_Motor_Unpack.recieve.recieve_Position	 = uint_to_float((Rx_massage_u4.Data[1]<<8)|(Rx_massage_u4.Data[2]), P_MIN, P_MAX, 16);
			Jiont_Motor_Unpack.recieve.recieve_Velocity  = uint_to_float((Rx_massage_u4.Data[3]<<4)|( Rx_massage_u4.Data[4]>>4), V_MIN, V_MAX, 12);
			Jiont_Motor_Unpack.recieve.recieve_Current   = uint_to_float(((Rx_massage_u4.Data[4]&0x0f)<<4)|( Rx_massage_u4.Data[5]), -T_MAX, T_MAX, 12);
		 
				switch (Jiont_Motor_Unpack.Motor_ID )
				{			
	
					//关节,左前
          case CAN1_JOINT_ID_LF_U: //0x01                                                                               
					{
						Joint_LF_U.recieve   =  Jiont_Motor_Unpack.recieve ; 			//上对应seigamar
						
						LF_U_Last_Position=Joint_LF_U.recieve.recieve_Position;
						
					}break ; 
					
					case CAN1_JOINT_ID_LF_D: //0x02                                                                              
					{
						Joint_LF_D.recieve    =  Jiont_Motor_Unpack.recieve ; 	 //下对应gama
						
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

  ///USART_ClearFlag ( UART4,USART_IT_IDLE  );
}




/**
  * @brief  关节电机命令发函数
  * @param  void
  * @retval void
  * @notes  
  */
void USART42CAN_TX_Send_Cmd (char CAN_DATA_CMD[8],char ID)
{
	                                      
	Tx_massage_u4.Data_Lenth  = 0x08;                                                     //帧长度为8
	Tx_massage_u4.Can_ID  = (ID<<21)&0xfff00000 ;                                                      //帧ID为接收信息的电机的ID
  

	Tx_massage_u4.Data[0] = CAN_DATA_CMD[0];             
	Tx_massage_u4.Data[1] = CAN_DATA_CMD[1];                
	Tx_massage_u4.Data[2] = CAN_DATA_CMD[2];           
	Tx_massage_u4.Data[3] = CAN_DATA_CMD[3];                
	Tx_massage_u4.Data[4] = CAN_DATA_CMD[4];               //203接收电流高8位
	Tx_massage_u4.Data[5] = CAN_DATA_CMD[5];                  //203接收电流低8位
	Tx_massage_u4.Data[6] = CAN_DATA_CMD[6];               //204接收电流高8位
	Tx_massage_u4.Data[7] = CAN_DATA_CMD[7];                  //204接收电流低8位
	
	 UART4_CAN_Tx[0]=0x41;
	 UART4_CAN_Tx[1]=0x54;
	 UART4_CAN_Tx[2]=bit32TObit8(3,Tx_massage_u4.Can_ID);
	 UART4_CAN_Tx[3]=bit32TObit8(2,Tx_massage_u4.Can_ID);
	 UART4_CAN_Tx[4]=bit32TObit8(1,Tx_massage_u4.Can_ID);
	 UART4_CAN_Tx[5]=bit32TObit8(0,Tx_massage_u4.Can_ID);
	 UART4_CAN_Tx[6]=Tx_massage_u4.Data_Lenth ;
	 UART4_CAN_Tx[7]=  Tx_massage_u4.Data[0]  ;
	 UART4_CAN_Tx[8]=  Tx_massage_u4.Data[1]  ;
	 UART4_CAN_Tx[9]=  Tx_massage_u4.Data[2]  ;
	 UART4_CAN_Tx[10]= Tx_massage_u4.Data[3]  ;
	 UART4_CAN_Tx[11]= Tx_massage_u4.Data[4]  ;
	 UART4_CAN_Tx[12]= Tx_massage_u4.Data[5]  ;
	 UART4_CAN_Tx[13]= Tx_massage_u4.Data[6]  ;
	 UART4_CAN_Tx[14]= Tx_massage_u4.Data[7]  ;
	 UART4_CAN_Tx[15]= 0x0d ;
	 UART4_CAN_Tx[16]= 0x0a ;

	

  DMA_Cmd(DMA1_Stream4,ENABLE);
	
}

/**
  * @brief  关节电机控制发送函数
  * @param  void
  * @retval void
  * @notes  
  */

void USART42CAN_TX_Jiont (CAN1_Data_TypeDef* motor)
{

	Tx_massage_u4.Data_Lenth = 0x08;                                                                      //帧长度为8
	Tx_massage_u4.Can_ID  = (motor->Motor_ID<<21)&0xfff00000;                                             //帧ID为电机ID
	
	
	
	//基于位置差的pid,kp增强控制
	motor->send.send_kp=BASE_JIONT_KP;//+fabs(Pid_Calc(motor->Motor_PID,motor->recieve .recieve_Position ,motor->send .send_Position));

	//KP限制
	if(motor->send.send_kp  >450)motor->send.send_kp=450;
	if(motor->send.send_kp  <=0)motor->send.send_kp=0;

	//位置限位
	if(motor->send.send_Position >=1.57f)motor->send.send_Position=1.57f;
	if(motor->send.send_Position <=-1.57f)motor->send.send_Position=-1.57f;

	Tx_massage_u4.Data[0] = (float_to_uint(motor->send.send_Position, P_MIN, P_MAX, 16)>>8)&0xff;             
	Tx_massage_u4.Data[1] = (float_to_uint(motor->send.send_Position, P_MIN, P_MAX, 16))&0xff;                
	Tx_massage_u4.Data[2] = (float_to_uint(motor->send.send_Velocity, V_MIN, V_MAX, 12)>>4)&0xff;           
	Tx_massage_u4.Data[3] = (float_to_uint(motor->send.send_Velocity, V_MIN, V_MAX, 12)<<4)|((float_to_uint(motor->send.send_kp, KP_MIN, KP_MAX, 12)>>8)&0x0f);                
	Tx_massage_u4.Data[4] = (float_to_uint(motor->send.send_kp, KP_MIN, KP_MAX, 12))&0xff;                     //203接收电流高8位
	Tx_massage_u4.Data[5] = (float_to_uint(motor->send.send_kd, KD_MIN, KD_MAX, 12)>>4)&0xff;                  //203接收电流低8位
	Tx_massage_u4.Data[6] = (float_to_uint(motor->send.send_kd, KD_MIN, KD_MAX, 12)<<4)|((float_to_uint(motor->send.send_torque, T_MIN, T_MAX, 12)>>8)&0x0f);               //204接收电流高8位
	Tx_massage_u4.Data[7] = (float_to_uint(motor->send.send_torque, T_MIN, T_MAX, 12) )&0xff;                  //204接收电流低8位
	
	

	
	 UART4_CAN_Tx[0]=0x41;
	 UART4_CAN_Tx[1]=0x54;
	 UART4_CAN_Tx[2]=bit32TObit8(3,Tx_massage_u4.Can_ID);
	 UART4_CAN_Tx[3]=bit32TObit8(2,Tx_massage_u4.Can_ID);
	 UART4_CAN_Tx[4]=bit32TObit8(1,Tx_massage_u4.Can_ID);
	 UART4_CAN_Tx[5]=bit32TObit8(0,Tx_massage_u4.Can_ID);
	 UART4_CAN_Tx[6]=Tx_massage_u4.Data_Lenth;
	 UART4_CAN_Tx[7]=  Tx_massage_u4.Data[0]  ;
	 UART4_CAN_Tx[8]=  Tx_massage_u4.Data[1]  ;
	 UART4_CAN_Tx[9]=  Tx_massage_u4.Data[2]  ;
	 UART4_CAN_Tx[10]= Tx_massage_u4.Data[3]  ;
	 UART4_CAN_Tx[11]= Tx_massage_u4.Data[4]  ;
	 UART4_CAN_Tx[12]= Tx_massage_u4.Data[5]  ;
	 UART4_CAN_Tx[13]= Tx_massage_u4.Data[6]  ;
	 UART4_CAN_Tx[14]= Tx_massage_u4.Data[7]  ;
	 UART4_CAN_Tx[15]= 0x0d ;
	 UART4_CAN_Tx[16]= 0x0a ;

	
	
  DMA_Cmd(DMA1_Stream4,ENABLE);
}








 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 

