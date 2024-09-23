#include "main.h"


/****************************************数据定义区*********************************/
CAN1_Data_TypeDef Joint_LF_U,
                  Joint_LF_D;


									
static int  CAN1_Tx_Flag;                         //CAN发送完成标志位




 #define P_MIN -12.5f
 #define P_MAX 12.5f
 #define V_MIN -45.0f
 #define V_MAX 45.0f
 #define KP_MIN 0.0f
 #define KP_MAX 500.0f
 #define KD_MIN 0.0f
 #define KD_MAX 5.0f
 #define T_MIN -18.0f
 #define T_MAX 18.0f
 
 
 

/*****************************************函数定义区********************************/
/**
  * @brief  CAN通信初始化函数
  * @param  void
  * @retval void
  * @notes  发送中断优先级为8，接收中断优先级为9
	  *CAN1 GPIO Configuration    
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX 
    *CAN2 GPIO Configuration    
    PB5     ------> CAN2_RX
    PB6     ------> CAN2_TX 
		
  */
void CAN1_Init(void)
{
	GPIO_InitTypeDef      GPIO_InitStruct;
	NVIC_InitTypeDef      NVIC_InitStruct;
	CAN_InitTypeDef       CAN_InitStruct;
	CAN_FilterInitTypeDef CAN_FilterInitStruct;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_CAN1);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_CAN1);
	
	GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(GPIOD,&GPIO_InitStruct);
	
	NVIC_InitStruct.NVIC_IRQChannel                    = CAN1_RX0_IRQn;             //注意与中断向量表区别  refer to stm32f4xx.h file
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority  = 9;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority         = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd                 = ENABLE ;
	NVIC_Init(&NVIC_InitStruct);
	
   NVIC_InitStruct.NVIC_IRQChannel                    = CAN1_TX_IRQn;              //注意与中断向量表区别  refer to stm32f4xx.h file
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority  = 8;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority         = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd                 = ENABLE ;
	NVIC_Init(&NVIC_InitStruct);
  
   CAN_DeInit(CAN1);
   CAN_StructInit(&CAN_InitStruct);
    
	CAN_InitStruct.CAN_Prescaler = 3;                                               //CAN BaudRate 42/(1+9+4)/3=1Mbps  //分频系数(Fdiv)为3+1
	CAN_InitStruct.CAN_Mode      = CAN_Mode_Normal;                                 //模式设置 
	CAN_InitStruct.CAN_SJW       = CAN_SJW_1tq;                                     //重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1tq~CAN_SJW_4tq
	CAN_InitStruct.CAN_BS1       = CAN_BS1_9tq;                                     //Tbs1范围CAN_BS1_1tq ~CAN_BS1_16tq
	CAN_InitStruct.CAN_BS2       = CAN_BS2_4tq;                                     //Tbs2范围CAN_BS2_1tq ~	CAN_BS2_8tq
	CAN_InitStruct.CAN_TTCM      = DISABLE;                                         //非时间触发通信模式
	CAN_InitStruct.CAN_ABOM      = DISABLE;                                         //软件自动离线管理	  
	CAN_InitStruct.CAN_AWUM      = DISABLE;                                         //睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
	CAN_InitStruct.CAN_NART      = DISABLE;	                                        //禁止报文自动传送
	CAN_InitStruct.CAN_RFLM      = DISABLE;                                         //报文不锁定,新的覆盖旧的
	CAN_InitStruct.CAN_TXFP      = ENABLE;	                                        //优先级由报文标识符决定
	CAN_Init(CAN1,&CAN_InitStruct);
	
	CAN_FilterInitStruct.CAN_FilterIdHigh         = 0x0000;                         //32位ID
	CAN_FilterInitStruct.CAN_FilterIdLow          = 0x0000;                         //32位ID
	CAN_FilterInitStruct.CAN_FilterMaskIdHigh     = 0x0000;
	CAN_FilterInitStruct.CAN_FilterMaskIdLow      = 0x0000;
	CAN_FilterInitStruct.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;               //过滤器0关联到FIFO0;
	CAN_FilterInitStruct.CAN_FilterNumber         = 0;                              //过滤器0,范围0---13
	CAN_FilterInitStruct.CAN_FilterMode           = CAN_FilterMode_IdMask;
	CAN_FilterInitStruct.CAN_FilterScale          = CAN_FilterScale_32bit;
	CAN_FilterInitStruct.CAN_FilterActivation     = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStruct);
	
   CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);                                          //FIFO0消息挂号中断允许，打开接收中断
	CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE);                                           //打开发送中断	
	
	
	//电机
	
	
	
	
	
	
	
	
	
	
}












/**
  * @brief  摩擦轮CAN1发送函数
  * @param  void
  * @retval void
  * @notes  底盘数据发送（广播ID 0x200）给定电流，电流值范围：-16384~+16384
  */
CanTxMsg CAN1_Tx_Message;
void CAN1_TX_Jiont (CAN1_Data_TypeDef* motor)
{
	

	CAN1_Tx_Message.IDE = CAN_ID_STD;                                               //标准帧
	CAN1_Tx_Message.RTR = CAN_RTR_DATA;                                             //数据帧
	CAN1_Tx_Message.DLC = 0x08;                                                     //帧长度为8
	CAN1_Tx_Message.StdId = motor->Motor_ID;                                   //帧ID为传入参数的CAN_ID


	CAN1_Tx_Message.Data[0] = (float_to_uint(motor->send_Position, P_MIN, P_MAX, 16)>>8)&0xff;             
	CAN1_Tx_Message.Data[1] = (float_to_uint(motor->send_Position, P_MIN, P_MAX, 16))&0xff;                
	CAN1_Tx_Message.Data[2] = (float_to_uint(motor->send_Velocity, V_MIN, V_MAX, 12)>>4)&0xff;           
	CAN1_Tx_Message.Data[3] = (float_to_uint(motor->send_Velocity, V_MIN, V_MAX, 12)<<4)|((float_to_uint(motor->send_kp, KP_MIN, KP_MAX, 12)>>8)&0x0f);                
	CAN1_Tx_Message.Data[4] = (float_to_uint(motor->send_kp, KP_MIN, KP_MAX, 12))&0xff;                     //203接收电流高8位
	CAN1_Tx_Message.Data[5] = (float_to_uint(motor->send_kd, KD_MIN, KD_MAX, 12)>>4)&0xff;                  //203接收电流低8位
	CAN1_Tx_Message.Data[6] = (float_to_uint(motor->send_kd, KD_MIN, KD_MAX, 12)<<4)|((float_to_uint(motor->send_torque, T_MIN, T_MAX, 12)>>8)&0x0f);               //204接收电流高8位
	CAN1_Tx_Message.Data[7] = (float_to_uint(motor->send_torque, T_MIN, T_MAX, 12) )&0xff;                  //204接收电流低8位
	
	CAN1_Tx_Flag = 0; 
	CAN_Transmit(CAN1,&CAN1_Tx_Message);

	while(CAN1_Tx_Flag == 0); 
}

void CAN1_Enlab ()
{
	CanTxMsg CAN1_Tx_Message;

	CAN1_Tx_Message.IDE = CAN_ID_STD;                                               //标准帧
	CAN1_Tx_Message.RTR = CAN_RTR_DATA;                                             //数据帧
	CAN1_Tx_Message.DLC = 0x08;                                                     //帧长度为8
	CAN1_Tx_Message.StdId = 1;                                                     //帧ID为传入参数的CAN_ID


	CAN1_Tx_Message.Data[0] = 0xff;             
	CAN1_Tx_Message.Data[1] = 0xff;                
	CAN1_Tx_Message.Data[2] = 0xff;           
	CAN1_Tx_Message.Data[3] = 0xff;                
	CAN1_Tx_Message.Data[4] = 0xff;               //203接收电流高8位
	CAN1_Tx_Message.Data[5] = 0xff;                  //203接收电流低8位
	CAN1_Tx_Message.Data[6] = 0xff;               //204接收电流高8位
	CAN1_Tx_Message.Data[7] = 0xfc;                  //204接收电流低8位
	
	CAN1_Tx_Flag = 0; 
	CAN_Transmit(CAN1,&CAN1_Tx_Message);

	while(CAN1_Tx_Flag == 0); 
}

/**
  * @brief  CAN1发送中断函数
  * @param  void
  * @retval void
  * @notes  void
  */
void CAN1_TX_IRQHandler(void)
{
  if (CAN_GetITStatus (CAN1,CAN_IT_TME)!=RESET)                                 //if transmit mailbox is empty     
	{
		CAN1_Tx_Flag=1;
		CAN_ClearITPendingBit(CAN1,CAN_IT_TME);    
	}
}

	CanRxMsg CAN1_Rx_Message;

	CAN1_Data_TypeDef  Jiont_Motor_Unpack;
/**
  * @brief  CAN1接收中断函数
  * @param  void
  * @retval void
  * @notes  每1ms接收一次，接收完之后分别对云台进行PITCH角度处理（为了防越界跳变），
            对YAW轴进行处理，主要是判断云台是否处于锁定状态
  */
void CAN1_RX0_IRQHandler(void)                                          
{   

	
	if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET)
	{
		  CAN_Receive(CAN1,CAN_FIFO0,&CAN1_Rx_Message);
		  
			if ( (CAN1_Rx_Message.IDE == CAN_Id_Standard) && (CAN1_Rx_Message.RTR == CAN_RTR_Data) && (CAN1_Rx_Message.DLC == 8) )//标准帧、数据帧、数据长度为8字节
			{	
			  Jiont_Motor_Unpack.Motor_ID = CAN1_Rx_Message.Data[0];
		    Jiont_Motor_Unpack.recieve_Position = (CAN1_Rx_Message.Data[1]>>8)|(CAN1_Rx_Message.Data[2]);
		    Jiont_Motor_Unpack.recieve_Velocity = (CAN1_Rx_Message.Data[3]<<4)|( CAN1_Rx_Message.Data[4]>>4);
		    Jiont_Motor_Unpack.recieve_Current = ((CAN1_Rx_Message.Data[4]&0x0f)<<4)|( CAN1_Rx_Message.Data[5]);
				switch (Jiont_Motor_Unpack.Motor_ID )
				{			
	
					//关节
          case CAN1_JOINT_ID_LF_U:                                                                                
					{
						Joint_LF_U   =  Jiont_Motor_Unpack; 			//转速
					}break ;
					
					case CAN1_JOINT_ID_LF_D:                                                                               
					{
						Joint_LF_U   =  Jiont_Motor_Unpack; 		 //转速	
					}break ;
					

					default:
					{}
					break ;	   
				}
			}
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);		
	}
}







/*********************************************************************************************/
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



