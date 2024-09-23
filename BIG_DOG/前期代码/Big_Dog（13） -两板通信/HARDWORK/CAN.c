#include "main.h"


/****************************************数据定义区*********************************/
static int  CAN1_Tx_Flag;                         //CAN1发送完成标志位
static int  CAN2_Tx_Flag;                         //CAN1发送完成标志位
//关节电机CAN数据结构
CAN1_Data_TypeDef  Joint_LF_U,//左前上角度电机1
                   Joint_LF_D,//左前下角度电机2
									 Joint_LB_U,//左后上角度电机3
                   Joint_LB_D,//左后下角度电机4
									 Joint_RB_U,//右后上角度电机5
                   Joint_RB_D,//右后下角度电机6
									 Joint_RF_U,//右前上角度电机7
                   Joint_RF_D;//右前下角度电机8

//轮电机
CAN2_Data_TypeDef  Wheel_LF,//左前轮
                   Wheel_LB,//左后轮
									 Wheel_RB,//右后轮
									 Wheel_RF;//右前轮
		
//关节电机命令
 char CAN_DATA_CMD_ON[8]    =       {0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFC};
 char CAN_DATA_CMD_OFF[8]   =       {0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFD};
 char CAN_DATA_CMD_SET_0[8] =       {0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF1,0X00};
 char CAN_DATA_CMD_Change_ID[8] =   {0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XF0,0X06}; //最后一位为ID号


/***************************************************************函数定义区*************************************************************/
/*****************************************CAN1********************************/
 /**
  * @brief  CAN1通信初始化函数
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
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority  = 5;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority         = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd                 = ENABLE ;
	NVIC_Init(&NVIC_InitStruct);
	
   NVIC_InitStruct.NVIC_IRQChannel                    = CAN1_TX_IRQn;              //注意与中断向量表区别  refer to stm32f4xx.h file
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority  = 5;
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
	CAN_InitStruct.CAN_TXFP      = DISABLE;	                                        //优先级由报文标识符决定
	CAN_Init(CAN1,&CAN_InitStruct);
	


	
	CAN_FilterInitStruct.CAN_FilterIdHigh         = 0x0000;                         //32位ID
	CAN_FilterInitStruct.CAN_FilterIdLow          = 0x0000;                         //32位ID
	CAN_FilterInitStruct.CAN_FilterMaskIdHigh     = 0x0000;
	CAN_FilterInitStruct.CAN_FilterMaskIdLow      = 0x0000;
	CAN_FilterInitStruct.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;               //过滤器0关联到FIFO0;
	CAN_FilterInitStruct.CAN_FilterNumber         = 2;                              //过滤器0,范围0---13
	CAN_FilterInitStruct.CAN_FilterMode           = CAN_FilterMode_IdMask;
	CAN_FilterInitStruct.CAN_FilterScale          = CAN_FilterScale_32bit;
	CAN_FilterInitStruct.CAN_FilterActivation     = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStruct);
	
   CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);                                          //FIFO0消息挂号中断允许，打开接收中断
	 CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE);                                           //打开发送中断		
	
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

/**
  * @brief  关节电机命令发函数
  * @param  void
  * @retval void
  * @notes  
  */
void CAN1_Send_Cmd (char CAN_DATA_CMD[8],char ID)
{
	CanTxMsg CAN1_Tx_Message;

	CAN1_Tx_Message.IDE = CAN_ID_STD;                                               //标识符类型，标准帧
	CAN1_Tx_Message.RTR = CAN_RTR_DATA;                                             //架构类型，数据帧
	CAN1_Tx_Message.DLC = 0x08;                                                     //帧长度为8
	CAN1_Tx_Message.StdId = ID ;                                                    //帧ID为接收信息的电机的ID
  CAN1_Tx_Message.ExtId =0;                                                       //拓展ID

	CAN1_Tx_Message.Data[0] = CAN_DATA_CMD[0];             
	CAN1_Tx_Message.Data[1] = CAN_DATA_CMD[1];                
	CAN1_Tx_Message.Data[2] = CAN_DATA_CMD[2];           
	CAN1_Tx_Message.Data[3] = CAN_DATA_CMD[3];                
	CAN1_Tx_Message.Data[4] = CAN_DATA_CMD[4];               //203接收电流高8位
	CAN1_Tx_Message.Data[5] = CAN_DATA_CMD[5];                  //203接收电流低8位
	CAN1_Tx_Message.Data[6] = CAN_DATA_CMD[6];               //204接收电流高8位
	CAN1_Tx_Message.Data[7] = CAN_DATA_CMD[7];                  //204接收电流低8位
	
	CAN1_Tx_Flag = 0; 
	CAN_Transmit(CAN1,&CAN1_Tx_Message);

	while(CAN1_Tx_Flag == 0); 
}


/**
  * @brief  关节电机控制发送函数
  * @param  void
  * @retval void
  * @notes  
  */

void CAN1_TX_Jiont (CAN1_Data_TypeDef* motor)
{
	CanTxMsg CAN1_Tx_Message;
	CAN1_Tx_Message.IDE = CAN_ID_STD;                                               //标准帧
	CAN1_Tx_Message.RTR = CAN_RTR_DATA;                                             //数据帧
	CAN1_Tx_Message.DLC = 0x08;                                                     //帧长度为8
	CAN1_Tx_Message.StdId = motor->Motor_ID;                                        //帧ID为电机ID
	
	
	
	//基于位置差的pid,kp增强控制
	motor->send.send_kp=BASE_JIONT_KP;//+fabs(Pid_Calc(motor->Motor_PID,motor->recieve .recieve_Position ,motor->send .send_Position));

	//KP限制
	if(motor->send.send_kp  >450)motor->send.send_kp=450;
	if(motor->send.send_kp  <=0)motor->send.send_kp=0;

	//位置限位
	if(motor->send.send_Position >=1.57f)motor->send.send_Position=1.57f;
	if(motor->send.send_Position <=0)motor->send.send_Position=0;

	CAN1_Tx_Message.Data[0] = (float_to_uint(motor->send.send_Position, P_MIN, P_MAX, 16)>>8)&0xff;             
	CAN1_Tx_Message.Data[1] = (float_to_uint(motor->send.send_Position, P_MIN, P_MAX, 16))&0xff;                
	CAN1_Tx_Message.Data[2] = (float_to_uint(motor->send.send_Velocity, V_MIN, V_MAX, 12)>>4)&0xff;           
	CAN1_Tx_Message.Data[3] = (float_to_uint(motor->send.send_Velocity, V_MIN, V_MAX, 12)<<4)|((float_to_uint(motor->send.send_kp, KP_MIN, KP_MAX, 12)>>8)&0x0f);                
	CAN1_Tx_Message.Data[4] = (float_to_uint(motor->send.send_kp, KP_MIN, KP_MAX, 12))&0xff;                     //203接收电流高8位
	CAN1_Tx_Message.Data[5] = (float_to_uint(motor->send.send_kd, KD_MIN, KD_MAX, 12)>>4)&0xff;                  //203接收电流低8位
	CAN1_Tx_Message.Data[6] = (float_to_uint(motor->send.send_kd, KD_MIN, KD_MAX, 12)<<4)|((float_to_uint(motor->send.send_torque, T_MIN, T_MAX, 12)>>8)&0x0f);               //204接收电流高8位
	CAN1_Tx_Message.Data[7] = (float_to_uint(motor->send.send_torque, T_MIN, T_MAX, 12) )&0xff;                  //204接收电流低8位
	
	CAN1_Tx_Flag = 0; 
	CAN_Transmit(CAN1,&CAN1_Tx_Message);

	while(CAN1_Tx_Flag == 0); 
}









/**
  * @brief  CAN1接收中断函数
  * @param  void
  * @retval void
  * @notes  接收单个电机的角度、速度、电流
  */
	CAN1_Data_TypeDef  Jiont_Motor_Unpack_CAN1;
  CanRxMsg CAN1_Rx_Message;
void CAN1_RX0_IRQHandler(void)                                          
{   

	if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET)
	{
		  CAN_Receive(CAN1,CAN_FIFO0,&CAN1_Rx_Message);
		  
			if ((CAN1_Rx_Message.IDE == CAN_Id_Standard) && (CAN1_Rx_Message.RTR == CAN_RTR_Data) && (CAN1_Rx_Message.DLC == 6) )//标准帧、数据帧、数据长度为6字节
			{	
				//接收参数
				Jiont_Motor_Unpack_CAN1.Motor_ID                  = CAN1_Rx_Message.Data[0];				
				Jiont_Motor_Unpack_CAN1.recieve.recieve_Position	 = uint_to_float((CAN1_Rx_Message.Data[1]<<8)|(CAN1_Rx_Message.Data[2]), P_MIN, P_MAX, 16);
				Jiont_Motor_Unpack_CAN1.recieve.recieve_Velocity  = uint_to_float((CAN1_Rx_Message.Data[3]<<4)|( CAN1_Rx_Message.Data[4]>>4), V_MIN, V_MAX, 12);
				Jiont_Motor_Unpack_CAN1.recieve.recieve_Current   = uint_to_float(((CAN1_Rx_Message.Data[4]&0x0f)<<4)|( CAN1_Rx_Message.Data[5]), -T_MAX, T_MAX, 12);
       
				switch (Jiont_Motor_Unpack_CAN1.Motor_ID )
				{			
	
					//关节,左前
          case CAN1_JOINT_ID_LF_U:                                                                                
					{
						Joint_LF_U.recieve   =  Jiont_Motor_Unpack_CAN1.recieve ; 			//上对应seigamar
					}break ; 
					
					case CAN1_JOINT_ID_LF_D:                                                                               
					{
						Joint_LF_D.recieve    =  Jiont_Motor_Unpack_CAN1.recieve ; 	 //下对应gama
					}break ;
					
				  case CAN1_JOINT_ID_LB_U:                                                                                
					{
						Joint_LB_U.recieve   =  Jiont_Motor_Unpack_CAN1.recieve ; 			
					}break ;
					
					case CAN1_JOINT_ID_LB_D:                                                                               
					{
						Joint_LB_D.recieve    =  Jiont_Motor_Unpack_CAN1.recieve ; 		 
					}break ;
					
					case CAN1_JOINT_ID_RB_U:                                                                                
					{
						Joint_RB_U.recieve   =  Jiont_Motor_Unpack_CAN1.recieve ; 			
					}break ;
					
					case CAN1_JOINT_ID_RB_D:                                                                               
					{
						Joint_RB_D.recieve    =  Jiont_Motor_Unpack_CAN1.recieve ; 		 	
					}break ;
					
					
					case CAN1_JOINT_ID_RF_U:                                                                                
					{
						Joint_RF_U.recieve   =  Jiont_Motor_Unpack_CAN1.recieve ; 			
					}break ;
					
					case CAN1_JOINT_ID_RF_D:                                                                               
					{
						Joint_RF_D.recieve    =  Jiont_Motor_Unpack_CAN1.recieve ; 		
					}break ;
					

					default:
					{}
					break ;	   
				}




			}
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);		
	}
}

/*****************************************CAN2********************************/
/**
  * @brief  CAN2通信初始化函数
  * @param  void
  * @retval void
  * @notes  发送中断优先级为8，接收中断优先级为
    *CAN2 GPIO Configuration    
    PB5     ------> CAN2_RX
    PB6     ------> CAN2_TX 
		
  */
void CAN2_Init(void)
{
	GPIO_InitTypeDef      GPIO_InitStruct;
	NVIC_InitTypeDef      NVIC_InitStruct;
	CAN_InitTypeDef       CAN_InitStruct;
	CAN_FilterInitTypeDef CAN_FilterInitStruct;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE); //先CAN1 后CAN2
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_CAN2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_CAN2);
	
	GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(GPIOB,&GPIO_InitStruct);
	
	NVIC_InitStruct.NVIC_IRQChannel                    = CAN2_RX0_IRQn;             //注意与中断向量表区别  refer to stm32f4xx.h file
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority  = 7;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority         = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd                 = ENABLE ;
	NVIC_Init(&NVIC_InitStruct);
	
  NVIC_InitStruct.NVIC_IRQChannel                    = CAN2_TX_IRQn;              //注意与中断向量表区别  refer to stm32f4xx.h file
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority  = 8;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority         = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd                 = ENABLE ;
	NVIC_Init(&NVIC_InitStruct);
  
  CAN_DeInit(CAN2);
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
	CAN_Init(CAN2,&CAN_InitStruct);
	
	CAN_FilterInitStruct.CAN_FilterIdHigh         = 0x0000;                         //32位ID
	CAN_FilterInitStruct.CAN_FilterIdLow          = 0x0000;                         //32位ID
	CAN_FilterInitStruct.CAN_FilterMaskIdHigh     = 0x0000;
	CAN_FilterInitStruct.CAN_FilterMaskIdLow      = 0x0000;
	CAN_FilterInitStruct.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;               //过滤器0关联到FIFO0;
	CAN_FilterInitStruct.CAN_FilterNumber         = 14;                             //CAN2的过滤器必须修改为14，否则进不了接收中断
	CAN_FilterInitStruct.CAN_FilterMode           = CAN_FilterMode_IdMask;
	CAN_FilterInitStruct.CAN_FilterScale          = CAN_FilterScale_32bit;
	CAN_FilterInitStruct.CAN_FilterActivation     = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStruct);
	
  CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);                                          //FIFO0消息挂号中断允许，打开接收中断
	CAN_ITConfig(CAN2,CAN_IT_TME,ENABLE);                                           //打开发送中断



}


/**
 * @Function : CAN2 send interrupt
 * @Input    : void
 * @Output   : void
 * @Notes    : void
 * @Copyright: Aword
 **/
void CAN2_TX_IRQHandler(void)
{
  if (CAN_GetITStatus (CAN2,CAN_IT_TME)!=RESET)                                 //if transmit mailbox is empty     
	{
		CAN2_Tx_Flag=1;
		CAN_ClearITPendingBit(CAN2,CAN_IT_TME);    
	}
}

/**
 * @Function : CAN2 Shoot send
 * @Input    : void
 * @Output   : void
 * @Notes    : ID 0x200
 * @Copyright: Aword
 **/
void CAN2_TX(void)
{
	
	//pid计算
	Wheel_LF.Send_Current=Pid_Calc(&Wheel_Motor_PID_201,Wheel_LF.Real_Speed,-Wheel_LF.Target_Speed);
	Wheel_LB.Send_Current=Pid_Calc(&Wheel_Motor_PID_202,Wheel_LB.Real_Speed,-Wheel_LB.Target_Speed);
  Wheel_RB.Send_Current=Pid_Calc(&Wheel_Motor_PID_203,Wheel_RB.Real_Speed, Wheel_RB.Target_Speed);
	Wheel_RF.Send_Current=Pid_Calc(&Wheel_Motor_PID_204,Wheel_RF.Real_Speed, Wheel_RF.Target_Speed);
	
	
	CanTxMsg CAN2_Tx_Message;

	CAN2_Tx_Message.IDE = CAN_ID_STD;                                               //标准帧
	CAN2_Tx_Message.RTR = CAN_RTR_DATA;                                             //数据帧
	CAN2_Tx_Message.DLC = 0x08;                                                     //帧长度为8
	CAN2_Tx_Message.StdId = CAN2_TRANSMIT_ID;                                //帧ID为传入参数的CAN_ID
	
	CAN2_Tx_Message.Data[0] = (Wheel_LF.Send_Current >>8)&0xff;              //201接收电流高8位
	CAN2_Tx_Message.Data[1] =  Wheel_LF.Send_Current&0xff;                   //201接收电流低8位
	CAN2_Tx_Message.Data[2] = (Wheel_LB.Send_Current>>8)&0xff;               //202接收电流高8位
	CAN2_Tx_Message.Data[3] =  Wheel_LB.Send_Current&0xff;                   //202接收电流低8位
	CAN2_Tx_Message.Data[4] = (Wheel_RB.Send_Current>>8)&0xff;               //203接收电流高8位
	CAN2_Tx_Message.Data[5] =  Wheel_RB.Send_Current&0xff;                   //203接收电流低8位
	CAN2_Tx_Message.Data[6] = (Wheel_RF.Send_Current>>8)&0xff;               //204接收电流高8位
	CAN2_Tx_Message.Data[7] =  Wheel_RF.Send_Current&0xff;                   //204接收电流低8位
 
	CAN2_Tx_Flag = 0;
	CAN_Transmit(CAN2,&CAN2_Tx_Message);

	while(CAN2_Tx_Flag == 0); 
}



/**
 * @Function : CAN2 receive interrupt
 * @Input    : void
 * @Output   : void
 * @Notes    : void
 * @Copyright: Aword
 **/

	
void CAN2_RX0_IRQHandler(void)            //中断向量表参考“startup_stm32f40_41xxx.s”
{   
  CanRxMsg CAN2_Rx_Message;
	if (CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET)
	{
		CAN_Receive(CAN2,CAN_FIFO0,&CAN2_Rx_Message);
		if ( (CAN2_Rx_Message.IDE == CAN_Id_Standard) && (CAN2_Rx_Message.RTR == CAN_RTR_Data) && (CAN2_Rx_Message.DLC == 8) )//标准帧、数据帧、数据长度为8字节
		{
			switch (CAN2_Rx_Message.StdId)
			{
				case CAN2_LF:     //轮电机--左前-->0x201                  
				{
					
					Wheel_LF.Real_Speed=            (CAN2_Rx_Message.Data[2]<<8)|(CAN2_Rx_Message.Data[3]);   //转速	
				
				}break ;
				
				
				case CAN2_LB:     //轮电机--左后-->0x202   
				{
					
					Wheel_LB.Real_Speed=          (CAN2_Rx_Message.Data[2]<<8)|(CAN2_Rx_Message.Data[3]);		//转速
				
				}break ; 
				
				
				case CAN2_RB:      //轮电机--右后-->0x203                                                                  
				{
					
					Wheel_RB.Real_Speed          =(CAN2_Rx_Message.Data[2]<<8)|(CAN2_Rx_Message.Data[3]);    //转速
				
				}break ;
				
				
				case CAN2_RF:      //轮电机--右前-->0x204                                                                  
				{
					
					Wheel_RF.Real_Speed          =(CAN2_Rx_Message.Data[2]<<8)|(CAN2_Rx_Message.Data[3]);   //转速
				
				}break ;
				
				default:
        {}
        break ;
					
			}
	
		}
		CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);		
	}
}




















/*******************************************数据格式转换函数**************************************************/
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



