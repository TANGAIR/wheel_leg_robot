#include "main.h"

/**********************************���ݶ�����*****************************/
Send_TX2_Data_Typedef    Send_TX2_Data_TypeStruct;
Receive_TX2_Data_Typedef Receive_TX2_Data_TypeStruct;

char   Usart3Rx_Info[10];//��������������
char        Send_TX2[10];//��������������


/**********************************����6��ӡ***************************/
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 
FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
//���ڴ�ӡ����
int fputc(int ch, FILE *f)
{
	while (USART_GetFlagStatus(USART6,USART_FLAG_TC) == RESET);//�ȴ�֮ǰ���ַ��������
	USART_SendData(USART6, (uint8_t)ch);
	return (ch);
}
#endif
/**
  * @brief  ��ʼ��USART6
  * @param  void
  * @retval void
  * @notes  ����6��ӡ��ʼ��    USART6_TX-PG14      USART6_RX -----PG9
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
	GPIO_InitSturct.GPIO_OType=GPIO_OType_PP;                                      //�������
	GPIO_InitSturct.GPIO_Mode=GPIO_Mode_AF;                                        //����ģʽ
	GPIO_InitSturct.GPIO_PuPd=GPIO_PuPd_NOPULL;                                    //������ 
	GPIO_InitSturct.GPIO_Speed=GPIO_High_Speed;                                    //100MHZ
	GPIO_Init(GPIOG,&GPIO_InitSturct);
	
	USART_InitStruct.USART_BaudRate=115200;
	USART_InitStruct.USART_HardwareFlowControl=USART_HardwareFlowControl_None;     //Ӳ����������
	USART_InitStruct.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;                       //�շ�ģʽ
	USART_InitStruct.USART_Parity=USART_Parity_No;                                 //����żУ��λ
	USART_InitStruct.USART_StopBits=USART_StopBits_1;                              //�ֽ�
	USART_InitStruct.USART_WordLength=USART_WordLength_8b;                         //���ݳ���
	USART_Init(USART6,&USART_InitStruct);
	USART_Cmd(USART6,ENABLE);
	USART_ITConfig(USART6,USART_IT_RXNE,ENABLE);
	
	NVIC_InitStruct.NVIC_IRQChannel=USART6_IRQn;                                   //�ж�ͨ��
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;                                     //ʹ��
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=15;                           //��ռ���ȼ�
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=0;                                  //�����ȼ�
	NVIC_Init(&NVIC_InitStruct);	
}


/**
  * @brief  ����6��ӡ�ж�
  * @param  void
  * @retval void
  * @notes  ��ʱû��
  */
void USART6_IRQHandler(void)
{
  if ( USART_GetITStatus( USART6, USART_IT_RXNE ) != RESET )
  { //���������յ������ݸ���
  }
  if ( USART_GetITStatus( USART6, USART_IT_ORE_RX ) != RESET )
  { 
		
  }
  USART_ClearFlag ( USART6,USART_IT_RXNE | USART_IT_ORE_RX );
}



/*********************************************������TX2����ͨ��**********************************************************************/
/**
  * @brief  USART3-TX2ͨ�ų�ʼ��
  * @param  void
  * @retval void
  * @notes  USART3_RX -----PD9    USART3_TX-----PD8     
            ������DMA1_Stream3��DMA_Channel_4��ʹ��DMA1_Stream3_IRQHandler
            ������DMA1_Stream1��DMA_Channel_4��ʹ��USART3_IRQHandler
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
//�����ж�
	NVIC_InitStruct.NVIC_IRQChannel                   =USART3_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority =6;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority        =0;
	NVIC_InitStruct.NVIC_IRQChannelCmd                =ENABLE ;
	NVIC_Init(&NVIC_InitStruct);
//���ͺ��ж�
	NVIC_InitStruct.NVIC_IRQChannel                   =DMA1_Stream3_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority =6;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority        =0;
	NVIC_InitStruct.NVIC_IRQChannelCmd                =ENABLE ;
	NVIC_Init(&NVIC_InitStruct);
	
/* ------------------------------------- Configure DMA  ����---------------------------------------*/

	USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);
	
	DMA_Cmd(DMA1_Stream1,DISABLE);
	DMA_DeInit(DMA1_Stream1);
	
	DMA_InitStruct.DMA_Channel=DMA_Channel_4;                                      //ͨ�����ã�ͨ��5��
	DMA_InitStruct.DMA_BufferSize=15;                                              //����������Ŀ                
	DMA_InitStruct.DMA_DIR=DMA_DIR_PeripheralToMemory;                             //���ݴ��䷽�����衪���洢����                   
	DMA_InitStruct.DMA_FIFOMode=ENABLE;                                            //ʹ��FIFOģʽ
	DMA_InitStruct.DMA_FIFOThreshold=DMA_FIFOThreshold_1QuarterFull;               //��ֵ1/4      
	DMA_InitStruct.DMA_Memory0BaseAddr=(uint32_t)Usart3Rx_Info;                    //�洢�����ݵ�ַ
	DMA_InitStruct.DMA_MemoryBurst=DMA_MemoryBurst_Single;                         //�洢��ͻ������
	DMA_InitStruct.DMA_MemoryDataSize=DMA_MemoryDataSize_Byte;                     //�洢�����ݸ�ʽ���ֽڣ�
	DMA_InitStruct.DMA_MemoryInc=DMA_MemoryInc_Enable;                             //�ڴ��ַ����
	DMA_InitStruct.DMA_Mode=DMA_Mode_Circular;                                     //�Ƿ�ѭ�����ͣ�ѭ������ͨģʽֻ�ܽ���һ�Σ�
	DMA_InitStruct.DMA_PeripheralBaseAddr=(uint32_t)&(USART3->DR);                 //�����ַ 
	DMA_InitStruct.DMA_PeripheralBurst=DMA_PeripheralBurst_Single;                 //����ͻ������         
	DMA_InitStruct.DMA_PeripheralDataSize=DMA_PeripheralDataSize_Byte;             //�������ݸ�ʽ
	DMA_InitStruct.DMA_PeripheralInc=DMA_PeripheralInc_Disable;                    //�����ַ������
	DMA_InitStruct.DMA_Priority=DMA_Priority_VeryHigh;                             //���ȼ��ǳ���      
	DMA_Init(DMA1_Stream1,&DMA_InitStruct);
	DMA_Cmd(DMA1_Stream1,ENABLE);

///*********************************************DMA����************************************/
	
	USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);
	
	DMA_Cmd(DMA1_Stream3,DISABLE);
  DMA_DeInit(DMA1_Stream3);                                                 //ΪDMA����ͨ��
	DMA_InitStruct.DMA_Channel           =DMA_Channel_4;
	DMA_InitStruct.DMA_PeripheralBaseAddr=(uint32_t)&(	USART3->DR);          //��ʼ��ַ
	DMA_InitStruct.DMA_Memory0BaseAddr   =(uint32_t)Send_TX2;                 //�洢����
	DMA_InitStruct.DMA_DIR               =DMA_DIR_MemoryToPeripheral;         //���䷽��
	DMA_InitStruct.DMA_BufferSize        =15; //����������
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
	DMA_Init(DMA1_Stream3,&DMA_InitStruct);
	
	DMA_ITConfig(DMA1_Stream3,DMA_IT_TC,ENABLE);
	
 
	delay_ms(3);
}
/**
  * @brief  TX2 DMA�����������жϷ�����
  * @param  void
  * @retval void
  * @notes  ÿ����������������֮����������жϷ����������ͺ����������ǻ�ȡ�����б����ã�ÿ1msִ��һ��
            �ж��н�DMA�������رղ�����������DNA���ݳ���
            
  */
void DMA1_Stream3_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_Stream3,DMA_IT_TCIF3)==SET)
	{
	 //�ر�dma������
	 DMA_Cmd(DMA1_Stream3,DISABLE);
	 //��������dma���ݳ���
	 DMA_SetCurrDataCounter(DMA1_Stream3,10);
	}
	DMA_ClearITPendingBit(DMA1_Stream3,DMA_IT_TCIF3);	
}
/**
  * @brief  TX2 ����3���ݽ����жϷ�����
  * @param  void
  * @retval void
  * @notes  ���������յ�TX2����������֮���Ƚ�dma�������ر������������ݳ���֮���ٴ�������
            Ȼ��ת��TX2���ݴ�������DMA����USART3->DR��Usart3Rx_Info
  */
void USART3_IRQHandler(void)
{
	
	BaseType_t pxHigherPriorityTaskWoken;
	char num=0;
	if(USART_GetITStatus(USART3,USART_IT_IDLE)==SET)
	{
		 num = USART3->SR;
     num = USART3->DR; //������ڿ����жϣ������ֲ�712ҳ��
		 num++;            //û�����壬ֻ����������
		
		 DMA_Cmd(DMA1_Stream1,DISABLE); 
		 DMA_SetCurrDataCounter(DMA1_Stream1,10);      //�������ý������ݳ���
		 DMA_Cmd(DMA1_Stream1,ENABLE);                 //���䣬������3���ݼĴ��������ݴ��䵽Usart3Rx_Info
		
		//��TX2���ݽ���������֪ͨ���������������б�
		 vTaskNotifyGiveFromISR(TX2_Decode_Task_Handler,&pxHigherPriorityTaskWoken);
		//���������л�
		 portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
	}	
}
/******************************���嵽TX2ͨ�����ݷ���****************************
	
  char  Left_Front_Leg_X;  //��ǰ������ڹؽ�����ϵX������,��λCM���Թؽڶ������Ϊԭ�㣬������ǰ������ΪX��������
	char  Left_Front_Leg_Y;  //��ǰ������ڹؽ�����ϵY������,��λCM���Թؽڶ������Ϊԭ�㣬����ΪY��������
	char  Right_Front_Leg_X; //��ǰ������ڹؽ�����ϵX�����꣬��λCM���Թؽڶ������Ϊԭ�㣬������ǰ������ΪX��������
	char  Right_Front_Leg_Y; //��ǰ������ڹؽ�����ϵY�����꣬��λCM���Թؽڶ������Ϊԭ�㣬����ΪY��������
	short IMU_PITCH;         //�����Ǹ�����PITCH����λΪ�㣬G_Y������Ϊ��������Ϊ��
	short IMU_YAW;           //�����Ǻ����YAW����λΪ�㣬G_Z����������Ϊ��������Ϊ��
	char  Left_Front_Wheel_Speed;//��ǰ�ֵ���ٶȣ���λCM/S
	
*************************************************************************************/
/**
  * @brief  ���������ݷ��ͺ���
  * @param  void
  * @retval void
  * @notes  ����̨yaw���pitch������ݾ�������֮�󴢴�Send_TX2������
            Ȼ������dma�����������ݷ��ͳ�ȥ�������������ݽ��뺯���б�����
            ÿ1ms����һ��Send_TX2��Send_TX2���ٷ���
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
	//��Send_TX2��Send_TX2
	DMA_Cmd(DMA1_Stream3,ENABLE);
}




/*******************************TX2������ͨ�����ݷ���*****************************
	char  Dog_Mode;      //������ģʽ,1����ʽ����ģʽ��2����¥��ģʽ��3����¥��ģʽ��0��ֹͣ
	char  Climb_Status;  //��¥״̬,1��ǰ�����к��Ȼ��У�2��ǰ���ȶ����У�3��ǰ�Ȼ��к�������
	char  Front_Leg_Distance; //��¥ʱ����ǰ�������¥���е��ˮƽ����,��λCM����ǰ������Ϊ�������е�Ϊ0
	char  Stair_Higth;        //¥�ݸ߶ȣ���λCM
	char  Stair_Width;        //¥�ݿ�ȣ���λCM
	char  Slide_Speed;        //ƽ�ػ����ٶȣ���λCM/S����¥ʱΪ0�����ڿ��ƻ���������ģʽʱ�Ļ����ٶ�
	char  Turn_Angle;         //��������ת��,��λ�㣬���ڿ��ƻ�����ת��
	char  Head_Pitch_Angle;   //ͷ���PITCH�Ƕ�,��λ�㣬���ڿ��ƻ�����̧ͷ��ͷ
	char  Head_Yaw_Angle;     //ͷ���YAW�Ƕ�,��λ�㣬���ڿ��ƻ���������תͷ
**********************************************************************************/
/**
  * @brief  TX2���ݽ�������
  * @param  void
  * @retval void
  * @notes  ÿ�ν�����TX2������������ʱִ��һ��
            TX2��ʼ������ʼ������3������TX2��ͨѶ������TX2�������񣬵����ݾ���У��֮��˵��TX2�Ѿ��������ˣ����ʱ��TX2��ʧʱ�����0��
            ��TX2�������������е�װ�װ����Ͳ�Ϊ0��ʱ��˵���Ѿ�ʶ���˵��ˣ���ʱ����̨ģʽ�л�Ϊʶ�𵽵��ˣ�
            �����������װ�װ�������0�Ļ���˵��û�м�⵽װ�װ壬���ʱ����̨����ģʽ�л�Ϊû��ʶ�𵽵���            
  */
void TX2_Decode_Task(void *pvParameters)
{
	uint32_t err;

	TX2_Init();
	vTaskDelay(200);
	while(1)
	{
		//�ȴ�����֪ͨ
		err=ulTaskNotifyTake(pdTRUE,portMAX_DELAY);

		if(err==1)
		{//����У��
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

























/*********************************************��������ת������**********************************************************************/


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



