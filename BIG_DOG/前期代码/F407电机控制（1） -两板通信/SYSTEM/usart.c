#include "sys.h"
#include "usart.h"	
////////////////////////////////////////////////////////////////////////////////// 	 
//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos ʹ��	  
#endif
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F4̽���߿�����
//����1��ʼ��		   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2014/6/10
//�汾��V1.5
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved
//********************************************************************************
//V1.3�޸�˵�� 
//֧����Ӧ��ͬƵ���µĴ��ڲ���������.
//�����˶�printf��֧��
//�����˴��ڽ��������.
//������printf��һ���ַ���ʧ��bug
//V1.4�޸�˵��
//1,�޸Ĵ��ڳ�ʼ��IO��bug
//2,�޸���USART_RX_STA,ʹ�ô����������ֽ���Ϊ2��14�η�
//3,������USART_REC_LEN,���ڶ��崮�����������յ��ֽ���(������2��14�η�)
//4,�޸���EN_USART1_RX��ʹ�ܷ�ʽ
//V1.5�޸�˵��
//1,�����˶�UCOSII��֧��
////////////////////////////////////////////////////////////////////////////////// 	  
 

//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
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
// ��������
int fputc(int ch, FILE *f)
{
	while (USART_GetFlagStatus(USART1,USART_FLAG_TC) == RESET);//�ȴ�֮ǰ���ַ��������
	USART_SendData(USART1, (uint8_t)ch);
	return (ch);
}
#endif
 

//��ʼ��IO ����1 
//bound:������
void uart_init(u32 bound)
{
	GPIO_InitTypeDef       GPIO_InitSturct;
	USART_InitTypeDef      USART_InitStruct;
	NVIC_InitTypeDef       NVIC_InitStruct;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);
	
	GPIO_InitSturct.GPIO_Pin=GPIO_Pin_9|GPIO_Pin_10;
	GPIO_InitSturct.GPIO_OType=GPIO_OType_PP;                                      //�������
	GPIO_InitSturct.GPIO_Mode=GPIO_Mode_AF;                                        //����ģʽ
	GPIO_InitSturct.GPIO_PuPd=GPIO_PuPd_NOPULL;                                    //������ 
	GPIO_InitSturct.GPIO_Speed=GPIO_High_Speed;                                    //100MHZ
	GPIO_Init(GPIOA,&GPIO_InitSturct);
	
	USART_InitStruct.USART_BaudRate=bound;
	USART_InitStruct.USART_HardwareFlowControl=USART_HardwareFlowControl_None;     //Ӳ����������
	USART_InitStruct.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;                       //�շ�ģʽ
	USART_InitStruct.USART_Parity=USART_Parity_No;                                 //����żУ��λ
	USART_InitStruct.USART_StopBits=USART_StopBits_1;                              //�ֽ�
	USART_InitStruct.USART_WordLength=USART_WordLength_8b;                         //���ݳ���
	USART_Init(USART1,&USART_InitStruct);
	USART_Cmd(USART1,ENABLE);
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
	
	NVIC_InitStruct.NVIC_IRQChannel=USART1_IRQn;                                   //�ж�ͨ��
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;                                     //ʹ��
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=7;                           //��ռ���ȼ�
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=0;                                  //�����ȼ�
	NVIC_Init(&NVIC_InitStruct);	
}


void USART1_IRQHandler(void)
{
  if ( USART_GetITStatus( USART1, USART_IT_RXNE | USART_IT_ORE_RX ) != RESET )
  {    
  }
  USART_ClearFlag ( USART1,USART_IT_RXNE | USART_IT_ORE_RX );
}

 



