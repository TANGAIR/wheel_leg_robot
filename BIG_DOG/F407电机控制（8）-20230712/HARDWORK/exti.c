#include "main.h"

u16 Ramount=0,Gamount=0,Bamount=0;//���ڴ�Ŷ�ȡ��RGBֵ
u16 amount=0;//�жϼ���




void EXTI4_IRQHandler(void)//�жϷ�����
{
   EXTI_ClearITPendingBit(EXTI_Line4);    //���LINE2�ϵ��жϱ�־λ
	if(amount<9999)
	{
		amount++;
	}
}

void EXTI4_Init(void)//4�жϳ�ʼ��
{
   
//����PD4Ϊ����ģʽ	
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//?? GPIOI ??
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4  ;//
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//??????
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//????
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//??
  GPIO_Init(GPIOD, &GPIO_InitStructure);//GPIOF2
	
	
	NVIC_InitTypeDef NVIC_InitStructure;
	 EXTI_InitTypeDef EXTI_InitStructure;
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE); 
	 SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD,EXTI_PinSource4);//�ж�ʱ�ӣ�PF2������PF2�����ֵ�����ж�
  EXTI_InitStructure.EXTI_Line = EXTI_Line4;//�ⲿ�ж���·4
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//ģʽ���ж�
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; //�����ش���
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;//ʹ��
  EXTI_Init(&EXTI_InitStructure);//��ʼ��
	
 
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;//����EXTI2�ж�
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;//��ռ���ȼ�0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//��Ӧ���ȼ�2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//IRQͨ��ʹ��
  NVIC_Init(&NVIC_InitStructure);//����ָ���Ĳ�����ʼ��NVIC�Ĵ���
	
	
}




