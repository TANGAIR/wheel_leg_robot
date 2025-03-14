#include "main.h"

u16 Ramount=0,Gamount=0,Bamount=0;//用于存放读取的RGB值
u16 amount=0;//中断计数




void EXTI4_IRQHandler(void)//中断服务函数
{
   EXTI_ClearITPendingBit(EXTI_Line4);    //清除LINE2上的中断标志位
	if(amount<9999)
	{
		amount++;
	}
}

void EXTI4_Init(void)//4中断初始化
{
   
//设置PD4为输入模式	
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
	 SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD,EXTI_PinSource4);//中断时钟：PF2，利用PF2输入的值触发中断
  EXTI_InitStructure.EXTI_Line = EXTI_Line4;//外部中断线路4
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//模式：中断
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; //上升沿触发
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;//使能
  EXTI_Init(&EXTI_InitStructure);//初始化
	
 
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;//开启EXTI2中断
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;//抢占优先级0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//响应优先级2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//IRQ通道使能
  NVIC_Init(&NVIC_InitStructure);//根据指定的参数初始化NVIC寄存器
	
	
}




