#include "BEEP.h"


void Beep_Init()
{
   TIM_TimeBaseInitTypeDef  TIM_TimeInitstruct;
	TIM_OCInitTypeDef      TIM_OCInitstruct;
	GPIO_InitTypeDef  GPIO_Initstruct;
	
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12,ENABLE);
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOH,ENABLE);
	GPIO_PinAFConfig(GPIOH,GPIO_PinSource6,GPIO_AF_TIM12);
	
	GPIO_Initstruct.GPIO_Mode =GPIO_Mode_AF;
	GPIO_Initstruct.GPIO_OType =GPIO_OType_PP;
	GPIO_Initstruct.GPIO_Pin  =GPIO_Pin_6;
	GPIO_Initstruct.GPIO_PuPd =GPIO_PuPd_UP;
	GPIO_Initstruct.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOH,&GPIO_Initstruct);
	
	TIM_TimeInitstruct.TIM_ClockDivision        =TIM_CKD_DIV1;
	TIM_TimeInitstruct.TIM_CounterMode          =TIM_CounterMode_Up;
	TIM_TimeInitstruct.TIM_Period               =499;    
	TIM_TimeInitstruct.TIM_Prescaler            =83;                 
	TIM_TimeBaseInit(TIM12,&TIM_TimeInitstruct);
	
	TIM_OCInitstruct.TIM_OCMode         =TIM_OCMode_PWM1;
	TIM_OCInitstruct.TIM_OCPolarity=TIM_OCPolarity_High;
	TIM_OCInitstruct.TIM_OutputState  =TIM_OutputState_Enable;
	TIM_OC1Init(TIM12,&TIM_OCInitstruct);
	
	TIM_OC1PreloadConfig(TIM12,TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM12,ENABLE); 
	TIM_Cmd(TIM12,ENABLE);
	Beep_OFF();
}


void Beep_ON(void)
{	
   TIM_SetCompare1(TIM12,100);	
}


void Beep_OFF(void)
{
   TIM_SetCompare1(TIM12,0);	
}
