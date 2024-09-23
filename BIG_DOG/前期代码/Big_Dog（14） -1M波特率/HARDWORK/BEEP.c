#include "BEEP.h"

 //PD14     ------> TIM4_CH3 
void Beep_Init()
{
   TIM_TimeBaseInitTypeDef  TIM_TimeInitstruct;
	 TIM_OCInitTypeDef      TIM_OCInitstruct;
	 GPIO_InitTypeDef  GPIO_Initstruct;
	
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOD,ENABLE);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource14,GPIO_AF_TIM4);
	
	GPIO_Initstruct.GPIO_Mode =GPIO_Mode_AF;
	GPIO_Initstruct.GPIO_OType =GPIO_OType_PP;
	GPIO_Initstruct.GPIO_Pin  =GPIO_Pin_14;
	GPIO_Initstruct.GPIO_PuPd =GPIO_PuPd_UP;
	GPIO_Initstruct.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOD,&GPIO_Initstruct);
	
	TIM_TimeInitstruct.TIM_ClockDivision        =TIM_CKD_DIV1;
	TIM_TimeInitstruct.TIM_CounterMode          =TIM_CounterMode_Up;
	TIM_TimeInitstruct.TIM_Period               =499;    
	TIM_TimeInitstruct.TIM_Prescaler            =83;                 
	TIM_TimeBaseInit(TIM4,&TIM_TimeInitstruct);
	
	TIM_OCInitstruct.TIM_OCMode         =TIM_OCMode_PWM1;
	TIM_OCInitstruct.TIM_OCPolarity=TIM_OCPolarity_High;
	TIM_OCInitstruct.TIM_OutputState  =TIM_OutputState_Enable;
	TIM_OC3Init(TIM4,&TIM_OCInitstruct);
	
	TIM_OC3PreloadConfig(TIM4,TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM4,ENABLE); 
	TIM_Cmd(TIM4,ENABLE);
	Beep_OFF();
}


void Beep_ON(void)
{	
   TIM_SetCompare3(TIM4,100);	
}


void Beep_OFF(void)
{
   TIM_SetCompare3(TIM4,0);	
}



//频率变更,
void Beep_Change_Music(int pra)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeInitstruct;
	 TIM_OCInitTypeDef      TIM_OCInitstruct;
	 GPIO_InitTypeDef  GPIO_Initstruct;
	
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOD,ENABLE);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource14,GPIO_AF_TIM4);
	
	GPIO_Initstruct.GPIO_Mode =GPIO_Mode_AF;
	GPIO_Initstruct.GPIO_OType =GPIO_OType_PP;
	GPIO_Initstruct.GPIO_Pin  =GPIO_Pin_14;
	GPIO_Initstruct.GPIO_PuPd =GPIO_PuPd_UP;
	GPIO_Initstruct.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOD,&GPIO_Initstruct);
	
	TIM_TimeInitstruct.TIM_ClockDivision        =TIM_CKD_DIV1;
	TIM_TimeInitstruct.TIM_CounterMode          =TIM_CounterMode_Up;
	TIM_TimeInitstruct.TIM_Period               =pra;    
	TIM_TimeInitstruct.TIM_Prescaler            =83;                 
	TIM_TimeBaseInit(TIM4,&TIM_TimeInitstruct);
	
	TIM_OCInitstruct.TIM_OCMode         =TIM_OCMode_PWM1;
	TIM_OCInitstruct.TIM_OCPolarity=TIM_OCPolarity_High;
	TIM_OCInitstruct.TIM_OutputState  =TIM_OutputState_Enable;
	TIM_OC3Init(TIM4,&TIM_OCInitstruct);
	
	TIM_OC3PreloadConfig(TIM4,TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM4,ENABLE); 
	TIM_Cmd(TIM4,ENABLE);
	
	Beep_OFF();
 
}



//乐谱
int tune[] =  
{ 
	
	ZERO,
	
	//新闻联播
  L5,M1,L5,M3,M1,M5,ZERO,
	M1,M5,M7,M6,M3,ZERO,
	M5,M7,M6,M3,ZERO,
	L1,M1,M3,M2,L6,ZERO,
	M1,M3,M2,L6,ZERO,
	M2,M2,M3,M2,M6,M3,M5,ZERO,
	L5,L6,M1,
	
	ZERO
	
////欢乐斗地主	
// M3,M3,M2,M1,M1,L6,M2,M3,M2,M3,L5,
// L6,L6,L5,L6,M1,M5,M6,M3,M5,M2,
// M3,M3,M2,M3,M5,M6,H1,M6,H1,M6,M5,M3,
// M2,M2,M3,M5,L5,M2,M3,M2,M3,M1,
//	
//	ZERO,
//	//
//	M5,L6,M1,M5,
//	L6,M1,M5,
//	L6,M1,M5,
//	L6,M1,
//	M1,L6,M1,
//	M1,L6,M1,
//	M2,M3,half_M3//全音符800ms
 	

};

int delay[] =  
{ 
	1000,
	//新闻联播
	250,250,250,250,250,900,600,
	250,200,250,250,600,150,
	250,200,200,650,250,
	250,200,250,250,600,150,
	250,200,200,650,500,
	250,250,250,250,500,500,1000,250,
  300,300,1000,
	
	1000
	
//  //欢乐斗地主	
//  200,200,100,200,100,100,100,100,100,100,400,
//  200,200,100,200,100,100,100,100,100,400,
//  200,200,100,200,200,100,100,100,100,300,100,100,
//  200,200,150,200,200,150,150,150,150,400,
//	
//	1000,
//	
//	//
//	400,200,200,400,
//	200,200,400,
//	200,200,400,
//	400,400,
//	200,200,400,
//	200,200,300,
//	300,300,300
	
};







int Music_length = (sizeof(tune)/sizeof(tune[0]));




