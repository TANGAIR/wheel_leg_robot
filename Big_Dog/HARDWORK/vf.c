#include "main.h"

/**
  * @brief  电压控制初始化
  * @param  void
  * @retval void
  * @notes  
  */
void Vf_Init(void)
{
  GPIO_InitTypeDef GPIO_InitTypeStruct;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH,ENABLE);
	
	GPIO_InitTypeStruct.GPIO_Pin=GPIO_Pin_4;
	GPIO_InitTypeStruct.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitTypeStruct.GPIO_OType=GPIO_OType_PP;
	GPIO_InitTypeStruct.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitTypeStruct.GPIO_Speed=GPIO_High_Speed;
  GPIO_Init(GPIOH,&GPIO_InitTypeStruct);
	 
	GPIO_InitTypeStruct.GPIO_Pin=GPIO_Pin_5;
	GPIO_Init(GPIOH,&GPIO_InitTypeStruct);
	
	GPIO_ResetBits(GPIOH, GPIO_Pin_4);
  GPIO_SetBits(GPIOH, GPIO_Pin_5);
}





























