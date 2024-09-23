#include "main.h"

/**
  * @brief  LED³õÊ¼»¯
  * @param  void
  * @retval void
  * @notes  ºìµÆ-PE11      ÂÌµÆ-PE14
  */
void LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitTypeStruct;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);
	
	GPIO_InitTypeStruct.GPIO_Pin=GPIO_Pin_11;
	GPIO_InitTypeStruct.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitTypeStruct.GPIO_OType=GPIO_OType_PP;
	GPIO_InitTypeStruct.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitTypeStruct.GPIO_Speed=GPIO_High_Speed;
   GPIO_Init(GPIOE,&GPIO_InitTypeStruct);
	
	GPIO_InitTypeStruct.GPIO_Pin=GPIO_Pin_14;
	GPIO_InitTypeStruct.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitTypeStruct.GPIO_OType=GPIO_OType_PP;
	GPIO_InitTypeStruct.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitTypeStruct.GPIO_Speed=GPIO_High_Speed;
   GPIO_Init(GPIOF,&GPIO_InitTypeStruct);

   LED_RED_OFF;
	LED_GREEN_OFF;
}


void Heart_Task(void *pvParameters)
{

	while(1)
	{
		 LED_RED_TOGGLE;
		 vTaskDelay(1000);

	}	
}

