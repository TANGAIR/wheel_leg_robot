#include "main.h"

/**
  * @brief  LED初始化
  * @param  void
  * @retval void
  * @notes  红灯-PE11      绿灯-PE14
  */
void LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitTypeStruct;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH,ENABLE);

	
	GPIO_InitTypeStruct.GPIO_Pin=GPIO_Pin_11|GPIO_Pin_10|GPIO_Pin_12;
	GPIO_InitTypeStruct.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitTypeStruct.GPIO_OType=GPIO_OType_PP;
	GPIO_InitTypeStruct.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitTypeStruct.GPIO_Speed=GPIO_High_Speed;
  GPIO_Init(GPIOH,&GPIO_InitTypeStruct);
  
	GPIO_InitTypeStruct.GPIO_Pin=GPIO_Pin_11;
  GPIO_Init(GPIOH,&GPIO_InitTypeStruct);
	
	
   LED_RED_OFF;
	 LED_GREEN_OFF;
	 LED_BULE_OFF;
}

char NUC_Lost_Flag_Music=0;
void Heart_Task(void *pvParameters)
{
  vTaskDelay(1000);

	while(1)
	{

		if(NUC_Lost_Flag_Music==1)
		{ 
			for(int x=0;x<Music_length;x++)//循环音符的次数
			{ 
			 Beep_Change_Music(tune[x]);
			if(tune[x]==ZERO)Beep_OFF();
			else  Beep_ON();
			 delay_ms(1.25f*delay[x]);
			 Beep_OFF();
			 delay_ms(50);

			}
			
			NUC_Lost_Flag_Music=0;
		}
		
		
		
		 LED_RED_TOGGLE;
		 vTaskDelay(1000);

	}	
}



