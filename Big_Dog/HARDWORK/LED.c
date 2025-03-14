#include "main.h"

/**
  * @brief  LED��ʼ��
  * @param  void
  * @retval void
  * @notes  ���-PE11      �̵�-PE14
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

		if(Receive_TX2_Data_TypeStruct.Dog_Mode!=0)
		{ 
			
			if(Receive_TX2_Data_TypeStruct.Dog_Mode==1)
			{
					for(int x=0;x<Music_length1;x++)//ѭ�������Ĵ���
					{ 
					 Beep_Change_Music(tune1[x]);
					if(tune1[x]==ZERO)Beep_OFF();
					else  Beep_ON();
					 delay_ms(1.25f*delay1[x]);
					 Beep_OFF();
					 delay_ms(50);
					if(Receive_TX2_Data_TypeStruct.Dog_Mode==0)	 break;
					}
			}
			else if(Receive_TX2_Data_TypeStruct.Dog_Mode==2)
			{
					for(int x=0;x<Music_length2;x++)//ѭ�������Ĵ���
					{ 
					 Beep_Change_Music(tune2[x]);
					if(tune2[x]==ZERO)Beep_OFF();
					else  Beep_ON();
					 delay_ms(1.25f*delay2[x]);
					 Beep_OFF();
					 delay_ms(50);
						if(Receive_TX2_Data_TypeStruct.Dog_Mode==0)	 break;
					}
			}
		
			
			Receive_TX2_Data_TypeStruct.Dog_Mode=0;
		}
		 LED_RED_TOGGLE;
		 vTaskDelay(1000);
	}
}	




