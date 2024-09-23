#include "main.h"   

 
void LED_GPIO_Config(void)
{		
	  RCC_AHB1PeriphClockCmd ( LED1_GPIO_CLK,ENABLE); /*开启LED相关的GPIO外设时钟*/
		GPIO_InitTypeDef GPIO_InitStructure;/*定义一个GPIO_InitTypeDef类型的结构体*/
														   
		GPIO_InitStructure.GPIO_Pin = LED1_PIN;	/*选择要控制的GPIO引脚*/
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;/*设置引脚模式为输出模式*/
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; /*设置引脚的输出类型为推挽输出*/
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;/*设置引脚为上拉模式*/
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; 	/*设置引脚速率为2MHz */
		GPIO_Init(LED1_GPIO_PORT, &GPIO_InitStructure);	/*调用库函数，使用上面配置的GPIO_InitStructure初始化GPIO*/
    														   
		GPIO_InitStructure.GPIO_Pin = LED2_PIN;	/*选择要控制的GPIO引脚*/	
    GPIO_Init(LED2_GPIO_PORT, &GPIO_InitStructure);	
		/*关闭RGB灯*/
    LED1_OFF;
	  LED2_OFF;
	
}



int led_t=5;//计时单位秒


void led_against(void)
{
 while(led_t-->0)
 {
    LED1_ON;
	  LED2_OFF;
	 delay_ms(100);
    LED1_OFF;
	  LED2_ON;
	 delay_ms(100);
 }
 LED1_ON;//绿灯常亮表示程序正在正常运行
 LED2_OFF;//关闭红灯
}


void led_R_twinkle(void )//红灯闪烁
{
while(1)//红灯一直闪烁
 {
    LED1_OFF;
	 delay_ms(300);
    LED1_ON;
	 delay_ms(300);
 }

}


void led_Y_twinkle(void )//黄色灯闪烁闪烁
{
while(1)
 {
    LED2_OFF;
	 delay_ms(300);
    LED2_ON;
	 delay_ms(300);
 }

}













