#include "main.h"



/*****************函数定义区***************/
/**
  * @brief  基本硬件初始化
  * @param  void
  * @retval void
  * @notes  配置优先级分组，初始化一些不需要任务的硬件
  */
void BaseHardwork_Init(void)
{ 
	/*中断优先级分组配置*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
	delay_init(168);                             	//延时函数初始化
   
	LED_Init();                                   //LED初始化
	
	Laser_InitConfig();                           //激光初始化
	
	Beep_Init();                                  //蜂鸣器初始化

	USART1_Init();																//串口6打印初始化
	
	TIM9_Init();																	//FreeRTOS系统计时初始化
	

	Beep_ON ();
	delay_ms(500);
	Beep_OFF();
} 



/**
  * @brief  主函数
  * @param  void
  * @retval void
  * @notes  void
  */
int main(void)
{

  //基本硬件初始化
   BaseHardwork_Init();
  
	//亮绿灯
	 LED_GREEN_ON;
	
	//创建启动任务
   startTask(); 
	
	//任务调度
   vTaskStartScheduler();    

}














