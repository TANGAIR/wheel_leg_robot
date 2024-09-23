/*************************************************************
  *文件名称：TIM定时器外设功能源文件
	
	*数据定义：无
	
  *外部数据使用：无
	
	*函数定义：TIM9定时器初始化函数、
	           TIM9定时器中断服务函数
	
  *外部自定义函数使用：电机控制函数
  
	*功能说明：开启TIM9定时器进行定时，使用定时器中断定时运行控制函数，
	           从遥控器接收命令，控制电机
***************************************************************/


/****************头文件包含区******************************/
#include "main.h"

/****************数据定义区********************************/
int GetTick=0;//用于计算系统运行时间

/****************函数功能区******************************/
/*
  *函数名称：TIM9定时器初始化函数
  *参数输入：无
  *返回值：  无
  *硬件信息：无
  *功能说明：将TIM9初始化,开启定时器，中断产生频率设置，中断优先级设置，尽量设低
*/
void  TIM9_Init(void)
{

    NVIC_InitTypeDef NVIC_InitStructure; 
		// 设置中断来源
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_TIM9_IRQn; 	
		// 设置抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 12;	 
	  // 设置子优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 12;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	
	
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	// 开启TIMx_CLK,x[6,7] 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE); 
  /* 累计 TIM_Period个后产生一个更新或者中断*/		
  //当定时器从0计数到999，即为1000次，为一个定时周期
  TIM_TimeBaseStructure.TIM_Period = 1000-1;       
	
	// 通用控制定时器时钟源TIMxCLK = HCLK/2=84MHz 
	// 设定定时器频率为=TIMxCLK/(TIM_Prescaler+1)=1000000Hz
  TIM_TimeBaseStructure.TIM_Prescaler = 168-1;	
  // 采样时钟分频
  TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
  // 计数方式
  TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	// 初始化定时器TIMx, x[1,8]
	TIM_TimeBaseInit(GENERAL_TIM, &TIM_TimeBaseStructure);
	// 清除定时器更新中断标志位
	TIM_ClearFlag(GENERAL_TIM, TIM_FLAG_Update);
	// 开启定时器更新中断
	TIM_ITConfig(GENERAL_TIM,TIM_IT_Update,ENABLE);
	// 使能定时器
	TIM_Cmd(GENERAL_TIM, ENABLE);	



}



/*
  *函数名称：TIM9定时器中断服务函数
  *参数输入：无
  *返回值：  无
  *硬件信息：无
  *功能说明：每隔一个固定的时间运行一次控制函数，
*/

volatile uint32_t CPU_RunTime=0ul;
void TIM1_BRK_TIM9_IRQHandler(void)//每1MS产生一次中断
{
	// printf("GetTick=%d\r\n",GetTick);
if ( TIM_GetITStatus( TIM9, TIM_IT_Update) != RESET ) 
	{	
		
		//CPU_RunTime++;
		 GetTick++;
		if(GetTick%1000==0)
		 printf("GetTick=%d\r\n",GetTick);
		
	 TIM_ClearITPendingBit (TIM9,TIM_IT_Update);	
		
	}			 
}


















