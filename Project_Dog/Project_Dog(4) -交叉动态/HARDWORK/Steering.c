#include "main.h"


/**
  * @brief  头舵机初始化函数
  * @param  void
  * @retval void
  * @notes  初始化头舵机，设置初始PITCH角度和YAW角度都为90度，计数器单次计数为0.01ms
	          舵机控制，高电平1ms/20ms对应0度，1.5ms/20ms对应90度，2ms/20ms对应180度，从舵机轴向看逆时针为正向
						设置比较值，100~200对应0~180度，150对应90度
						PITCH-C-PH11-TIM5C2
						YAW-D-PH10-TIM5C1
  */
void Head_Steering_Init(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeInitstruct;
	TIM_OCInitTypeDef        TIM_OCInitstruct;
	GPIO_InitTypeDef         GPIO_Initstruct;
	
  RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM5,ENABLE);
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOH,ENABLE);
	
	GPIO_PinAFConfig(GPIOH,GPIO_PinSource11,GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOH,GPIO_PinSource10,GPIO_AF_TIM5);
	
	GPIO_Initstruct.GPIO_Mode =GPIO_Mode_AF;
	GPIO_Initstruct.GPIO_OType =GPIO_OType_PP;
	GPIO_Initstruct.GPIO_PuPd =GPIO_PuPd_UP;
	GPIO_Initstruct.GPIO_Speed=GPIO_Speed_100MHz;
	
	GPIO_Initstruct.GPIO_Pin  =GPIO_Pin_10;	
	GPIO_Init(GPIOH,&GPIO_Initstruct);
	GPIO_Initstruct.GPIO_Pin  =GPIO_Pin_11;	
	GPIO_Init(GPIOH,&GPIO_Initstruct);
	
	TIM_TimeInitstruct.TIM_ClockDivision        =TIM_CKD_DIV1;
	TIM_TimeInitstruct.TIM_CounterMode          =TIM_CounterMode_Down;
	TIM_TimeInitstruct.TIM_Period               =2000-1;    
	TIM_TimeInitstruct.TIM_Prescaler            =900-1;                 
	TIM_TimeBaseInit(TIM5,&TIM_TimeInitstruct);
	
	TIM_OCInitstruct.TIM_OCMode         =TIM_OCMode_PWM1;
	TIM_OCInitstruct.TIM_OCPolarity     =TIM_OCPolarity_High;
	TIM_OCInitstruct.TIM_OutputState    =TIM_OutputState_Enable;
	
	TIM_OC1Init(TIM5,&TIM_OCInitstruct);
	TIM_OC1PreloadConfig(TIM5,TIM_OCPreload_Enable);
	TIM_OC2Init(TIM5,&TIM_OCInitstruct);
	TIM_OC2PreloadConfig(TIM5,TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM5,ENABLE); 
	TIM_Cmd(TIM5,ENABLE);
  
	//初始化舵机
	Head_PITCH_Angle(150)
  Head_YAW_Angle(150)
  
}


/**
  * @brief  轮电机PWM初始化函数
  * @param  void
  * @retval void
  * @notes  初始化轮电机，使其静止，
            控制频率100HZ，即周期为10ms，行程0.4-2.2ms，以180分频，10000计数值，则行程400-2200，
            启动时要先调到最低行程即400，再调高这样才能驱动，驱动行程为1100-1500
	          左前轮-W-PI5-TIM8C1
						左后轮-X-PI6-TIM8C2
            右前轮-Y-PI7-TIM8C3
            右后轮-A-PI0-TIM5C4
					
  */
void Wheel_Steering_Init(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeInitstruct;
	TIM_OCInitTypeDef        TIM_OCInitstruct;
	GPIO_InitTypeDef         GPIO_Initstruct;
	
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM5,ENABLE);
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_TIM8,ENABLE);
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOI,ENABLE);
	
	GPIO_PinAFConfig(GPIOI,GPIO_PinSource5,GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOI,GPIO_PinSource6,GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOI,GPIO_PinSource7,GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOI,GPIO_PinSource0,GPIO_AF_TIM5);
	
	GPIO_Initstruct.GPIO_Mode  =GPIO_Mode_AF;
	GPIO_Initstruct.GPIO_OType =GPIO_OType_PP;
	GPIO_Initstruct.GPIO_PuPd  =GPIO_PuPd_NOPULL;
	GPIO_Initstruct.GPIO_Speed =GPIO_Speed_100MHz;
	
	GPIO_Initstruct.GPIO_Pin  =GPIO_Pin_5;	
	GPIO_Init(GPIOI,&GPIO_Initstruct);
	GPIO_Initstruct.GPIO_Pin  =GPIO_Pin_6;	
	GPIO_Init(GPIOI,&GPIO_Initstruct);
	GPIO_Initstruct.GPIO_Pin  =GPIO_Pin_7;	
	GPIO_Init(GPIOI,&GPIO_Initstruct);
	GPIO_Initstruct.GPIO_Pin  =GPIO_Pin_0;	
	GPIO_Init(GPIOI,&GPIO_Initstruct);
	
	
	TIM_TimeInitstruct.TIM_ClockDivision        =TIM_CKD_DIV1;
	TIM_TimeInitstruct.TIM_CounterMode          =TIM_CounterMode_Up;
	TIM_TimeInitstruct.TIM_Period               =2000-1;//10000-1;    
	TIM_TimeInitstruct.TIM_Prescaler            =1800-1;//180-1; 
  TIM_TimeInitstruct.TIM_RepetitionCounter    =0;
	TIM_TimeBaseInit(TIM8,&TIM_TimeInitstruct);
	
	TIM_OCInitstruct.TIM_OCMode         =TIM_OCMode_PWM1;
	TIM_OCInitstruct.TIM_OCPolarity     =TIM_OCPolarity_High;
	TIM_OCInitstruct.TIM_OutputState    =TIM_OutputState_Enable;
	
	TIM_OC1Init(TIM8,&TIM_OCInitstruct);
	TIM_OC1PreloadConfig(TIM8,TIM_OCPreload_Enable);
	TIM_OC2Init(TIM8,&TIM_OCInitstruct);
	TIM_OC2PreloadConfig(TIM8,TIM_OCPreload_Enable);
	TIM_OC3Init(TIM8,&TIM_OCInitstruct);
	TIM_OC3PreloadConfig(TIM8,TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM8,ENABLE); 
	
	TIM_Cmd(TIM8,ENABLE);

  TIM_CtrlPWMOutputs(TIM8, ENABLE); //主输出使能，
	
	
	
	TIM_TimeInitstruct.TIM_ClockDivision        =TIM_CKD_DIV1;
	TIM_TimeInitstruct.TIM_CounterMode          =TIM_CounterMode_Down;
	TIM_TimeInitstruct.TIM_Period               =2000-1;    
	TIM_TimeInitstruct.TIM_Prescaler            =900-1;                 
	TIM_TimeBaseInit(TIM5,&TIM_TimeInitstruct);
	
	TIM_OCInitstruct.TIM_OCMode         =TIM_OCMode_PWM1;
	TIM_OCInitstruct.TIM_OCPolarity     =TIM_OCPolarity_High;
	TIM_OCInitstruct.TIM_OutputState    =TIM_OutputState_Enable;
	
	TIM_OC4Init(TIM5,&TIM_OCInitstruct);
	TIM_OC4PreloadConfig(TIM5,TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM5,ENABLE); 
	TIM_Cmd(TIM5,ENABLE);

 

}

/**
  * @brief  关节舵机初始化函数--左侧
  * @param  void
  * @retval void
  * @notes  初始化关节舵机，设置初始角度使得足端坐标为（0，190）Y坐标正向为下，X坐标正向为前进方向，计数器单次计数为0.01ms
	          舵机控制，高电平1ms/20ms对应0度，1.5ms/20ms对应90度，2ms/20ms对应180度，从舵机轴向看逆时针为正向
						设置比较值，100~200对应0~180度，150对应90度
						左前内-S-PA0-TIM2C1
						左前外-T-PA1-TIM2C2
            左后内-U-PA2-TIM2C3
            左后外-V-PA3-TIM2C4
  */
void Joint_Steering_Init_1(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeInitstruct;
	TIM_OCInitTypeDef        TIM_OCInitstruct;
	GPIO_InitTypeDef         GPIO_Initstruct;
	
  RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM2,ENABLE);
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA,ENABLE);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_TIM2);
	
	GPIO_Initstruct.GPIO_Mode =GPIO_Mode_AF;
	GPIO_Initstruct.GPIO_OType =GPIO_OType_PP;
	GPIO_Initstruct.GPIO_PuPd =GPIO_PuPd_UP;
	GPIO_Initstruct.GPIO_Speed=GPIO_Speed_100MHz;
	
	GPIO_Initstruct.GPIO_Pin  =GPIO_Pin_0;	
	GPIO_Init(GPIOA,&GPIO_Initstruct);
	GPIO_Initstruct.GPIO_Pin  =GPIO_Pin_1;	
	GPIO_Init(GPIOA,&GPIO_Initstruct);
	GPIO_Initstruct.GPIO_Pin  =GPIO_Pin_2;	
	GPIO_Init(GPIOA,&GPIO_Initstruct);
	GPIO_Initstruct.GPIO_Pin  =GPIO_Pin_3;	
	GPIO_Init(GPIOA,&GPIO_Initstruct);
	
	TIM_TimeInitstruct.TIM_ClockDivision        =TIM_CKD_DIV1;
	TIM_TimeInitstruct.TIM_CounterMode          =TIM_CounterMode_Down;
	TIM_TimeInitstruct.TIM_Period               =2000;    
	TIM_TimeInitstruct.TIM_Prescaler            =900-1;                 
	TIM_TimeBaseInit(TIM2,&TIM_TimeInitstruct);
	
	TIM_OCInitstruct.TIM_OCMode         =TIM_OCMode_PWM1;
	TIM_OCInitstruct.TIM_OCPolarity     =TIM_OCPolarity_High;
	TIM_OCInitstruct.TIM_OutputState    =TIM_OutputState_Enable;
	
	TIM_OC1Init(TIM2,&TIM_OCInitstruct);
	TIM_OC1PreloadConfig(TIM2,TIM_OCPreload_Enable);
	TIM_OC2Init(TIM2,&TIM_OCInitstruct);
	TIM_OC2PreloadConfig(TIM2,TIM_OCPreload_Enable);
	TIM_OC3Init(TIM2,&TIM_OCInitstruct);
	TIM_OC3PreloadConfig(TIM2,TIM_OCPreload_Enable);
	TIM_OC4Init(TIM2,&TIM_OCInitstruct);
	TIM_OC4PreloadConfig(TIM2,TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM2,ENABLE); 
	TIM_Cmd(TIM2,ENABLE);



}

/**
  * @brief  关节舵机初始化函数——右侧
  * @param  void
  * @retval void
  * @notes  初始化关节舵机，设置初始角度使得足端坐标为（0，190）Y坐标正向为下，X坐标正向为前进方向，计数器单次计数为0.01ms
	          舵机控制，高电平1ms/20ms对应0度，1.5ms/20ms对应90度，2ms/20ms对应180度，从舵机轴向看逆时针为正向
						设置比较值，100~200对应0~180度，150对应90度
            右前内-H-PD12-TIM4C1
						右前外-G-PD13-TIM4C2
            右后内-F-PD14-TIM4C3
            右后外-E-PD15-TIM4C4
						
  */
void Joint_Steering_Init_2(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeInitstruct;
	TIM_OCInitTypeDef        TIM_OCInitstruct;
	GPIO_InitTypeDef         GPIO_Initstruct;
	
  RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM4,ENABLE);
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOD,ENABLE);
	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource12,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource13,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource14,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource15,GPIO_AF_TIM4);
	
	GPIO_Initstruct.GPIO_Mode =GPIO_Mode_AF;
	GPIO_Initstruct.GPIO_OType =GPIO_OType_PP;
	GPIO_Initstruct.GPIO_PuPd =GPIO_PuPd_UP;
	GPIO_Initstruct.GPIO_Speed=GPIO_Speed_100MHz;
	
	GPIO_Initstruct.GPIO_Pin  =GPIO_Pin_12;	
	GPIO_Init(GPIOD,&GPIO_Initstruct);
	GPIO_Initstruct.GPIO_Pin  =GPIO_Pin_13;	
	GPIO_Init(GPIOD,&GPIO_Initstruct);
	GPIO_Initstruct.GPIO_Pin  =GPIO_Pin_14;	
	GPIO_Init(GPIOD,&GPIO_Initstruct);
	GPIO_Initstruct.GPIO_Pin  =GPIO_Pin_15;	
	GPIO_Init(GPIOD,&GPIO_Initstruct);
	
	TIM_TimeInitstruct.TIM_ClockDivision        =TIM_CKD_DIV1;
	TIM_TimeInitstruct.TIM_CounterMode          =TIM_CounterMode_Down;
	TIM_TimeInitstruct.TIM_Period               =2000;//10000;    
	TIM_TimeInitstruct.TIM_Prescaler            =900-1;//84-1;                 
	TIM_TimeBaseInit(TIM4,&TIM_TimeInitstruct);
	
	TIM_OCInitstruct.TIM_OCMode         =TIM_OCMode_PWM1;
	TIM_OCInitstruct.TIM_OCPolarity     =TIM_OCPolarity_High;
	TIM_OCInitstruct.TIM_OutputState    =TIM_OutputState_Enable;
	
	TIM_OC1Init(TIM4,&TIM_OCInitstruct);
	TIM_OC1PreloadConfig(TIM4,TIM_OCPreload_Enable);
	TIM_OC2Init(TIM4,&TIM_OCInitstruct);
	TIM_OC2PreloadConfig(TIM4,TIM_OCPreload_Enable);
	TIM_OC3Init(TIM4,&TIM_OCInitstruct);
	TIM_OC3PreloadConfig(TIM4,TIM_OCPreload_Enable);
	TIM_OC4Init(TIM4,&TIM_OCInitstruct);
	TIM_OC4PreloadConfig(TIM4,TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM4,ENABLE); 
	TIM_Cmd(TIM4,ENABLE); 

     
}

/**
  * @brief  舵机初始化函数
  * @param  void
  * @retval void
  * @notes  初始化所有舵机
  */
void Steering_Init(void)
{
  //Head_Steering_Init();
	Wheel_Steering_Init();

	Joint_Steering_Init_1();
	Joint_Steering_Init_2();
}









