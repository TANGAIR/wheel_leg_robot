#include "main.h"


/**
  * @brief  ͷ�����ʼ������
  * @param  void
  * @retval void
  * @notes  ��ʼ��ͷ��������ó�ʼPITCH�ǶȺ�YAW�Ƕȶ�Ϊ90�ȣ����������μ���Ϊ0.01ms
	          ������ƣ��ߵ�ƽ1ms/20ms��Ӧ0�ȣ�1.5ms/20ms��Ӧ90�ȣ�2ms/20ms��Ӧ180�ȣ��Ӷ��������ʱ��Ϊ����
						���ñȽ�ֵ��100~200��Ӧ0~180�ȣ�150��Ӧ90��
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
  
	//��ʼ�����
	Head_PITCH_Angle(150)
  Head_YAW_Angle(150)
  
}


/**
  * @brief  �ֵ��PWM��ʼ������
  * @param  void
  * @retval void
  * @notes  ��ʼ���ֵ����ʹ�侲ֹ��
            ����Ƶ��100HZ��������Ϊ10ms���г�0.4-2.2ms����180��Ƶ��10000����ֵ�����г�400-2200��
            ����ʱҪ�ȵ�������г̼�400���ٵ����������������������г�Ϊ1100-1500
	          ��ǰ��-W-PI5-TIM8C1
						�����-X-PI6-TIM8C2
            ��ǰ��-Y-PI7-TIM8C3
            �Һ���-A-PI0-TIM5C4
					
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

  TIM_CtrlPWMOutputs(TIM8, ENABLE); //�����ʹ�ܣ�
	
	
	
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
  * @brief  �ؽڶ����ʼ������--���
  * @param  void
  * @retval void
  * @notes  ��ʼ���ؽڶ�������ó�ʼ�Ƕ�ʹ���������Ϊ��0��190��Y��������Ϊ�£�X��������Ϊǰ�����򣬼��������μ���Ϊ0.01ms
	          ������ƣ��ߵ�ƽ1ms/20ms��Ӧ0�ȣ�1.5ms/20ms��Ӧ90�ȣ�2ms/20ms��Ӧ180�ȣ��Ӷ��������ʱ��Ϊ����
						���ñȽ�ֵ��100~200��Ӧ0~180�ȣ�150��Ӧ90��
						��ǰ��-S-PA0-TIM2C1
						��ǰ��-T-PA1-TIM2C2
            �����-U-PA2-TIM2C3
            �����-V-PA3-TIM2C4
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
  * @brief  �ؽڶ����ʼ�����������Ҳ�
  * @param  void
  * @retval void
  * @notes  ��ʼ���ؽڶ�������ó�ʼ�Ƕ�ʹ���������Ϊ��0��190��Y��������Ϊ�£�X��������Ϊǰ�����򣬼��������μ���Ϊ0.01ms
	          ������ƣ��ߵ�ƽ1ms/20ms��Ӧ0�ȣ�1.5ms/20ms��Ӧ90�ȣ�2ms/20ms��Ӧ180�ȣ��Ӷ��������ʱ��Ϊ����
						���ñȽ�ֵ��100~200��Ӧ0~180�ȣ�150��Ӧ90��
            ��ǰ��-H-PD12-TIM4C1
						��ǰ��-G-PD13-TIM4C2
            �Һ���-F-PD14-TIM4C3
            �Һ���-E-PD15-TIM4C4
						
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
  * @brief  �����ʼ������
  * @param  void
  * @retval void
  * @notes  ��ʼ�����ж��
  */
void Steering_Init(void)
{
  //Head_Steering_Init();
	Wheel_Steering_Init();

	Joint_Steering_Init_1();
	Joint_Steering_Init_2();
}









