/*************************************************************
  *�ļ����ƣ�TIM��ʱ�����蹦��Դ�ļ�
	
	*���ݶ��壺��
	
  *�ⲿ����ʹ�ã���
	
	*�������壺TIM3��ʱ����ʼ��������
	           TIM3��ʱ���жϷ�����
	
  *�ⲿ�Զ��庯��ʹ�ã�������ƺ���
  
	*����˵��������TIM3��ʱ�����ж�ʱ��ʹ�ö�ʱ���ж϶�ʱ���п��ƺ�����
	           ��ң��������������Ƶ��
***************************************************************/


/****************ͷ�ļ�������******************************/
#include "main.h"

/****************���ݶ�����********************************/

/****************����������******************************/
/*
  *�������ƣ�TIM3��ʱ����ʼ������
  *�������룺��
  *����ֵ��  ��
  *Ӳ����Ϣ����
  *����˵������TIM3��ʼ��,������ʱ�����жϲ���Ƶ�����ã��ж����ȼ����ã��������
*/
void  TIM3_Init(void)
{

    NVIC_InitTypeDef NVIC_InitStructure; 
		// �����ж���Դ
    NVIC_InitStructure.NVIC_IRQChannel = GENERAL_TIM_IRQn; 	
		// ������ռ���ȼ�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 12;	 
	  // ���������ȼ�
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 12;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	
	
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	// ����TIMx_CLK,x[6,7] 
  RCC_APB1PeriphClockCmd(GENERAL_TIM_CLK, ENABLE); 
  /* �ۼ� TIM_Period�������һ�����»����ж�*/		
  //����ʱ����0������999����Ϊ1000�Σ�Ϊһ����ʱ����
  TIM_TimeBaseStructure.TIM_Period = 50-1;       
	
	// ͨ�ÿ��ƶ�ʱ��ʱ��ԴTIMxCLK = HCLK/2=90MHz 
	// �趨��ʱ��Ƶ��Ϊ=TIMxCLK/(TIM_Prescaler+1)=100000Hz
  TIM_TimeBaseStructure.TIM_Prescaler = 90-1;	
  // ����ʱ�ӷ�Ƶ
  TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
  // ������ʽ
  TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	// ��ʼ����ʱ��TIMx, x[1,8]
	TIM_TimeBaseInit(GENERAL_TIM, &TIM_TimeBaseStructure);
	// �����ʱ�������жϱ�־λ
	TIM_ClearFlag(GENERAL_TIM, TIM_FLAG_Update);
	// ������ʱ�������ж�
	TIM_ITConfig(GENERAL_TIM,TIM_IT_Update,ENABLE);
	// ʹ�ܶ�ʱ��
	TIM_Cmd(GENERAL_TIM, ENABLE);	



}



/*
  *�������ƣ�TIM3��ʱ���жϷ�����
  *�������룺��
  *����ֵ��  ��
  *Ӳ����Ϣ����
  *����˵����ÿ��һ���̶���ʱ������һ�ο��ƺ�������ң������������
*/

volatile uint32_t CPU_RunTime=0ul;
void TIM3_IRQHandler(void)//ÿ50us����һ���ж�
{
if ( TIM_GetITStatus( TIM3, TIM_IT_Update) != RESET ) 
	{	
		
		CPU_RunTime++;
		
		
		
	 TIM_ClearITPendingBit (TIM3,TIM_IT_Update);	
		
	}			 
}


















