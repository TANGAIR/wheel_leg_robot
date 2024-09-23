#include "main.h"



/*****************����������***************/
/**
  * @brief  ����Ӳ����ʼ��
  * @param  void
  * @retval void
  * @notes  �������ȼ����飬��ʼ��һЩ����Ҫ�����Ӳ��
  */
void BaseHardwork_Init(void)
{ 
	/*�ж����ȼ���������*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
	delay_init(168);                             	//��ʱ������ʼ��
   
	LED_Init();                                   //LED��ʼ��
	
	Laser_InitConfig();                           //�����ʼ��
	
	Beep_Init();                                  //��������ʼ��

	USART1_Init();																//����6��ӡ��ʼ��
	
	TIM9_Init();																	//FreeRTOSϵͳ��ʱ��ʼ��
	

	Beep_ON ();
	delay_ms(500);
	Beep_OFF();
} 



/**
  * @brief  ������
  * @param  void
  * @retval void
  * @notes  void
  */
int main(void)
{

  //����Ӳ����ʼ��
   BaseHardwork_Init();
  
	//���̵�
	 LED_GREEN_ON;
	
	//������������
   startTask(); 
	
	//�������
   vTaskStartScheduler();    

}














