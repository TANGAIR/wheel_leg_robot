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
	
	delay_init(180);                             	//��ʱ������ʼ��
   
	LED_Init();                                   //LED��ʼ��
	
	Laser_InitConfig();                           //�����ʼ��
	
	Beep_Init();                                  //��������ʼ��

	
} 




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














