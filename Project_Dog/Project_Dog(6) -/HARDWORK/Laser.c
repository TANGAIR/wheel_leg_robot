#include "Laser.h"

/***************************************���ݶ�����***************************************/


/***************************************����������***************************************/
/**
  * @brief  �����ʼ��
  * @param  void
  * @retval void
  * @notes  PG13  
  */
void Laser_InitConfig(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct;

	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOG,ENABLE);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;//����
	GPIO_Init(GPIOG, &GPIO_InitStruct);//��ʼ��GPIO
	LASER_ON;
}	
