#ifndef __KEY_H
#define __KEY_H

#include "stm32f4xx.h"

//key0���Ŷ���
#define KEY0_PIN                  GPIO_Pin_4                 
#define KEY0_GPIO_PORT            GPIOE                     
#define KEY0_GPIO_CLK             RCC_AHB1Periph_GPIOE

//KEY1���Ŷ���
#define KEY1_PIN                  GPIO_Pin_3                 
#define KEY1_GPIO_PORT            GPIOE                      
#define KEY1_GPIO_CLK             RCC_AHB1Periph_GPIOE


#define KEY1   GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3)//�͵�ƽ��ʾ����
#define KEY0   GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_4)//�͵�ƽ��ʾ����

void key_init(void);//����ʼ������
















#endif
























