#ifndef __KEY_H
#define __KEY_H

#include "stm32f4xx.h"

//key0引脚定义
#define KEY0_PIN                  GPIO_Pin_4                 
#define KEY0_GPIO_PORT            GPIOE                     
#define KEY0_GPIO_CLK             RCC_AHB1Periph_GPIOE

//KEY1引脚定义
#define KEY1_PIN                  GPIO_Pin_3                 
#define KEY1_GPIO_PORT            GPIOE                      
#define KEY1_GPIO_CLK             RCC_AHB1Periph_GPIOE


#define KEY1   GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3)//低电平表示按下
#define KEY0   GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_4)//低电平表示按下

void key_init(void);//按键始化函数
















#endif
























