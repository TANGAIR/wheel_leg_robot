#ifndef _LASER_H
#define _LASER_H
#include "stm32f4xx.h" 

#define  LASER_ON      GPIO_SetBits(GPIOC, GPIO_Pin_8)
#define  LASER_OFF     GPIO_ResetBits(GPIOC, GPIO_Pin_8)

void Laser_InitConfig(void);
	
#endif
