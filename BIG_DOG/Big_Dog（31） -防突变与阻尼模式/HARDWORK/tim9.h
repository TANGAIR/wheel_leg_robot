#ifndef __TIM2_H
#define __TIM2_H 	
#include "sys.h"



#define GENERAL_TIM           		TIM9
#define GENERAL_TIM_CLK       		RCC_APB2Periph_TIM9

#define GENERAL_TIM_IRQn		      TIM1_BRK_TIM9_IRQn
#define GENERAL_TIM_IRQHandler    TIM1_BRK_TIM9_IRQHandler

extern int GetTick;

void TIM9_Init(void);






#endif


