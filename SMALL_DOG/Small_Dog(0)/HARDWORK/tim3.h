#ifndef __TIM2_H
#define __TIM2_H 	
#include "sys.h"



#define GENERAL_TIM           		TIM3
#define GENERAL_TIM_CLK       		RCC_APB1Periph_TIM3

#define GENERAL_TIM_IRQn		      TIM3_IRQn
#define GENERAL_TIM_IRQHandler    TIM3_IRQHandler

void TIM3_Init(void);






#endif


