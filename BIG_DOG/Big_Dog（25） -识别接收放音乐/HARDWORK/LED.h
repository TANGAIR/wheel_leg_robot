#ifndef LED_H
#define LED_H
#include "stm32f4xx.h" 

#define  LED_RED_ON       GPIO_ResetBits(GPIOH, GPIO_Pin_12)
#define  LED_RED_OFF      GPIO_SetBits(GPIOH, GPIO_Pin_12)
#define  LED_RED_TOGGLE   GPIO_ToggleBits(GPIOH, GPIO_Pin_12)

#define  LED_GREEN_OFF        GPIO_ResetBits(GPIOH, GPIO_Pin_11)
#define  LED_GREEN_ON       GPIO_SetBits(GPIOH, GPIO_Pin_11)
#define  LED_GREEN_TOGGLE    GPIO_ToggleBits(GPIOH, GPIO_Pin_11)


#define  LED_BULE_ON         GPIO_SetBits(GPIOH, GPIO_Pin_10)
#define  LED_BULE_OFF        GPIO_ResetBits(GPIOH, GPIO_Pin_10)
#define  LED_BULE_TOGGLE    GPIO_ToggleBits(GPIOH, GPIO_Pin_10)

void LED_Init(void);
void Heart_Task(void *pvParameters);

#endif
