#ifndef __LED_H
#define	__LED_H

#include "stm32f4xx.h"

//���Ŷ���
/*******************************************************/
//R ��ɫ��
#define LED1_PIN                  GPIO_Pin_9                 
#define LED1_GPIO_PORT            GPIOF                      
#define LED1_GPIO_CLK             RCC_AHB1Periph_GPIOF

//G ��ɫ��
#define LED2_PIN                  GPIO_Pin_10                 
#define LED2_GPIO_PORT            GPIOF                      
#define LED2_GPIO_CLK             RCC_AHB1Periph_GPIOF


/************************************************************/


/* ֱ�Ӳ����Ĵ����ķ�������IO */
#define	digitalHi(p,i)			 {p->BSRRL=i;}		//����Ϊ�ߵ�ƽ
#define digitalLo(p,i)			 {p->BSRRH=i;}		//����͵�ƽ
#define digitalToggle(p,i)	 {p->ODR ^=i;}		//�����ת״̬

/* �������IO�ĺ� */
#define LED1_TOGGLE		digitalToggle(LED1_GPIO_PORT,LED1_PIN)//����LED��ת
#define LED1_OFF			digitalHi(LED1_GPIO_PORT,LED1_PIN)
#define LED1_ON				GPIO_ResetBits(LED1_GPIO_PORT,LED1_PIN)

#define LED2_TOGGLE		digitalToggle(LED2_GPIO_PORT,LED2_PIN)
#define LED2_OFF			digitalHi(LED2_GPIO_PORT,LED2_PIN)
#define LED2_ON				digitalLo(LED2_GPIO_PORT,LED2_PIN)


//��
#define LED_RED  \
					LED1_ON;\
					LED2_OFF;
//��
#define LED_GREEN		\
					LED1_OFF;\
					LED2_ON;	


void LED_GPIO_Config(void);

void led_against(void);//����Ƴ���

void led_R_twinkle(void );//�����˸

void led_Y_twinkle(void );//�̵���˸













#endif /* __LED_H */
