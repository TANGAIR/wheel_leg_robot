#include "main.h"   

 
void LED_GPIO_Config(void)
{		
	  RCC_AHB1PeriphClockCmd ( LED1_GPIO_CLK,ENABLE); /*����LED��ص�GPIO����ʱ��*/
		GPIO_InitTypeDef GPIO_InitStructure;/*����һ��GPIO_InitTypeDef���͵Ľṹ��*/
														   
		GPIO_InitStructure.GPIO_Pin = LED1_PIN;	/*ѡ��Ҫ���Ƶ�GPIO����*/
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;/*��������ģʽΪ���ģʽ*/
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; /*�������ŵ��������Ϊ�������*/
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;/*��������Ϊ����ģʽ*/
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; 	/*������������Ϊ2MHz */
		GPIO_Init(LED1_GPIO_PORT, &GPIO_InitStructure);	/*���ÿ⺯����ʹ���������õ�GPIO_InitStructure��ʼ��GPIO*/
    														   
		GPIO_InitStructure.GPIO_Pin = LED2_PIN;	/*ѡ��Ҫ���Ƶ�GPIO����*/	
    GPIO_Init(LED2_GPIO_PORT, &GPIO_InitStructure);	
		/*�ر�RGB��*/
    LED1_OFF;
	  LED2_OFF;
	
}



int led_t=5;//��ʱ��λ��


void led_against(void)
{
 while(led_t-->0)
 {
    LED1_ON;
	  LED2_OFF;
	 delay_ms(100);
    LED1_OFF;
	  LED2_ON;
	 delay_ms(100);
 }
 LED1_ON;//�̵Ƴ�����ʾ����������������
 LED2_OFF;//�رպ��
}


void led_R_twinkle(void )//�����˸
{
while(1)//���һֱ��˸
 {
    LED1_OFF;
	 delay_ms(300);
    LED1_ON;
	 delay_ms(300);
 }

}


void led_Y_twinkle(void )//��ɫ����˸��˸
{
while(1)
 {
    LED2_OFF;
	 delay_ms(300);
    LED2_ON;
	 delay_ms(300);
 }

}













