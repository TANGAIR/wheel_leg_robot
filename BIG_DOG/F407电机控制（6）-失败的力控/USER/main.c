/*
*程序名称：STM32F407模板程序
*芯片型号：STM32F407ZGT6
*程序描述：这是一个经过测试后可以使用的程序模板，下载之后，按主板上的复位键或者重新连接主板电源
*          主板上的LED灯会交错闪烁最后只有一个灯亮， 串口会打印出初始化OK的字样
*         
*硬件连接：电源：找到主板上的5伏和GND分别接5伏电源正极和负极
*          程序下载：程序使用ST link下载,主板下载接口与ST link接口连接(四根线如下所述
*                    正对主板的下载接正对主板的下载接口，接口序号如下：
*                            19 17 15 13 11 9  7 5 3 1
*                            20 18 16 14 12 10 8 6 4 2
*                    其中序号为1的接口接3.3伏，序号为7的接口接SWDIO,序号为9的接口接SWCLK，序号为20的接口接GND
*          串口通讯：程序使用串口一,主板上的PA9和pA10分别接CH340串口的RX和TX,再接由主板的GND到串口的GND
*/
#include "main.h"

 /*************************************************数据定义区*************************************************/
 int mode=1;
 char Contron_ID=5; 
 char Contron_ID_2=6;
 char Contron_ID_3=7;
 char Contron_ID_4=5;
 /*************************************************函数定义区**************************************************/


char USART_CAN_SEND_WAIT=0;    //发送等待
int delay_time=10;  //以15ms为命令周期，就可以都收到 ，共计0.12s的八条腿指令周期
int led_num=0;
int main(void)//主函数
{
	
	
	/*初始化*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);/* 配置NVIC为优先级组2 */
  delay_init (168);        //延迟初始化
	LED_GPIO_Config();       //LED初始化
  key_init();              //按键初始化函数
	
	
	
	//F407需要比C板慢一些启动
	delay_ms(3000);
	
	//串口转CAN
	USART2_Init();           //串口2初始化 T---PA2   R---PA3
	USART3_Init();           //串口3初始化 T---PD8   R---PD9
	USART4_Init();           //串口4初始化 T---PC10   R---PC11
	USART5_Init();           //串口5初始化 T---PC12   R---PD2
	delay_ms(100);
	Jiont_Motor_FOC_Init();  //电机CAN数据结构体初始化
	
	//两板通讯
	delay_ms(10);
	USART1_Init();
	
	
	led_against();
/**************************************************************/
	 //循环
  while(1)
  {
		
		
	  	//反馈数据
			F407_to_C_Send();
			delay_ms(delay_time);
		
		
		
		 /*************LED闪烁****************************/
		 led_num++;
		 if(led_num >=50){led_num=0;LED2_TOGGLE;}
	   
 }
}


