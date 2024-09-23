#ifndef USART_H
#define USART_H

#include "sys.h"       
/*********************************************结构体定义区**********************************************************************/
//TX2接收数据结构体
typedef struct
{
	char Left_Front_Leg_X;  //左前腿足端在关节坐标系X的坐标,单位CM，以关节舵机轴心为原点，机器狗前进方向为X轴正方向
	char Left_Front_Leg_Y;  //左前腿足端在关节坐标系Y的坐标,单位CM，以关节舵机轴心为原点，向下为Y轴正方向
	char Right_Front_Leg_X; //
	char Right_Front_Leg_Y;
	short IMU_PITCH;
	short IMU_YAW;
	char  Left_Front_Wheel_Speed;
	
}Send_TX2_Data_Typedef;


/*********************************************函数声明区**********************************************************************/
void USART6_Init(void);
void TX2_Init(void);


void USART3_Send_TX2(void);



void TX2_Decode_Task(void *pvParameters);

short  u8ToShort(char a[],char b);
char   floatTou8(char bit,float data);
short  int8ToShort(int8_t a[],char b);
char   shortTou8(char bit,short data);
int8_t shortToint8(char bit,short data);

/*********************************************数据定义域拓展区**********************************************************************/
extern TX2_Data_Typedef    Down_TX2_Data_TypeStruct;
extern char   Usart3Rx_Info[10];



#endif

