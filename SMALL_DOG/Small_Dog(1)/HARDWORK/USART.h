#ifndef USART_H
#define USART_H

#include "sys.h"       
/*********************************************�ṹ�嶨����**********************************************************************/
//TX2�������ݽṹ��
typedef struct
{
	char Left_Front_Leg_X;  //��ǰ������ڹؽ�����ϵX������,��λCM���Թؽڶ������Ϊԭ�㣬������ǰ������ΪX��������
	char Left_Front_Leg_Y;  //��ǰ������ڹؽ�����ϵY������,��λCM���Թؽڶ������Ϊԭ�㣬����ΪY��������
	char Right_Front_Leg_X; //
	char Right_Front_Leg_Y;
	short IMU_PITCH;
	short IMU_YAW;
	char  Left_Front_Wheel_Speed;
	
}Send_TX2_Data_Typedef;


/*********************************************����������**********************************************************************/
void USART6_Init(void);
void TX2_Init(void);


void USART3_Send_TX2(void);



void TX2_Decode_Task(void *pvParameters);

short  u8ToShort(char a[],char b);
char   floatTou8(char bit,float data);
short  int8ToShort(int8_t a[],char b);
char   shortTou8(char bit,short data);
int8_t shortToint8(char bit,short data);

/*********************************************���ݶ�������չ��**********************************************************************/
extern TX2_Data_Typedef    Down_TX2_Data_TypeStruct;
extern char   Usart3Rx_Info[10];



#endif

