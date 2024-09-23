#ifndef __USART1_H
#define	__USART1_H

#include "stm32f4xx.h"
#include "FOC.h"


typedef struct
{
	char Motor_ID;
	//����
  struct 
	{
	  float send_Position;
	  float send_Velocity;
	  float send_kp;
	  float send_kd;
	  float send_torque;	
	}send;
	
	//����
	struct
	{
		float recieve_Position;
		float recieve_Velocity;
		float recieve_Current;	
	}recieve;
	
	
	PID_TypeDef* Motor_PID;
	
}CAN1_Data_TypeDef;




//����תCAN���ݽṹ��
typedef struct
{
	int Can_ID;
	uint8_t  Data_Lenth;
	uint8_t  Data[8];
	
}USART2CAN_Typedef;

void USART1_Init(void);
void F407_to_C_Send (void);
short  u8ToShort(char a[],char b);
char   floatTou8(char bit,float data);
short  int8ToShort(int8_t a[],char b);
char   shortTou8(char bit,short data);
int8_t shortToint8(char bit,short data);
float bit8TObit32(uint8_t *change_info);
uint8_t bit32TObit8(int index_need,int bit32);
  
float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x, float x_min, float x_max, int bits);
/***********************************************���ݶ�������չ********************************************************/
extern CAN1_Data_TypeDef   Joint_LF_U,//��ǰ�ϽǶȵ��1
													 Joint_LF_D,//��ǰ�½Ƕȵ��2
													 Joint_LB_U,//����ϽǶȵ��3
													 Joint_LB_D,//����½Ƕȵ��4
													 Joint_RB_U,//�Һ��ϽǶȵ��5
													 Joint_RB_D,//�Һ��½Ƕȵ��6
													 Joint_RF_U,//��ǰ�ϽǶȵ��7
													 Joint_RF_D;//��ǰ�½Ƕȵ��8



//�ؽڵ��CAN1����
extern char CAN_DATA_CMD_ON[8];
extern char CAN_DATA_CMD_OFF[8];
extern char CAN_DATA_CMD_SET_0[8];
extern char CAN_DATA_CMD_Change_ID[8];

#endif 



