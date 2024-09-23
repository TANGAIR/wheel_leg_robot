#ifndef CAN_H
#define CAN_H
#include "stm32f4xx.h"


typedef struct
{
	char Motor_ID;
	//∑¢ÀÕ
	float send_Position;
	float send_Velocity;
	float send_kp;
	float send_kd;
	float send_torque;
	//Ω” ’
	int recieve_Position;
	int recieve_Velocity;
	int recieve_Current;
	
	
}CAN1_Data_TypeDef;




#define CAN1_JOINT_ID_LF_U                0x01
#define CAN1_JOINT_ID_LF_D                0x02




float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x, float x_min, float x_max, int bits);


void CAN1_Init(void);




extern CAN1_Data_TypeDef Joint_LF_U,
                         Joint_LF_D;

extern void CAN1_Enlab (void);

void CAN1_TX_PTZ(void);
void CAN1_TX_Jiont (CAN1_Data_TypeDef* motor);






#endif
