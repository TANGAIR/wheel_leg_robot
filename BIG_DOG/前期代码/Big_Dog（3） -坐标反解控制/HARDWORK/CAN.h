#ifndef CAN_H
#define CAN_H
#include "stm32f4xx.h"


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
	
	

	
}CAN1_Data_TypeDef;


 #define P_MIN -12.5f
 #define P_MAX 12.5f
 #define V_MIN -45.0f
 #define V_MAX 45.0f
 #define KP_MIN 0.0f
 #define KP_MAX 500.0f
 #define KD_MIN 0.0f
 #define KD_MAX 5.0f
 #define T_MIN -18.0f
 #define T_MAX 18.0f
 


#define CAN1_JOINT_ID_LF_U                0x01          //��ǰ�ϽǶȵ��1
#define CAN1_JOINT_ID_LF_D                0x02          //��ǰ�½Ƕȵ��2
#define CAN1_JOINT_ID_LB_U                0X03          //����ϽǶȵ��3
#define	CAN1_JOINT_ID_LB_D                0X04          //����½Ƕȵ��4
#define	CAN1_JOINT_ID_RB_U								0X05					//�Һ��ϽǶȵ��5
#define	CAN1_JOINT_ID_RB_D								0X06					//�Һ��½Ƕȵ��6
#define	CAN1_JOINT_ID_RF_U								0X07					//��ǰ�ϽǶȵ��7
#define	CAN1_JOINT_ID_RF_D								0X08					//��ǰ�½Ƕȵ��8












float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x, float x_min, float x_max, int bits);


void CAN1_Init(void);




extern CAN1_Data_TypeDef   Joint_LF_U,//��ǰ�ϽǶȵ��1
													 Joint_LF_D,//��ǰ�½Ƕȵ��2
													 Joint_LB_U,//����ϽǶȵ��3
													 Joint_LB_D,//����½Ƕȵ��4
													 Joint_RB_U,//�Һ��ϽǶȵ��5
													 Joint_RB_D,//�Һ��½Ƕȵ��6
													 Joint_RF_U,//��ǰ�ϽǶȵ��7
													 Joint_RF_D;//��ǰ�½Ƕȵ��8

void CAN1_Send_Cmd (char CAN_DATA_CMD[8],char ID);

void CAN1_TX_PTZ(void);
void CAN1_TX_Jiont (CAN1_Data_TypeDef* motor);






#endif
