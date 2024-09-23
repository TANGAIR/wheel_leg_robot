#ifndef CAN_H
#define CAN_H
#include "stm32f4xx.h"


typedef struct
{
	char Motor_ID;
	//发送
  struct 
	{
	  float send_Position;
	  float send_Velocity;
	  float send_kp;
	  float send_kd;
	  float send_torque;	
	}send;
	
	//接收
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
 


#define CAN1_JOINT_ID_LF_U                0x01          //左前上角度电机1
#define CAN1_JOINT_ID_LF_D                0x02          //左前下角度电机2
#define CAN1_JOINT_ID_LB_U                0X03          //左后上角度电机3
#define	CAN1_JOINT_ID_LB_D                0X04          //左后下角度电机4
#define	CAN1_JOINT_ID_RB_U								0X05					//右后上角度电机5
#define	CAN1_JOINT_ID_RB_D								0X06					//右后下角度电机6
#define	CAN1_JOINT_ID_RF_U								0X07					//右前上角度电机7
#define	CAN1_JOINT_ID_RF_D								0X08					//右前下角度电机8












float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x, float x_min, float x_max, int bits);


void CAN1_Init(void);




extern CAN1_Data_TypeDef   Joint_LF_U,//左前上角度电机1
													 Joint_LF_D,//左前下角度电机2
													 Joint_LB_U,//左后上角度电机3
													 Joint_LB_D,//左后下角度电机4
													 Joint_RB_U,//右后上角度电机5
													 Joint_RB_D,//右后下角度电机6
													 Joint_RF_U,//右前上角度电机7
													 Joint_RF_D;//右前下角度电机8

void CAN1_Send_Cmd (char CAN_DATA_CMD[8],char ID);

void CAN1_TX_PTZ(void);
void CAN1_TX_Jiont (CAN1_Data_TypeDef* motor);






#endif
