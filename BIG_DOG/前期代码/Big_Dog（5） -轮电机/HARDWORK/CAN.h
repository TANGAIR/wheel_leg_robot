#ifndef CAN_H
#define CAN_H
#include "stm32f4xx.h"

/**********************************************************结构体定义*******************************************************/
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




typedef struct
{
	short Real_Speed;
	short Real_MechanicalAngle;
	short Target_Speed;
	short Target_MechanicalAngle;
	short Send_Current;
	
}CAN2_Data_TypeDef;






/*******************************************************宏定义*******************************************************/
//关节电机FOC控制参数
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
 

//关节电机CAN1ID
#define CAN1_JOINT_ID_LF_U                0x01          //左前上角度电机1
#define CAN1_JOINT_ID_LF_D                0x02          //左前下角度电机2
#define CAN1_JOINT_ID_LB_U                0X03          //左后上角度电机3
#define	CAN1_JOINT_ID_LB_D                0X04          //左后下角度电机4
#define	CAN1_JOINT_ID_RB_U								0X05					//右后上角度电机5
#define	CAN1_JOINT_ID_RB_D								0X06					//右后下角度电机6
#define	CAN1_JOINT_ID_RF_U								0X07					//右前上角度电机7
#define	CAN1_JOINT_ID_RF_D								0X08					//右前下角度电机8

//轮电机ID
#define CAN2_TRANSMIT_ID   0x200
#define CAN2_LF            0x201
#define CAN2_LB            0x202
#define CAN2_RB            0x203
#define CAN2_RF            0x204






/************************************************函数声明***********************************************************/
void CAN1_Init(void);
void CAN1_Send_Cmd (char CAN_DATA_CMD[8],char ID);
void CAN1_TX_Jiont (CAN1_Data_TypeDef* motor); 
 
void CAN2_Init(void);
void CAN2_TX(void); 
 
 
 
float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x, float x_min, float x_max, int bits);
 /***********************************************数据定义域拓展********************************************************/
extern CAN1_Data_TypeDef   Joint_LF_U,//左前上角度电机1
													 Joint_LF_D,//左前下角度电机2
													 Joint_LB_U,//左后上角度电机3
													 Joint_LB_D,//左后下角度电机4
													 Joint_RB_U,//右后上角度电机5
													 Joint_RB_D,//右后下角度电机6
													 Joint_RF_U,//右前上角度电机7
													 Joint_RF_D;//右前下角度电机8

//轮电机
extern CAN2_Data_TypeDef   Wheel_LF,//左前轮
													 Wheel_LB,//左后轮
													 Wheel_RB,//右后轮
													 Wheel_RF;//右前轮



//关节电机CAN1命令
extern char CAN_DATA_CMD_ON[8];
extern char CAN_DATA_CMD_OFF[8];
extern char CAN_DATA_CMD_SET_0[8];
extern char CAN_DATA_CMD_Change_ID[8];







#endif
