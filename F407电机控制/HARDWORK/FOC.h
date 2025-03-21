#ifndef __FOC_H
#define	__FOC_H

#include "stm32f4xx.h"

typedef struct
{
	float P;
	float I;
	float D;
	float CurrentError;
	float LastError;
	float ErrorIgnored;
	float Pout;
	float Iout;
	float Dout;
	float PIDout;
	float PIDOutCompensate;
	float PMax;	
	float IMax;
	float DMax;
	float PIDOutMax;
	float PIDOutLast;
	float Target_Speed_Last;
	float Speed_Ratio;
	float Acceleration;
	float LastLastError;
	float Err_Change;              //误差变化量
	float Ke;                      //误差量化因子  Ke=N/emax  在这里N=3       e=Ke*e
	float Kec;                     //误差变化量量化因子
	float Ku_P;                    //Kp的缩放因子    Kp=Ku_P*Kp_M   Kp_M为根据模糊规则算出的在【-3,3】范围内的值  Ku=umax/N
	float Ku_I;                    //KI的缩放因子
	float Ku_D;                    //Kd的缩放因子
}PID_TypeDef;





void Jiont_Motor_PID_Init(void);
void Jiont_Motor_FOC_Init(void);
 //正常模式
void Jiont_Motor_Normal(void);
 //阻尼模式
void Jiont_Motor_Passive(void);

float Pid_Calc(PID_TypeDef *PID,float Current_Speed,float Target_Speed);









extern PID_TypeDef  Jiont_Motor_PID_LF_U,
										Jiont_Motor_PID_LF_D,
										Jiont_Motor_PID_LB_U,
										Jiont_Motor_PID_LB_D,
										Jiont_Motor_PID_RB_U,
										Jiont_Motor_PID_RB_D,
										Jiont_Motor_PID_RF_U,
										Jiont_Motor_PID_RF_D;






#endif 

