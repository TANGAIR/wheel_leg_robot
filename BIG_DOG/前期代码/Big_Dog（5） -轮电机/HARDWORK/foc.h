#ifndef FOC_H
#define FOC_H
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
	float Err_Change;              //���仯��
	float Ke;                      //�����������  Ke=N/emax  ������N=3       e=Ke*e
	float Kec;                     //���仯����������
	float Ku_P;                    //Kp����������    Kp=Ku_P*Kp_M   Kp_MΪ����ģ������������ڡ�-3,3����Χ�ڵ�ֵ  Ku=umax/N
	float Ku_I;                    //KI����������
	float Ku_D;                    //Kd����������
}PID_TypeDef;



void Jiont_Motor_FOC_Init(void);
void Wheel_Motor_PID_Init(void);
float Pid_Calc(PID_TypeDef *PID,float Current_Speed,float Target_Speed);








extern  PID_TypeDef Wheel_Motor_PID_201,
										Wheel_Motor_PID_202,
										Wheel_Motor_PID_203,
										Wheel_Motor_PID_204;








#endif




