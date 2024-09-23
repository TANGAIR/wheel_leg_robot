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
	float Err_Change;              //Îó²î±ä»¯Á¿

}PID_TypeDef;





void Jiont_Motor_Angle_PID_Init(void);
void Jiont_Motor_FOC_Init(void);

float Pid_Calc(PID_TypeDef *PID,float Current_Speed,float Target_Speed);









extern PID_TypeDef  Jiont_Motor_Angle_PID_LF_U,
										Jiont_Motor_Angle_PID_LF_D,
										Jiont_Motor_Angle_PID_LB_U,
										Jiont_Motor_Angle_PID_LB_D,
										Jiont_Motor_Angle_PID_RB_U,
										Jiont_Motor_Angle_PID_RB_D,
										Jiont_Motor_Angle_PID_RF_U,
										Jiont_Motor_Angle_PID_RF_D;


extern PID_TypeDef Jiont_Motor_Speed_PID_LF_U,
            Jiont_Motor_Speed_PID_LF_D,
						Jiont_Motor_Speed_PID_LB_U,
						Jiont_Motor_Speed_PID_LB_D,
            Jiont_Motor_Speed_PID_RB_U,
            Jiont_Motor_Speed_PID_RB_D,
						Jiont_Motor_Speed_PID_RF_U,
						Jiont_Motor_Speed_PID_RF_D;



#endif 

