#ifndef STEERING_H
#define STEERING_H

#include "sys.h"

//ͷ���
#define Head_PITCH_Angle(x)  TIM_SetCompare2(TIM5,x);
#define Head_YAW_Angle(x)    TIM_SetCompare1(TIM5,x);



#define Left_Front_Speed(x)    TIM_SetCompare1(TIM8,x);   //��ǰ��-W-PI5-TIM8C1
#define Left_Front_IN(x)       TIM_SetCompare1(TIM2,x);   //��ǰ��-S-PA0-TIM2C1
#define Left_Front_OUT(x)      TIM_SetCompare2(TIM2,x);   //��ǰ��-T-PA1-TIM2C2


#define Left_Back_Speed(x)     TIM_SetCompare2(TIM8,x);  // �����-X-PI6-TIM8C2
#define Left_Back_In(x)        TIM_SetCompare3(TIM2,x);  // �����-U-PA2-TIM2C3
#define Left_Back_OUT(x)       TIM_SetCompare4(TIM2,x);  // �����-V-PA3-TIM2C4


#define Right_Front_Speed(x)    TIM_SetCompare3(TIM8,x); // ��ǰ��-Y-PI7-TIM8C3
#define Right_Front_IN(x)       TIM_SetCompare1(TIM4,x); // ��ǰ��-H-PD12-TIM4C1
#define Right_Front_OUT(x)      TIM_SetCompare2(TIM4,x); // ��ǰ��-G-PD13-TIM4C2


#define Right_Back_Speed(x)     TIM_SetCompare4(TIM5,x); //  �Һ���-A-PI0-TIM5C4
#define Right_Back_In(x)        TIM_SetCompare3(TIM4,x); //  �Һ���-F-PD14-TIM4C3
#define Right_Back_OUT(x)       TIM_SetCompare4(TIM4,x); //  �Һ���-E-PD15-TIM4C4

void Steering_Init(void);

#endif


