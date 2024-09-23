#ifndef STEERING_H
#define STEERING_H

#include "sys.h"

//头舵机
#define Head_PITCH_Angle(x)  TIM_SetCompare2(TIM5,x);
#define Head_YAW_Angle(x)    TIM_SetCompare1(TIM5,x);

//轮舵机
//停1000，转动1250
#define Left_Front_Speed(x)    TIM_SetCompare1(TIM4,x);
#define Left_Back_Speed(x)     TIM_SetCompare2(TIM4,x);
#define Right_Front_Speed(x)   TIM_SetCompare3(TIM4,x);
#define Right_Back_Speed(x)    TIM_SetCompare4(TIM4,x);

//关节舵机
#define Left_Front_IN(x)       TIM_SetCompare1(TIM2,x);
#define Left_Front_OUT(x)      TIM_SetCompare2(TIM2,x);
#define Left_Back_In(x)        TIM_SetCompare3(TIM2,x);
#define Left_Back_OUT(x)       TIM_SetCompare4(TIM2,x);

#define Right_Front_IN(x)       TIM_SetCompare1(TIM8,x);
#define Right_Front_OUT(x)      TIM_SetCompare2(TIM8,x);
#define Right_Back_In(x)        TIM_SetCompare3(TIM8,x);
#define Right_Back_OUT(x)       TIM_SetCompare4(TIM8,x);

void Steering_Init(void);

#endif


