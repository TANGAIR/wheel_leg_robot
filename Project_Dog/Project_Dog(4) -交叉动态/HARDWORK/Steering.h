#ifndef STEERING_H
#define STEERING_H

#include "sys.h"

//头舵机
#define Head_PITCH_Angle(x)  TIM_SetCompare2(TIM5,x);
#define Head_YAW_Angle(x)    TIM_SetCompare1(TIM5,x);



#define Left_Front_Speed(x)    TIM_SetCompare1(TIM8,x);   //左前轮-W-PI5-TIM8C1
#define Left_Front_IN(x)       TIM_SetCompare1(TIM2,x);   //左前内-S-PA0-TIM2C1
#define Left_Front_OUT(x)      TIM_SetCompare2(TIM2,x);   //左前外-T-PA1-TIM2C2


#define Left_Back_Speed(x)     TIM_SetCompare2(TIM8,x);  // 左后轮-X-PI6-TIM8C2
#define Left_Back_In(x)        TIM_SetCompare3(TIM2,x);  // 左后内-U-PA2-TIM2C3
#define Left_Back_OUT(x)       TIM_SetCompare4(TIM2,x);  // 左后外-V-PA3-TIM2C4


#define Right_Front_Speed(x)    TIM_SetCompare3(TIM8,x); // 右前轮-Y-PI7-TIM8C3
#define Right_Front_IN(x)       TIM_SetCompare1(TIM4,x); // 右前内-H-PD12-TIM4C1
#define Right_Front_OUT(x)      TIM_SetCompare2(TIM4,x); // 右前外-G-PD13-TIM4C2


#define Right_Back_Speed(x)     TIM_SetCompare4(TIM5,x); //  右后轮-A-PI0-TIM5C4
#define Right_Back_In(x)        TIM_SetCompare3(TIM4,x); //  右后内-F-PD14-TIM4C3
#define Right_Back_OUT(x)       TIM_SetCompare4(TIM4,x); //  右后外-E-PD15-TIM4C4

void Steering_Init(void);

#endif


