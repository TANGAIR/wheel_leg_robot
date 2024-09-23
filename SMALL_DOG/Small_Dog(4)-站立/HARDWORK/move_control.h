#ifndef MOVE_CONTROL_H
#define MOVE_CONTROL_H
#include "sys.h"


//结构体定义
typedef struct
{
	double  Leg_X;            //足端在关节坐标系X的坐标,单位CM，以关节舵机轴心为原点，机器狗前进方向为X轴正方向
	double  Leg_Y;            //足端在关节坐标系Y的坐标,单位CM，以关节舵机轴心为原点，向下为Y轴正方向
	double Steerint_IN;       //腿内侧舵机控制值
	double Steerint_OUT;      //腿外侧舵机控制值
	char   Leg_Type;          //腿的类型
	double step_forward_time; //迈步周期计时
	double quiesecence_time;  //静退周期计时
		
}Leg_Data_Typedef;


/*宏定义*/
//腿的类型
#define LEFT_FRONT_LEG  11
#define LEFT_BACK_LEG   22
#define RIGHT_BACK_LEG  33
#define RIGHT_FRONT_LEG 44
//轨迹
#define QUIESECENCE  (-1) //静止退后
#define STEP_FORWARD  (1) //迈步前进
//方向
#define DIRECTION_FORWARD  (1) //前进
#define DIRECTION_BACK  (-1)   //后退
//收缩步态
#define SHRINK (0)
//圆周律
#define pi 3.14159

//轨迹计算参数
//腿杆长(MM)
#define CLUB_LENGTH 120

//修正摆线起始X坐标（mm），足端初始X坐标
#define FOOT_X 0
//足端静退直线Y坐标（mm），足端初始Y坐标
#define FOOT_Y 190

//修正摆线步长（mm）
#define STEP_LENTH 80
//修正摆线步高（mm）
#define STEP_HIGHT 30

//迈步用时ms
#define STEP_FORWARD_TIME 2000
//静退用时ms
#define QUIESECENCE_TIME  1800

//轮电机速度定义
#define WHEEL_STOP  1000


void Foot_Direction_Init(void);
void Shrink(void);
void Move_Control_Task(void *pvParameters);


//腿数据结构体
extern  Leg_Data_Typedef Left_Front_Leg_Struct;
extern  Leg_Data_Typedef Left_Back_Leg_Struct;
extern  Leg_Data_Typedef Right_Back_Leg_Struct;
extern  Leg_Data_Typedef Right_Front_Leg_Struct;








#endif 
