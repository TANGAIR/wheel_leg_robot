#ifndef  GAIT_H
#define  GAIT_H	 
#include "stm32f4xx.h" 
#include "CAN.h"

/**********************************************************结构体定义*******************************************************/
typedef struct
{
  char   Leg_Type;          //腿的类型
	
	char  Leg_Mode;           //腿的模式，1轨迹，0静退

	float  Leg_X;             //目标X，足端在关节坐标系X的坐标,单位CM，以关节舵机轴心为原点，机器狗前进方向为X轴正方向
	float  Leg_Y;             //目标Y，足端在关节坐标系Y的坐标,单位CM，以关节舵机轴心为原点，向下为Y轴正方向
	
  
  float real_x;             //通过关节角度解算出的真实足端坐标
  float real_y; 						//通过关节角度解算出的真实足端坐标


	float  step_forward_time; //迈步周期计时
	float  quiesecence_time;  //静退周期计时


	float  Y_Add;                    //Y轴自适应补偿

  CAN1_Data_TypeDef *Jiont_Motor_U;//上电机，序号较低，对应seigamar，角度单位是弧度
		
  CAN1_Data_TypeDef *Jiont_Motor_D;//下电机，序号较高，对应gama，角度单位是弧度
	
	
	
	
	

}Leg_Data_Typedef;


/*******************************************************宏定义*******************************************************/
//腿的类型
#define LEFT_FRONT_LEG  11
#define LEFT_BACK_LEG   22
#define RIGHT_BACK_LEG  33
#define RIGHT_FRONT_LEG 44
//轨迹
#define QUIESECENCE  (1) //静止退后
#define STEP_FORWARD  (2) //迈步前进
//方向
#define DIRECTION_FORWARD  (2) //前进
#define DIRECTION_BACK  (1)   //后退
//收缩步态
#define SHRINK (0)
//圆周律
#define pi (3.141592654)

//轨迹计算参数
//腿杆长(MM)
#define CLUB_LENGTH (230)

//修正摆线起始X坐标（mm），足端初始X坐标
#define FOOT_X   (-90)
//足端静退直线Y坐标（mm），足端初始Y坐标
#define FOOT_Y   (370)

//修正摆线步长（mm）
#define STEP_LENTH  (180)
//修正摆线步高（mm）
#define STEP_HIGHT  (100)



//迈步用时10ms,或者说迈步周期数
#define STEP_FORWARD_TIME (75)
//静退用时10ms
#define QUIESECENCE_TIME  (40)
//轨迹拟合偏差，单位mm,大于偏差则电机一直输出之前的数值，防止轨迹不完整
#define DISTANCE_IGNORE  (10)




//轮电机步行基本速度定义
#define WHEEL_STEP_SPEED  (50)






/************************************************函数声明***********************************************************/
//收缩
void Shrink(void);
//自适应初始化
void Leg_Deinit(Leg_Data_Typedef *Leg_Struct_Point);

//滑行
void Slide(void);

//坐标反解控制
void Coordinate_Inverse(Leg_Data_Typedef* Leg_Data);
//坐标控制
void Leg_Diretion_Control(Leg_Data_Typedef* Leg_Data,float foot_x,float foot_y);
//足端轨迹控制
void Foot_Control(Leg_Data_Typedef* Leg_Data,char trail,char direction);
//单腿循环
 void signal_leg(Leg_Data_Typedef *Leg_Data);
//四足步态控制
void Gait_Control(char gait);




/***********************************************数据定义域拓展********************************************************/
//腿数据结构体
extern  Leg_Data_Typedef Left_Front_Leg_Struct;
extern  Leg_Data_Typedef Left_Back_Leg_Struct;
extern  Leg_Data_Typedef Right_Back_Leg_Struct;
extern  Leg_Data_Typedef Right_Front_Leg_Struct;




#endif


