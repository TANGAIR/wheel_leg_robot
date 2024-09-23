#ifndef USART_H
#define USART_H

#include "sys.h" 
 #include "can.h" 
/*********************************************结构体定义区**********************************************************************/
//发送给TX2数据结构体
typedef struct
{
	char  Left_Front_Leg_X;  //左前腿足端在关节坐标系X的坐标,单位CM，以关节舵机轴心为原点，机器狗前进方向为X轴正方向
	char  Left_Front_Leg_Y;  //左前腿足端在关节坐标系Y的坐标,单位CM，以关节舵机轴心为原点，向下为Y轴正方向
	char  Right_Front_Leg_X; //右前腿足端在关节坐标系X的坐标，单位CM，以关节舵机轴心为原点，机器狗前进方向为X轴正方向
	char  Right_Front_Leg_Y; //右前腿足端在关节坐标系Y的坐标，单位CM，以关节舵机轴心为原点，向下为Y轴正方向
	short IMU_PITCH;         //陀螺仪俯仰角PITCH，单位为°，G_Y，向上为正，向下为负
	short IMU_YAW;           //陀螺仪航向角YAW，单位为°，G_Z，俯视向左为正，向右为负
	char  Left_Front_Wheel_Speed;//右前轮电机速度，单位CM/S
	
}Send_TX2_Data_Typedef;

//接收TX2数据结构体
typedef struct
{
	char  Dog_Mode;      //机器狗模式,1，轮式跟随模式；2，上楼梯模式；3，下楼梯模式；0，停止
	char  Climb_Status;  //爬楼状态,1，前腿爬行后腿滑行；2，前后腿都爬行；3，前腿滑行后腿爬行
	char  Front_Leg_Distance; //爬楼时，左前腿落点与楼梯中点的水平距离,单位CM，以前进方向为正，以中点为0
	char  Stair_Higth;        //楼梯高度，单位CM
	char  Stair_Width;        //楼梯宽度，单位CM
	char  Slide_Speed;        //平地滑行速度，单位CM/S，爬楼时为0，用于控制机器狗跟随模式时的滑行速度
	char  Turn_Angle;         //机器狗的转角,单位°，用于控制机器狗转向
	char  Head_Pitch_Angle;   //头舵机PITCH角度,单位°，用于控制机器狗抬头低头
	char  Head_Yaw_Angle;     //头舵机YAW角度,单位°，用于控制机器狗左右转头
	
}Receive_TX2_Data_Typedef;








//串口转CAN数据结构体
typedef struct
{
	int Can_ID;
	uint8_t  Data_Lenth;
	uint8_t  Data[8];
	
}USART2CAN_Typedef;

















/*********************************************函数声明区**********************************************************************/
void USART1_Init(void);
void TX2_Init(void);


void USART3_Send_TX2(void);



void USART12CAN_TX_Send_Cmd (char CAN_DATA_CMD[8],char ID);
void USART12CAN_TX_Jiont (CAN1_Data_TypeDef* motor) ;


void TX2_Decode_Task(void *pvParameters);

short  u8ToShort(char a[],char b);
char   floatTou8(char bit,float data);
short  int8ToShort(int8_t a[],char b);
char   shortTou8(char bit,short data);
int8_t shortToint8(char bit,short data);
float bit8TObit32(uint8_t *change_info);
uint8_t bit32TObit8(int index_need,int bit32);
/*********************************************数据定义域拓展区**********************************************************************/
extern Receive_TX2_Data_Typedef Receive_TX2_Data_TypeStruct;
extern char   Usart3Rx_Info[10];

extern uint8_t USART2CAN_start[7];

#endif

