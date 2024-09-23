#ifndef MOVE_CONTROL_H
#define MOVE_CONTROL_H
#include "sys.h"


//�ṹ�嶨��
typedef struct
{
	double  Leg_X;            //����ڹؽ�����ϵX������,��λCM���Թؽڶ������Ϊԭ�㣬������ǰ������ΪX��������
	double  Leg_Y;            //����ڹؽ�����ϵY������,��λCM���Թؽڶ������Ϊԭ�㣬����ΪY��������
	double Steerint_IN;       //���ڲ�������ֵ
	double Steerint_OUT;      //�����������ֵ
	char   Leg_Type;          //�ȵ�����
	double step_forward_time; //�������ڼ�ʱ
	double quiesecence_time;  //�������ڼ�ʱ
		
}Leg_Data_Typedef;


/*�궨��*/
//�ȵ�����
#define LEFT_FRONT_LEG  11
#define LEFT_BACK_LEG   22
#define RIGHT_BACK_LEG  33
#define RIGHT_FRONT_LEG 44
//�켣
#define QUIESECENCE  (-1) //��ֹ�˺�
#define STEP_FORWARD  (1) //����ǰ��
//����
#define DIRECTION_FORWARD  (1) //ǰ��
#define DIRECTION_BACK  (-1)   //����
//������̬
#define SHRINK (0)
//Բ����
#define pi 3.14159

//�켣�������
//�ȸ˳�(MM)
#define CLUB_LENGTH 120

//����������ʼX���꣨mm������˳�ʼX����
#define FOOT_X 0
//��˾���ֱ��Y���꣨mm������˳�ʼY����
#define FOOT_Y 190

//�������߲�����mm��
#define STEP_LENTH 80
//�������߲��ߣ�mm��
#define STEP_HIGHT 30

//������ʱms
#define STEP_FORWARD_TIME 2000
//������ʱms
#define QUIESECENCE_TIME  1800

//�ֵ���ٶȶ���
#define WHEEL_STOP  1000


void Foot_Direction_Init(void);
void Shrink(void);
void Move_Control_Task(void *pvParameters);


//�����ݽṹ��
extern  Leg_Data_Typedef Left_Front_Leg_Struct;
extern  Leg_Data_Typedef Left_Back_Leg_Struct;
extern  Leg_Data_Typedef Right_Back_Leg_Struct;
extern  Leg_Data_Typedef Right_Front_Leg_Struct;








#endif 
