#ifndef MOVE_CONTROL_H
#define MOVE_CONTROL_H
#include "sys.h"


/**********************************************************�ṹ�嶨��*******************************************************/
typedef struct
{
	double  Leg_X;             //����ڹؽ�����ϵX������,��λCM���Թؽڶ������Ϊԭ�㣬������ǰ������ΪX��������
	double  Leg_Y;             //����ڹؽ�����ϵY������,��λCM���Թؽڶ������Ϊԭ�㣬����ΪY��������
	char    Leg_Type;          //�ȵ�����
	double  step_forward_time; //�������ڼ�ʱ
	double  quiesecence_time;  //�������ڼ�ʱ
	double  Y_Add;
		
}Leg_Data_Typedef;


/*******************************************************�궨��*******************************************************/
//�ȵ�����
#define LEFT_FRONT_LEG  11
#define LEFT_BACK_LEG   22
#define RIGHT_BACK_LEG  33
#define RIGHT_FRONT_LEG 44
//�켣
#define QUIESECENCE  (1) //��ֹ�˺�
#define STEP_FORWARD  (2) //����ǰ��
//����
#define DIRECTION_FORWARD  (2) //ǰ��
#define DIRECTION_BACK  (1)   //����
//������̬
#define SHRINK (0)
//Բ����
#define pi 3.1415926

//�켣�������
//�ȸ˳�(MM)
#define CLUB_LENGTH 230

//����������ʼX���꣨mm������˳�ʼX����
#define FOOT_X (-90)
//��˾���ֱ��Y���꣨mm������˳�ʼY����
#define FOOT_Y 350

//�������߲�����mm��
#define STEP_LENTH 180
//�������߲��ߣ�mm��
#define STEP_HIGHT 90

//������ʱ2ms
#define STEP_FORWARD_TIME 50
//������ʱ2ms
#define QUIESECENCE_TIME  25

//�ֵ���ٶȶ���
#define WHEEL_STOP  1000

//ת��X






/************************************************��������***********************************************************/

void Gait_Control(char gait);
void Foot_Direction_Init(void);
void Shrink(void);
void Slide(void);
void Move_Control_Task(void *pvParameters);
//���귴��
void Coordinate_Inverse(Leg_Data_Typedef* Leg_Data);
//������

//�������
void Leg_Diretion_Control(Leg_Data_Typedef* Leg_Data,double foot_x,double foot_y);
//��˹켣����
void Foot_Control(Leg_Data_Typedef* Leg_Data,char trail,char direction);
//���㲽̬����

/***********************************************���ݶ�������չ********************************************************/
//�����ݽṹ��
extern  Leg_Data_Typedef Left_Front_Leg_Struct;
extern  Leg_Data_Typedef Left_Back_Leg_Struct;
extern  Leg_Data_Typedef Right_Back_Leg_Struct;
extern  Leg_Data_Typedef Right_Front_Leg_Struct;

//�������
extern char CAN_DATA_CMD_ON[8];






#endif 
