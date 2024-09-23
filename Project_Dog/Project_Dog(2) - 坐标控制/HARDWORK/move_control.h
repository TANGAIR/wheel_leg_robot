#ifndef MOVE_CONTROL_H
#define MOVE_CONTROL_H
#include "sys.h"


/**********************************************************�ṹ�嶨��*******************************************************/
typedef struct
{
  char   Leg_Type;          //�ȵ�����
	
	char  Leg_Mode;           //�ȵ�ģʽ��1�켣��0����

	float  Leg_X;             //Ŀ��X������ڹؽ�����ϵX������,��λCM���Թؽڶ������Ϊԭ�㣬������ǰ������ΪX��������
	float  Leg_Y;             //Ŀ��Y������ڹؽ�����ϵY������,��λCM���Թؽڶ������Ϊԭ�㣬����ΪY��������
	
  
  float real_x;             //ͨ���ؽڽǶȽ��������ʵ�������
  float real_y; 						//ͨ���ؽڽǶȽ��������ʵ�������

	float Steerint_IN ;
	float Steerint_OUT ;
	
	float  step_forward_time; //�������ڼ�ʱ
	float  quiesecence_time;  //�������ڼ�ʱ


	float  Y_Add;                    //Y������Ӧ����
	
	float FOOT_x;                   //�������߳�ʼX����
	float FOOT_y;                   //�������߳�ʼY����
		
}Leg_Data_Typedef;


/*******************************************************�궨��*******************************************************/
//�ȵ�����    
#define LEFT_BACK_LEG   11
#define LEFT_FRONT_LEG  22

#define LEFT_FRONT_BACKFORWARD  33


#define RIGHT_BACK_LEG  44
#define RIGHT_FRONT_LEG 55

#define RIGHT_FRONT_BACKFORWARD  66
//�켣
#define QUIESECENCE  (1) //��ֹ�˺�
#define STEP_FORWARD  (2) //����ǰ��
//����
#define DIRECTION_FORWARD  (2) //ǰ��
#define DIRECTION_BACK  (1)   //����
//������̬
#define SHRINK (0)
//Բ����
#define pi 3.14159

//�켣�������
//�ȸ˳�(MM)
#define CLUB_LENGTH 120     //90  75
#define GIG_LENGTH_1 90     //90  75
#define SMALL_LENGTH_2 75     //90  75
//����������ʼX���꣨mm������˳�ʼX����
#define FOOT_X (0)
//��˾���ֱ��Y���꣨mm������˳�ʼY����
#define FOOT_Y 140

//�������߲�����mm��
#define STEP_LENTH 70
//�������߲��ߣ�mm��
#define STEP_HIGHT 20

//������ʱ10ms
#define STEP_FORWARD_TIME 100
//������ʱ10ms
#define QUIESECENCE_TIME  50

//�ֵ���ٶȶ���
#define WHEEL_STOP  1000

//ת��X






/************************************************��������***********************************************************/


void Foot_Direction_Init(void);
void Shrink(void);
void Slide(void);
void Move_Control_Task(void *pvParameters);
//���귴��
void Coordinate_Inverse(Leg_Data_Typedef* Leg_Data);
//������
void Steering_PWM_Out(Leg_Data_Typedef* Leg_Data);
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








#endif 
