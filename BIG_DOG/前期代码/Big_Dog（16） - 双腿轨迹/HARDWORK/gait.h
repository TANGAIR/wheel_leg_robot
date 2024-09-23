#ifndef  GAIT_H
#define  GAIT_H	 
#include "stm32f4xx.h" 
#include "CAN.h"

/**********************************************************�ṹ�嶨��*******************************************************/
typedef struct
{
  char   Leg_Type;          //�ȵ�����
	
	char  Leg_Mode;           //�ȵ�ģʽ��1�켣��0����

	float  Leg_X;             //Ŀ��X������ڹؽ�����ϵX������,��λCM���Թؽڶ������Ϊԭ�㣬������ǰ������ΪX��������
	float  Leg_Y;             //Ŀ��Y������ڹؽ�����ϵY������,��λCM���Թؽڶ������Ϊԭ�㣬����ΪY��������
	
  
  float real_x;             //ͨ���ؽڽǶȽ��������ʵ�������
  float real_y; 						//ͨ���ؽڽǶȽ��������ʵ�������


	float  step_forward_time; //�������ڼ�ʱ
	float  quiesecence_time;  //�������ڼ�ʱ


	float  Y_Add;                    //Y������Ӧ����

  CAN1_Data_TypeDef *Jiont_Motor_U;//�ϵ������Žϵͣ���Ӧseigamar���Ƕȵ�λ�ǻ���
		
  CAN1_Data_TypeDef *Jiont_Motor_D;//�µ������Žϸߣ���Ӧgama���Ƕȵ�λ�ǻ���
	
	
	
	
	

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
#define pi (3.141592654)

//�켣�������
//�ȸ˳�(MM)
#define CLUB_LENGTH (230)

//����������ʼX���꣨mm������˳�ʼX����
#define FOOT_X   (-90)
//��˾���ֱ��Y���꣨mm������˳�ʼY����
#define FOOT_Y   (370)

//�������߲�����mm��
#define STEP_LENTH  (180)
//�������߲��ߣ�mm��
#define STEP_HIGHT  (100)



//������ʱ10ms,����˵����������
#define STEP_FORWARD_TIME (75)
//������ʱ10ms
#define QUIESECENCE_TIME  (40)
//�켣���ƫ���λmm,����ƫ������һֱ���֮ǰ����ֵ����ֹ�켣������
#define DISTANCE_IGNORE  (10)




//�ֵ�����л����ٶȶ���
#define WHEEL_STEP_SPEED  (50)






/************************************************��������***********************************************************/
//����
void Shrink(void);
//����Ӧ��ʼ��
void Leg_Deinit(Leg_Data_Typedef *Leg_Struct_Point);

//����
void Slide(void);

//���귴�����
void Coordinate_Inverse(Leg_Data_Typedef* Leg_Data);
//�������
void Leg_Diretion_Control(Leg_Data_Typedef* Leg_Data,float foot_x,float foot_y);
//��˹켣����
void Foot_Control(Leg_Data_Typedef* Leg_Data,char trail,char direction);
//����ѭ��
 void signal_leg(Leg_Data_Typedef *Leg_Data);
//���㲽̬����
void Gait_Control(char gait);




/***********************************************���ݶ�������չ********************************************************/
//�����ݽṹ��
extern  Leg_Data_Typedef Left_Front_Leg_Struct;
extern  Leg_Data_Typedef Left_Back_Leg_Struct;
extern  Leg_Data_Typedef Right_Back_Leg_Struct;
extern  Leg_Data_Typedef Right_Front_Leg_Struct;




#endif


