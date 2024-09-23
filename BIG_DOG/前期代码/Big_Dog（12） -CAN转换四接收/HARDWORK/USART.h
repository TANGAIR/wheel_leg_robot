#ifndef USART_H
#define USART_H

#include "sys.h" 
 #include "can.h" 
/*********************************************�ṹ�嶨����**********************************************************************/
//���͸�TX2���ݽṹ��
typedef struct
{
	char  Left_Front_Leg_X;  //��ǰ������ڹؽ�����ϵX������,��λCM���Թؽڶ������Ϊԭ�㣬������ǰ������ΪX��������
	char  Left_Front_Leg_Y;  //��ǰ������ڹؽ�����ϵY������,��λCM���Թؽڶ������Ϊԭ�㣬����ΪY��������
	char  Right_Front_Leg_X; //��ǰ������ڹؽ�����ϵX�����꣬��λCM���Թؽڶ������Ϊԭ�㣬������ǰ������ΪX��������
	char  Right_Front_Leg_Y; //��ǰ������ڹؽ�����ϵY�����꣬��λCM���Թؽڶ������Ϊԭ�㣬����ΪY��������
	short IMU_PITCH;         //�����Ǹ�����PITCH����λΪ�㣬G_Y������Ϊ��������Ϊ��
	short IMU_YAW;           //�����Ǻ����YAW����λΪ�㣬G_Z����������Ϊ��������Ϊ��
	char  Left_Front_Wheel_Speed;//��ǰ�ֵ���ٶȣ���λCM/S
	
}Send_TX2_Data_Typedef;

//����TX2���ݽṹ��
typedef struct
{
	char  Dog_Mode;      //������ģʽ,1����ʽ����ģʽ��2����¥��ģʽ��3����¥��ģʽ��0��ֹͣ
	char  Climb_Status;  //��¥״̬,1��ǰ�����к��Ȼ��У�2��ǰ���ȶ����У�3��ǰ�Ȼ��к�������
	char  Front_Leg_Distance; //��¥ʱ����ǰ�������¥���е��ˮƽ����,��λCM����ǰ������Ϊ�������е�Ϊ0
	char  Stair_Higth;        //¥�ݸ߶ȣ���λCM
	char  Stair_Width;        //¥�ݿ�ȣ���λCM
	char  Slide_Speed;        //ƽ�ػ����ٶȣ���λCM/S����¥ʱΪ0�����ڿ��ƻ���������ģʽʱ�Ļ����ٶ�
	char  Turn_Angle;         //��������ת��,��λ�㣬���ڿ��ƻ�����ת��
	char  Head_Pitch_Angle;   //ͷ���PITCH�Ƕ�,��λ�㣬���ڿ��ƻ�����̧ͷ��ͷ
	char  Head_Yaw_Angle;     //ͷ���YAW�Ƕ�,��λ�㣬���ڿ��ƻ���������תͷ
	
}Receive_TX2_Data_Typedef;








//����תCAN���ݽṹ��
typedef struct
{
	int Can_ID;
	uint8_t  Data_Lenth;
	uint8_t  Data[8];
	
}USART2CAN_Typedef;

















/*********************************************����������**********************************************************************/
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
/*********************************************���ݶ�������չ��**********************************************************************/
extern Receive_TX2_Data_Typedef Receive_TX2_Data_TypeStruct;
extern char   Usart3Rx_Info[10];

extern uint8_t USART2CAN_start[7];

#endif

