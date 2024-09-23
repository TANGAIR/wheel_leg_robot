#ifndef _IMU_H
#define _IMU_H
#include "sys.h"

/**********************************************************�ṹ�嶨��*******************************************************/
typedef struct
{
   float Accel_X;  //�Ĵ���ԭ��X����ٶȱ�ʾֵ
   float Accel_Y;  //�Ĵ���ԭ��Y����ٶȱ�ʾֵ
   float Accel_Z;  //�Ĵ���ԭ��Z����ٶȱ�ʾֵ
   float Temp;     //�Ĵ���ԭ���¶ȱ�ʾֵ
   float Gyro_X;   //�Ĵ���ԭ��X�������Ǳ�ʾֵ
   float Gyro_Y;   //�Ĵ���ԭ��Y�������Ǳ�ʾֵ
   float Gyro_Z;   //�Ĵ���ԭ��Z�������Ǳ�ʾֵ
   float Mag_X;    //������ԭʼ����
   float Mag_Y;
   float Mag_Z;
}IMU_RAW_DATA_Typedef;//ԭ�����ݣ�û�о������㣩

typedef struct
{
   float Accel_X; 
   float Accel_Y;  
   float Accel_Z;  
   float Temp;     
   float Gyro_X;   
   float Gyro_Y;  
   float Gyro_Z;   
   float Mag_X;    
   float Mag_Y;
   float Mag_Z;
	 float YAW;         //��λ�Ƕ�
	 float Pitch;       //
	 float Roll;        //
	
	
}IMU_REAL_DATA_Typedef;
/*******************************************************�궨��*******************************************************/
/*��ƽ����ؽṹ����*/
//��е��ǰ�󳤶ȣ�mm
#define FRONT_BACK_LENGTH  (447.0f)
//��е�����ҿ��
#define LEFT_RIGHT_WIDTH  (341.2f)
//�Ƕ�ת��Ϊ���Ȳ���
#define PI_180  (0.01745329252f)








/************************************************��������***********************************************************/
void IMU_Get_Data(void);
void MPU6500_Gyro_Cali(void);

void MahonyAHRSupdateIMU(float q[4], float gx, float gy, float gz, float ax, float ay, float az);
float invSqrt(float x) ;




/***********************************************���ݶ�������չ********************************************************/






extern  uint8_t MPU_id;
extern  IMU_RAW_DATA_Typedef   IMU_Raw_Data;//ԭ������
extern  IMU_REAL_DATA_Typedef  IMU_Real_Data;
extern  IMU_REAL_DATA_Typedef  IMU_Offset_Data;











void IMU_Get_Data_Task(void *pvParameters);












#endif
