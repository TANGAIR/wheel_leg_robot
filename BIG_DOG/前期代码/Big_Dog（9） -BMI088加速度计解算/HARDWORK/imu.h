#ifndef _IMU_H
#define _IMU_H
#include "sys.h"


typedef struct
{
   float Accel_X;  //寄存器原生X轴加速度表示值
   float Accel_Y;  //寄存器原生Y轴加速度表示值
   float Accel_Z;  //寄存器原生Z轴加速度表示值
   float Temp;     //寄存器原生温度表示值
   float Gyro_X;   //寄存器原生X轴陀螺仪表示值
   float Gyro_Y;   //寄存器原生Y轴陀螺仪表示值
   float Gyro_Z;   //寄存器原生Z轴陀螺仪表示值
   float Mag_X;    //磁力计原始数据
   float Mag_Y;
   float Mag_Z;
}IMU_RAW_DATA_Typedef;//原生数据（没有经过解算）

typedef struct
{
   float Accel_X;  //寄存器原生X轴加速度表示值
   float Accel_Y;  //寄存器原生Y轴加速度表示值
   float Accel_Z;  //寄存器原生Z轴加速度表示值
   float Temp;     //寄存器原生温度表示值
   float Gyro_X;   //寄存器原生X轴陀螺仪表示值
   float Gyro_Y;   //寄存器原生Y轴陀螺仪表示值
   float Gyro_Z;   //寄存器原生Z轴陀螺仪表示值
   float Mag_X;    //磁力计原始数据
   float Mag_Y;
   float Mag_Z;
	 float YAW;
	 float Pitch;
	 float Roll;
	
	
}IMU_REAL_DATA_Typedef;

extern  uint8_t MPU_id;
extern  IMU_RAW_DATA_Typedef   IMU_Raw_Data;//原生数据
extern  IMU_REAL_DATA_Typedef  IMU_Real_Data;
extern  IMU_REAL_DATA_Typedef  IMU_Offset_Data;


void IMU_Get_Data(void);
void MPU6500_Gyro_Cali(void);



void MahonyAHRSupdateIMU(float q[4], float gx, float gy, float gz, float ax, float ay, float az);
float invSqrt(float x) ;








void IMU_Get_Data_Task(void *pvParameters);












#endif
