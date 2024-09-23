#ifndef _IMU_H
#define _IMU_H
#include "sys.h"

//mpu Reg -- Map
#define MPU6500_SELF_TEST_XG        (0x00)
#define MPU6500_SELF_TEST_YG        (0x01)
#define MPU6500_SELF_TEST_ZG        (0x02)
#define MPU6500_SELF_TEST_XA        (0x0D) 	//�Լ�Ĵ���X
#define MPU6500_SELF_TEST_YA        (0x0E)	//�Լ�Ĵ���Y
#define MPU6500_SELF_TEST_ZA        (0x0F) 	//�Լ�Ĵ���Z
#define MPU6500_XG_OFFSET_H         (0x13)
#define MPU6500_XG_OFFSET_L         (0x14)
#define MPU6500_YG_OFFSET_H         (0x15)
#define MPU6500_YG_OFFSET_L         (0x16)
#define MPU6500_ZG_OFFSET_H         (0x17)
#define MPU6500_ZG_OFFSET_L         (0x18)
#define MPU6500_SMPLRT_DIV          (0x19)
#define MPU6500_CONFIG              (0x1A)
#define MPU6500_GYRO_CONFIG         (0x1B)
#define MPU6500_ACCEL_CONFIG        (0x1C)
#define MPU6500_ACCEL_CONFIG_2      (0x1D)
#define MPU6500_LP_ACCEL_ODR        (0x1E)
#define MPU6500_MOT_THR             (0x1F)
#define MPU6500_FIFO_EN             (0x23)
#define MPU6500_I2C_MST_CTRL        (0x24)
#define MPU6500_I2C_SLV0_ADDR       (0x25)
#define MPU6500_I2C_SLV0_REG        (0x26)
#define MPU6500_I2C_SLV0_CTRL       (0x27)
#define MPU6500_I2C_SLV1_ADDR       (0x28)
#define MPU6500_I2C_SLV1_REG        (0x29)
#define MPU6500_I2C_SLV1_CTRL       (0x2A)
#define MPU6500_I2C_SLV2_ADDR       (0x2B)
#define MPU6500_I2C_SLV2_REG        (0x2C)
#define MPU6500_I2C_SLV2_CTRL       (0x2D)
#define MPU6500_I2C_SLV3_ADDR       (0x2E)
#define MPU6500_I2C_SLV3_REG        (0x2F)
#define MPU6500_I2C_SLV3_CTRL       (0x30)
#define MPU6500_I2C_SLV4_ADDR       (0x31)
#define MPU6500_I2C_SLV4_REG        (0x32)
#define MPU6500_I2C_SLV4_DO         (0x33)
#define MPU6500_I2C_SLV4_CTRL       (0x34)
#define MPU6500_I2C_SLV4_DI         (0x35)
#define MPU6500_I2C_MST_STATUS      (0x36)
#define MPU6500_INT_PIN_CFG         (0x37)
#define MPU6500_INT_ENABLE          (0x38)
#define MPU6500_INT_STATUS          (0x3A)
#define MPU6500_ACCEL_XOUT_H        (0x3B)//���ٶ�ֵ,X���8λ�Ĵ���
#define MPU6500_ACCEL_XOUT_L        (0x3C)//���ٶ�ֵ,X���8λ�Ĵ���
#define MPU6500_ACCEL_YOUT_H        (0x3D)//���ٶ�ֵ,Y���8λ�Ĵ���
#define MPU6500_ACCEL_YOUT_L        (0x3E)//���ٶ�ֵ,Y���8λ�Ĵ���
#define MPU6500_ACCEL_ZOUT_H        (0x3F)//���ٶ�ֵ,Z���8λ�Ĵ���
#define MPU6500_ACCEL_ZOUT_L        (0x40)//���ٶ�ֵ,Z���8λ�Ĵ���
#define MPU6500_TEMP_OUT_H          (0x41)
#define MPU6500_TEMP_OUT_L          (0x42)
#define MPU6500_GYRO_XOUT_H         (0x43)
#define MPU6500_GYRO_XOUT_L         (0x44)
#define MPU6500_GYRO_YOUT_H         (0x45)
#define MPU6500_GYRO_YOUT_L         (0x46)
#define MPU6500_GYRO_ZOUT_H         (0x47)
#define MPU6500_GYRO_ZOUT_L         (0x48)
#define MPU6500_EXT_SENS_DATA_00    (0x49)
#define MPU6500_EXT_SENS_DATA_01    (0x4A)
#define MPU6500_EXT_SENS_DATA_02    (0x4B)
#define MPU6500_EXT_SENS_DATA_03    (0x4C)
#define MPU6500_EXT_SENS_DATA_04    (0x4D)
#define MPU6500_EXT_SENS_DATA_05    (0x4E)
#define MPU6500_EXT_SENS_DATA_06    (0x4F)
#define MPU6500_EXT_SENS_DATA_07    (0x50)
#define MPU6500_EXT_SENS_DATA_08    (0x51)
#define MPU6500_EXT_SENS_DATA_09    (0x52)
#define MPU6500_EXT_SENS_DATA_10    (0x53)
#define MPU6500_EXT_SENS_DATA_11    (0x54)
#define MPU6500_EXT_SENS_DATA_12    (0x55)
#define MPU6500_EXT_SENS_DATA_13    (0x56)
#define MPU6500_EXT_SENS_DATA_14    (0x57)
#define MPU6500_EXT_SENS_DATA_15    (0x58)
#define MPU6500_EXT_SENS_DATA_16    (0x59)
#define MPU6500_EXT_SENS_DATA_17    (0x5A)
#define MPU6500_EXT_SENS_DATA_18    (0x5B)
#define MPU6500_EXT_SENS_DATA_19    (0x5C)
#define MPU6500_EXT_SENS_DATA_20    (0x5D)
#define MPU6500_EXT_SENS_DATA_21    (0x5E)
#define MPU6500_EXT_SENS_DATA_22    (0x5F)
#define MPU6500_EXT_SENS_DATA_23    (0x60)
#define MPU6500_I2C_SLV0_DO         (0x63)
#define MPU6500_I2C_SLV1_DO         (0x64)
#define MPU6500_I2C_SLV2_DO         (0x65)
#define MPU6500_I2C_SLV3_DO         (0x66)
#define MPU6500_I2C_MST_DELAY_CTRL  (0x67)
#define MPU6500_SIGNAL_PATH_RESET   (0x68)
#define MPU6500_MOT_DETECT_CTRL     (0x69)
#define MPU6500_USER_CTRL           (0x6A)
#define MPU6500_PWR_MGMT_1          (0x6B)//��Դ����1
#define MPU6500_PWR_MGMT_2          (0x6C)//��Դ����2
#define MPU6500_FIFO_COUNTH         (0x72)
#define MPU6500_FIFO_COUNTL         (0x73)
#define MPU6500_FIFO_R_W            (0x74)
#define MPU6500_WHO_AM_I            (0x75)	// mpu6500 id = 0x71
#define MPU6500_XA_OFFSET_H         (0x77)
#define MPU6500_XA_OFFSET_L         (0x78)
#define MPU6500_YA_OFFSET_H         (0x7A)
#define MPU6500_YA_OFFSET_L         (0x7B)
#define MPU6500_ZA_OFFSET_H         (0x7D)
#define MPU6500_ZA_OFFSET_L         (0x7E)	
#define MPU6050_ID					(0x68)
#define MPU6500_ID					(0x70)			// mpu6500 id = 0x70
#define MPU_IIC_ADDR				(0x68)

#define MPU6500_NSS_Low()  GPIO_ResetBits(GPIOF,GPIO_Pin_6)
#define MPU6500_NSS_High() GPIO_SetBits(GPIOF,GPIO_Pin_6)

typedef struct
{
   short Accel_X;  //�Ĵ���ԭ��X����ٶȱ�ʾֵ
   short Accel_Y;  //�Ĵ���ԭ��Y����ٶȱ�ʾֵ
   short Accel_Z;  //�Ĵ���ԭ��Z����ٶȱ�ʾֵ
   short Temp;     //�Ĵ���ԭ���¶ȱ�ʾֵ
   short Gyro_X;   //�Ĵ���ԭ��X�������Ǳ�ʾֵ
   short Gyro_Y;   //�Ĵ���ԭ��Y�������Ǳ�ʾֵ
   short Gyro_Z;   //�Ĵ���ԭ��Z�������Ǳ�ʾֵ
   short Mag_X;    //������ԭʼ����
   short Mag_Y;
   short Mag_Z;
}IMU_RAW_DATA_Typedef;//ԭ�����ݣ�û�о������㣩

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
	 float YAW;
	 float Pitch;
	 float Roll;
	
	
}IMU_REAL_DATA_Typedef;

extern  uint8_t MPU_id;
extern  IMU_RAW_DATA_Typedef   IMU_Raw_Data;//ԭ������
extern  IMU_REAL_DATA_Typedef  IMU_Real_Data;
extern  IMU_REAL_DATA_Typedef  IMU_Offset_Data;


uint8_t IMU_Init(void);
void IMU_Get_Data(void);
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) ;
void Heat_Init(void);
float inv_sqrt(float x) ;
extern float exInt, eyInt, ezInt;//��ǰ�ӼƲ�õ��������ٶ��������ϵķ��� ���õ�ǰ��̬��������������������ϵķ��������Ļ���
extern float q0 , q1 , q2 , q3 ;// qҪΪ��λ������q1Ϊ��z��ת����q2Ϊ��y��ת����q3Ϊ��x��ת��

void MPU_6500_Interrupt_InitConfig(void);
void IMU_Get_Data_Task(void *pvParameters);












#endif
