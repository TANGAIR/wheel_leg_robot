#include "main.h"
/***************************************���ݶ�����***************************************/
uint8_t MPU_id;
IMU_RAW_DATA_Typedef   IMU_Raw_Data;
IMU_REAL_DATA_Typedef  IMU_Real_Data;
IMU_REAL_DATA_Typedef  IMU_Offset_Data;
volatile uint32_t   last_update, now_update;
/***************************************����������***************************************/
/**
  * @brief  MPU6500����д��
  * @param  void
  * @retval void
  * @notes  Write a register to MPU6500     
  */
uint8_t MPU6500_Write_Reg(uint8_t const reg, uint8_t const data)  //�����ǵļĴ���8λ �Ĵ�����ַҲ��8λ
{
  MPU6500_NSS_Low();
  SPI5_ReadWriteByte(reg&0x7f);//spi����Э�飺ÿ�ζ�д������Ҫ16��ʱ�����ڻ���࣬����ĵ�һ���ֽ�ΪSPI��ַ���ڶ����ֽ�ΪSPI���ݡ����ֽڵĵ�һλΪ��дλ��0λд��1Ϊ��������7λΪ�Ĵ�����ַ
  SPI5_ReadWriteByte(data);
  MPU6500_NSS_High(); 
  return 0;
}

/**
  * @brief  MPU6500�ǶȻ�ȡ
  * @param  void
  * @retval void
  * @notes  Read a register from MPU6500      
  */
uint8_t MPU6500_Read_Reg(uint8_t  reg)
{
  uint8_t MPU_Rx;
  
  MPU6500_NSS_Low();
  SPI5_ReadWriteByte(reg|0x80);   
  MPU_Rx=SPI5_ReadWriteByte(0xff);
  MPU6500_NSS_High();
  
  return MPU_Rx;
}

/**
  * @brief  MPU6500�ǶȻ�ȡ
  * @param  void
  * @retval void
  * @notes  Read registers from MPU6500,address begin with regAddr      
  */
uint8_t MPU6500_Read_Regs(uint8_t  regAddr, uint8_t *pData, uint8_t len)
{
  int i; 
  int a;
		
  for (i=0;i<len;i++)
  {
    *pData = MPU6500_Read_Reg(regAddr+i);
    pData++;
    a=10;
    while(a--);

  }
  return 0;
}

/**
  * @brief  MPU6500�ǶȻ�ȡ
  * @param  void
  * @retval ������=atan2(AY, AZ)  �����= ��atan2(AX, AZ )
  * @notes  Get 6 axis data from MPU6500       
  */
	
#define g   9.81F
#define pi  3.1415926F 
#define PTZ_MPU_Ignore  0.02f
void IMU_Get_Data()
{
	kalman p,a; 
	kalmanCreate(&p,1,0.005);
	kalmanCreate(&a,1,0.008);
	uint8_t mpu_buff[14];

	MPU6500_Read_Regs(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);

	IMU_Raw_Data.Accel_X = mpu_buff[0]<<8 |mpu_buff[1]; 
	IMU_Raw_Data.Accel_Y = mpu_buff[2]<<8 |mpu_buff[3];
	IMU_Raw_Data.Accel_Z = mpu_buff[4]<<8 |mpu_buff[5];  //���ٶ�
	IMU_Raw_Data.Temp   = mpu_buff[6]<<8 |mpu_buff[7];	//�¶�
	IMU_Raw_Data.Gyro_X = mpu_buff[8] <<8 |mpu_buff[9];
	IMU_Raw_Data.Gyro_Y = mpu_buff[10]<<8 |mpu_buff[11] ;
	IMU_Raw_Data.Gyro_Z = mpu_buff[12]<<8 |mpu_buff[13] ; //���ٶ�
	
	
	IMU_Real_Data.Accel_X=IMU_Raw_Data.Accel_X*g/4096;
	IMU_Real_Data.Accel_Y=IMU_Raw_Data.Accel_Y*g/4096;
	IMU_Real_Data.Accel_Z=IMU_Raw_Data.Accel_Z*g/4096;
	
	IMU_Real_Data.Gyro_X = (IMU_Raw_Data.Gyro_X - IMU_Offset_Data.Gyro_X )/ 16.384f / 57.3f;
	if((IMU_Real_Data.Gyro_X<PTZ_MPU_Ignore)&&(IMU_Real_Data.Gyro_X>-PTZ_MPU_Ignore))IMU_Real_Data.Gyro_X=0;
	IMU_Real_Data.Gyro_Y = (IMU_Raw_Data.Gyro_Y - IMU_Offset_Data.Gyro_Y )/ 16.384f / 57.3f;
	if((IMU_Real_Data.Gyro_Y<PTZ_MPU_Ignore)&&(IMU_Real_Data.Gyro_Y>-PTZ_MPU_Ignore))IMU_Real_Data.Gyro_Y=0;
	IMU_Real_Data.Gyro_Z = (IMU_Raw_Data.Gyro_Z - IMU_Offset_Data.Gyro_Z )/ 16.384f / 57.3f;
	if((IMU_Real_Data.Gyro_Z<PTZ_MPU_Ignore)&&(IMU_Real_Data.Gyro_Z>-PTZ_MPU_Ignore))IMU_Real_Data.Gyro_Z=0;


    IMU_Real_Data.Gyro_X=KalmanFilter(&p,IMU_Real_Data.Gyro_X);
		IMU_Real_Data.Gyro_Y=KalmanFilter(&p,IMU_Real_Data.Gyro_Y);
		IMU_Real_Data.Gyro_Z=KalmanFilter(&p,IMU_Real_Data.Gyro_Z);
		IMU_Real_Data.Accel_X=KalmanFilter(&a,IMU_Real_Data.Accel_X);
		IMU_Real_Data.Accel_Y=KalmanFilter(&a,IMU_Real_Data.Accel_Y);
		IMU_Real_Data.Accel_Z=KalmanFilter(&a,IMU_Real_Data.Accel_Z);
		MadgwickAHRSupdateIMU(IMU_Real_Data.Gyro_X,IMU_Real_Data.Gyro_Y,IMU_Real_Data.Gyro_Z,IMU_Real_Data.Accel_X,IMU_Real_Data.Accel_Y,IMU_Real_Data.Accel_Z);

}

/**
  * @brief  MPU6500��ʼ���Ƕȴ���
  * @param  void
  * @retval void
  * @notes  void       
  */ 
void MPU6500_Gyro_Cali(void)					//temp����ʱ
{
  u16 i;
  float Gyro_X_temp=0,Gyro_Y_temp=0,Gyro_Z_temp=0,Accel_X_temp,Accel_Y_temp,Accel_Z_temp,Temp_temp;
  
	IMU_Offset_Data.Accel_X = 0;
	IMU_Offset_Data.Accel_Y = 0;
	IMU_Offset_Data.Accel_Z = 0;
	IMU_Offset_Data.Temp    = 0;
	IMU_Offset_Data.Gyro_X  = 0;
	IMU_Offset_Data.Gyro_Y  = 0;
	IMU_Offset_Data.Gyro_Z  = 0;
	IMU_Offset_Data.Mag_X   = 0;
	IMU_Offset_Data.Mag_Y   = 0;
	IMU_Offset_Data.Mag_Z   = 0;
	
  for (i=0;i<300;i++)
  {
    IMU_Get_Data();
		Gyro_X_temp += IMU_Raw_Data.Gyro_X;
		Gyro_Y_temp += IMU_Raw_Data.Gyro_Y;
		Gyro_Z_temp += IMU_Raw_Data.Gyro_Z;
		Accel_X_temp+=IMU_Raw_Data.Accel_X;
		Accel_Y_temp+=IMU_Raw_Data.Accel_Y;
		Accel_Z_temp+=IMU_Raw_Data.Accel_Z;
		Temp_temp   +=   IMU_Raw_Data.Temp;
  }
		IMU_Offset_Data.Gyro_X =  Gyro_X_temp/300;
		IMU_Offset_Data.Gyro_Y =  Gyro_Y_temp/300;
		IMU_Offset_Data.Gyro_Z =  Gyro_Z_temp/300;
		IMU_Offset_Data.Accel_X = Accel_X_temp/300;
		IMU_Offset_Data.Accel_Y = Accel_Y_temp/300;
		IMU_Offset_Data.Accel_Z = Accel_Z_temp/300;
		IMU_Offset_Data.Temp    = Temp_temp/300;   //offset��һ������ֵ����С���
}

/**
  * @brief  IMU��ʼ��
  * @param  void
  * @retval void
  * @notes  Initialize the MPU6500  500Hz������       
  */
uint8_t IMU_Init(void)
{
	/*
	 * Some important registers in MPU6500
	 * Name  Sample rate divider SMPLRT_DIV  | Bit7  Bit6  Bit5  Bit4  Bit3  Bit2  Bit1  Bit0
	 *                                       |     SMPLRT_DIV[7:0]
	 * �üĴ���ָ������������ʵķ�Ƶ����������MPU-60X0�Ĳ����ʡ� �������Ĵ����������FIFO�����DMP�������˶����Ķ��ǻ��ڸ�
	 * �����ʡ������ʵļ��㹫ʽ:
	 *          ������ = �����ǵ������ / (1 + SMPLRT_DIV)
	 * �����ֵ�ͨ�˲��� (LPF) û��ʹ�ܵ�ʱ�������ǵ����ƽ·����8KHZ����֮����1KHZ��
	 *
	 * Name  Configuration  CONFIG           | Bit7 |    Bit6     | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0
	 *                                       |   -  |  FIFO_MODE  |  EXT_SYNC_SET[2:0] |  DLPF_CFG[2:0]
	 * 1) FIFO_MODE һλFIFO�Ĵ���������FIFO�洢ģʽ
	 * 1) EXT_SYNC_SET 3λ�޷���ֵ������֡ͬ�����ŵĲ���
	 * 2) DLPF_CFG 3λ�޷���ֵ���������ֵ�ͨ�˲���
	 *
	 * �üĴ���ΪMPU6500 FIFOģʽ�������Ǻͼ��ٶȼ������ⲿ֡ͬ����FSYNC�����Ų��������ֵ�ͨ�˲�����DLPF����ͨ������
	 * EXT_SYNC_SET�����Զ����ӵ�FSYNC���ŵ�һ���ⲿ�źŽ��в����� FSYNC�����ϵ��źű仯�ᱻ���棬�������ܲ��񵽺̵ܶ�Ƶ����
	 * �š���������������������λ����ǰ��FSYNC�ź�״̬����������ı�����ֵ���ɼ��������ݻ��滻�����ݼĴ������ϴν��յ���
	 * ��Ч����
	 * EXT_SYNC_SET | FSYNC Bit Location
	 *      0       | Function disabled
	 *      1       | TEMP_OUT_L[0]
	 *      2       | GYRO_XOUT_L[0]
	 *      3       | GYRO_YOUT_L[0]
	 *      4       | GYRO_ZOUT_L[0]
	 *      5       | ACCEL_XOUT_L[0]
	 *      6       | ACCEL_YOUT_L[0]
	 *      7       | ACCEL_ZOUT_L[0]
	 * ���ֵ�ͨ�˲�������DLPF_CFG�����á���DLPFΪ����ʱ��FCHOICE[1:0]������11��FCHOICE_B[1:0]������00��
	 *
	 * Name Gyroscope Configuration GYRO_CONFIG   | Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0
	 *                                            | XG_ST| YG_ST| ZG_ST|  FS_SEL[1:0]|  -   | FCHOICE_B[1:0]
	 * 1) XG_ST ���ô�λ��X�������ǽ������Ҳ��ԡ�
	 * 2��YG_ST ���ô�λ��Y�������ǽ������Ҳ��ԡ�
	 * 3��ZG_ST ���ô�λ��Z�������ǽ������Ҳ��ԡ�
	 * 4��FS_SEL 2λ�޷���ֵ��ѡ�������ǵ����̡�
	 *
	 * GYRO_FS_SEL[1:0] | 00 = ��250��/s
	 *                  | 01 = ��500��/s
	 *                  | 10 = ��1000��/s
	 *                  | 11 = ��2000��/s
	 *
	 * Name Accelerometer Configuration ACCEL_CONFIG | Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0
	 *                                               | XA_ST| YA_ST| ZA_ST|  FS_SEL[1:0]|   -  |   -  |   -
	 * 1��XA_ST    ����Ϊ1ʱ��X����ٶȸ�Ӧ�������Լ졣
	 * 2��YA_ST    ����Ϊ1ʱ��Y����ٶȸ�Ӧ�������Լ졣
	 * 3��ZA_ST    ����Ϊ1ʱ��Z����ٶȸ�Ӧ�������Լ졣
	 * 4��AFS_SEL  2λ�޷���ֵ��ѡ����ٶȼƵ����̡�
	 *
	 * ACCEL_FS_SEL[1:0]| 00 = ��2g
	 *                  | 01 = ��4g
	 *                  | 10 = ��8g
	 *                  | 11 = ��16g
	 *
	 * Accelerometer Measurements
	 * ACCEL_XOUT_H
	 * ACCEL_XOUT_L
	 * ACCEL_YOUT_H
	 * ACCEL_YOUT_L
	 * ACCEL_ZOUT_H
	 * ACCEL_ZOUT_L
	 *
	 * 16 bits 2's complement
	 *
	 * Temperature Measurement
	 * TEMP_OUT_H
	 * TEMP_OUT_L
	 *
	 * 16 bits 2's complement
	 *
	 * Gyroscope Measurements
	 * GYRO_XOUT_H,
	 * GYRO_XOUT_L,
	 * GYRO_YOUT_H,
	 * GYRO_YOUT_L,
	 * GYRO_ZOUT_H,
	 * GYRO_ZOUT_L
	 *
	 * 16 bits 2's complement
	 *
	 * Name   Power Management 1     PWR_MGMT_1   | Bit7 | Bit6 | Bit5 | Bit4 |   Bit3   | Bit2 | Bit1 | Bit0
	 *                                            | RESET| SLEEP| CYCLE|   -  | TEMP_DIS |   CLKSEL[2:0]
	 *
	 * 1��DEVICE_RESET  ��1�����еļĴ�����λ�����DEVICE_RESET�Զ���0.
	 * 2��SLEEP         ��1�����˯��ģʽ
	 * 3��CYCLE         ��CYCLE������Ϊ1����SLEEPû�����ã�MPU-60X0����ѭ��ģʽ��Ϊ�˴��ٶȴ������л�ò���ֵ����˯��ģʽ����
	 * �����ݲɼ�ģʽ֮���л���ÿ�λ��һ���������ݡ���LP_WAKE_CTRL��108���Ĵ����У��������û��Ѻ�Ĳ����ʺͱ����ѵ�Ƶ�ʡ�
	 * 4��TEMP_DIS      ��1��ر��¶ȴ�����
	 * 5��CLKSEL        ָ���豸��ʱ��Դ
	 *
	 * ʱ��Դѡ��
	 *    CLKSEL    | Clock Source
	 *      0       | Internal 20M OC
	 *      1       | Auto selects the best available clock source �C PLL if ready, else use the Internal oscillator
	 *      2       | Auto selects the best available clock source �C PLL if ready, else use the Internal oscillator
	 *      3       | Auto selects the best available clock source �C PLL if ready, else use the Internal oscillator
	 *      4       | Auto selects the best available clock source �C PLL if ready, else use the Internal oscillator
	 *      5       | Auto selects the best available clock source �C PLL if ready, else use the Internal oscillator
	 *      6       | Internal 20M OC
	 *      7       | Stops the clock and keeps timing generator in reset
	 *
	 * Name   Power Management 1     PWR_MGMT_2   |   Bit7  |  Bit6  |  Bit5 |  Bit4 |  Bit3 |  Bit2 |  Bit1 |  Bit0
	 *                                            | LP_WAKE_CTRL[1:0]| DIS_XA| DIS_YA| DIS_ZA| DIS_XG| DIS_YG| DIS_ZG
	 * LP_WAKE_CTRL   �͹���ģʽ��ģ�黽��Ƶ��
	 */
	uint8_t index = 0;
	uint8_t MPU6500_Init_Data[10][2] = 
	{
		{MPU6500_PWR_MGMT_1,    0x80},      // Reset Device  ��λ
		{MPU6500_SIGNAL_PATH_RESET, 0x07}, // Reset accelerometer, gyroscope and thermometer ��λ
		{MPU6500_PWR_MGMT_1,    0x03},      // Select clock Source - Gyro-Z ѡ��ο�ʱ��Դ
		{MPU6500_PWR_MGMT_2,    0x00},      // Enable Acc & Gyro ʹ�ܼ��ٶȺͽ��ٶ�
		{MPU6500_SMPLRT_DIV, 	0x00},      // Sample rate = gyroscope output rate / (0 + 1) = 1000Hz ���ò���Ƶ��
		{MPU6500_CONFIG,        0x02},      // FSYNC disabled, Low pass filter enabled, Gyroscope Bandwidth 92Hz, delay , FSYNC��ʹ�ܣ�FSYNC�ⲿ���Ų���������ֵ�����洫�����Ĵ��������λ��Ч������ͨ�˲���ʹ��
				// Temperature sensor Bandwidth 98Hz, delay 2.8ms                                   ����DLPF_CFG=2 ���������Ƶ��Ϊ1khz
		{MPU6500_GYRO_CONFIG,   0x18},      // Set gyroscope no self-test, calibration range ��2000dps, set FCHOICE_B 2b'11
		{MPU6500_ACCEL_CONFIG,  0x10},      // Set accelerometer no self-test, calibration range ��8G
		{MPU6500_ACCEL_CONFIG_2,0x02},      // Enable Accel LPF, Accel Bandwidth 92Hz, delay 7.8ms, sample rate 1Hz
		{MPU6500_USER_CTRL,     0x30},      // Enable AUX
	};  

	SPI_GPIO_InitConfig();
	Heat_Init();//���ö�ʱ��3 
	while(MPU6500_ID != MPU6500_Read_Reg(MPU6500_WHO_AM_I))
    {
		LED_RED_ON;	   
    };  //read id of device,check if MPU6500 or not
		LED_RED_ON;

	for(index = 0; index < 10; index++)//����6500
	{
		MPU6500_Write_Reg(MPU6500_Init_Data[index][0], MPU6500_Init_Data[index][1]);//��ʼ��������
		delay_ms(1);
	}
	delay_ms(10);
	MPU6500_Gyro_Cali();//�����ʼ���ǶȲ���ֵ
	MPU_6500_Interrupt_InitConfig();
	return 0;
}

/***********************************************��Ԫ��******************************************************/
/***********************************************��Ԫ��******************************************************/
/***********************************************��Ԫ��******************************************************/
/***********************************************��Ԫ��******************************************************/
/***********************************************��Ԫ��******************************************************/
/***********************************************��Ԫ��******************************************************/

#define Kp   2.0f                               // �����KpKi�����ڵ������ٶȼ����������ǵ��ٶ�
#define Ki  0.0275f
//#define halfT	0.005835f		    		           	//�������ڵ�һ�룬���������Ԫ��΢�ַ���ʱ���������   ���������Գ�����
float exInt=0, eyInt=0, ezInt=0;                //��ǰ�ӼƲ�õ��������ٶ��������ϵķ��� ���õ�ǰ��̬��������������������ϵķ��������Ļ���
float q0 = 1.0f, q1 =0.0f, q2 = 0.0f, q3 = 0.0f;// qҪΪ��λ������q1Ϊ��z��ת����q2Ϊ��y��ת����q3Ϊ��x��ת��
/**
* @brief Madgwick����̬�����㷨(����)
* @param gx gy gz������������� ��λ����/s ax ay az�Ӽ�������� ��λ�޹�
* @detail ������ɺ�����ݱ��洢��ȫ�ֱ�����
* @return NULL
*/
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) 
{
	float q0temp,q1temp,q2temp,q3temp;					//��Ԫ���ݴ���������΢�ַ���ʱҪ��
	float norm; 									            	//ʸ����ģ����Ԫ���ķ���
	float vx, vy, vz;									          //��ǰ��̬��������������������ϵķ���
	float ex, ey, ez;									          //��ǰ�ӼƲ�õ��������ٶ��������ϵķ��� ���õ�ǰ��̬��������������������ϵķ��������
	float halfT;
  float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
  float q1q1 = q1*q1;
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;
  float q2q3 = q2*q3;
  float q3q3 = q3*q3; 

	now_update  =GetTick; //��Ҫ��systickʱ�Ӽ����������ʱ�䣬������tim������
	halfT       = ((float)(now_update - last_update) / 1850.0f);//��������1850 ����ֵӦ����2000

	last_update = now_update;

	
	//ֻ���ڼ��ٶȼƲ�����Ч������²��ܼ��㷴��(������ٶȼƵ�������ʱ����NaN)
	if(ax*ay*az==0)  return;//�Ӽƴ�����������״̬ʱ��������̬���㣬��Ϊ�������ĸ���������
	
	norm = inv_sqrt(ax*ax + ay*ay + az*az);//��λ�����ٶȼƣ�    
	ax = ax * norm;                        // �������������Ҳ����Ҫ�޸�KP��������Ϊ�����һ����
	ay = ay * norm;
	az = az * norm;  
	
	//�õ�ǰ��̬������������������ϵķ�����
  //�ο�����nϵת������������bϵ������Ԫ����ʾ�ķ������Ҿ�������м��ǣ�����һ�����ᵽ��
	vx = 2.0f*(q1q3 - q0q2);        
	vy = 2.0f*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3 ;
	
	//�����õ������������������������������Ա�ʾ��һ���
	ex = (ay*vz - az*vy) ;                                                                  
	ey = (az*vx - ax*vz) ;
	ez = (ax*vy - ay*vx) ;												
	
	
	exInt = exInt + ex * Ki * halfT;                                           //�������л���
	eyInt = eyInt + ey * Ki * halfT;
	ezInt = ezInt + ez * Ki * halfT;
	
	// adjusted gyroscope measurements
	gx = gx + Kp*ex + exInt;  //�����PI�󲹳��������ǣ����������Ư��
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt; 	//�����gz����û�й۲��߽��н��������Ư�ƣ����ֳ����ľ��ǻ����������Լ�
	
	//���������̬�ĸ��£�Ҳ������Ԫ��΢�ַ��̵����
	q0temp=q0;//�ݴ浱ǰֵ���ڼ���
	q1temp=q1;//���ϴ�������㷨���û��ע��������⣬�ڴ˸���
	q2temp=q2;
	q3temp=q3;
	
	//����һ�ױϿ��ⷨ�����֪ʶ�ɲμ���������������Ե���ϵͳ��P212
	q0 = q0temp + (-q1temp*gx - q2temp*gy -q3temp*gz)*halfT;
	q1 = q1temp + (q0temp*gx + q2temp*gz -q3temp*gy)*halfT;
	q2 = q2temp + (q0temp*gy - q1temp*gz +q3temp*gx)*halfT;
	q3 = q3temp + (q0temp*gz + q1temp*gy -q2temp*gx)*halfT;
	
	//��λ����Ԫ���ڿռ���תʱ�������죬������ת�Ƕȣ����������Դ�����������任
	norm = inv_sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 * norm;
	q1 = q1 * norm;
	q2 = q2 * norm;
	q3 = q3 * norm;
	  
	IMU_Real_Data.Roll  = atan2(2 * q2 * q3 + 2 * q0 * q1,1-2 * q1 * q1 - 2 * q2* q2 )* 57.3f;       //ת��Ϊŷ����X
	IMU_Real_Data.Pitch =asin(-2 * q1 * q3 + 2 * q0* q2)*57.3f;//Y
	IMU_Real_Data.YAW =atan2(2*(q1*q2 + q0*q3),1 - 2*q2*q2 - 2*q3*q3) * 57.3f;

}


float inv_sqrt(float x) 
{
	float halfx = 0.5f * x;
	float y     = x;
	long  i     = *(long*)&y;
	
	i = 0x5f3759df - (i >> 1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	
	return y;
}





void Heat_Init(void)
{
	GPIO_InitTypeDef         GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef        TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);  	     //TIM12ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 	     //ʹ��PORTAʱ��	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource5,GPIO_AF_TIM3);
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;                     //GPIOH0
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                  //���ù���
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	           //�ٶ�100MHz
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                //���츴�����
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;                  //����
  GPIO_Init(GPIOB,&GPIO_InitStructure);                         //��ʼ��GPIOH0
	
	TIM_TimeBaseStructure.TIM_Prescaler=168-1;                     //��ʱ����Ƶ 1M
  TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;     //���ϼ���ģʽ
  TIM_TimeBaseStructure.TIM_Period=1500;                       //�Զ���װ��ֵ 100Hz  10ms
  TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
  TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);                //��ʼ����ʱ��5

  //��ʼ��TIM12 Channel1 PWM1ģʽ	 
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;             //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //�������:TIM����Ƚϼ��Ե�
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);                      //����Tָ���Ĳ�����ʼ������TIM12 OC1
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);             //ʹ��TIM12��CCR1�ϵ�Ԥװ�ؼĴ���
   
  TIM_ARRPreloadConfig(TIM3,ENABLE);                            //ARPEʹ�� 
  TIM_Cmd(TIM3, ENABLE);  
}


/**
  * @brief  6500��������ж϶�Ӧ�����벶���ʼ��
  * @param  void
  * @retval void
  * @notes  EXTI_Line8-->PB8���������ڲ�ÿ�ν��������֮������һ����ƽ�仯����PB8����
            ��GPIO8��ȡ���ƽ������һ���ⲿ�жϣ��ж����ȼ�Ϊ7��Ƶ�ʴ��Ϊ1000Hz
  */
void MPU_6500_Interrupt_InitConfig(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	EXTI_InitTypeDef EXIT_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE );
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE );

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init (GPIOB,&GPIO_InitStruct);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB ,GPIO_PinSource8);

	EXIT_InitStruct.EXTI_Line = EXTI_Line8;
	EXIT_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXIT_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
	EXIT_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXIT_InitStruct);

	NVIC_InitStruct.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 8;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct); 
}

/**
  * @brief  ��ʱ��4�ⲿ�жϴ�����()���벶��  1KHz
  * @param  void
  * @retval void
  * @notes  �����������ݻ�ȡ������֪ͨ������ӵ������б�ͬʱ����һ�������л�
  */
void EXTI9_5_IRQHandler(void)
{
	BaseType_t pxHigherPriorityTaskWoken;
   if (EXTI_GetITStatus (EXTI_Line8 ) == SET)
   {
     vTaskNotifyGiveFromISR(IMU_Get_Data_Task_Handler,&pxHigherPriorityTaskWoken);

   }
   EXTI_ClearITPendingBit(EXTI_Line8);
	 portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
}

/**
  * @brief  �����ǽǶȻ�ȡ����
  * @param  void
  * @retval void
  * @notes  ÿ1���뱻PB8�ⲿ�жϻ���һ�Σ���ȡ���������ݣ�����洢��IMU_Real_Data��Ա������
            ͬʱ��USART3_Send_TX2�����п���DMA����������NUC������Ϣ��
  */
void IMU_Get_Data_Task(void *pvParameters)
{
	uint32_t err;
	while(1)
	{
		//�ȴ��ж�����֪ͨ
		err=ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
		if(err==1)
		{
			IMU_Get_Data();
			//USART3_Send_TX2();
		}
	}
}



