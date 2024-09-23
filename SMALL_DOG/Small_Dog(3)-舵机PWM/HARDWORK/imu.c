#include "main.h"
/***************************************数据定义区***************************************/
uint8_t MPU_id;
IMU_RAW_DATA_Typedef   IMU_Raw_Data;
IMU_REAL_DATA_Typedef  IMU_Real_Data;
IMU_REAL_DATA_Typedef  IMU_Offset_Data;
volatile uint32_t   last_update, now_update;
/***************************************函数处理区***************************************/
/**
  * @brief  MPU6500配置写入
  * @param  void
  * @retval void
  * @notes  Write a register to MPU6500     
  */
uint8_t MPU6500_Write_Reg(uint8_t const reg, uint8_t const data)  //陀螺仪的寄存是8位 寄存器地址也是8位
{
  MPU6500_NSS_Low();
  SPI5_ReadWriteByte(reg&0x7f);//spi传输协议：每次读写操作需要16个时钟周期或更多，传输的第一个字节为SPI地址，第二个字节为SPI数据。首字节的第一位为读写位，0位写，1为读，后面7位为寄存器地址
  SPI5_ReadWriteByte(data);
  MPU6500_NSS_High(); 
  return 0;
}

/**
  * @brief  MPU6500角度获取
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
  * @brief  MPU6500角度获取
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
  * @brief  MPU6500角度获取
  * @param  void
  * @retval 俯仰角=atan2(AY, AZ)  横滚角= —atan2(AX, AZ )
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
	IMU_Raw_Data.Accel_Z = mpu_buff[4]<<8 |mpu_buff[5];  //加速度
	IMU_Raw_Data.Temp   = mpu_buff[6]<<8 |mpu_buff[7];	//温度
	IMU_Raw_Data.Gyro_X = mpu_buff[8] <<8 |mpu_buff[9];
	IMU_Raw_Data.Gyro_Y = mpu_buff[10]<<8 |mpu_buff[11] ;
	IMU_Raw_Data.Gyro_Z = mpu_buff[12]<<8 |mpu_buff[13] ; //角速度
	
	
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
  * @brief  MPU6500初始化角度处理
  * @param  void
  * @retval void
  * @notes  void       
  */ 
void MPU6500_Gyro_Cali(void)					//temp：临时
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
		IMU_Offset_Data.Temp    = Temp_temp/300;   //offset是一个补偿值来减小误差
}

/**
  * @brief  IMU初始化
  * @param  void
  * @retval void
  * @notes  Initialize the MPU6500  500Hz采样率       
  */
uint8_t IMU_Init(void)
{
	/*
	 * Some important registers in MPU6500
	 * Name  Sample rate divider SMPLRT_DIV  | Bit7  Bit6  Bit5  Bit4  Bit3  Bit2  Bit1  Bit0
	 *                                       |     SMPLRT_DIV[7:0]
	 * 该寄存器指定陀螺仪输出率的分频，用来产生MPU-60X0的采样率。 传感器寄存器的输出、FIFO输出、DMP采样和运动检测的都是基于该
	 * 采样率。采样率的计算公式:
	 *          采样率 = 陀螺仪的输出率 / (1 + SMPLRT_DIV)
	 * 当数字低通滤波器 (LPF) 没有使能的时候，陀螺仪的输出平路等于8KHZ，反之等于1KHZ。
	 *
	 * Name  Configuration  CONFIG           | Bit7 |    Bit6     | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0
	 *                                       |   -  |  FIFO_MODE  |  EXT_SYNC_SET[2:0] |  DLPF_CFG[2:0]
	 * 1) FIFO_MODE 一位FIFO寄存器，配置FIFO存储模式
	 * 1) EXT_SYNC_SET 3位无符号值，配置帧同步引脚的采样
	 * 2) DLPF_CFG 3位无符号值，配置数字低通滤波器
	 *
	 * 该寄存器为MPU6500 FIFO模式，陀螺仪和加速度计配置外部帧同步（FSYNC）引脚采样和数字低通滤波器（DLPF）。通过配置
	 * EXT_SYNC_SET，可以对连接到FSYNC引脚的一个外部信号进行采样。 FSYNC引脚上的信号变化会被锁存，这样就能捕获到很短的频闪信
	 * 号。采样结束后，锁存器将复位到当前的FSYNC信号状态。根据下面的表格定义的值，采集到的数据会替换掉数据寄存器中上次接收到的
	 * 有效数据
	 * EXT_SYNC_SET | FSYNC Bit Location
	 *      0       | Function disabled
	 *      1       | TEMP_OUT_L[0]
	 *      2       | GYRO_XOUT_L[0]
	 *      3       | GYRO_YOUT_L[0]
	 *      4       | GYRO_ZOUT_L[0]
	 *      5       | ACCEL_XOUT_L[0]
	 *      6       | ACCEL_YOUT_L[0]
	 *      7       | ACCEL_ZOUT_L[0]
	 * 数字低通滤波器是由DLPF_CFG来配置。当DLPF为非零时，FCHOICE[1:0]必须置11，FCHOICE_B[1:0]必须置00。
	 *
	 * Name Gyroscope Configuration GYRO_CONFIG   | Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0
	 *                                            | XG_ST| YG_ST| ZG_ST|  FS_SEL[1:0]|  -   | FCHOICE_B[1:0]
	 * 1) XG_ST 设置此位，X轴陀螺仪进行自我测试。
	 * 2）YG_ST 设置此位，Y轴陀螺仪进行自我测试。
	 * 3）ZG_ST 设置此位，Z轴陀螺仪进行自我测试。
	 * 4）FS_SEL 2位无符号值。选择陀螺仪的量程。
	 *
	 * GYRO_FS_SEL[1:0] | 00 = ±250°/s
	 *                  | 01 = ±500°/s
	 *                  | 10 = ±1000°/s
	 *                  | 11 = ±2000°/s
	 *
	 * Name Accelerometer Configuration ACCEL_CONFIG | Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0
	 *                                               | XA_ST| YA_ST| ZA_ST|  FS_SEL[1:0]|   -  |   -  |   -
	 * 1）XA_ST    设置为1时，X轴加速度感应器进行自检。
	 * 2）YA_ST    设置为1时，Y轴加速度感应器进行自检。
	 * 3）ZA_ST    设置为1时，Z轴加速度感应器进行自检。
	 * 4）AFS_SEL  2位无符号值。选择加速度计的量程。
	 *
	 * ACCEL_FS_SEL[1:0]| 00 = ±2g
	 *                  | 01 = ±4g
	 *                  | 10 = ±8g
	 *                  | 11 = ±16g
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
	 * 1）DEVICE_RESET  置1后所有的寄存器复位，随后DEVICE_RESET自动置0.
	 * 2）SLEEP         置1后进入睡眠模式
	 * 3）CYCLE         当CYCLE被设置为1，且SLEEP没有设置，MPU-60X0进入循环模式，为了从速度传感器中获得采样值，在睡眠模式和正
	 * 常数据采集模式之间切换，每次获得一个采样数据。在LP_WAKE_CTRL（108）寄存器中，可以设置唤醒后的采样率和被唤醒的频率。
	 * 4）TEMP_DIS      置1后关闭温度传感器
	 * 5）CLKSEL        指定设备的时钟源
	 *
	 * 时钟源选择
	 *    CLKSEL    | Clock Source
	 *      0       | Internal 20M OC
	 *      1       | Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
	 *      2       | Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
	 *      3       | Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
	 *      4       | Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
	 *      5       | Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
	 *      6       | Internal 20M OC
	 *      7       | Stops the clock and keeps timing generator in reset
	 *
	 * Name   Power Management 1     PWR_MGMT_2   |   Bit7  |  Bit6  |  Bit5 |  Bit4 |  Bit3 |  Bit2 |  Bit1 |  Bit0
	 *                                            | LP_WAKE_CTRL[1:0]| DIS_XA| DIS_YA| DIS_ZA| DIS_XG| DIS_YG| DIS_ZG
	 * LP_WAKE_CTRL   低功耗模式下模块唤醒频率
	 */
	uint8_t index = 0;
	uint8_t MPU6500_Init_Data[10][2] = 
	{
		{MPU6500_PWR_MGMT_1,    0x80},      // Reset Device  复位
		{MPU6500_SIGNAL_PATH_RESET, 0x07}, // Reset accelerometer, gyroscope and thermometer 复位
		{MPU6500_PWR_MGMT_1,    0x03},      // Select clock Source - Gyro-Z 选择参考时钟源
		{MPU6500_PWR_MGMT_2,    0x00},      // Enable Acc & Gyro 使能加速度和角速度
		{MPU6500_SMPLRT_DIV, 	0x00},      // Sample rate = gyroscope output rate / (0 + 1) = 1000Hz 设置采样频率
		{MPU6500_CONFIG,        0x02},      // FSYNC disabled, Low pass filter enabled, Gyroscope Bandwidth 92Hz, delay , FSYNC不使能（FSYNC外部引脚采样，采样值将代替传感器寄存器的最低位有效），低通滤波器使能
				// Temperature sensor Bandwidth 98Hz, delay 2.8ms                                   配置DLPF_CFG=2 陀螺仪输出频率为1khz
		{MPU6500_GYRO_CONFIG,   0x18},      // Set gyroscope no self-test, calibration range ±2000dps, set FCHOICE_B 2b'11
		{MPU6500_ACCEL_CONFIG,  0x10},      // Set accelerometer no self-test, calibration range ±8G
		{MPU6500_ACCEL_CONFIG_2,0x02},      // Enable Accel LPF, Accel Bandwidth 92Hz, delay 7.8ms, sample rate 1Hz
		{MPU6500_USER_CTRL,     0x30},      // Enable AUX
	};  

	SPI_GPIO_InitConfig();
	Heat_Init();//设置定时器3 
	while(MPU6500_ID != MPU6500_Read_Reg(MPU6500_WHO_AM_I))
    {
		LED_RED_ON;	   
    };  //read id of device,check if MPU6500 or not
		LED_RED_ON;

	for(index = 0; index < 10; index++)//配置6500
	{
		MPU6500_Write_Reg(MPU6500_Init_Data[index][0], MPU6500_Init_Data[index][1]);//初始化陀螺仪
		delay_ms(1);
	}
	delay_ms(10);
	MPU6500_Gyro_Cali();//计算初始化角度补偿值
	MPU_6500_Interrupt_InitConfig();
	return 0;
}

/***********************************************四元数******************************************************/
/***********************************************四元数******************************************************/
/***********************************************四元数******************************************************/
/***********************************************四元数******************************************************/
/***********************************************四元数******************************************************/
/***********************************************四元数******************************************************/

#define Kp   2.0f                               // 这里的KpKi是用于调整加速度计修正陀螺仪的速度
#define Ki  0.0275f
//#define halfT	0.005835f		    		           	//采样周期的一半，用于求解四元数微分方程时计算角增量   我这里是试出来的
float exInt=0, eyInt=0, ezInt=0;                //当前加计测得的重力加速度在三轴上的分量 与用当前姿态计算得来的重力在三轴上的分量的误差的积分
float q0 = 1.0f, q1 =0.0f, q2 = 0.0f, q3 = 0.0f;// q要为单位向量，q1为绕z轴转动，q2为绕y轴转动，q3为绕x轴转动
/**
* @brief Madgwick的姿态解算算法(六轴)
* @param gx gy gz陀螺仪三轴分量 单位弧度/s ax ay az加计三轴分量 单位无关
* @detail 解算完成后的数据被存储于全局变量中
* @return NULL
*/
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) 
{
	float q0temp,q1temp,q2temp,q3temp;					//四元数暂存变量，求解微分方程时要用
	float norm; 									            	//矢量的模或四元数的范数
	float vx, vy, vz;									          //当前姿态计算得来的重力在三轴上的分量
	float ex, ey, ez;									          //当前加计测得的重力加速度在三轴上的分量 与用当前姿态计算得来的重力在三轴上的分量的误差
	float halfT;
  float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
  float q1q1 = q1*q1;
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;
  float q2q3 = q2*q3;
  float q3q3 = q3*q3; 

	now_update  =GetTick; //需要用systick时钟计算程序运行时间，具体在tim函数里
	halfT       = ((float)(now_update - last_update) / 1850.0f);//调出来的1850 理论值应该是2000

	last_update = now_update;

	
	//只有在加速度计测量有效的情况下才能计算反馈(避免加速度计的正常化时发生NaN)
	if(ax*ay*az==0)  return;//加计处于自由落体状态时不进行姿态解算，因为会产生分母无穷大的情况
	
	norm = inv_sqrt(ax*ax + ay*ay + az*az);//单位化加速度计，    
	ax = ax * norm;                        // 这样变更了量程也不需要修改KP参数，因为这里归一化了
	ay = ay * norm;
	az = az * norm;  
	
	//用当前姿态计算出重力在三个轴上的分量，
  //参考坐标n系转化到载体坐标b系的用四元数表示的方向余弦矩阵第三列即是（博文一中有提到）
	vx = 2.0f*(q1q3 - q0q2);        
	vy = 2.0f*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3 ;
	
	//计算测得的重力与计算得重力间的误差，向量外积可以表示这一误差
	ex = (ay*vz - az*vy) ;                                                                  
	ey = (az*vx - ax*vz) ;
	ez = (ax*vy - ay*vx) ;												
	
	
	exInt = exInt + ex * Ki * halfT;                                           //对误差进行积分
	eyInt = eyInt + ey * Ki * halfT;
	ezInt = ezInt + ez * Ki * halfT;
	
	// adjusted gyroscope measurements
	gx = gx + Kp*ex + exInt;  //将误差PI后补偿到陀螺仪，即补偿零点漂移
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt; 	//这里的gz由于没有观测者进行矫正会产生漂移，表现出来的就是积分自增或自减
	
	//下面进行姿态的更新，也就是四元数微分方程的求解
	q0temp=q0;//暂存当前值用于计算
	q1temp=q1;//网上传的这份算法大多没有注意这个问题，在此更正
	q2temp=q2;
	q3temp=q3;
	
	//采用一阶毕卡解法，相关知识可参见《惯性器件与惯性导航系统》P212
	q0 = q0temp + (-q1temp*gx - q2temp*gy -q3temp*gz)*halfT;
	q1 = q1temp + (q0temp*gx + q2temp*gz -q3temp*gy)*halfT;
	q2 = q2temp + (q0temp*gy - q1temp*gz +q3temp*gx)*halfT;
	q3 = q3temp + (q0temp*gz + q1temp*gy -q2temp*gx)*halfT;
	
	//单位化四元数在空间旋转时不会拉伸，仅有旋转角度，这类似线性代数里的正交变换
	norm = inv_sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 * norm;
	q1 = q1 * norm;
	q2 = q2 * norm;
	q3 = q3 * norm;
	  
	IMU_Real_Data.Roll  = atan2(2 * q2 * q3 + 2 * q0 * q1,1-2 * q1 * q1 - 2 * q2* q2 )* 57.3f;       //转换为欧拉角X
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
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);  	     //TIM12时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 	     //使能PORTA时钟	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource5,GPIO_AF_TIM3);
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;                     //GPIOH0
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                  //复用功能
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	           //速度100MHz
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                //推挽复用输出
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;                  //上拉
  GPIO_Init(GPIOB,&GPIO_InitStructure);                         //初始化GPIOH0
	
	TIM_TimeBaseStructure.TIM_Prescaler=168-1;                     //定时器分频 1M
  TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;     //向上计数模式
  TIM_TimeBaseStructure.TIM_Period=1500;                       //自动重装载值 100Hz  10ms
  TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
  TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);                //初始化定时器5

  //初始化TIM12 Channel1 PWM1模式	 
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;             //选择定时器模式:TIM脉冲宽度调制模式1
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //输出极性:TIM输出比较极性低
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);                      //根据T指定的参数初始化外设TIM12 OC1
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);             //使能TIM12在CCR1上的预装载寄存器
   
  TIM_ARRPreloadConfig(TIM3,ENABLE);                            //ARPE使能 
  TIM_Cmd(TIM3, ENABLE);  
}


/**
  * @brief  6500数据溢出中断对应的输入捕获初始化
  * @param  void
  * @retval void
  * @notes  EXTI_Line8-->PB8，陀螺仪内部每次解算出数据之后会产生一个电平变化，从PB8输入
            从GPIO8读取其电平，产生一个外部中断，中断优先级为7，频率大概为1000Hz
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
  * @brief  定时器4外部中断处理函数()输入捕获  1KHz
  * @param  void
  * @retval void
  * @notes  向陀螺仪数据获取任务发送通知将其添加到就绪列表，同时进行一次任务切换
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
  * @brief  陀螺仪角度获取任务
  * @param  void
  * @retval void
  * @notes  每1毫秒被PB8外部中断唤醒一次，获取陀螺仪数据，将其存储到IMU_Real_Data成员变量中
            同时在USART3_Send_TX2函数中开启DMA数据流，给NUC发送消息，
  */
void IMU_Get_Data_Task(void *pvParameters)
{
	uint32_t err;
	while(1)
	{
		//等待中断任务通知
		err=ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
		if(err==1)
		{
			IMU_Get_Data();
			//USART3_Send_TX2();
		}
	}
}



