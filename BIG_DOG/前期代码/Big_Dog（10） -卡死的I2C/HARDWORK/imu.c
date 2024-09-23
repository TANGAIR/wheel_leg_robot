#include "main.h"
/***************************************数据定义区***************************************/
uint8_t MPU_id;
IMU_RAW_DATA_Typedef   IMU_Raw_Data;
IMU_REAL_DATA_Typedef  IMU_Real_Data;
IMU_REAL_DATA_Typedef  IMU_Offset_Data;
volatile uint32_t   last_update, now_update;
char IMU_OFFset_Flag=0;
fp32 quat[4]={1.0f,0.0f,0.0f,0.0f};
/***************************************函数处理区***************************************/
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
			IMU_Get_Data();//陀螺仪数据解算
			//向TX2发送数据
		//	USART3_Send_TX2();
		}
	}
}



/**
  * @brief  
  * @param  void
  * @retval void
  * @notes  向陀螺仪数据获取任务发送通知将其添加到就绪列表，同时进行一次任务切换
  */
void EXTI4_IRQHandler(void)
{
	BaseType_t pxHigherPriorityTaskWoken;
   if (EXTI_GetITStatus (EXTI_Line4 ) == SET)
   {
     
		 vTaskNotifyGiveFromISR(IMU_Get_Data_Task_Handler,&pxHigherPriorityTaskWoken);

   }
   EXTI_ClearITPendingBit(EXTI_Line4);
	 portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
}






/**
  * @brief  MPU6500角度获取
  * @param  void
  * @retval 俯仰角=atan2(AY, AZ)  横滚角= —atan2(AX, AZ )
  * @notes  Get 6 axis data from MPU6500       
  */
	
#define g   9.81F
#define PTZ_MPU_Ignore  0.02f
void IMU_Get_Data(void)
{
	fp32 gyro[3], accel[3], temp, mag[3];
	kalman p,a; 
	kalmanCreate(&p,1,0.005);
	kalmanCreate(&a,1,0.008);
	

	//读取数据
	BMI088_read(gyro, accel, &temp);
	//ist8310_read_mag(mag);

	IMU_Raw_Data.Accel_X = accel[0];   
	IMU_Raw_Data.Accel_Y = accel[1];
	IMU_Raw_Data.Accel_Z = accel[2];    //加速度
	
	IMU_Real_Data.Temp   = temp;	      //温度
	IMU_Raw_Data.Gyro_X = gyro[0];
	IMU_Raw_Data.Gyro_Y = gyro[1];
	IMU_Raw_Data.Gyro_Z = gyro[2];      //角速度
	
	IMU_Raw_Data.Mag_X =mag[0];
	IMU_Raw_Data.Mag_Y =mag[1];
	IMU_Raw_Data.Mag_Z =mag[2];         //磁力数值
	
	
	
	
	
	//加速度，g的单位
	IMU_Real_Data.Accel_X=IMU_Raw_Data.Accel_X;
	IMU_Real_Data.Accel_Y=IMU_Raw_Data.Accel_Y;
	IMU_Real_Data.Accel_Z=IMU_Raw_Data.Accel_Z;
	
	//角速度,
	IMU_Real_Data.Gyro_X = (IMU_Raw_Data.Gyro_X - IMU_Offset_Data.Gyro_X );
	if((IMU_Real_Data.Gyro_X<PTZ_MPU_Ignore)&&(IMU_Real_Data.Gyro_X>-PTZ_MPU_Ignore))IMU_Real_Data.Gyro_X=0;
	IMU_Real_Data.Gyro_Y = (IMU_Raw_Data.Gyro_Y - IMU_Offset_Data.Gyro_Y );
	if((IMU_Real_Data.Gyro_Y<PTZ_MPU_Ignore)&&(IMU_Real_Data.Gyro_Y>-PTZ_MPU_Ignore))IMU_Real_Data.Gyro_Y=0;
	IMU_Real_Data.Gyro_Z = (IMU_Raw_Data.Gyro_Z - IMU_Offset_Data.Gyro_Z );
	if((IMU_Real_Data.Gyro_Z<PTZ_MPU_Ignore)&&(IMU_Real_Data.Gyro_Z>-PTZ_MPU_Ignore))IMU_Real_Data.Gyro_Z=0;


  //磁力数值
   IMU_Real_Data.Mag_X=IMU_Raw_Data.Mag_X; 
	 IMU_Real_Data.Mag_Y=IMU_Raw_Data.Mag_Y; 
	 IMU_Real_Data.Mag_Z=IMU_Raw_Data.Mag_Z;        //磁力数值
	




    IMU_Real_Data.Gyro_X=KalmanFilter(&p,IMU_Real_Data.Gyro_X);
		IMU_Real_Data.Gyro_Y=KalmanFilter(&p,IMU_Real_Data.Gyro_Y);
		IMU_Real_Data.Gyro_Z=KalmanFilter(&p,IMU_Real_Data.Gyro_Z);
		IMU_Real_Data.Accel_X=KalmanFilter(&a,IMU_Real_Data.Accel_X);
		IMU_Real_Data.Accel_Y=KalmanFilter(&a,IMU_Real_Data.Accel_Y);
		IMU_Real_Data.Accel_Z=KalmanFilter(&a,IMU_Real_Data.Accel_Z);
		
		
		if(IMU_OFFset_Flag==1)
		 MahonyAHRSupdateIMU(quat,IMU_Real_Data.Gyro_X,IMU_Real_Data.Gyro_Y,IMU_Real_Data.Gyro_Z,IMU_Real_Data.Accel_X,IMU_Real_Data.Accel_Y,IMU_Real_Data.Accel_Z);

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
		
		delay_ms(2);
  }
		IMU_Offset_Data.Gyro_X =  Gyro_X_temp/300;
		IMU_Offset_Data.Gyro_Y =  Gyro_Y_temp/300;
		IMU_Offset_Data.Gyro_Z =  Gyro_Z_temp/300;
		IMU_Offset_Data.Accel_X = Accel_X_temp/300;
		IMU_Offset_Data.Accel_Y = Accel_Y_temp/300;
		IMU_Offset_Data.Accel_Z = Accel_Z_temp/300;
		IMU_Offset_Data.Temp    = Temp_temp/300;   //offset是一个补偿值来减小误差
	
	  IMU_OFFset_Flag=1;
}




/*********************************************四元数解算************************************************************/
//
#define sampleFreq	1000.0f			// sample frequency in Hz
#define twoKpDef	(2.0f * 0.5f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 0.0f)	// 2 * integral gain


volatile float twoKp = twoKpDef;											// 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;											// 2 * integral gain (Ki)
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki


void MahonyAHRSupdateIMU(float q[4], float gx, float gy, float gz, float ax, float ay, float az)
{
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;        

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q[1] * q[3] - q[0] * q[2];
		halfvy = q[0] * q[1] + q[2] * q[3];
		halfvz = q[0] * q[0] - 0.5f + q[3] * q[3];
	
		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q[0];
	qb = q[1];
	qc = q[2];
	q[0] += (-qb * gx - qc * gy - q[3] * gz);
	q[1] += (qa * gx + qc * gz - q[3] * gy);
	q[2] += (qa * gy - qb * gz + q[3] * gx);
	q[3] += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	q[0] *= recipNorm;
	q[1] *= recipNorm;
	q[2] *= recipNorm;
	q[3] *= recipNorm;
	
	
	
	  IMU_Real_Data.YAW  = 74.07f*atan2f(2.0f*(q[0]*q[3]+q[1]*q[2]), 2.0f*(q[0]*q[0]+q[1]*q[1])-1.0f);
    IMU_Real_Data.Pitch = 74.07f*asinf(-2.0f*(q[1]*q[3]-q[0]*q[2]));
    IMU_Real_Data.Roll  = 74.07f*atan2f(2.0f*(q[0]*q[1]+q[2]*q[3]),2.0f*(q[0]*q[0]+q[3]*q[3])-1.0f);
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) 
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}














