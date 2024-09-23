 #include "main.h"  



PID_TypeDef Jiont_Motor_Angle_PID_LF_U,
            Jiont_Motor_Angle_PID_LF_D,
						Jiont_Motor_Angle_PID_LB_U,
						Jiont_Motor_Angle_PID_LB_D,
            Jiont_Motor_Angle_PID_RB_U,
            Jiont_Motor_Angle_PID_RB_D,
						Jiont_Motor_Angle_PID_RF_U,
						Jiont_Motor_Angle_PID_RF_D;
						
PID_TypeDef Jiont_Motor_Speed_PID_LF_U,
            Jiont_Motor_Speed_PID_LF_D,
						Jiont_Motor_Speed_PID_LB_U,
						Jiont_Motor_Speed_PID_LB_D,
            Jiont_Motor_Speed_PID_RB_U,
            Jiont_Motor_Speed_PID_RB_D,
						Jiont_Motor_Speed_PID_RF_U,
						Jiont_Motor_Speed_PID_RF_D;						
						

//关节角速PID设置
void Jiont_Motor_Angle_PID_Init(void)
{
	
	Jiont_Motor_Angle_PID_LF_U.P=30;
	Jiont_Motor_Angle_PID_LF_U.I=0;
	Jiont_Motor_Angle_PID_LF_U.D=0;
	
	Jiont_Motor_Angle_PID_LF_D.P=6;                     //10
	Jiont_Motor_Angle_PID_LF_D.I=0;                   //0.01
	Jiont_Motor_Angle_PID_LF_D.D=0;  
		
	Jiont_Motor_Angle_PID_LB_U.P=6;
	Jiont_Motor_Angle_PID_LB_U.I=0;
	Jiont_Motor_Angle_PID_LB_U.D=0;
	
	Jiont_Motor_Angle_PID_LB_D.P=6;                     //10
	Jiont_Motor_Angle_PID_LB_D.I=0;                   //0.01
	Jiont_Motor_Angle_PID_LB_D.D=0;  

	
	Jiont_Motor_Angle_PID_RB_U.P=6;
	Jiont_Motor_Angle_PID_RB_U.I=0;
	Jiont_Motor_Angle_PID_RB_U.D=0;
	
	Jiont_Motor_Angle_PID_RB_D.P=6;                     //10
	Jiont_Motor_Angle_PID_RB_D.I=0;                   //0.01
	Jiont_Motor_Angle_PID_RB_D.D=0;  
		
	Jiont_Motor_Angle_PID_RF_U.P=6;
	Jiont_Motor_Angle_PID_RF_U.I=0;
	Jiont_Motor_Angle_PID_RF_U.D=0;
	
	Jiont_Motor_Angle_PID_RF_D.P=6;                     //10
	Jiont_Motor_Angle_PID_RF_D.I=0;                   //0.01
	Jiont_Motor_Angle_PID_RF_D.D=0;  
	
	
	Jiont_Motor_Angle_PID_LF_U.CurrentError=0;
	Jiont_Motor_Angle_PID_LF_U.LastError=0;
	Jiont_Motor_Angle_PID_LF_U.ErrorIgnored=0;
	Jiont_Motor_Angle_PID_LF_U.Pout=0;
	Jiont_Motor_Angle_PID_LF_U.Iout=0;
	Jiont_Motor_Angle_PID_LF_U.Dout=0;
	Jiont_Motor_Angle_PID_LF_U.PIDout=0;
	Jiont_Motor_Angle_PID_LF_U.PIDOutCompensate=0;
	Jiont_Motor_Angle_PID_LF_U.PMax=300;
	Jiont_Motor_Angle_PID_LF_U.IMax=5000;
	Jiont_Motor_Angle_PID_LF_U.DMax=3000;
	Jiont_Motor_Angle_PID_LF_U.PIDOutMax=16;
	Jiont_Motor_Angle_PID_LF_U.PIDOutLast=0;
	Jiont_Motor_Angle_PID_LF_U.Target_Speed_Last=0;
	Jiont_Motor_Angle_PID_LF_U.Speed_Ratio=1;
	Jiont_Motor_Angle_PID_LF_U.Acceleration=0;
	
	
	
	Jiont_Motor_Angle_PID_LF_D.CurrentError=0;
	Jiont_Motor_Angle_PID_LF_D.LastError=0;
	Jiont_Motor_Angle_PID_LF_D.ErrorIgnored=0;
	Jiont_Motor_Angle_PID_LF_D.Pout=0;
	Jiont_Motor_Angle_PID_LF_D.Iout=0;
	Jiont_Motor_Angle_PID_LF_D.Dout=0;
	Jiont_Motor_Angle_PID_LF_D.PIDout=0;
	Jiont_Motor_Angle_PID_LF_D.PIDOutCompensate=0;
	Jiont_Motor_Angle_PID_LF_D.PMax=300;
	Jiont_Motor_Angle_PID_LF_D.IMax=5000;
	Jiont_Motor_Angle_PID_LF_D.DMax=3000;
	Jiont_Motor_Angle_PID_LF_D.PIDOutMax=6;
	Jiont_Motor_Angle_PID_LF_D.PIDOutLast=0;
	Jiont_Motor_Angle_PID_LF_D.Target_Speed_Last=0;
	Jiont_Motor_Angle_PID_LF_D.Speed_Ratio=1;
	Jiont_Motor_Angle_PID_LF_D.Acceleration=0;
	


  Jiont_Motor_Angle_PID_LB_U.CurrentError=0;
	Jiont_Motor_Angle_PID_LB_U.LastError=0;
	Jiont_Motor_Angle_PID_LB_U.ErrorIgnored=0;
	Jiont_Motor_Angle_PID_LB_U.Pout=0;
	Jiont_Motor_Angle_PID_LB_U.Iout=0;
	Jiont_Motor_Angle_PID_LB_U.Dout=0;
	Jiont_Motor_Angle_PID_LB_U.PIDout=0;
	Jiont_Motor_Angle_PID_LB_U.PIDOutCompensate=0;
	Jiont_Motor_Angle_PID_LB_U.PMax=300;
	Jiont_Motor_Angle_PID_LB_U.IMax=5000;
	Jiont_Motor_Angle_PID_LB_U.DMax=3000;
	Jiont_Motor_Angle_PID_LB_U.PIDOutMax=6;
	Jiont_Motor_Angle_PID_LB_U.PIDOutLast=0;
	Jiont_Motor_Angle_PID_LB_U.Target_Speed_Last=0;
	Jiont_Motor_Angle_PID_LB_U.Speed_Ratio=1;
	Jiont_Motor_Angle_PID_LB_U.Acceleration=0;
	
	
	Jiont_Motor_Angle_PID_LB_D.CurrentError=0;
	Jiont_Motor_Angle_PID_LB_D.LastError=0;
	Jiont_Motor_Angle_PID_LB_D.ErrorIgnored=0;
	Jiont_Motor_Angle_PID_LB_D.Pout=0;
	Jiont_Motor_Angle_PID_LB_D.Iout=0;
	Jiont_Motor_Angle_PID_LB_D.Dout=0;
	Jiont_Motor_Angle_PID_LB_D.PIDout=0;
	Jiont_Motor_Angle_PID_LB_D.PIDOutCompensate=0;
	Jiont_Motor_Angle_PID_LB_D.PMax=300;
	Jiont_Motor_Angle_PID_LB_D.IMax=5000;
	Jiont_Motor_Angle_PID_LB_D.DMax=3000;
	Jiont_Motor_Angle_PID_LB_D.PIDOutMax=6;
	Jiont_Motor_Angle_PID_LB_D.PIDOutLast=0;
	Jiont_Motor_Angle_PID_LB_D.Target_Speed_Last=0;
	Jiont_Motor_Angle_PID_LB_D.Speed_Ratio=1;
	Jiont_Motor_Angle_PID_LB_D.Acceleration=0;
	

	
	Jiont_Motor_Angle_PID_RB_U.CurrentError=0;
	Jiont_Motor_Angle_PID_RB_U.LastError=0;
	Jiont_Motor_Angle_PID_RB_U.ErrorIgnored=0;
	Jiont_Motor_Angle_PID_RB_U.Pout=0;
	Jiont_Motor_Angle_PID_RB_U.Iout=0;
	Jiont_Motor_Angle_PID_RB_U.Dout=0;
	Jiont_Motor_Angle_PID_RB_U.PIDout=0;
	Jiont_Motor_Angle_PID_RB_U.PIDOutCompensate=0;
	Jiont_Motor_Angle_PID_RB_U.PMax=300;
	Jiont_Motor_Angle_PID_RB_U.IMax=5000;
	Jiont_Motor_Angle_PID_RB_U.DMax=3000;
	Jiont_Motor_Angle_PID_RB_U.PIDOutMax=6;
	Jiont_Motor_Angle_PID_RB_U.PIDOutLast=0;
	Jiont_Motor_Angle_PID_RB_U.Target_Speed_Last=0;
	Jiont_Motor_Angle_PID_RB_U.Speed_Ratio=1;
	Jiont_Motor_Angle_PID_RB_U.Acceleration=0;
	
	
	Jiont_Motor_Angle_PID_RB_D.CurrentError=0;
	Jiont_Motor_Angle_PID_RB_D.LastError=0;
	Jiont_Motor_Angle_PID_RB_D.ErrorIgnored=0;
	Jiont_Motor_Angle_PID_RB_D.Pout=0;
	Jiont_Motor_Angle_PID_RB_D.Iout=0;
	Jiont_Motor_Angle_PID_RB_D.Dout=0;
	Jiont_Motor_Angle_PID_RB_D.PIDout=0;
	Jiont_Motor_Angle_PID_RB_D.PIDOutCompensate=0;
	Jiont_Motor_Angle_PID_RB_D.PMax=300;
	Jiont_Motor_Angle_PID_RB_D.IMax=5000;
	Jiont_Motor_Angle_PID_RB_D.DMax=3000;
	Jiont_Motor_Angle_PID_RB_D.PIDOutMax=6;
	Jiont_Motor_Angle_PID_RB_D.PIDOutLast=0;
	Jiont_Motor_Angle_PID_RB_D.Target_Speed_Last=0;
	Jiont_Motor_Angle_PID_RB_D.Speed_Ratio=1;
	Jiont_Motor_Angle_PID_RB_D.Acceleration=0;
	


  Jiont_Motor_Angle_PID_RF_U.CurrentError=0;
	Jiont_Motor_Angle_PID_RF_U.LastError=0;
	Jiont_Motor_Angle_PID_RF_U.ErrorIgnored=0;
	Jiont_Motor_Angle_PID_RF_U.Pout=0;
	Jiont_Motor_Angle_PID_RF_U.Iout=0;
	Jiont_Motor_Angle_PID_RF_U.Dout=0;
	Jiont_Motor_Angle_PID_RF_U.PIDout=0;
	Jiont_Motor_Angle_PID_RF_U.PIDOutCompensate=0;
	Jiont_Motor_Angle_PID_RF_U.PMax=300;
	Jiont_Motor_Angle_PID_RF_U.IMax=5000;
	Jiont_Motor_Angle_PID_RF_U.DMax=3000;
	Jiont_Motor_Angle_PID_RF_U.PIDOutMax=6;
	Jiont_Motor_Angle_PID_RF_U.PIDOutLast=0;
	Jiont_Motor_Angle_PID_RF_U.Target_Speed_Last=0;
	Jiont_Motor_Angle_PID_RF_U.Speed_Ratio=1;
	Jiont_Motor_Angle_PID_RF_U.Acceleration=0;
	
	
	
	Jiont_Motor_Angle_PID_RF_D.CurrentError=0;
	Jiont_Motor_Angle_PID_RF_D.LastError=0;
	Jiont_Motor_Angle_PID_RF_D.ErrorIgnored=0;
	Jiont_Motor_Angle_PID_RF_D.Pout=0;
	Jiont_Motor_Angle_PID_RF_D.Iout=0;
	Jiont_Motor_Angle_PID_RF_D.Dout=0;
	Jiont_Motor_Angle_PID_RF_D.PIDout=0;
	Jiont_Motor_Angle_PID_RF_D.PIDOutCompensate=0;
	Jiont_Motor_Angle_PID_RF_D.PMax=300;
	Jiont_Motor_Angle_PID_RF_D.IMax=5000;
	Jiont_Motor_Angle_PID_RF_D.DMax=3000;
	Jiont_Motor_Angle_PID_RF_D.PIDOutMax=6;
	Jiont_Motor_Angle_PID_RF_D.PIDOutLast=0;
	Jiont_Motor_Angle_PID_RF_D.Target_Speed_Last=0;
	Jiont_Motor_Angle_PID_RF_D.Speed_Ratio=1;
	Jiont_Motor_Angle_PID_RF_D.Acceleration=0;



  //PID,标定
  Joint_LF_U.Motor_Angle_PID = &Jiont_Motor_Angle_PID_LF_U;
  Joint_LF_D.Motor_Angle_PID = &Jiont_Motor_Angle_PID_LF_D;
	Joint_LB_U.Motor_Angle_PID = &Jiont_Motor_Angle_PID_LB_U;
	Joint_LB_D.Motor_Angle_PID = &Jiont_Motor_Angle_PID_LB_D;
	Joint_RB_U.Motor_Angle_PID = &Jiont_Motor_Angle_PID_RB_U;
	Joint_RB_D.Motor_Angle_PID = &Jiont_Motor_Angle_PID_RB_D;
	Joint_RF_U.Motor_Angle_PID = &Jiont_Motor_Angle_PID_RF_U;
	Joint_RF_D.Motor_Angle_PID = &Jiont_Motor_Angle_PID_RF_D;

}



//关节PID用于差值反馈控制关节电机FOC的KP
void Jiont_Motor_Speed_PID_Init(void)
{
	
	Jiont_Motor_Speed_PID_LF_U.P=1;
	Jiont_Motor_Speed_PID_LF_U.I=0;
	Jiont_Motor_Speed_PID_LF_U.D=0;
	
	Jiont_Motor_Speed_PID_LF_D.P=1;                     //10
	Jiont_Motor_Speed_PID_LF_D.I=0;                   //0.01
	Jiont_Motor_Speed_PID_LF_D.D=0;  
		
	Jiont_Motor_Speed_PID_LB_U.P=1;
	Jiont_Motor_Speed_PID_LB_U.I=0;
	Jiont_Motor_Speed_PID_LB_U.D=0;
	
	Jiont_Motor_Speed_PID_LB_D.P=6;                     //10
	Jiont_Motor_Speed_PID_LB_D.I=0;                   //0.01
	Jiont_Motor_Speed_PID_LB_D.D=0;  

	
	Jiont_Motor_Speed_PID_RB_U.P=1;
	Jiont_Motor_Speed_PID_RB_U.I=0;
	Jiont_Motor_Speed_PID_RB_U.D=0;
	
	Jiont_Motor_Speed_PID_RB_D.P=1;                     //10
	Jiont_Motor_Speed_PID_RB_D.I=0;                   //0.01
	Jiont_Motor_Speed_PID_RB_D.D=0;  
		
	Jiont_Motor_Speed_PID_RF_U.P=1;
	Jiont_Motor_Speed_PID_RF_U.I=0;
	Jiont_Motor_Speed_PID_RF_U.D=0;
	
	Jiont_Motor_Speed_PID_RF_D.P=1;                     //10
	Jiont_Motor_Speed_PID_RF_D.I=0;                   //0.01
	Jiont_Motor_Speed_PID_RF_D.D=0;  
	
	
	Jiont_Motor_Speed_PID_LF_U.CurrentError=0;
	Jiont_Motor_Speed_PID_LF_U.LastError=0;
	Jiont_Motor_Speed_PID_LF_U.ErrorIgnored=0;
	Jiont_Motor_Speed_PID_LF_U.Pout=0;
	Jiont_Motor_Speed_PID_LF_U.Iout=0;
	Jiont_Motor_Speed_PID_LF_U.Dout=0;
	Jiont_Motor_Speed_PID_LF_U.PIDout=0;
	Jiont_Motor_Speed_PID_LF_U.PIDOutCompensate=0;
	Jiont_Motor_Speed_PID_LF_U.PMax=300;
	Jiont_Motor_Speed_PID_LF_U.IMax=5000;
	Jiont_Motor_Speed_PID_LF_U.DMax=3000;
	Jiont_Motor_Speed_PID_LF_U.PIDOutMax=11;
	Jiont_Motor_Speed_PID_LF_U.PIDOutLast=0;
	Jiont_Motor_Speed_PID_LF_U.Target_Speed_Last=0;
	Jiont_Motor_Speed_PID_LF_U.Speed_Ratio=1;
	Jiont_Motor_Speed_PID_LF_U.Acceleration=0;
	
	
	
	Jiont_Motor_Speed_PID_LF_D.CurrentError=0;
	Jiont_Motor_Speed_PID_LF_D.LastError=0;
	Jiont_Motor_Speed_PID_LF_D.ErrorIgnored=0;
	Jiont_Motor_Speed_PID_LF_D.Pout=0;
	Jiont_Motor_Speed_PID_LF_D.Iout=0;
	Jiont_Motor_Speed_PID_LF_D.Dout=0;
	Jiont_Motor_Speed_PID_LF_D.PIDout=0;
	Jiont_Motor_Speed_PID_LF_D.PIDOutCompensate=0;
	Jiont_Motor_Speed_PID_LF_D.PMax=300;
	Jiont_Motor_Speed_PID_LF_D.IMax=5000;
	Jiont_Motor_Speed_PID_LF_D.DMax=3000;
	Jiont_Motor_Speed_PID_LF_D.PIDOutMax=16;
	Jiont_Motor_Speed_PID_LF_D.PIDOutLast=0;
	Jiont_Motor_Speed_PID_LF_D.Target_Speed_Last=0;
	Jiont_Motor_Speed_PID_LF_D.Speed_Ratio=1;
	Jiont_Motor_Speed_PID_LF_D.Acceleration=0;
	

  Jiont_Motor_Speed_PID_LB_U.CurrentError=0;
	Jiont_Motor_Speed_PID_LB_U.LastError=0;
	Jiont_Motor_Speed_PID_LB_U.ErrorIgnored=0;
	Jiont_Motor_Speed_PID_LB_U.Pout=0;
	Jiont_Motor_Speed_PID_LB_U.Iout=0;
	Jiont_Motor_Speed_PID_LB_U.Dout=0;
	Jiont_Motor_Speed_PID_LB_U.PIDout=0;
	Jiont_Motor_Speed_PID_LB_U.PIDOutCompensate=0;
	Jiont_Motor_Speed_PID_LB_U.PMax=300;
	Jiont_Motor_Speed_PID_LB_U.IMax=5000;
	Jiont_Motor_Speed_PID_LB_U.DMax=3000;
	Jiont_Motor_Speed_PID_LB_U.PIDOutMax=16;
	Jiont_Motor_Speed_PID_LB_U.PIDOutLast=0;
	Jiont_Motor_Speed_PID_LB_U.Target_Speed_Last=0;
	Jiont_Motor_Speed_PID_LB_U.Speed_Ratio=1;
	Jiont_Motor_Speed_PID_LB_U.Acceleration=0;
	
	
	Jiont_Motor_Speed_PID_LB_D.CurrentError=0;
	Jiont_Motor_Speed_PID_LB_D.LastError=0;
	Jiont_Motor_Speed_PID_LB_D.ErrorIgnored=0;
	Jiont_Motor_Speed_PID_LB_D.Pout=0;
	Jiont_Motor_Speed_PID_LB_D.Iout=0;
	Jiont_Motor_Speed_PID_LB_D.Dout=0;
	Jiont_Motor_Speed_PID_LB_D.PIDout=0;
	Jiont_Motor_Speed_PID_LB_D.PIDOutCompensate=0;
	Jiont_Motor_Speed_PID_LB_D.PMax=300;
	Jiont_Motor_Speed_PID_LB_D.IMax=5000;
	Jiont_Motor_Speed_PID_LB_D.DMax=3000;
	Jiont_Motor_Speed_PID_LB_D.PIDOutMax=16;
	Jiont_Motor_Speed_PID_LB_D.PIDOutLast=0;
	Jiont_Motor_Speed_PID_LB_D.Target_Speed_Last=0;
	Jiont_Motor_Speed_PID_LB_D.Speed_Ratio=1;
	Jiont_Motor_Speed_PID_LB_D.Acceleration=0;
	

	
	Jiont_Motor_Speed_PID_RB_U.CurrentError=0;
	Jiont_Motor_Speed_PID_RB_U.LastError=0;
	Jiont_Motor_Speed_PID_RB_U.ErrorIgnored=0;
	Jiont_Motor_Speed_PID_RB_U.Pout=0;
	Jiont_Motor_Speed_PID_RB_U.Iout=0;
	Jiont_Motor_Speed_PID_RB_U.Dout=0;
	Jiont_Motor_Speed_PID_RB_U.PIDout=0;
	Jiont_Motor_Speed_PID_RB_U.PIDOutCompensate=0;
	Jiont_Motor_Speed_PID_RB_U.PMax=300;
	Jiont_Motor_Speed_PID_RB_U.IMax=5000;
	Jiont_Motor_Speed_PID_RB_U.DMax=3000;
	Jiont_Motor_Speed_PID_RB_U.PIDOutMax=16;
	Jiont_Motor_Speed_PID_RB_U.PIDOutLast=0;
	Jiont_Motor_Speed_PID_RB_U.Target_Speed_Last=0;
	Jiont_Motor_Speed_PID_RB_U.Speed_Ratio=1;
	Jiont_Motor_Speed_PID_RB_U.Acceleration=0;
	
	
	
	Jiont_Motor_Speed_PID_RB_D.CurrentError=0;
	Jiont_Motor_Speed_PID_RB_D.LastError=0;
	Jiont_Motor_Speed_PID_RB_D.ErrorIgnored=0;
	Jiont_Motor_Speed_PID_RB_D.Pout=0;
	Jiont_Motor_Speed_PID_RB_D.Iout=0;
	Jiont_Motor_Speed_PID_RB_D.Dout=0;
	Jiont_Motor_Speed_PID_RB_D.PIDout=0;
	Jiont_Motor_Speed_PID_RB_D.PIDOutCompensate=0;
	Jiont_Motor_Speed_PID_RB_D.PMax=300;
	Jiont_Motor_Speed_PID_RB_D.IMax=5000;
	Jiont_Motor_Speed_PID_RB_D.DMax=3000;
	Jiont_Motor_Speed_PID_RB_D.PIDOutMax=16;
	Jiont_Motor_Speed_PID_RB_D.PIDOutLast=0;
	Jiont_Motor_Speed_PID_RB_D.Target_Speed_Last=0;
	Jiont_Motor_Speed_PID_RB_D.Speed_Ratio=1;
	Jiont_Motor_Speed_PID_RB_D.Acceleration=0;
	


  Jiont_Motor_Speed_PID_RF_U.CurrentError=0;
	Jiont_Motor_Speed_PID_RF_U.LastError=0;
	Jiont_Motor_Speed_PID_RF_U.ErrorIgnored=0;
	Jiont_Motor_Speed_PID_RF_U.Pout=0;
	Jiont_Motor_Speed_PID_RF_U.Iout=0;
	Jiont_Motor_Speed_PID_RF_U.Dout=0;
	Jiont_Motor_Speed_PID_RF_U.PIDout=0;
	Jiont_Motor_Speed_PID_RF_U.PIDOutCompensate=0;
	Jiont_Motor_Speed_PID_RF_U.PMax=300;
	Jiont_Motor_Speed_PID_RF_U.IMax=5000;
	Jiont_Motor_Speed_PID_RF_U.DMax=3000;
	Jiont_Motor_Speed_PID_RF_U.PIDOutMax=16;
	Jiont_Motor_Speed_PID_RF_U.PIDOutLast=0;
	Jiont_Motor_Speed_PID_RF_U.Target_Speed_Last=0;
	Jiont_Motor_Speed_PID_RF_U.Speed_Ratio=1;
	Jiont_Motor_Speed_PID_RF_U.Acceleration=0;
	
	
	
	Jiont_Motor_Speed_PID_RF_D.CurrentError=0;
	Jiont_Motor_Speed_PID_RF_D.LastError=0;
	Jiont_Motor_Speed_PID_RF_D.ErrorIgnored=0;
	Jiont_Motor_Speed_PID_RF_D.Pout=0;
	Jiont_Motor_Speed_PID_RF_D.Iout=0;
	Jiont_Motor_Speed_PID_RF_D.Dout=0;
	Jiont_Motor_Speed_PID_RF_D.PIDout=0;
	Jiont_Motor_Speed_PID_RF_D.PIDOutCompensate=0;
	Jiont_Motor_Speed_PID_RF_D.PMax=300;
	Jiont_Motor_Speed_PID_RF_D.IMax=5000;
	Jiont_Motor_Speed_PID_RF_D.DMax=3000;
	Jiont_Motor_Speed_PID_RF_D.PIDOutMax=16;
	Jiont_Motor_Speed_PID_RF_D.PIDOutLast=0;
	Jiont_Motor_Speed_PID_RF_D.Target_Speed_Last=0;
	Jiont_Motor_Speed_PID_RF_D.Speed_Ratio=1;
	Jiont_Motor_Speed_PID_RF_D.Acceleration=0;
	



  //PID,标定
  Joint_LF_U.Motor_Speed_PID = &Jiont_Motor_Speed_PID_LF_U;
  Joint_LF_D.Motor_Speed_PID = &Jiont_Motor_Speed_PID_LF_D;
	Joint_LB_U.Motor_Speed_PID = &Jiont_Motor_Speed_PID_LB_U;
	Joint_LB_D.Motor_Speed_PID = &Jiont_Motor_Speed_PID_LB_D;
	Joint_RB_U.Motor_Speed_PID = &Jiont_Motor_Speed_PID_RB_U;
	Joint_RB_D.Motor_Speed_PID = &Jiont_Motor_Speed_PID_RB_D;
	Joint_RF_U.Motor_Speed_PID = &Jiont_Motor_Speed_PID_RF_U;
	Joint_RF_D.Motor_Speed_PID = &Jiont_Motor_Speed_PID_RF_D;

}

//设置关节电机FOC的各参数
void Jiont_Motor_FOC_Init(void)
{

	Jiont_Motor_Speed_PID_Init();
	Jiont_Motor_Angle_PID_Init();
	
  Joint_LF_U.Motor_ID=1;	
	Joint_LF_U.send.send_Velocity =0;
	Joint_LF_U.send.send_Position=-1;
	Joint_LF_U.send.send_kp =0;
	Joint_LF_U.send.send_kd =0;
	Joint_LF_U.send.send_torque =0;
	
	
	Joint_LF_D.Motor_ID=2;
	Joint_LF_D.send.send_Velocity =0;
	Joint_LF_D.send.send_kp =0;
	Joint_LF_D.send.send_kd =0;
	Joint_LF_D.send.send_torque =0;

	

  Joint_LB_U.Motor_ID=3;
	Joint_LB_U.send.send_Velocity =0;
	Joint_LB_U.send.send_kp =0;
	Joint_LB_U.send.send_kd =0;
	Joint_LB_U.send.send_torque =0;

  Joint_LB_D.Motor_ID=4;
	Joint_LB_D.send.send_Velocity =0;
	Joint_LB_D.send.send_kp =0;
	Joint_LB_D.send.send_kd =0;
	Joint_LB_D.send.send_torque =0;


  Joint_RB_U.Motor_ID=5;
	Joint_RB_U.send.send_Velocity =0;
	Joint_RB_U.send.send_kp =0;
	Joint_RB_U.send.send_kd =0;
	Joint_RB_U.send.send_torque =0;

  Joint_RB_D.Motor_ID=6;
	Joint_RB_D.send.send_Velocity =0;
	Joint_RB_D.send.send_kp =0;
	Joint_RB_D.send.send_kd =0;
	Joint_RB_D.send.send_torque =0;
	

	Joint_RF_U.Motor_ID=7;
	Joint_RF_U.send.send_Velocity =0;
	Joint_RF_U.send.send_kp =0;
	Joint_RF_U.send.send_kd =0;
	Joint_RF_U.send.send_torque =0;

  Joint_RF_D.Motor_ID=8;
	Joint_RF_D.send.send_Velocity =0;
	Joint_RF_D.send.send_kp =0;
	Joint_RF_D.send.send_kd =0;
	Joint_RF_D.send.send_torque =0;



  //启动FOC控制模式
  USART22CAN_TX_Send_Cmd(CAN_DATA_CMD_ON,1);
	delay_ms(20);
	
	USART32CAN_TX_Send_Cmd(CAN_DATA_CMD_ON,3);
	delay_ms(20);
	
	 USART42CAN_TX_Send_Cmd(CAN_DATA_CMD_ON,5);
	delay_ms(20);
	
	USART52CAN_TX_Send_Cmd(CAN_DATA_CMD_ON,7);
	delay_ms(20);
	
	USART22CAN_TX_Send_Cmd(CAN_DATA_CMD_ON,2);
	delay_ms(20);
		
	
	
	USART42CAN_TX_Send_Cmd(CAN_DATA_CMD_ON,6);
	delay_ms(20);
	
	USART52CAN_TX_Send_Cmd(CAN_DATA_CMD_ON,8);
	delay_ms(20);

 USART32CAN_TX_Send_Cmd(CAN_DATA_CMD_ON,4);
	delay_ms(10);

}





/**
  * @brief  PID计算
  * @param  void
  * @retval void
  * @notes  void  
*/
 float Pid_Calc(PID_TypeDef *PID,float Current_Speed,float Target_Speed)
{   	
	Current_Speed = Current_Speed/PID->Speed_Ratio;
	
	PID->Target_Speed_Last = Target_Speed;
	
	PID->CurrentError = Target_Speed - Current_Speed;
	PID->Err_Change=PID->CurrentError-PID->LastError;
	
	if ( fabs(PID->CurrentError)< PID->ErrorIgnored )  PID->CurrentError = 0;
//	if ( abs(PID->CurrentError)< 30 )  PID->CurrentError = PID->CurrentError+PID->PIDOutCompensate;
	//P  
	PID->Pout = PID->P * PID->CurrentError;
	if(PID->Pout> PID->PMax) PID->Pout= PID->PMax;
	if(PID->Pout<(-PID->PMax)) PID->Pout=(-PID->PMax);
	//I
		PID->Iout += PID->I * PID->CurrentError;
		if(PID->Iout> PID->IMax)   PID->Iout= PID->IMax;
		if(PID->Iout<(-PID->IMax)) PID->Iout=(-PID->IMax);
	//D 
	PID->Dout = PID->D * ( PID->CurrentError - PID->LastError );
	if(PID->Dout> PID->DMax) PID->Dout= PID->DMax;
	if(PID->Dout<(-PID->DMax)) PID->Dout=(-PID->DMax);
	//PID
//	if( (abs(PID->CurrentError)<40) && (abs(PID->CurrentError)>0) )
//	{
		if ( PID->CurrentError > 0 ) PID->PIDout = PID->Pout + PID->Iout + PID->Dout + PID->PIDOutCompensate;
		else  if(PID->CurrentError < 0) PID->PIDout = PID->Pout + PID->Iout + PID->Dout - PID->PIDOutCompensate;
//	}
//////
     else  PID->PIDout = PID->Pout + PID->Iout + PID->Dout;
	if ( (fabs( PID->PIDout - PID->PIDOutLast)>3000)  && (fabs(Current_Speed) !=0) )    
	{
	  PID->PIDout =( PID->PIDout>PID->PIDOutLast) ?  (PID->PIDOutLast+3000) : (PID->PIDOutLast-3000);
	}
  // + PID->PIDOutCompensate;
	
	if(PID->PIDout> PID->PIDOutMax)   PID->PIDout= PID->PIDOutMax;
	if(PID->PIDout<(-PID->PIDOutMax)) PID->PIDout=(-PID->PIDOutMax);

    PID->PIDOutLast = PID->PIDout;
	PID->LastError = PID->CurrentError;
	return PID->PIDout;
}






