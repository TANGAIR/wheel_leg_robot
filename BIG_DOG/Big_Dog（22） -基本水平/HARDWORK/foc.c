#include "main.h"



PID_TypeDef Wheel_Motor_PID_201,
            Wheel_Motor_PID_202,
						Wheel_Motor_PID_203,
						Wheel_Motor_PID_204;
						
PID_TypeDef Jiont_Motor_PID_LF_U,
            Jiont_Motor_PID_LF_D,
						Jiont_Motor_PID_LB_U,
						Jiont_Motor_PID_LB_D,
            Jiont_Motor_PID_RB_U,
            Jiont_Motor_PID_RB_D,
						Jiont_Motor_PID_RF_U,
						Jiont_Motor_PID_RF_D;

//关节PID用于差值反馈控制关节电机FOC的KP
void Jiont_Motor_PID_Init(void)
{
	
	Jiont_Motor_PID_LF_U.P=300;
	Jiont_Motor_PID_LF_U.I=0;
	Jiont_Motor_PID_LF_U.D=0;
	
	Jiont_Motor_PID_LF_D.P=300;                     //10
	Jiont_Motor_PID_LF_D.I=0;                   //0.01
	Jiont_Motor_PID_LF_D.D=0;  
		
	Jiont_Motor_PID_LB_U.P=300;
	Jiont_Motor_PID_LB_U.I=0;
	Jiont_Motor_PID_LB_U.D=0;
	
	Jiont_Motor_PID_LB_D.P=300;                     //10
	Jiont_Motor_PID_LB_D.I=0;                   //0.01
	Jiont_Motor_PID_LB_D.D=0;  

	
	Jiont_Motor_PID_RB_U.P=300;
	Jiont_Motor_PID_RB_U.I=0;
	Jiont_Motor_PID_RB_U.D=0;
	
	Jiont_Motor_PID_RB_D.P=300;                     //10
	Jiont_Motor_PID_RB_D.I=0;                   //0.01
	Jiont_Motor_PID_RB_D.D=0;  
		
	Jiont_Motor_PID_RF_U.P=500;
	Jiont_Motor_PID_RF_U.I=0;
	Jiont_Motor_PID_RF_U.D=0;
	
	Jiont_Motor_PID_RF_D.P=500;                     //10
	Jiont_Motor_PID_RF_D.I=0;                   //0.01
	Jiont_Motor_PID_RF_D.D=0;  
	
	
	Jiont_Motor_PID_LF_U.CurrentError=0;
	Jiont_Motor_PID_LF_U.LastError=0;
	Jiont_Motor_PID_LF_U.ErrorIgnored=0;
	Jiont_Motor_PID_LF_U.Pout=0;
	Jiont_Motor_PID_LF_U.Iout=0;
	Jiont_Motor_PID_LF_U.Dout=0;
	Jiont_Motor_PID_LF_U.PIDout=0;
	Jiont_Motor_PID_LF_U.PIDOutCompensate=0;
	Jiont_Motor_PID_LF_U.PMax=300;
	Jiont_Motor_PID_LF_U.IMax=5000;
	Jiont_Motor_PID_LF_U.DMax=3000;
	Jiont_Motor_PID_LF_U.PIDOutMax=300;
	Jiont_Motor_PID_LF_U.PIDOutLast=0;
	Jiont_Motor_PID_LF_U.Target_Speed_Last=0;
	Jiont_Motor_PID_LF_U.Speed_Ratio=1;
	Jiont_Motor_PID_LF_U.Acceleration=0;
	Jiont_Motor_PID_LF_U.Ke=0;
	Jiont_Motor_PID_LF_U.Kec=0;
	Jiont_Motor_PID_LF_U.Ku_D=0;
	Jiont_Motor_PID_LF_U.Ku_I=0;
	Jiont_Motor_PID_LF_U.Ku_P=0;
	
	
	Jiont_Motor_PID_LF_D.CurrentError=0;
	Jiont_Motor_PID_LF_D.LastError=0;
	Jiont_Motor_PID_LF_D.ErrorIgnored=0;
	Jiont_Motor_PID_LF_D.Pout=0;
	Jiont_Motor_PID_LF_D.Iout=0;
	Jiont_Motor_PID_LF_D.Dout=0;
	Jiont_Motor_PID_LF_D.PIDout=0;
	Jiont_Motor_PID_LF_D.PIDOutCompensate=0;
	Jiont_Motor_PID_LF_D.PMax=300;
	Jiont_Motor_PID_LF_D.IMax=5000;
	Jiont_Motor_PID_LF_D.DMax=3000;
	Jiont_Motor_PID_LF_D.PIDOutMax=300;
	Jiont_Motor_PID_LF_D.PIDOutLast=0;
	Jiont_Motor_PID_LF_D.Target_Speed_Last=0;
	Jiont_Motor_PID_LF_D.Speed_Ratio=1;
	Jiont_Motor_PID_LF_D.Acceleration=0;
	Jiont_Motor_PID_LF_D.Ke=0;
	Jiont_Motor_PID_LF_D.Kec=0;
	Jiont_Motor_PID_LF_D.Ku_D=0;
	Jiont_Motor_PID_LF_D.Ku_I=0;
	Jiont_Motor_PID_LF_D.Ku_P=0;


  Jiont_Motor_PID_LB_U.CurrentError=0;
	Jiont_Motor_PID_LB_U.LastError=0;
	Jiont_Motor_PID_LB_U.ErrorIgnored=0;
	Jiont_Motor_PID_LB_U.Pout=0;
	Jiont_Motor_PID_LB_U.Iout=0;
	Jiont_Motor_PID_LB_U.Dout=0;
	Jiont_Motor_PID_LB_U.PIDout=0;
	Jiont_Motor_PID_LB_U.PIDOutCompensate=0;
	Jiont_Motor_PID_LB_U.PMax=300;
	Jiont_Motor_PID_LB_U.IMax=5000;
	Jiont_Motor_PID_LB_U.DMax=3000;
	Jiont_Motor_PID_LB_U.PIDOutMax=300;
	Jiont_Motor_PID_LB_U.PIDOutLast=0;
	Jiont_Motor_PID_LB_U.Target_Speed_Last=0;
	Jiont_Motor_PID_LB_U.Speed_Ratio=1;
	Jiont_Motor_PID_LB_U.Acceleration=0;
	Jiont_Motor_PID_LB_U.Ke=0;
	Jiont_Motor_PID_LB_U.Kec=0;
	Jiont_Motor_PID_LB_U.Ku_D=0;
	Jiont_Motor_PID_LB_U.Ku_I=0;
	Jiont_Motor_PID_LB_U.Ku_P=0;
	
	
	Jiont_Motor_PID_LB_D.CurrentError=0;
	Jiont_Motor_PID_LB_D.LastError=0;
	Jiont_Motor_PID_LB_D.ErrorIgnored=0;
	Jiont_Motor_PID_LB_D.Pout=0;
	Jiont_Motor_PID_LB_D.Iout=0;
	Jiont_Motor_PID_LB_D.Dout=0;
	Jiont_Motor_PID_LB_D.PIDout=0;
	Jiont_Motor_PID_LB_D.PIDOutCompensate=0;
	Jiont_Motor_PID_LB_D.PMax=300;
	Jiont_Motor_PID_LB_D.IMax=5000;
	Jiont_Motor_PID_LB_D.DMax=3000;
	Jiont_Motor_PID_LB_D.PIDOutMax=300;
	Jiont_Motor_PID_LB_D.PIDOutLast=0;
	Jiont_Motor_PID_LB_D.Target_Speed_Last=0;
	Jiont_Motor_PID_LB_D.Speed_Ratio=1;
	Jiont_Motor_PID_LB_D.Acceleration=0;
	Jiont_Motor_PID_LB_D.Ke=0;
	Jiont_Motor_PID_LB_D.Kec=0;
	Jiont_Motor_PID_LB_D.Ku_D=0;
	Jiont_Motor_PID_LB_D.Ku_I=0;
	Jiont_Motor_PID_LB_D.Ku_P=0;

	
	Jiont_Motor_PID_RB_U.CurrentError=0;
	Jiont_Motor_PID_RB_U.LastError=0;
	Jiont_Motor_PID_RB_U.ErrorIgnored=0;
	Jiont_Motor_PID_RB_U.Pout=0;
	Jiont_Motor_PID_RB_U.Iout=0;
	Jiont_Motor_PID_RB_U.Dout=0;
	Jiont_Motor_PID_RB_U.PIDout=0;
	Jiont_Motor_PID_RB_U.PIDOutCompensate=0;
	Jiont_Motor_PID_RB_U.PMax=300;
	Jiont_Motor_PID_RB_U.IMax=5000;
	Jiont_Motor_PID_RB_U.DMax=3000;
	Jiont_Motor_PID_RB_U.PIDOutMax=300;
	Jiont_Motor_PID_RB_U.PIDOutLast=0;
	Jiont_Motor_PID_RB_U.Target_Speed_Last=0;
	Jiont_Motor_PID_RB_U.Speed_Ratio=1;
	Jiont_Motor_PID_RB_U.Acceleration=0;
	Jiont_Motor_PID_RB_U.Ke=0;
	Jiont_Motor_PID_RB_U.Kec=0;
	Jiont_Motor_PID_RB_U.Ku_D=0;
	Jiont_Motor_PID_RB_U.Ku_I=0;
	Jiont_Motor_PID_RB_U.Ku_P=0;
	
	
	Jiont_Motor_PID_RB_D.CurrentError=0;
	Jiont_Motor_PID_RB_D.LastError=0;
	Jiont_Motor_PID_RB_D.ErrorIgnored=0;
	Jiont_Motor_PID_RB_D.Pout=0;
	Jiont_Motor_PID_RB_D.Iout=0;
	Jiont_Motor_PID_RB_D.Dout=0;
	Jiont_Motor_PID_RB_D.PIDout=0;
	Jiont_Motor_PID_RB_D.PIDOutCompensate=0;
	Jiont_Motor_PID_RB_D.PMax=300;
	Jiont_Motor_PID_RB_D.IMax=5000;
	Jiont_Motor_PID_RB_D.DMax=3000;
	Jiont_Motor_PID_RB_D.PIDOutMax=300;
	Jiont_Motor_PID_RB_D.PIDOutLast=0;
	Jiont_Motor_PID_RB_D.Target_Speed_Last=0;
	Jiont_Motor_PID_RB_D.Speed_Ratio=1;
	Jiont_Motor_PID_RB_D.Acceleration=0;
	Jiont_Motor_PID_RB_D.Ke=0;
	Jiont_Motor_PID_RB_D.Kec=0;
	Jiont_Motor_PID_RB_D.Ku_D=0;
	Jiont_Motor_PID_RB_D.Ku_I=0;
	Jiont_Motor_PID_RB_D.Ku_P=0;


  Jiont_Motor_PID_RF_U.CurrentError=0;
	Jiont_Motor_PID_RF_U.LastError=0;
	Jiont_Motor_PID_RF_U.ErrorIgnored=0;
	Jiont_Motor_PID_RF_U.Pout=0;
	Jiont_Motor_PID_RF_U.Iout=0;
	Jiont_Motor_PID_RF_U.Dout=0;
	Jiont_Motor_PID_RF_U.PIDout=0;
	Jiont_Motor_PID_RF_U.PIDOutCompensate=0;
	Jiont_Motor_PID_RF_U.PMax=300;
	Jiont_Motor_PID_RF_U.IMax=5000;
	Jiont_Motor_PID_RF_U.DMax=3000;
	Jiont_Motor_PID_RF_U.PIDOutMax=300;
	Jiont_Motor_PID_RF_U.PIDOutLast=0;
	Jiont_Motor_PID_RF_U.Target_Speed_Last=0;
	Jiont_Motor_PID_RF_U.Speed_Ratio=1;
	Jiont_Motor_PID_RF_U.Acceleration=0;
	Jiont_Motor_PID_RF_U.Ke=0;
	Jiont_Motor_PID_RF_U.Kec=0;
	Jiont_Motor_PID_RF_U.Ku_D=0;
	Jiont_Motor_PID_RF_U.Ku_I=0;
	Jiont_Motor_PID_RF_U.Ku_P=0;
	
	
	Jiont_Motor_PID_RF_D.CurrentError=0;
	Jiont_Motor_PID_RF_D.LastError=0;
	Jiont_Motor_PID_RF_D.ErrorIgnored=0;
	Jiont_Motor_PID_RF_D.Pout=0;
	Jiont_Motor_PID_RF_D.Iout=0;
	Jiont_Motor_PID_RF_D.Dout=0;
	Jiont_Motor_PID_RF_D.PIDout=0;
	Jiont_Motor_PID_RF_D.PIDOutCompensate=0;
	Jiont_Motor_PID_RF_D.PMax=300;
	Jiont_Motor_PID_RF_D.IMax=5000;
	Jiont_Motor_PID_RF_D.DMax=3000;
	Jiont_Motor_PID_RF_D.PIDOutMax=300;
	Jiont_Motor_PID_RF_D.PIDOutLast=0;
	Jiont_Motor_PID_RF_D.Target_Speed_Last=0;
	Jiont_Motor_PID_RF_D.Speed_Ratio=1;
	Jiont_Motor_PID_RF_D.Acceleration=0;
	Jiont_Motor_PID_RF_D.Ke=0;
	Jiont_Motor_PID_RF_D.Kec=0;
	Jiont_Motor_PID_RF_D.Ku_D=0;
	Jiont_Motor_PID_RF_D.Ku_I=0;
	Jiont_Motor_PID_RF_D.Ku_P=0;



  //PID,标定
  Joint_LF_U.Motor_PID = &Jiont_Motor_PID_LF_U;
  Joint_LF_D.Motor_PID = &Jiont_Motor_PID_LF_D;
	Joint_LB_U.Motor_PID = &Jiont_Motor_PID_LB_U;
	Joint_LB_D.Motor_PID = &Jiont_Motor_PID_LB_D;
	Joint_RB_U.Motor_PID = &Jiont_Motor_PID_RB_U;
	Joint_RB_D.Motor_PID = &Jiont_Motor_PID_RB_D;
	Joint_RF_U.Motor_PID = &Jiont_Motor_PID_RF_U;
	Joint_RF_D.Motor_PID = &Jiont_Motor_PID_RF_D;

}


//设置关节电机FOC的各参数
void Jiont_Motor_FOC_Init(void)
{

  Joint_LF_U.Motor_ID=1;	
	Joint_LF_U.send.send_Velocity =0;
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
//  CAN1_Send_Cmd(CAN_DATA_CMD_ON,1);
//	CAN1_Send_Cmd(CAN_DATA_CMD_ON,2);
//	CAN1_Send_Cmd(CAN_DATA_CMD_ON,3);
//	CAN1_Send_Cmd(CAN_DATA_CMD_ON,4);
//	CAN1_Send_Cmd(CAN_DATA_CMD_ON,5);
//	CAN1_Send_Cmd(CAN_DATA_CMD_ON,6);
//	CAN1_Send_Cmd(CAN_DATA_CMD_ON,7);
//	CAN1_Send_Cmd(CAN_DATA_CMD_ON,8);


}


void Wheel_Motor_PID_Init(void)
{
  Wheel_Motor_PID_201.P=9;
	Wheel_Motor_PID_201.I=0;
	Wheel_Motor_PID_201.D=1;
	
	Wheel_Motor_PID_202.P=9;                     //10
	Wheel_Motor_PID_202.I=0;                   //0.01
	Wheel_Motor_PID_202.D=1;  
		
	Wheel_Motor_PID_203.P=9;
	Wheel_Motor_PID_203.I=0;
	Wheel_Motor_PID_203.D=1;
	
	Wheel_Motor_PID_204.P=9;                     //10
	Wheel_Motor_PID_204.I=0;                   //0.01
	Wheel_Motor_PID_204.D=1;  

	
	Wheel_Motor_PID_201.CurrentError=0;
	Wheel_Motor_PID_201.LastError=0;
	Wheel_Motor_PID_201.ErrorIgnored=0;
	Wheel_Motor_PID_201.Pout=0;
	Wheel_Motor_PID_201.Iout=0;
	Wheel_Motor_PID_201.Dout=0;
	Wheel_Motor_PID_201.PIDout=0;
	Wheel_Motor_PID_201.PIDOutCompensate=0;
	Wheel_Motor_PID_201.PMax=16000;
	Wheel_Motor_PID_201.IMax=5000;
	Wheel_Motor_PID_201.DMax=3000;
	Wheel_Motor_PID_201.PIDOutMax=16000;
	Wheel_Motor_PID_201.PIDOutLast=0;
	Wheel_Motor_PID_201.Target_Speed_Last=0;
	Wheel_Motor_PID_201.Speed_Ratio=1;
	Wheel_Motor_PID_201.Acceleration=0;
	Wheel_Motor_PID_201.Ke=0;
	Wheel_Motor_PID_201.Kec=0;
	Wheel_Motor_PID_201.Ku_D=0;
	Wheel_Motor_PID_201.Ku_I=0;
	Wheel_Motor_PID_201.Ku_P=0;
	
	
	Wheel_Motor_PID_202.CurrentError=0;
	Wheel_Motor_PID_202.LastError=0;
	Wheel_Motor_PID_202.ErrorIgnored=0;
	Wheel_Motor_PID_202.Pout=0;
	Wheel_Motor_PID_202.Iout=0;
	Wheel_Motor_PID_202.Dout=0;
	Wheel_Motor_PID_202.PIDout=0;
	Wheel_Motor_PID_202.PIDOutCompensate=0;
	Wheel_Motor_PID_202.PMax=16000;
	Wheel_Motor_PID_202.IMax=5000;
	Wheel_Motor_PID_202.DMax=3000;
	Wheel_Motor_PID_202.PIDOutMax=16000;
	Wheel_Motor_PID_202.PIDOutLast=0;
	Wheel_Motor_PID_202.Target_Speed_Last=0;
	Wheel_Motor_PID_202.Speed_Ratio=1;
	Wheel_Motor_PID_202.Acceleration=0;
	Wheel_Motor_PID_202.Ke=0;
	Wheel_Motor_PID_202.Kec=0;
	Wheel_Motor_PID_202.Ku_D=0;
	Wheel_Motor_PID_202.Ku_I=0;
	Wheel_Motor_PID_202.Ku_P=0;


  Wheel_Motor_PID_203.CurrentError=0;
	Wheel_Motor_PID_203.LastError=0;
	Wheel_Motor_PID_203.ErrorIgnored=0;
	Wheel_Motor_PID_203.Pout=0;
	Wheel_Motor_PID_203.Iout=0;
	Wheel_Motor_PID_203.Dout=0;
	Wheel_Motor_PID_203.PIDout=0;
	Wheel_Motor_PID_203.PIDOutCompensate=0;
	Wheel_Motor_PID_203.PMax=16000;
	Wheel_Motor_PID_203.IMax=5000;
	Wheel_Motor_PID_203.DMax=3000;
	Wheel_Motor_PID_203.PIDOutMax=16000;
	Wheel_Motor_PID_203.PIDOutLast=0;
	Wheel_Motor_PID_203.Target_Speed_Last=0;
	Wheel_Motor_PID_203.Speed_Ratio=1;
	Wheel_Motor_PID_203.Acceleration=0;
	Wheel_Motor_PID_203.Ke=0;
	Wheel_Motor_PID_203.Kec=0;
	Wheel_Motor_PID_203.Ku_D=0;
	Wheel_Motor_PID_203.Ku_I=0;
	Wheel_Motor_PID_203.Ku_P=0;
	
	
	Wheel_Motor_PID_204.CurrentError=0;
	Wheel_Motor_PID_204.LastError=0;
	Wheel_Motor_PID_204.ErrorIgnored=0;
	Wheel_Motor_PID_204.Pout=0;
	Wheel_Motor_PID_204.Iout=0;
	Wheel_Motor_PID_204.Dout=0;
	Wheel_Motor_PID_204.PIDout=0;
	Wheel_Motor_PID_204.PIDOutCompensate=0;
	Wheel_Motor_PID_204.PMax=16000;
	Wheel_Motor_PID_204.IMax=5000;
	Wheel_Motor_PID_204.DMax=3000;
	Wheel_Motor_PID_204.PIDOutMax=16000;
	Wheel_Motor_PID_204.PIDOutLast=0;
	Wheel_Motor_PID_204.Target_Speed_Last=0;
	Wheel_Motor_PID_204.Speed_Ratio=1;
	Wheel_Motor_PID_204.Acceleration=0;
	Wheel_Motor_PID_204.Ke=0;
	Wheel_Motor_PID_204.Kec=0;
	Wheel_Motor_PID_204.Ku_D=0;
	Wheel_Motor_PID_204.Ku_I=0;
	Wheel_Motor_PID_204.Ku_P=0;

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

















