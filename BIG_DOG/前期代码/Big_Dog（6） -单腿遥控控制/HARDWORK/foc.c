#include "main.h"



PID_TypeDef Wheel_Motor_PID_201,
            Wheel_Motor_PID_202,
						Wheel_Motor_PID_203,
						Wheel_Motor_PID_204;


void Jiont_Motor_FOC_Init(void)
{

  Joint_LF_U.Motor_ID=1;	
	Joint_LF_U.send.send_Velocity =0;
	Joint_LF_U.send.send_kp =50;
	Joint_LF_U.send.send_kd =2;
	Joint_LF_U.send.send_torque =0;
	
	
	Joint_LF_D.Motor_ID=2;
	Joint_LF_D.send.send_Velocity =0;
	Joint_LF_D.send.send_kp =50;
	Joint_LF_D.send.send_kd =2;
	Joint_LF_D.send.send_torque =0;

	

  Joint_LB_U.Motor_ID=3;
	Joint_LB_U.send.send_Velocity =0;
	Joint_LB_U.send.send_kp =50;
	Joint_LB_U.send.send_kd =2;
	Joint_LB_U.send.send_torque =0;

  Joint_LB_D.Motor_ID=4;
	Joint_LB_D.send.send_Velocity =0;
	Joint_LB_D.send.send_kp =50;
	Joint_LB_D.send.send_kd =2;
	Joint_LB_D.send.send_torque =0;


  Joint_RB_U.Motor_ID=5;
	Joint_RB_U.send.send_Velocity =0;
	Joint_RB_U.send.send_kp =50;
	Joint_RB_U.send.send_kd =2;
	Joint_RB_U.send.send_torque =0;

  Joint_RB_D.Motor_ID=6;
	Joint_RB_D.send.send_Velocity =0;
	Joint_RB_D.send.send_kp =50;
	Joint_RB_D.send.send_kd =2;
	Joint_RB_D.send.send_torque =0;
	
	
	
	Joint_RF_U.Motor_ID=7;
	Joint_RF_U.send.send_Velocity =0;
	Joint_RF_U.send.send_kp =50;
	Joint_RF_U.send.send_kd =2;
	Joint_RF_U.send.send_torque =0;

  Joint_RF_D.Motor_ID=8;
	Joint_RF_D.send.send_Velocity =0;
	Joint_RF_D.send.send_kp =50;
	Joint_RF_D.send.send_kd =2;
	Joint_RF_D.send.send_torque =0;



//启动FOC控制模式
  CAN1_Send_Cmd(CAN_DATA_CMD_ON,1);
	CAN1_Send_Cmd(CAN_DATA_CMD_ON,2);
	CAN1_Send_Cmd(CAN_DATA_CMD_ON,3);
	CAN1_Send_Cmd(CAN_DATA_CMD_ON,4);
	CAN1_Send_Cmd(CAN_DATA_CMD_ON,5);
	CAN1_Send_Cmd(CAN_DATA_CMD_ON,6);
	CAN1_Send_Cmd(CAN_DATA_CMD_ON,7);
	CAN1_Send_Cmd(CAN_DATA_CMD_ON,8);


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
	Wheel_Motor_PID_201.PMax=9000;
	Wheel_Motor_PID_201.IMax=5000;
	Wheel_Motor_PID_201.DMax=3000;
	Wheel_Motor_PID_201.PIDOutMax=15000;
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
	Wheel_Motor_PID_202.PMax=9000;
	Wheel_Motor_PID_202.IMax=5000;
	Wheel_Motor_PID_202.DMax=3000;
	Wheel_Motor_PID_202.PIDOutMax=15000;
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
	Wheel_Motor_PID_203.PMax=9000;
	Wheel_Motor_PID_203.IMax=5000;
	Wheel_Motor_PID_203.DMax=3000;
	Wheel_Motor_PID_203.PIDOutMax=15000;
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
	Wheel_Motor_PID_204.PMax=9000;
	Wheel_Motor_PID_204.IMax=5000;
	Wheel_Motor_PID_204.DMax=3000;
	Wheel_Motor_PID_204.PIDOutMax=15000;
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

















