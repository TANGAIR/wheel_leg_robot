#include "main.h"


void Jiont_Motor_FOC_Init(void)
{

  Joint_LF_U.Motor_ID=1;
	Joint_LF_U.send.send_Velocity =0;
	Joint_LF_U.send.send_kp =23;
	Joint_LF_U.send.send_kd =2;
	Joint_LF_U.send.send_torque =0;
	
	
	Joint_LF_D.Motor_ID=2;
	Joint_LF_D.send.send_Velocity =0;
	Joint_LF_D.send.send_kp =23;
	Joint_LF_D.send.send_kd =2;
	Joint_LF_D.send.send_torque =0;

	

  Joint_LB_U.Motor_ID=3;
	Joint_LB_U.send.send_Velocity =0;
	Joint_LB_U.send.send_kp =23;
	Joint_LB_U.send.send_kd =2;
	Joint_LB_U.send.send_torque =0;

  Joint_LB_D.Motor_ID=4;
	Joint_LB_D.send.send_Velocity =0;
	Joint_LB_D.send.send_kp =23;
	Joint_LB_D.send.send_kd =2;
	Joint_LB_D.send.send_torque =0;


  Joint_RB_U.Motor_ID=5;
	Joint_RB_U.send.send_Velocity =0;
	Joint_RB_U.send.send_kp =23;
	Joint_RB_U.send.send_kd =2;
	Joint_RB_U.send.send_torque =0;

  Joint_RB_D.Motor_ID=6;
	Joint_RB_D.send.send_Velocity =0;
	Joint_RB_D.send.send_kp =23;
	Joint_RB_D.send.send_kd =2;
	Joint_RB_D.send.send_torque =0;
	
	
	
	Joint_RF_U.Motor_ID=7;
	Joint_RF_U.send.send_Velocity =0;
	Joint_RF_U.send.send_kp =23;
	Joint_RF_U.send.send_kd =2;
	Joint_RF_U.send.send_torque =0;

  Joint_RF_D.Motor_ID=8;
	Joint_RF_D.send.send_Velocity =0;
	Joint_RF_D.send.send_kp =23;
	Joint_RF_D.send.send_kd =2;
	Joint_RF_D.send.send_torque =0;





}



























