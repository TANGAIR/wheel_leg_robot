#include "main.h"


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



//…Ë÷√0Œª÷√


















  CAN1_Send_Cmd(CAN_DATA_CMD_ON,1);
	CAN1_Send_Cmd(CAN_DATA_CMD_ON,2);
	CAN1_Send_Cmd(CAN_DATA_CMD_ON,3);
	CAN1_Send_Cmd(CAN_DATA_CMD_ON,4);
	CAN1_Send_Cmd(CAN_DATA_CMD_ON,5);
	CAN1_Send_Cmd(CAN_DATA_CMD_ON,6);
	CAN1_Send_Cmd(CAN_DATA_CMD_ON,7);
	CAN1_Send_Cmd(CAN_DATA_CMD_ON,8);


}



























