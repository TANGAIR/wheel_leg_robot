#include "main.h"
#include "move_control.h"
/*************************************************数据定义区*************************************************/
 int mode=0;
 char Contron_ID=2; 

 /*************************************************函数定义区**************************************************/
/**
  * @brief  移动控制任务函数
  * @param  void
  * @retval void
  * @notes  根据遥控器的模式选择不同的方式
  */
void Move_Control_Task(void *pvParameters)
{
	
	BMI088_init();            //陀螺仪初始化，开启陀螺仪中断
	
	USART1_Init();						//串口1，双板通讯初始化
	CAN1_Init();              //关节电机Can初始化
	CAN2_Init();              //轮电机初始化
	vTaskDelay(100);

	Beep_ON ();
	vTaskDelay(500);
	Beep_OFF();
  Foot_Direction_Init();    //腿部位置初始化，收缩状态
	vTaskDelay(1000);


	while(1)
	{	

	   //左侧拨杆步行
	  if(	DBUS.RC.Switch_Left==RC_SW_UP)
		{
      //腿
			 Shrink();
//		 signal_leg(&Right_Back_Leg_Struct);
//		 signal_leg(&Left_Back_Leg_Struct);
			//轮
			Slide();
		}
		//轮式前行
		else if(DBUS.RC.Switch_Left==RC_SW_DOWN)
		{
			//腿
		  Shrink();
			//轮
		  Slide();
		}
    //中间以及其他 ，停
		else 
		{
		  //腿部回原点
	    Shrink();
			//轮速度设为0
			Wheel_LF.Target_Speed =0;
			Wheel_LB.Target_Speed =0;
			Wheel_RB.Target_Speed =0;
			Wheel_RF.Target_Speed =0;
			
		} 
		
	//	Motor_Debug();
		//轮通讯输出
		CAN2_TX();
		//关节输出
	  
		
	 Contron_TX_Jiont();
    vTaskDelay(10);
			
	}	

	
	//电机调试
			

			

	
}


/**
  * @brief  电机调试函数
  * @param  void
  * @retval void
  * @notes  用于设置电机ID以及零角度位置
  */
char Set_0=0;
 void Motor_Debug(void)
 {
   	//电机调试模式
		//启动电机
		if(mode==1)
		{
			  CAN1_Send_Cmd(CAN_DATA_CMD_ON,Contron_ID);
		}
		
		//FOC控制
		else if (mode==2)
		{
			switch(Contron_ID)
			{
				 //1
				 case CAN1_JOINT_ID_LF_U:                                                                                
					{
						CAN1_TX_Jiont(&Joint_LF_U);
			
					}break ;
					//2
					case CAN1_JOINT_ID_LF_D:                                                                               
					{
						CAN1_TX_Jiont(&Joint_LF_D);
					}break ;
					//3
				  case CAN1_JOINT_ID_LB_U:                                                                                
					{
						CAN1_TX_Jiont(&Joint_LB_U);
			
					}break ;
					//4
					case CAN1_JOINT_ID_LB_D:                                                                               
					{
						CAN1_TX_Jiont(&Joint_LB_D);
					}break ;
					 //5
					case CAN1_JOINT_ID_RB_U:                                                                                
					{
						 CAN1_TX_Jiont(&Joint_RB_U);
			
					}break ;
					//6
					case CAN1_JOINT_ID_RB_D:                                                                               
					{
						CAN1_TX_Jiont(&Joint_RB_D);
						
					}break ;
					 //7
					case CAN1_JOINT_ID_RF_U:                                                                                
					{
						CAN1_TX_Jiont(&Joint_RF_U);
					}break ;
					 //8
					case CAN1_JOINT_ID_RF_D:                                                                               
					{
            CAN1_TX_Jiont(&Joint_RF_D);
						
					}break ;
					

					default:
					{}
					break ;	   

			}
			
		}		
		//改ID
		else if(mode==9)
		{
			 CAN1_Send_Cmd(CAN_DATA_CMD_Change_ID,Contron_ID);
		}
		//制0
		else if(mode==10)
		{
			if(Set_0==0)
			{
				 CAN1_Send_Cmd(CAN_DATA_CMD_SET_0,Contron_ID);
				 Set_0=1;
			}
			 
		}
		else
		{
	  	 CAN1_Send_Cmd(CAN_DATA_CMD_OFF,Contron_ID);		
		}

 }
 
 /**
  * @brief  足端位置初始化函数
  * @param  void
  * @retval void
  * @notes  腿类型标定，足端回位，轮电机激活以及速度设置为停止
  */
 void Foot_Direction_Init(void)
{
	//腿部类型标定
	Left_Front_Leg_Struct.Leg_Type  = LEFT_FRONT_LEG;
	Left_Back_Leg_Struct.Leg_Type   = LEFT_BACK_LEG;
  Right_Back_Leg_Struct.Leg_Type  = RIGHT_BACK_LEG;
	Right_Front_Leg_Struct.Leg_Type = RIGHT_FRONT_LEG;
  
	Left_Front_Leg_Struct.Leg_Mode   = 1;
	Left_Back_Leg_Struct.Leg_Mode    = 1;
  Right_Back_Leg_Struct.Leg_Mode   = 1;
	Right_Front_Leg_Struct.Leg_Mode  = 1;
	
	
  //将腿数据结构体对象与关节电机结构体对应
  Left_Front_Leg_Struct.Jiont_Motor_U=&Joint_LF_U;//左前上角度电机1
  Left_Front_Leg_Struct.Jiont_Motor_D=&Joint_LF_D;//左前下角度电机2
  Left_Back_Leg_Struct.Jiont_Motor_U=&Joint_LB_U;//左后上角度电机3
  Left_Back_Leg_Struct.Jiont_Motor_D=&Joint_LB_D;//左后下角度电机4
  
  Right_Back_Leg_Struct.Jiont_Motor_U=&Joint_RB_U;//右后上角度电机5
  Right_Back_Leg_Struct.Jiont_Motor_D=&Joint_RB_D;//右后下角度电机6
  Right_Front_Leg_Struct.Jiont_Motor_U=&Joint_RF_U;//右前上角度电机7
  Right_Front_Leg_Struct.Jiont_Motor_D=&Joint_RF_D;//右前下角度电机8


	
	//关节电机强化PID初始化
  Jiont_Motor_PID_Init();
	//轮电机PID初始化
	Wheel_Motor_PID_Init();
	
	
	//关节电机参数初始化,启动所有电机，启动后至少需要0.5秒冷却
	Jiont_Motor_FOC_Init(); 
	vTaskDelay(500);//必须加上，否则会疯
	
	
  //腿部回原点
 // Shrink();

  vTaskDelay(100);//必须加上，否则会疯

}



 

 










