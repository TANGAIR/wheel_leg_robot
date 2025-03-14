#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
#include "sys.h"
#include "delay.h"



#include "key.h"
#include "led.h"

#include "USART1.h"
#include "FOC.h"


#include "USART2.h"
#include "USART3.h"
#include "USART4.h"
#include "USART5.h"

/*******************************************************�궨��*******************************************************/
//�ؽڵ��CAN1ID
#define CAN1_JOINT_ID_LF_U                0x01          //��ǰ�ϽǶȵ��1��seigama
#define CAN1_JOINT_ID_LF_D                0x02          //��ǰ�½Ƕȵ��2��gama
#define CAN1_JOINT_ID_LB_U                0X03          //����ϽǶȵ��3
#define	CAN1_JOINT_ID_LB_D                0X04          //����½Ƕȵ��4
#define	CAN1_JOINT_ID_RB_U								0X05					//�Һ��ϽǶȵ��5
#define	CAN1_JOINT_ID_RB_D								0X06					//�Һ��½Ƕȵ��6
#define	CAN1_JOINT_ID_RF_U								0X07					//��ǰ�ϽǶȵ��7
#define	CAN1_JOINT_ID_RF_D								0X08					//��ǰ�½Ƕȵ��8

#define BASE_JIONT_KP (150)


//�ؽڵ��FOC���Ʋ���
 #define P_MIN -12.5f
 #define P_MAX 12.5f
 #define V_MIN -45.0f
 #define V_MAX 45.0f
 #define KP_MIN 0.0f
 #define KP_MAX 500.0f
 #define KD_MIN 0.0f
 #define KD_MAX 5.0f
 #define T_MIN -18.0f
 #define T_MAX 18.0f
 



extern  char USART_CAN_SEND_WAIT;  







#endif



