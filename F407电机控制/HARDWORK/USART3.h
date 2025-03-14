#ifndef __USART3_H
#define	__USART3_H

#include "stm32f4xx.h"
#include "USART1.h"


/*********************************************º¯ÊýÉùÃ÷Çø**********************************************************************/
 void USART3_Init(void);

void USART32CAN_TX_Send_Cmd (char CAN_DATA_CMD[8],char ID);
void USART32CAN_TX_Jiont (CAN1_Data_TypeDef* motor) ;


#endif /* __LED_H */

