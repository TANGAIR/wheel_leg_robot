#ifndef __USART2_H
#define	__USART2_H

#include "stm32f4xx.h"
#include "USART1.h"


/*********************************************º¯ÊýÉùÃ÷Çø**********************************************************************/
 void USART2_Init(void);

void USART22CAN_TX_Send_Cmd (char CAN_DATA_CMD[8],char ID);
void USART22CAN_TX_Jiont (CAN1_Data_TypeDef* motor) ;


#endif /* __LED_H */

