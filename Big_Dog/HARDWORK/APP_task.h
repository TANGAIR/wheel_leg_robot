#ifndef APP_TASK_H
#define APP_TASK_H

#include "task.h"

void startTask(void);

extern TaskHandle_t TX2_Decode_Task_Handler;

extern TaskHandle_t DBUS_Receive_Data_Task_Handler;

extern TaskHandle_t IMU_Get_Data_Task_Handler;



#endif
