/****************头文件包含区******************************/
#include "main.h"



/**********************************任务相关定义***************************************************/
/*
分别是
任务优先级：数字越大优先级越高
任务栈大小：单位是字，即4字节，决定任务中所有局部变量的定义的总大小上限，所有任务的栈总大小不能超过系统堆大小
任务句柄：  用于对任务进行操作
*/
//启动任务
#define START_TASK_PRIO 1
#define START_STK_SIZE 512
static TaskHandle_t StartTask_Handler;



//LED心跳显示任务
#define Heart_Task_PRIO 1
#define Heart_Task_SIZE 512
TaskHandle_t Heart_Task_Handler;

/**
  * @brief  启动任务函数
  * @param  void
  * @retval void
  * @notes  启动任务的函数主体，用于创建各种任务
	          前三个优先级的任务不会阻塞等待中断给出通知
						分别是：LED任务每秒一次，摩擦轮控制任务每1ms执行一次，云台控制任务每1ms执行一次				
  */
void start_task(void *pvParameters)
{
	 //进入临界区，防止中断打断
    taskENTER_CRITICAL();
	
  	/*动态创建任务*/
	
		//创建LED心跳任务
		//优先级为 1
		//红灯每隔1s闪烁一次，表明系统在正常运行
	  xTaskCreate((TaskFunction_t) Heart_Task,
                (const char *)"Heart_Task",
                (uint16_t)Heart_Task_SIZE,
                (void *)NULL,
                (UBaseType_t)Heart_Task_PRIO,
                (TaskHandle_t *)&Heart_Task_Handler);	
								
		//删除开始任务
    vTaskDelete(StartTask_Handler);
		 //退出临界区						
    taskEXIT_CRITICAL();           
}


/**
  * @brief  启动任务创建函数
  * @param  void
  * @retval void
  * @notes  创建一个启动任务，在开启任务调度以后系统只有一个启动任务，
            在启动任务中创建其他任务。最后将启动任务删除。
  */
void startTask(void)
{
	//动态任务创建
    xTaskCreate((TaskFunction_t)start_task,          //任务函数
                (const char *)"start_task",          //任务名称
                (uint16_t)START_STK_SIZE,            //任务堆栈大小
                (void *)NULL,                        //传递给任务函数的参数
                (UBaseType_t)START_TASK_PRIO,        //任务优先级
                (TaskHandle_t *)&StartTask_Handler); //任务句柄
}





