/****************ͷ�ļ�������******************************/
#include "main.h"



/**********************************������ض���***************************************************/
/*
�ֱ���
�������ȼ�������Խ�����ȼ�Խ��
����ջ��С����λ���֣���4�ֽڣ��������������оֲ������Ķ�����ܴ�С���ޣ����������ջ�ܴ�С���ܳ���ϵͳ�Ѵ�С
��������  ���ڶ�������в���
*/
//��������
#define START_TASK_PRIO 1
#define START_STK_SIZE 512
static TaskHandle_t StartTask_Handler;



//LED������ʾ����
#define Heart_Task_PRIO 1
#define Heart_Task_SIZE 512
TaskHandle_t Heart_Task_Handler;

/**
  * @brief  ����������
  * @param  void
  * @retval void
  * @notes  ��������ĺ������壬���ڴ�����������
	          ǰ�������ȼ������񲻻������ȴ��жϸ���֪ͨ
						�ֱ��ǣ�LED����ÿ��һ�Σ�Ħ���ֿ�������ÿ1msִ��һ�Σ���̨��������ÿ1msִ��һ��				
  */
void start_task(void *pvParameters)
{
	 //�����ٽ�������ֹ�жϴ��
    taskENTER_CRITICAL();
	
  	/*��̬��������*/
	
		//����LED��������
		//���ȼ�Ϊ 1
		//���ÿ��1s��˸һ�Σ�����ϵͳ����������
	  xTaskCreate((TaskFunction_t) Heart_Task,
                (const char *)"Heart_Task",
                (uint16_t)Heart_Task_SIZE,
                (void *)NULL,
                (UBaseType_t)Heart_Task_PRIO,
                (TaskHandle_t *)&Heart_Task_Handler);	
								
		//ɾ����ʼ����
    vTaskDelete(StartTask_Handler);
		 //�˳��ٽ���						
    taskEXIT_CRITICAL();           
}


/**
  * @brief  �������񴴽�����
  * @param  void
  * @retval void
  * @notes  ����һ�����������ڿ�����������Ժ�ϵͳֻ��һ����������
            �����������д����������������������ɾ����
  */
void startTask(void)
{
	//��̬���񴴽�
    xTaskCreate((TaskFunction_t)start_task,          //������
                (const char *)"start_task",          //��������
                (uint16_t)START_STK_SIZE,            //�����ջ��С
                (void *)NULL,                        //���ݸ��������Ĳ���
                (UBaseType_t)START_TASK_PRIO,        //�������ȼ�
                (TaskHandle_t *)&StartTask_Handler); //������
}





