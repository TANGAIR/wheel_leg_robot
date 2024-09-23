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

//TX2���ݽ�������
#define TX2_Decode_Task_PRIO 6
#define TX2_Decode_Task_SIZE 512
TaskHandle_t TX2_Decode_Task_Handler;

//ң�������ݽ�������
#define DBUS_Receive_Data_Task_PRIO 4
#define DBUS_Receive_Data_Task_SIZE 512
TaskHandle_t DBUS_Receive_Data_Task_Handler;

//���������ݻ�ȡ����
#define IMU_Get_Data_Task_PRIO 3
#define IMU_Get_Data_Task_SIZE 512
TaskHandle_t IMU_Get_Data_Task_Handler;



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
	
	  /**************************���ж�֪ͨ������û�б��жϻ���ʱ���ᱻִ�У�********************************/		

	  //����TX2���ݽ�������
	  //�������ȼ�Ϊ 7
	  //USART3_IRQHandler�ж����ȼ�Ϊ 6 ��������TX2����      ��������β�ж�DMA1_Stream3_IRQHandler���ȼ�Ϊ 6
	  //DMA:���ͣ�DMA1_Stream3�����գ�DMA1_Stream1
	  //���жϻ��Ѻ������ִ��Ȩ�ޣ�������̨��ͬʱ���ϰ巢������
    xTaskCreate((TaskFunction_t)TX2_Decode_Task,
                (const char *)"TX2_Decode_Task",
                (uint16_t)TX2_Decode_Task_SIZE,
                (void *)NULL,
                (UBaseType_t)TX2_Decode_Task_PRIO,
                (TaskHandle_t *)&TX2_Decode_Task_Handler);	 

	//����ң�������ݽ�������
		//�������ȼ�Ϊ 4
		//DMA2_Stream5_IRQHandler�ж����ȼ�Ϊ 7,����14ms		
		//DMA:DMA2_Stream5								
		//��ʼ��ң�������մ��ں�DMA�Լ�DMA�жϣ�ÿ14ms���ж�֪ͨ����һ�Σ�����ң�������ݽ��룬
		//�����������ݴ��浽DBUS�ṹ�������							
    xTaskCreate((TaskFunction_t)DBUS_Receive_Data_Task,
                (const char *)"DBUS_Receive_Data_Task",
                (uint16_t)DBUS_Receive_Data_Task_SIZE,
                (void *)NULL,
                (UBaseType_t)DBUS_Receive_Data_Task_PRIO,
                (TaskHandle_t *)&DBUS_Receive_Data_Task_Handler);	
	
	  //�������������ݽ�������					
		//�������ȼ�Ϊ 3
	  //EXTI9_5_IRQHandler�ж����ȼ�Ϊ 8������1ms
	  //ÿ1���뱻PB8�жϻ���һ�Σ���ȡ���������ݣ�   ͬʱ��TX2�������ݣ�
	  //IMU��ʼ����������̨����������
	  xTaskCreate((TaskFunction_t)IMU_Get_Data_Task,      			//�������
                (const char *)"IMU_Get_Data_Task",    				//��������
                (uint16_t)IMU_Get_Data_Task_SIZE,     				//����ջ��С
                (void *)NULL,                         				//���������
                (UBaseType_t)IMU_Get_Data_Task_PRIO,  				//�������ȼ�
                (TaskHandle_t *)&IMU_Get_Data_Task_Handler);  //������				
	
	
	
	  /**************************���ж�֪ͨ������********************************/	
								
								
								
								
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





