#include "main.h"

uint32_t WRITE_single_error=0;
uint32_t read_single_error=0;
uint32_t read_MUTIL_error=0;




/**
  * @brief  I2C��ʼ�����ú���
  * @param  void
  * @retval void
  * @notes  ����ң������ģʽѡ��ͬ�ķ�ʽ
  */
void I2C3_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure; 
  I2C_InitTypeDef  I2C_InitStructure; 
  /*!< IST8310_I2C Periph clock enable */
  IST8310_I2C_CLK_INIT(IST8310_I2C_CLK, ENABLE);
  
  /*!< IST8310_I2C_SCL_GPIO_CLK and IST8310_I2C_SDA_GPIO_CLK Periph clock enable */
  RCC_AHB1PeriphClockCmd(IST8310_I2C_SDA_GPIO_CLK, ENABLE);
  RCC_AHB1PeriphClockCmd(IST8310_I2C_SCL_GPIO_CLK , ENABLE);

  /*!< GPIO configuration */
  /* Connect PXx to I2C_SCL*/
  GPIO_PinAFConfig(IST8310_I2C_SCL_GPIO_PORT, IST8310_I2C_SCL_SOURCE, IST8310_I2C_SCL_AF);
  /* Connect PXx to I2C_SDA*/
  GPIO_PinAFConfig(IST8310_I2C_SDA_GPIO_PORT, IST8310_I2C_SDA_SOURCE, IST8310_I2C_SDA_AF);  
  
  /*!< Configure IST8310_I2C pins: SCL */   
  GPIO_InitStructure.GPIO_Pin = IST8310_I2C_SCL_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(IST8310_I2C_SCL_GPIO_PORT, &GPIO_InitStructure);

  /*!< Configure IST8310_I2C pins: SDA */
  GPIO_InitStructure.GPIO_Pin = IST8310_I2C_SDA_PIN;
  GPIO_Init(IST8310_I2C_SDA_GPIO_PORT, &GPIO_InitStructure);



  /* I2C ���� */
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;	
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;		                      /* �ߵ�ƽ�����ȶ����͵�ƽ���ݱ仯 SCL ʱ���ߵ�ռ�ձ� */
  I2C_InitStructure.I2C_OwnAddress1 =0; 
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable ;	
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;	  /* I2C��Ѱַģʽ */
  I2C_InitStructure.I2C_ClockSpeed = 400000-1;	                                /* ͨ������ */
  
	
	I2C_Init(IST8310_I2C, &I2C_InitStructure);	                                      /* I2C1 ��ʼ�� */
  I2C_Cmd(IST8310_I2C, ENABLE);  	                                                /* ʹ�� I2C1 */

  I2C_AcknowledgeConfig(IST8310_I2C, ENABLE);  
	
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//�������
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//����
	GPIO_Init(GPIOG, &GPIO_InitStructure);//��ʼ��


}




/**
  * @brief          ��ȡIST8310��һ���ֽ�ͨ��I2C
  * @param[in]      �Ĵ�����ַ
  * @retval         �Ĵ���ֵ
  */
uint8_t ist8310_IIC_read_single_reg(uint8_t reg)
{
    uint8_t res = 0;
    read_single_error=I2C_EE_BufferRead(I2C3, IST8310_IIC_ADDRESS<<1 , reg,1,&res);
    return res;
}
/**
  * @brief          ͨ��I2Cд��һ���ֽڵ�IST8310�ļĴ�����
  * @param[in]      �Ĵ�����ַ
  * @param[in]      д��ֵ
  * @retval         none
  */
void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t* data)
{
    WRITE_single_error=I2C_EE_PageWrite(I2C3, IST8310_IIC_ADDRESS<<1 , reg,1,data);
}

/**
  * @brief          ��ȡIST8310�Ķ���ֽ�ͨ��I2C
  * @param[in]      �Ĵ�����ʼ��ַ
  * @param[out]     ��ȡ������
  * @param[in]      ��ȡ�ֽ�����
  * @retval         none
  */
void ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    read_MUTIL_error=I2C_EE_BufferRead(I2C3, IST8310_IIC_ADDRESS <<1, reg,len,buf);
}

/**
  * @brief          д�����ֽڵ�IST8310�ļĴ���ͨ��I2C
  * @param[in]      �Ĵ�����ʼ��ַ
  * @param[out]     ��ȡ������
  * @param[in]      ��ȡ�ֽ�����
  * @retval         none
  */
void ist8310_IIC_write_muli_reg(uint8_t reg, uint8_t *data, uint8_t len)
{
    I2C_EE_PageWrite(I2C3, IST8310_IIC_ADDRESS <<1, reg,len,data);
}







/**
  * @brief   ��EEPROM��һ��дѭ���п���д����ֽڣ���һ��д����ֽ���
  *          ���ܳ���EEPROMҳ�Ĵ�С��AT24C02ÿҳ��8���ֽ�
  * @param   
  *		@arg pBuffer:������ָ��
  *		@arg WriteAddr:д��ַ
  *     @arg NumByteToWrite:д���ֽ���
  * @retval  ��I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout
  */
#define I2CT_FLAG_TIMEOUT         ((uint32_t)(1000*168))
#define I2CT_LONG_TIMEOUT         ((uint32_t)(10 * I2CT_FLAG_TIMEOUT))
uint32_t I2C_EE_PageWrite(I2C_TypeDef* EEPROM_I2C,u8 EEPROM_ADDRESS, u8 WriteAddr,u8 NumByteToWrite, u8* pBuffer)
{
 {
	 uint32_t I2CTimeout = I2CT_LONG_TIMEOUT;

  while(I2C_GetFlagStatus(EEPROM_I2C, I2C_FLAG_BUSY))  
   {
    if((I2CTimeout--) == 0) return I2C_TIMEOUT_UserCallback(4);
  } 
  
	
	
	
	
  /* Send START condition */
  I2C_GenerateSTART(EEPROM_I2C, ENABLE);
  
  
	
	
	
	
	
	
	
  I2CTimeout = I2CT_FLAG_TIMEOUT;

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(EEPROM_I2C, I2C_EVENT_MASTER_MODE_SELECT))
  {
    if((I2CTimeout--) == 0) return I2C_TIMEOUT_UserCallback(5);
  } 
  
	
	
	
	
	
	
  /* Send EEPROM address for write */
  I2C_Send7bitAddress(EEPROM_I2C, EEPROM_ADDRESS, I2C_Direction_Transmitter);
  
	
	
	
	
	
	
	
	
  I2CTimeout = I2CT_FLAG_TIMEOUT;

  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(EEPROM_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) 
  {
    if((I2CTimeout--) == 0) return I2C_TIMEOUT_UserCallback(6);
  } 
	
	
	
	
	
  /* Send the EEPROM's internal address to write to */    
  I2C_SendData(EEPROM_I2C, WriteAddr);  

	
	
	
	
	
	
	
	
	
  I2CTimeout = I2CT_FLAG_TIMEOUT;

  /* Test on EV8 and clear it */
  while(! I2C_CheckEvent(EEPROM_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) 
  {
    if((I2CTimeout--) == 0) return I2C_TIMEOUT_UserCallback(7);
  } 
	
	
	
	
	
	
	
	
	
	
  /* While there is data to be written */
  while(NumByteToWrite--)  
  {
		
		
    /* Send the current byte */
    I2C_SendData(EEPROM_I2C, *pBuffer); 

		
		
    /* Point to the next byte to be written */
    pBuffer++; 
		
    I2CTimeout = I2CT_FLAG_TIMEOUT;

    /* Test on EV8 and clear it */
    while (!I2C_CheckEvent(EEPROM_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
    if((I2CTimeout--) == 0) return I2C_TIMEOUT_UserCallback(8);
    } 
  }

	
	
	
	
	
	
	
	
  /* Send STOP condition */
  I2C_GenerateSTOP(EEPROM_I2C, ENABLE);
  
	
	
	
	
  return 1;
 } 
{
	



}


}

/**
  * @brief   ��EEPROM�����ȡһ������ 
  * @param   
  *		@arg pBuffer:��Ŵ�EEPROM��ȡ�����ݵĻ�����ָ��
  *		@arg WriteAddr:�������ݵ�EEPROM�ĵ�ַ
  *     @arg NumByteToWrite:Ҫ��EEPROM��ȡ���ֽ���
  * @retval  ��
  */
uint32_t I2C_EE_BufferRead(I2C_TypeDef* EEPROM_I2C,u8 EEPROM_ADDRESS, u8 ReadAddr,u8 NumByteToRead, u8* pBuffer)
{  
   uint32_t I2CTimeout = I2CT_LONG_TIMEOUT;

  //*((u8 *)0x4001080c) |=0x80; 
    while(I2C_GetFlagStatus(EEPROM_I2C, I2C_FLAG_BUSY))   
    {
    if((I2CTimeout--) == 0) return I2C_TIMEOUT_UserCallback(9);
    }
	
	
	
	
	
  /* Send START condition */
  I2C_GenerateSTART(EEPROM_I2C, ENABLE);
  //*((u8 *)0x4001080c) &=~0x80;
  
	
	
  I2CTimeout = I2CT_FLAG_TIMEOUT;
  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(EEPROM_I2C, I2C_EVENT_MASTER_MODE_SELECT))
  {
    if((I2CTimeout--) == 0) return I2C_TIMEOUT_UserCallback(10);
   }






  /* Send EEPROM address for write */
  I2C_Send7bitAddress(EEPROM_I2C, EEPROM_ADDRESS, I2C_Direction_Transmitter);









  I2CTimeout = I2CT_FLAG_TIMEOUT;
 
  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(EEPROM_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) 
    {
    if((I2CTimeout--) == 0) return I2C_TIMEOUT_UserCallback(11);
   }
		
	 
	 
	 
	 
  /* Clear EV6 by setting again the PE bit */
  I2C_Cmd(EEPROM_I2C, ENABLE);

  /* Send the EEPROM's internal address to write to */
  I2C_SendData(EEPROM_I2C, ReadAddr);  

   










	 I2CTimeout = I2CT_FLAG_TIMEOUT;

  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(EEPROM_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
    if((I2CTimeout--) == 0) return I2C_TIMEOUT_UserCallback(12);
   }
		
	 
	 
	 
	 
	 
	 
	 
  /* Send STRAT condition a second time */  
  I2C_GenerateSTART(EEPROM_I2C, ENABLE);
  
     I2CTimeout = I2CT_FLAG_TIMEOUT;

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(EEPROM_I2C, I2C_EVENT_MASTER_MODE_SELECT))
    {
    if((I2CTimeout--) == 0) return I2C_TIMEOUT_UserCallback(13);
   }




  /* Send EEPROM address for read */
  I2C_Send7bitAddress(EEPROM_I2C, EEPROM_ADDRESS, I2C_Direction_Receiver);
  
  




   I2CTimeout = I2CT_FLAG_TIMEOUT;

  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(EEPROM_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
    {
    if((I2CTimeout--) == 0) return I2C_TIMEOUT_UserCallback(14);
   }
		
	 
	 
	 
	 
	 
	 
	 
  /* While there is data to be read */
  while(NumByteToRead)  
  {
    if(NumByteToRead == 1)
    {
      /* Disable Acknowledgement */
      I2C_AcknowledgeConfig(EEPROM_I2C, DISABLE);
      
      /* Send STOP Condition */
      I2C_GenerateSTOP(EEPROM_I2C, ENABLE);
    }


		I2CTimeout = I2CT_LONG_TIMEOUT;
		while(I2C_CheckEvent(EEPROM_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED)==0)  
		{
			if((I2CTimeout--) == 0) return I2C_TIMEOUT_UserCallback(3);
		} 	
		
		
		
		
		  /* Read a byte from the device */
      *pBuffer = I2C_ReceiveData(EEPROM_I2C);

      /* Point to the next location where the byte read will be saved */
      pBuffer++; 
      
      /* Decrement the read bytes counter */
      NumByteToRead--;
		




		
  }

	
	
	
	
  /* Enable Acknowledgement to be ready for another reception */
  I2C_AcknowledgeConfig(EEPROM_I2C, ENABLE);
  
  return 1;
}






























static  uint32_t I2C_TIMEOUT_UserCallback(uint8_t errorCode)
{
  /* Block communication and all processes */
  printf("I2C �ȴ���ʱ!errorCode = %d",errorCode);
  
  return errorCode;
}




















