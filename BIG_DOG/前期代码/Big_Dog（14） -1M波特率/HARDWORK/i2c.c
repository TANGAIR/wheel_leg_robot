#include "main.h"

uint32_t WRITE_single_error=0;
uint32_t read_single_error=0;
uint32_t read_MUTIL_error=0;




/**
  * @brief  I2C初始化配置函数
  * @param  void
  * @retval void
  * @notes  根据遥控器的模式选择不同的方式
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



  /* I2C 配置 */
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;	
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;		                      /* 高电平数据稳定，低电平数据变化 SCL 时钟线的占空比 */
  I2C_InitStructure.I2C_OwnAddress1 =0; 
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable ;	
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;	  /* I2C的寻址模式 */
  I2C_InitStructure.I2C_ClockSpeed = 400000-1;	                                /* 通信速率 */
  
	
	I2C_Init(IST8310_I2C, &I2C_InitStructure);	                                      /* I2C1 初始化 */
  I2C_Cmd(IST8310_I2C, ENABLE);  	                                                /* 使能 I2C1 */

  I2C_AcknowledgeConfig(IST8310_I2C, ENABLE);  
	
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//输出功能
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//上拉
	GPIO_Init(GPIOG, &GPIO_InitStructure);//初始化


}




/**
  * @brief          读取IST8310的一个字节通过I2C
  * @param[in]      寄存器地址
  * @retval         寄存器值
  */
uint8_t ist8310_IIC_read_single_reg(uint8_t reg)
{
    uint8_t res = 0;
    read_single_error=I2C_EE_BufferRead(I2C3, IST8310_IIC_ADDRESS<<1 , reg,1,&res);
    return res;
}
/**
  * @brief          通过I2C写入一个字节到IST8310的寄存器中
  * @param[in]      寄存器地址
  * @param[in]      写入值
  * @retval         none
  */
void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t* data)
{
    WRITE_single_error=I2C_EE_PageWrite(I2C3, IST8310_IIC_ADDRESS<<1 , reg,1,data);
}

/**
  * @brief          读取IST8310的多个字节通过I2C
  * @param[in]      寄存器开始地址
  * @param[out]     存取缓冲区
  * @param[in]      读取字节总数
  * @retval         none
  */
void ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    read_MUTIL_error=I2C_EE_BufferRead(I2C3, IST8310_IIC_ADDRESS <<1, reg,len,buf);
}

/**
  * @brief          写入多个字节到IST8310的寄存器通过I2C
  * @param[in]      寄存器开始地址
  * @param[out]     存取缓冲区
  * @param[in]      读取字节总数
  * @retval         none
  */
void ist8310_IIC_write_muli_reg(uint8_t reg, uint8_t *data, uint8_t len)
{
    I2C_EE_PageWrite(I2C3, IST8310_IIC_ADDRESS <<1, reg,len,data);
}







/**
  * @brief   在EEPROM的一个写循环中可以写多个字节，但一次写入的字节数
  *          不能超过EEPROM页的大小，AT24C02每页有8个字节
  * @param   
  *		@arg pBuffer:缓冲区指针
  *		@arg WriteAddr:写地址
  *     @arg NumByteToWrite:写的字节数
  * @retval  无I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout
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
  * @brief   从EEPROM里面读取一块数据 
  * @param   
  *		@arg pBuffer:存放从EEPROM读取的数据的缓冲区指针
  *		@arg WriteAddr:接收数据的EEPROM的地址
  *     @arg NumByteToWrite:要从EEPROM读取的字节数
  * @retval  无
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
  printf("I2C 等待超时!errorCode = %d",errorCode);
  
  return errorCode;
}




















