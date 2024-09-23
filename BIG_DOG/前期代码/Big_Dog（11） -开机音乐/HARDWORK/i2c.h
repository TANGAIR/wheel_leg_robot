#ifndef I2C_H
#define I2C_H
#include "sys.h"


void I2C3_Init(void);

#define IST8310_IIC_ADDRESS 0x0E  //the I2C address of IST8310
#define I2C_MEMADD_SIZE_8BIT            0x00000001U



 /**I2C3 GPIO Configuration    
    PC9     ------> I2C3_SDA
    PA8     ------> I2C3_SCL 
    */

/*I2C�ӿ�*/
#define IST8310_I2C                          I2C3
#define IST8310_I2C_CLK                      RCC_APB1Periph_I2C3
#define IST8310_I2C_CLK_INIT								 RCC_APB1PeriphClockCmd

#define IST8310_I2C_SCL_PIN                  GPIO_Pin_8                 
#define IST8310_I2C_SCL_GPIO_PORT            GPIOA                       
#define IST8310_I2C_SCL_GPIO_CLK             RCC_AHB1Periph_GPIOA
#define IST8310_I2C_SCL_SOURCE               GPIO_PinSource8
#define IST8310_I2C_SCL_AF                   GPIO_AF_I2C3

#define IST8310_I2C_SDA_PIN                  GPIO_Pin_9                  
#define IST8310_I2C_SDA_GPIO_PORT            GPIOC                      
#define IST8310_I2C_SDA_GPIO_CLK             RCC_AHB1Periph_GPIOC
#define IST8310_I2C_SDA_SOURCE               GPIO_PinSource9
#define IST8310_I2C_SDA_AF                   GPIO_AF_I2C3




static  uint32_t I2C_TIMEOUT_UserCallback(uint8_t errorCode);
uint32_t I2C_EE_PageWrite(I2C_TypeDef* EEPROM_I2C,u8 EEPROM_ADDRESS, u8 WriteAddr,u8 NumByteToWrite, u8* pBuffer);
uint32_t I2C_EE_BufferRead(I2C_TypeDef* EEPROM_I2C,u8 EEPROM_ADDRESS, u8 ReadAddr,u8 NumByteToRead, u8* pBuffer);


/**
  * @brief          read a byte of ist8310 by i2c
  * @param[in]      register address
  * @retval         value of the register
  */
/**
  * @brief          ��ȡIST8310��һ���ֽ�ͨ��I2C
  * @param[in]      �Ĵ�����ַ
  * @retval         �Ĵ���ֵ
  */
extern uint8_t ist8310_IIC_read_single_reg(uint8_t reg);

/**
  * @brief          write a byte of ist8310 by i2c
  * @param[in]      register address
  * @param[in]      write value
  * @retval         value of the register
  */
/**
  * @brief          ͨ��I2Cд��һ���ֽڵ�IST8310�ļĴ�����
  * @param[in]      �Ĵ�����ַ
  * @param[in]      д��ֵ
  * @retval         none
  */
extern void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t* data);

/**
  * @brief          read multiple byte of ist8310 by i2c
  * @param[in]      register start address
  * @param[out]     read buffer
  * @param[in]      Size Amount of data to be read
  * @retval         none
  */
/**
  * @brief          ��ȡIST8310�Ķ���ֽ�ͨ��I2C
  * @param[in]      �Ĵ�����ʼ��ַ
  * @param[out]     ��ȡ������
  * @param[in]      ��ȡ�ֽ�����
  * @retval         none
  */
extern void ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);

/**
  * @brief          write multiple byte of ist8310 by i2c
  * @param[in]      register address
  * @param[out]     write buffer
  * @param[in]      Size Amount of data to be sent
  * @retval         none
  */
/**
  * @brief          д�����ֽڵ�IST8310�ļĴ���ͨ��I2C
  * @param[in]      �Ĵ�����ʼ��ַ
  * @param[out]     ��ȡ������
  * @param[in]      ��ȡ�ֽ�����
  * @retval         none
  */
extern void ist8310_IIC_write_muli_reg(uint8_t reg, uint8_t *data, uint8_t len);

/**
  * @brief          delay x millisecond
  * @param[in]      ms: ms millisecond
  * @retval         none
  */
/**
  * @brief          ��ʱx����
  * @param[in]      ms: ms����
  * @retval         none
  */







#endif


