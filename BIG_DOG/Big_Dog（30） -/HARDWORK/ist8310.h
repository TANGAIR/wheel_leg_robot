#ifndef IST8310_H
#define IST8310_H
#include "sys.h"


#define IST8310_DATA_READY_BIT 2

#define IST8310_NO_ERROR 0x00
#define IST8310_NO_SENSOR 0x40




uint8_t ist8310_init(void);
void ist8310_read_mag(float mag[3]);


typedef struct ist8310_real_data_t
{
  uint8_t status;
  float mag[3];
} ist8310_real_data_t;



/**
  * @brief          delay x millisecond
  * @param[in]      ms: ms millisecond
  * @retval         none
  */
/**
  * @brief          延时x毫秒
  * @param[in]      ms: ms毫秒
  * @retval         none
  */
extern void ist8310_delay_ms(uint16_t ms);
/**
  * @brief          delay x microsecond
  * @param[in]      us: us microsecond
  * @retval         none
  */
/**
  * @brief          延时x微秒
  * @param[in]      us: us微秒
  * @retval         none
  */
extern void ist8310_delay_us(uint16_t us);
/**
  * @brief          set the RSTN PIN to 1
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          设置RSTN引脚为1
  * @param[in]      none
  * @retval         none
  */
extern void ist8310_RST_H(void);
/**
  * @brief          set the RSTN PIN to 0
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          设置RSTN引脚为0
  * @param[in]      none
  * @retval         none
  */
extern void ist8310_RST_L(void);
















#endif



