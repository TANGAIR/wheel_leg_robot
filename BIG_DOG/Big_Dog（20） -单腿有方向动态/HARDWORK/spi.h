#ifndef _SPI_H
#define _SPI_H
#include "sys.h"

void SPI_GPIO_InitConfig(void);
uint8_t BMI088_read_write_byte(uint8_t reg);

#endif
