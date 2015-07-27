#ifdef STM32F407VG
#ifndef CONFIG_I2C_H
#define CONFIG_I2C_H

#include "stm32f4xx_i2c.h"

int Init_i2c(GPIO_TypeDef *sda_port, uint16_t sda_pin,GPIO_TypeDef *scl_port, uint16_t scl_pin);

#endif//CONFIG_I2C_H
#endif//STM32f407VG
