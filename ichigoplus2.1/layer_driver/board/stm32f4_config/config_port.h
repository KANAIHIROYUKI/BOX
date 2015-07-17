#ifdef STM32F407VG

#ifndef CONFIG_PORT_H
#define CONFIG_PORT_H

#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
//É|Å[ÉgÇÃê›íË
void Init_port(GPIOMode_TypeDef mode,GPIO_TypeDef *port,uint16_t pin,GPIOPuPd_TypeDef pupd,GPIOOType_TypeDef otype);
uint8_t Pin_select_source(uint16_t pin);
#endif // !CONFIG_PORT_H
#endif//STM32f407VG
