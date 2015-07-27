#ifdef STM32F407VG
#ifndef config_can_h
#define config_can_h
#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4xx_can.h"
#include "stm32f4xx_rcc.h"

#include "stm32f4xx_gpio.h"
#include "config_port.h"
#include "name_converter.h"


typedef enum{
	CAN_DATA=0,
	CAN_REMOTE
}can_flame_t;
typedef enum{
	CAN_MASK=0,
	CAN_LIST
}can_filter_mode_t;



//EXTIèâä˙âªä÷êî
void Init_CAN(int can_num,uint8_t CAN_Mode,GPIO_TypeDef *tx_port,uint16_t tx_pin,GPIO_TypeDef *rx_port,uint16_t rx_pin);
void Init_CANfilter(int filter_num,can_filter_mode_t mode ,short stid0,can_flame_t rtr0,short stid1,can_flame_t rtr1
											,short stid2,can_flame_t rtr2,short stid3,can_flame_t rtr3);
void send_can1(CanTxMsg *data);
#endif
#endif//STM32F407VG
