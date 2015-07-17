#ifdef STM32F407VG

#ifndef NAME_CONVERTER_H
#define NAME_CONVERTER_H

#include <math.h>

#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
//#include "stm32f4xx_adc.h"
#include "stm32f4xx_crc.h"
//#include "stm32f4xx_dma.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_gpio.h"
//#include "stm32f4xx_i2c.h"
//#include "stm32f4xx_pwr.h"
#include "stm32f4xx_rcc.h"
//#include "stm32f4xx_spi.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_usart.h"
#include "misc.h"


//ピンの情報の構造体
typedef struct{
    //ピン番号＝配列番号
	unsigned short user_pin[16];    //使用しているピンの情報用変数
	unsigned short user_pin_sum;		//使用しているピンの総数
	uint16_t	pin_address[16];          //使用しているピンのアドレス
}Pin_t;

void Analysis_GPIO_Pin(uint16_t pin, Pin_t *pin_state);
int USART_irqn_select(USART_TypeDef *usart);
uint8_t GPIO_af_USART_select(USART_TypeDef *usart);
int Tim_irqn_select(TIM_TypeDef *tim);
void RCC_PeriphClock_TIM(TIM_TypeDef *tim);
void RCC_PeriphClock_USART(USART_TypeDef *usart);
void NVIC_Configuration(int irqn);
void NVIC_config_USART(USART_TypeDef *usart);
uint8_t Pin_select_source(uint16_t pin);
uint8_t Tim_select_af(TIM_TypeDef *tim);

#endif // !NAME_CONVERTER_H
#endif//STM32F407VG
