#ifdef STM32F407VG

#ifndef  CONFIG_ADC_H
#define CONFIG_ADC_H

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_dma.h"

#include "config_port.h"
#include "name_converter.h"

uint8_t Select_ADC_channel(GPIO_TypeDef *GPIOx, uint16_t pin);		
void Init_ADC1(GPIO_TypeDef *GPIOx, uint16_t pin);
float get_ADC1_value(GPIO_TypeDef *GPIOx, uint16_t pin);

/*関数説明*/

/*Select_ADC_channel(GPIO_TypeDef *GPIOx, uint16_t pin)*/
/*指定したポートとピンに対応するADC_Channelを返す*/
/*使用例
	ADC_channel = Select_ADC_channel(GPIOA, GPIO_Pin_5);*/


/*Init_ADC1(GPIO_TypeDef *GPIOx, uint16_t pin,short *ADC_converted_value)*/
/*ADC1の初期設定*/
/*使用例
	Init_ADC1(GPIOA, GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3, &ad_value[3]);
	※設定できるのは1つのポートのみ。ピンは複数対応*/

/*get_ADC1_value(unsigned short ch)*/
/*指定したチャンネルのADC1の値を得る*/
/*使用例
	adc_value = get_ADC1_value(3)
	※チャンネルは0から7まで*/

#endif
#endif//STM32F407VG
