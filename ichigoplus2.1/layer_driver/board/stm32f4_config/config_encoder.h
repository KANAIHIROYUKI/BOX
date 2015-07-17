#ifdef STM32F407VG

#ifndef CONFIG_ENCODER_H
#define CONFIG_ENCODER_H

#include "stm32f4xx.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "misc.h"

#include "name_converter.h"
#include "config_port.h"


void Reset_encoder_over_under_flow(void);

void Init_encoder(TIM_TypeDef *tim,  GPIO_TypeDef *port, uint16_t pin);

int Select_encoder_count(TIM_TypeDef* TIMx);

void TIM1_UP_TIM10_IRQHandler(void);

void TIM2_IRQHandler(void);

void TIM3_IRQHandler(void);

void TIM4_IRQHandler(void);

void TIM5_IRQHandler(void);

void TIM8_UP_TIM13_IRQHandler(void);

void TIM8_BRK_TIM12_IRQHandler(void);

int tim1_count( void );

int tim2_count( void );

int tim3_count( void );

int tim4_count( void );

int tim5_count( void );

int tim8_count( void );

int tim12_count( void );
/* 使い方 */
/*Init_encoderを呼び出し引数に使いたい　タイマー、ポート、ピン　を入れるとエンコーダの設定ができます。*/
/*エンコーダのカウント値を見たい時は、 timx_count を呼び出すと値が取れます。(x は数字)*/
/*TIMx: where x can be 1, 2, 3, 4, 5, 8, 9 or 12 to select the TIM*/

#endif
#endif//STM32f407VG
