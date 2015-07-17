
#ifndef  CONFIG_PWM_H
#define CONFIG_PWM_H

#include "stm32f4xx.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
#include "system_stm32f4xx.h"

#include "config_port.h"
#include "name_converter.h"

/*//PWMの使用例

Init_PWM(TIM4,GPIOD,GPIO_Pin_13,10000);
TIM4をPWMに設定し、PD13から10000HzのPWM波を出す。

Set_duty(TIM4,2,0.5);
TIM4のch2をduty比50%に設定。
duty比はfloat型の0-1。小数点で指定。

GPIOのpin番号とチャンネルに関してはSTM32F407のデータシート(DocID022152 Rev 4)の60ページ以降でも見るといい。

#define LED3PWMSET TIM4,GPIOD,GPIO_Pin_13
#define LED3PWM TIM4,2
って宣言して

Init_PWM(LED3PWMSET,10000);
Set_duty(LED3PWM,0.1);

ってする方法もあるんだけど、関数の引数の数が見かけ減るから気持ち悪い。
嫌いな人がいるんじゃないかなって思って聞いたら、実際にいた。

Init_PWM(TIM4,GPIOD,GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15,10000);
ってこともできる。

Init_PWM関数は周波数設定がうまくいかなかったときに1が返ってくるようになってる。

#define PRINTF_AVAILABLE をヘッダファイルに追加すると、printfを設定していれば設定の過程が見れるよ。
*/
void  Set_duty(TIM_TypeDef* TIMx,uint8_t ch,float duty);										//デューティ比の設定
int Init_PWM(TIM_TypeDef * TIMx,GPIO_TypeDef *GPIOx,uint16_t pin,int frequency);	//PWM波形の設定

//2015_5_18 成宮追加
float Read_duty(TIM_TypeDef* TIMx,uint8_t ch);

#endif//CONFIG_PWM_H
