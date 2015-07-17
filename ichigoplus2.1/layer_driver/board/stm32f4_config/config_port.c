#ifdef STM32F407VG
#include "config_port.h"

/******************************************************************************
*	タイトル ： ポートの設定（固定）
*	  関数名 ： Init_port
*	  戻り値 ： void型						なし
*	   引数1 ： GPIOMode_TypeDef型	mode	モードの指定
*	   引数2 ： GPIO_TypeDef*型		port	ポートの指定
*	   引数3 ： uint16_t型			pin		ピンの指定
*	   引数4 ： GPIOPuPd_TypeDef型	pupd	プルアップ/ダウンの指定
*	   引数5 ： GPIOOType_TypeDef型	otype	出力タイプの指定
*	  作成者 ： 桐生
*	  作成日 ： 2014/11/10
******************************************************************************/
void Init_port(GPIOMode_TypeDef mode,GPIO_TypeDef *port,uint16_t pin,GPIOPuPd_TypeDef pupd,GPIOOType_TypeDef otype){
	GPIO_InitTypeDef GPIO_InitStructure;					//GPIO初期化のための構造体

	//モジュールストップの解除
	if(port == GPIOA){
	#ifndef RCC_AHB1Periph_GPIOA_SETEND
		#define RCC_AHB1Periph_GPIOA_SETEND
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	#endif
	}
	#ifndef RCC_AHB1Periph_GPIOB_SETEND
	else if(port == GPIOB){
		#define RCC_AHB1Periph_GPIOB_SETEND
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	}
	#endif
	#ifndef RCC_AHB1Periph_GPIOC_SETEND
	else if(port == GPIOC){
		#define RCC_AHB1Periph_GPIOC_SETEND
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	}
	#endif
	#ifndef RCC_AHB1Periph_GPIOD_SETEND
	else if(port == GPIOD){
		#define RCC_AHB1Periph_GPIOD_SETEND
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	}
	#endif
	#ifndef RCC_AHB1Periph_GPIOE_SETEND
	else if(port == GPIOE){
		#define RCC_AHB1Periph_GPIOE_SETEND
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	}
	#endif

	GPIO_StructInit(&GPIO_InitStructure);											//初期化用構造体にパラメータをセットしていくため、いったん初期値に戻す
	GPIO_InitStructure.GPIO_Pin = pin;												//設定するピンを指定する

	GPIO_InitStructure.GPIO_Mode = mode;										//ピンのモード設定
	GPIO_InitStructure.GPIO_OType = otype;										//出力タイプ設定
	GPIO_InitStructure.GPIO_PuPd = pupd;											//出力ポートをプッシュプルまたはオープンドレインに設定


	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;			//GPIOのスピードを100MHz（最高速）に設定する
	GPIO_Init(port, &GPIO_InitStructure);											//PORT設定入力

	if(mode == GPIO_Mode_OUT)	GPIO_ResetBits(port,pin);			//ポートのリセット
}
#endif//STM32F407VG
