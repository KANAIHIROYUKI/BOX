#ifdef STM32F407VG
#include "name_converter.h"




//------------------------------------------------------------------------------
//	設定のアシスト関数

/******************************************************************************
*	タイトル ： 使用するピンの情報分析関数
*	  関数名 ： Analysis_GPIO_Pin
*	  戻り値 ： void型
*	   引数1 ： uint16_t型 pin  使用するピン
*	   引数2 ： Pin_t型 *pin_state  ピンの情報の構造体(ポインタ)
*	  作成者 ： 坂下文彦
*	  作成日 ： 2014/11/12
******************************************************************************/
void Analysis_GPIO_Pin(uint16_t pin, Pin_t *pin_state)
{
	unsigned short i = 0;

	pin_state->user_pin_sum = 0;	//使用しているピンの数を初期化

	//使用するピンは１、しなければ０が格納される
	pin_state->user_pin[0] = pin & GPIO_Pin_0;
	pin_state->user_pin[1] = (pin & GPIO_Pin_1) >> 1;
	pin_state->user_pin[2] = (pin & GPIO_Pin_2)>> 2;
	pin_state->user_pin[3] = (pin & GPIO_Pin_3) >> 3;
	pin_state->user_pin[4] = (pin & GPIO_Pin_4) >> 4;
	pin_state->user_pin[5] = (pin & GPIO_Pin_5) >> 5;
	pin_state->user_pin[6] = (pin & GPIO_Pin_6) >> 6;
	pin_state->user_pin[7] = (pin & GPIO_Pin_7) >> 7;
	pin_state->user_pin[8] = (pin & GPIO_Pin_8) >> 8;
	pin_state->user_pin[9] = (pin & GPIO_Pin_9) >> 9;
	pin_state->user_pin[10] = (pin & GPIO_Pin_10) >> 10;
	pin_state->user_pin[11] = (pin & GPIO_Pin_11) >> 11;
	pin_state->user_pin[12] = (pin & GPIO_Pin_12) >> 12;
	pin_state->user_pin[13] = (pin & GPIO_Pin_13) >> 13;
	pin_state->user_pin[14] = (pin & GPIO_Pin_14) >> 14;
	pin_state->user_pin[15] = (pin & GPIO_Pin_15) >> 15;


	//使用するピンのアドレスと総数を格納
	for(i = 0; i < 16; i ++){
		if(pin_state->user_pin[i] == 1){
			pin_state->pin_address[i] =  (uint16_t)(pow(2, i));		//アドレス
			pin_state->user_pin_sum ++;										//総数
		}else{
		    pin_state->pin_address[i] = 0x0000;
		}
	}
}

/******************************************************************************
*	タイトル ： USARTに対応したAF_USARTを返す関数
*	  関数名 ： GPIO_af_USART_select
*	  戻り値 ： uint8_t型 AF_USART
*	   引数1 ： USART_TypeDef型 *usart  USART
*	  作成者 ： 石井岳史
*	  作成日 ： 2014/11/13
******************************************************************************/
uint8_t GPIO_af_USART_select(USART_TypeDef *usart){
	uint8_t af_usart=0;

	if(usart == USART1){
		af_usart = GPIO_AF_USART1;
	}else if(usart == USART2){
		af_usart = GPIO_AF_USART2;
	}else if(usart == USART3){
		af_usart = GPIO_AF_USART3;
	}else if(usart == UART4){
		af_usart = GPIO_AF_UART4;
	}else if(usart == UART5){
        af_usart = GPIO_AF_UART5;
	}else if(usart == USART6){
		af_usart = GPIO_AF_USART6;
	}
	return af_usart;
}

/******************************************************************************
*	タイトル ： USARTに対応したIRQnを返す関数
*	  関数名 ： USART_irqn_select
*	  戻り値 ： int型 IRQn
*	   引数1 ： USART_TypeDef型 *usart  USART
*	  作成者 ： 石井岳史
*	  作成日 ： 2014/11/13
******************************************************************************/
int USART_irqn_select(USART_TypeDef *usart){
	int irqn=0;

	if(usart == USART1){
		irqn = USART1_IRQn;
	}else if(usart == USART2){
		irqn = USART2_IRQn;
	}else if(usart == USART3){
		irqn = USART3_IRQn;
	}else if(usart == UART4){
		irqn = UART4_IRQn;
	}else if(usart == UART5){
        irqn = UART5_IRQn;
	}else if(usart == USART6){
		irqn = USART6_IRQn;
	}
	return irqn;
}

/******************************************************************************
*	タイトル ： Timに対応したIRQnを返す関数
*	  関数名 ： Tim_irqn_select
*	  戻り値 ： int型 IRQn
*	   引数1 ： USART_TypeDef型 *tim  TIM
*	  作成者 ： 成宮陽生
*	  作成日 ： 2014/11/13
******************************************************************************/
int Tim_irqn_select(TIM_TypeDef *tim)
{
	int irqn=0;

	if(tim == TIM1){
		irqn = TIM1_UP_TIM10_IRQn;
	}else if(tim == TIM2){
		irqn = TIM2_IRQn;
	}else if(tim == TIM3){
		irqn = TIM3_IRQn;
	}else if(tim == TIM4){
		irqn = TIM4_IRQn;
	}else if(tim == TIM5){
        irqn = TIM5_IRQn;
	}else if(tim == TIM7){
		irqn = TIM7_IRQn;
	}else if(tim == TIM8){
		irqn = TIM8_UP_TIM13_IRQn;
	}

	return irqn;
}

/******************************************************************************
*	タイトル ： タイマーのモジュールストップ解除
*	  関数名 ： RCC_PeriphClock_TIM
*	  戻り値 ： void型 
*	   引数1 ： TIM_TypeDef *tim型 {  クロックを供給するタイマー
*	  作成者 ： 桐生
*	  作成日 ： 2014/11/14
******************************************************************************/
void RCC_PeriphClock_TIM(TIM_TypeDef *tim){
	static unsigned int f=0;

	if(tim == TIM1 && (f&0x01) == 0){
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
		f |= 0x01;
	}else if(tim == TIM2 && (f&0x02) == 0){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
		f |= 0x02;
	}else if(tim == TIM3 && (f&0x04) == 0){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
		f |= 0x04;
	}else if(tim == TIM4 && (f&0x08) == 0){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
		f |= 0x08;
	}else if(tim == TIM5 && (f&0x10) == 0){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
		f |= 0x10;
	}else if(tim == TIM8 && (f&0x20) == 0){
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
		f |= 0x20;
	}else if(tim == TIM9 && (f&0x40) == 0){
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
		f |= 0x40;
	}else if(tim == TIM10 && (f&0x80) == 0){
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);
		f |= 0x80;
	}else if(tim == TIM11 && (f&0x100) == 0){
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11, ENABLE);
		f |= 0x100;
	}
}
/******************************************************************************
*	タイトル ： USARTのモジュールストップ解除
*	  関数名 ： RCC_PeriphClock_USART
*	  戻り値 ： void型 
*	   引数1 ： USART_TypeDef*型 usart  クロックを供給するUSART
*	  作成者 ： 桐生
*	  作成日 ： 2014/11/14
******************************************************************************/
void RCC_PeriphClock_USART(USART_TypeDef *usart){
	static unsigned int f=0;

	if(usart == USART1 && (f&0x01) == 0){
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
		f |= 0x01;
	}else if(usart == USART2 && (f&0x02) == 0){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
		f |= 0x02;
	}else if(usart == USART3 && (f&0x04) == 0){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
		f |= 0x04;
	}else if(usart == UART4 && (f&0x08) == 0){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
		f |= 0x08;
	}else if(usart == UART5 && (f&0x10) == 0){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
		f |= 0x10;
	}else if(usart == USART6 && (f&0x20) == 0){
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
		f |= 0x20;
	}
}

/******************************************************************************
*	タイトル ： 割り込み設定
*	  関数名 ： NVIC_Configuration
*	  戻り値 ： void型 
*	   引数1 ： int型 irqn  設定する割り込み
*	  作成者 ： 桐生
*	  作成日 ： 2014/11/14
******************************************************************************/
void NVIC_Configuration(int irqn){
	NVIC_InitTypeDef	NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = irqn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


/******************************************************************************
*	タイトル ： USART割り込み設定
*	  関数名 ： NVIC_config_USART
*	  戻り値 ： void型 
*	   引数1 ： USART_TypeDef *型 usart  USARTx
*	  作成者 ： 桐生
*	  作成日 ： 2014/11/14
******************************************************************************/
void NVIC_config_USART(USART_TypeDef *usart){
	if(usart == USART1){
		NVIC_Configuration(USART1_IRQn);
	}else if(usart == USART2){
		NVIC_Configuration(USART2_IRQn);
	}else if(usart == USART3){
		NVIC_Configuration(USART3_IRQn);
	}else if(usart == UART4){
		NVIC_Configuration(UART4_IRQn);
	}else if(usart == UART5){
		NVIC_Configuration(UART5_IRQn);
	}else if(usart == USART6){
		NVIC_Configuration(USART6_IRQn);
	}
}


/******************************************************************************
*	タイトル ： pinからsourceへ変換
*	  関数名 ： Pin_select_source
*	  戻り値 ： uint8_t型 
*	   引数1 ： uint16_t型 pin  PIN
*	  作成者 ： 桐生
*	  作成日 ： 2014/11/14
******************************************************************************/
uint8_t Pin_select_source(uint16_t pin){

	uint8_t source=0;

	switch(pin){
		case GPIO_Pin_0:	source = GPIO_PinSource0;
		break;
		case GPIO_Pin_1:	source = GPIO_PinSource1;
		break;
		case GPIO_Pin_2:	source = GPIO_PinSource2;
		break;
		case GPIO_Pin_3:	source = GPIO_PinSource3;
		break;
		case GPIO_Pin_4:	source = GPIO_PinSource4;
		break;
		case GPIO_Pin_5:	source = GPIO_PinSource5;
		break;
		case GPIO_Pin_6:	source = GPIO_PinSource6;
		break;
		case GPIO_Pin_7:	source = GPIO_PinSource7;
		break;
		case GPIO_Pin_8:	source = GPIO_PinSource8;
		break;
		case GPIO_Pin_9:	source = GPIO_PinSource9;
		break;
		case GPIO_Pin_10:	source = GPIO_PinSource10;
		break;
		case GPIO_Pin_11:	source = GPIO_PinSource11;
		break;
		case GPIO_Pin_12:	source = GPIO_PinSource12;
		break;
		case GPIO_Pin_13:	source = GPIO_PinSource13;
		break;
		case GPIO_Pin_14:	source = GPIO_PinSource14;
		break;
		case GPIO_Pin_15:	source = GPIO_PinSource15;
		break;
		default:break;
	}

	return source;
}

/******************************************************************************
*	タイトル ： TIMのAF選択
*	  関数名 ： Tim_select_af
*	  戻り値 ： uint8_t型 
*	   引数1 ： TIM_TypeDef *型 tim  TIMx
*	  作成者 ： 桐生
*	  作成日 ： 2014/11/14
******************************************************************************/
uint8_t Tim_select_af(TIM_TypeDef *tim){
	uint8_t select_af=0;

	if(tim == TIM1){
		select_af = GPIO_AF_TIM1;
	}else if(tim == TIM2){
		select_af = GPIO_AF_TIM2;
	}else if(tim == TIM3){
		select_af = GPIO_AF_TIM3;
	}else if(tim == TIM4){
		select_af = GPIO_AF_TIM4;
	}else if(tim == TIM5){
		select_af = GPIO_AF_TIM5;
	}else if(tim == TIM8){
		select_af = GPIO_AF_TIM8;
	}else if(tim == TIM9){
		select_af = GPIO_AF_TIM9;
	}else if(tim == TIM10){
		select_af = GPIO_AF_TIM10;
	}else if(tim == TIM11){
		select_af = GPIO_AF_TIM11;
	}

	return select_af;
}
#endif//STM32f407VG
