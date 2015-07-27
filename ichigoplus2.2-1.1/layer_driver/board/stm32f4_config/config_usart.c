#ifdef STM32F407VG
#include "config_usart.h"
//#include "settings.h"
#include <stdio.h>
#include <string.h>
//#include "config_usb.h"





/******************************************************************************
 *	タイトル ： USART1割込み関数
 *	  関数名 ： USART1_IRQHandler
 *	  戻り値 ： void型
 *	    引数 ： なし
 *	  作成者 ： 石井岳史
 *	  作成日 ： 2014/11/12
 ******************************************************************************/
/*void USART1_IRQHandler(void){
}*/

/******************************************************************************
 *	タイトル ： USART2割込み関数
 *	  関数名 ： USART2_IRQHandler
 *	  戻り値 ： void型
 *	    引数 ： なし
 *	  作成者 ： 石井岳史
 *	  作成日 ： 2014/11/12
 ******************************************************************************/
/*void USART2_IRQHandler(void){
}
*/
/******************************************************************************
 *	タイトル ： USART3割込み関数
 *	  関数名 ： USART3_IRQHandler
 *	  戻り値 ： void型
 *	    引数 ： なし
 *	  作成者 ： 石井岳史
 *	  作成日 ： 2014/11/12
 ******************************************************************************/
/*void USART3_IRQHandler(void){
}
*/
/******************************************************************************
 *	タイトル ： USART設定
 *	  関数名 ： Init_USART
 *	   引数1 ： USART_TypeDef型 *USARTx  USART番号
 *	   引数2 ： unsigned int型 baudrate  ボーレート
 *	   引数3 ： GPIO_TypeDef型 *GPIOx_TX  TXポート
 *	   引数4 ： uint16_t型 pin_TX  TXピン
 *	   引数5 ： GPIO_TypeDef型 *GPIOx_RX  RXポート
 *	   引数6 ： uint16_t型 pin_RX  RXピン
 *	  作成者 ： 石井岳史
 *	  作成日 ： 2014/11/12
 ******************************************************************************/
void Init_USART(USART_TypeDef *USARTx,unsigned int baudrate,
		GPIO_TypeDef *GPIOx_TX ,uint16_t pin_TX, GPIO_TypeDef *GPIOx_RX, uint16_t pin_RX,
		uint16_t Parity,uint16_t WordLength){

	//構造体変数を宣言
	USART_InitTypeDef 	USART_InitStructure;
	NVIC_InitTypeDef 	NVIC_InitStructure;

	//モジュールストップ状態の解除
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_PeriphClock_USART(USARTx);
	//GPIOの設定
	Init_port(GPIO_Mode_AF,GPIOx_TX,pin_TX,GPIO_PuPd_UP,GPIO_OType_PP);
	Init_port(GPIO_Mode_AF,GPIOx_RX,pin_RX,GPIO_PuPd_UP,GPIO_OType_PP);

	//PINをオルタネィテブファンクションに割り当て

	GPIO_PinAFConfig(GPIOx_TX, Pin_select_source(pin_TX), GPIO_af_USART_select(USARTx));//USART1 TX/PB6
	GPIO_PinAFConfig(GPIOx_RX, Pin_select_source(pin_RX), GPIO_af_USART_select(USARTx));//USART1 RX/PB7


	//USART1の設定
	USART_InitStructure.USART_BaudRate 				= baudrate;							//ボーレートの設定
	USART_InitStructure.USART_WordLength 			= WordLength;				//ビット長
	USART_InitStructure.USART_StopBits 				= USART_StopBits_1;					//ストップビットの長さ
	USART_InitStructure.USART_Parity 				= Parity;					//パリティの有無
	USART_InitStructure.USART_HardwareFlowControl 	= USART_HardwareFlowControl_None;	//ハードウェアフロー制御の有無
	USART_InitStructure.USART_Mode 					= USART_Mode_Rx | USART_Mode_Tx;	//送信受信の選択
	USART_Init(USARTx, &USART_InitStructure);											//USART1の設定
	USART_Cmd(USARTx, ENABLE);															//USART1周辺回路の有効化
	USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);										//USART1周辺回路の割込み有効化

	//割り込み設定
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);							//優先度のビット設定
	NVIC_InitStructure.NVIC_IRQChannel 						= USART_irqn_select(USARTx);	//有効化するIRQチャンネルの指定
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 	= 2;			//割り込みの優先順位（グループ）の指定。0が最優先
	NVIC_InitStructure.NVIC_IRQChannelSubPriority 			= 0;			//割り込みの優先順位(サブ)の指定。0が最優先
	NVIC_InitStructure.NVIC_IRQChannelCmd 					= ENABLE;		//割り込みの有効化
	NVIC_Init(&NVIC_InitStructure);											//割り込み設定
}

#endif//STM32F407VG
