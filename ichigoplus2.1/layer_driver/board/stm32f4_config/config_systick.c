#ifdef STM32F407VG
#include "config_systick.h"
#include "stm32f4xx_conf.h"



#define WEAK __attribute__ ((weak))
/******************************************************************************
*	タイトル ： 割り込みハンドラ　関数場所固定
*	  関数名 ： SysTick_Handler
*	  戻り値 ： void型 
*	   引数1 ： void型  
*	  作成者 ： いいの		
*	  作成日 ： 2014/11/10
******************************************************************************/
/*void WEAK SysTick_Handler(void){
	//任意ms毎に割り込んで、1カウントする　単位:[ms]
}*/

/******************************************************************************
*	タイトル ： 割り込みタイマ設定関数
*	  関数名 ： Init_SysTick
*	  戻り値 ： void型 なし
*	   引数1 ： float型 time  割り込み周期を設定
*	  作成者 ： いいの
*	  作成日 ： 2014/11/10
******************************************************************************/
void Init_SysTick(float time){
	//systick割り込みの設定	timeの単位:[s]
	SystemCoreClockUpdate();									//SystemCoreClockの更新
	NVIC_SetPriority(SysTick_IRQn,1);						//割り込み優先度の設定		優先度を1に
	if(SysTick_Config(SystemCoreClock*time)){		//systickによる割り込み周期を設定
						//失敗時動作　書き込み自由
	}
}


#endif//STM32f407VG
