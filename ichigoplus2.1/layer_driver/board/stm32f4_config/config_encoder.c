#ifdef STM32F407VG
#include "config_encoder.h"

unsigned  int	g_over_under_tim1 = 0,g_over_under_tim2 = 0,
				g_over_under_tim3 = 0,g_over_under_tim4 = 0,
				g_over_under_tim5 = 0,g_over_under_tim8 = 0,
				g_over_under_tim12 = 0;

/******************************************************************************
*	タイトル ： timer1割り込み関数
*	  関数名 ： TIM1_UP_TIM10_IRQHandler
*	  戻り値 ： void型
*	    引数 ： なし
*	  作成者 ： 成宮陽生
*	  作成日 ： 2014/11/11
******************************************************************************/
void TIM1_UP_TIM10_IRQHandler(void)
{
	if( ( (TIM1 -> CR1) >> 4 ) != 0 ){
		g_over_under_tim1 --;
	}else if( ( (TIM1 -> CR1) >> 4 ) == 0 ){
		g_over_under_tim1 ++;
	}

	TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
}

/******************************************************************************
*	タイトル ： timer2割り込み関数
*	  関数名 ： TIM2_IRQHandler
*	  戻り値 ： void型
*	    引数 ： なし
*	  作成者 ： 成宮陽生
*	  作成日 ： 2014/11/11
******************************************************************************/
void TIM2_IRQHandler(void)
{
	if( ( (TIM2 -> CR1) >> 4 ) != 0 ){
		g_over_under_tim2 --;
	}else if( ( (TIM2 -> CR1) >> 4 ) == 0 ){
		g_over_under_tim2 ++;
	}

	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
}

/******************************************************************************
*	タイトル ： timer3割り込み関数
*	  関数名 ： TIM3_IRQHandler
*	  戻り値 ： void型
*	    引数 ： なし
*	  作成者 ： 成宮陽生
*	  作成日 ： 2014/11/11
******************************************************************************/
void TIM3_IRQHandler(void)
{
	if( ( (TIM3 -> CR1) >> 4 ) != 0 ){
		g_over_under_tim3 --;
	}else if( ( (TIM3 -> CR1) >> 4 ) == 0 ){
		g_over_under_tim3 ++;
	}

	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
}

/******************************************************************************
*	タイトル ： timer4割り込み関数
*	  関数名 ： TIM4_IRQHandler
*	  戻り値 ： void型
*	    引数 ： なし
*	  作成者 ： 成宮陽生
*	  作成日 ： 2014/11/11
******************************************************************************/
void TIM4_IRQHandler(void)
{
	if( ( (TIM4 -> CR1) >> 4 ) != 0 ){
		g_over_under_tim4 --;
	}else if(( (TIM4 -> CR1) >> 4 ) == 0 ){
		g_over_under_tim4 ++;
	}

	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
}

/******************************************************************************
*	タイトル ： timer5割り込み関数
*	  関数名 ： TIM5_IRQHandler
*	  戻り値 ： void型
*	    引数 ： なし
*	  作成者 ： 成宮陽生
*	  作成日 ： 2014/11/11
******************************************************************************/
void TIM5_IRQHandler(void)
{
	if( ( (TIM5 -> CR1) >> 4 ) != 0 ){
		g_over_under_tim5 --;
	}else if(( (TIM5 -> CR1) >> 4 ) == 0 ){
		g_over_under_tim5 ++;
	}

	TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
}

/******************************************************************************
*	タイトル ： timer8割り込み関数
*	  関数名 ： TIM8_UP_TIM13_IRQHandler
*	  戻り値 ： void型
*	    引数 ： なし
*	  作成者 ： 成宮陽生
*	  作成日 ： 2014/11/11
******************************************************************************/
void TIM8_UP_TIM13_IRQHandler(void)
{
	if( ( (TIM8 -> CR1) >> 4 ) != 0 ){
		g_over_under_tim8 --;
	}else if(( (TIM8 -> CR1) >> 4 ) == 0 ){
		g_over_under_tim8 ++;
	}

	TIM_ClearITPendingBit(TIM8, TIM_IT_Update);
}

/******************************************************************************
*	タイトル ： timer12割り込み関数
*	  関数名 ： TIM12_IRQHandler
*	  戻り値 ： void型
*	    引数 ： なし
*	  作成者 ： 成宮陽生
*	  作成日 ： 2014/11/11
******************************************************************************/
void TIM8_BRK_TIM12_IRQHandler(void)
{
	if( ( (TIM12 -> CR1) >> 4 ) != 0 ){
		g_over_under_tim12 --;
	}else if(( (TIM12-> CR1) >> 4 ) == 0 ){
		g_over_under_tim12 ++;
	}

	TIM_ClearITPendingBit(TIM12, TIM_IT_Update);
}


/******************************************************************************
*	タイトル ： timer1カウント呼び出し
*	  関数名 ： tim1_count
*	  戻り値 ： int型
*	    引数 ： なし
*	  作成者 ： 成宮
*	  作成日 ： 2014/11/11
******************************************************************************/
int tim1_count( void )
{
	return ( TIM_GetCounter(TIM1) + g_over_under_tim1 * 65536);
}

/******************************************************************************
*	タイトル ： timer2カウント呼び出し
*	  関数名 ： tim2_count
*	  戻り値 ： int型
*	    引数 ： なし
*	  作成者 ： 成宮
*	  作成日 ： 2014/11/11
******************************************************************************/
int tim2_count( void )
{
	return ( TIM_GetCounter(TIM2) + g_over_under_tim2 * 65536 );
}

/******************************************************************************
*	タイトル ： timer3カウント呼び出し
*	  関数名 ： tim3_count
*	  戻り値 ： int型
*	    引数 ： なし
*	  作成者 ： 成宮
*	  作成日 ： 2014/11/11
******************************************************************************/
int tim3_count( void )
{
	return ( TIM_GetCounter(TIM3) + g_over_under_tim3 * 65536 );
}

/******************************************************************************
*	タイトル ： timer4カウント呼び出し
*	  関数名 ： tim4_count
*	  戻り値 ： int型
*	    引数 ： なし
*	  作成者 ： 成宮陽生
*	  作成日 ： 2014/11/11
******************************************************************************/
int tim4_count( void )
{
	return ( TIM_GetCounter(TIM4) + g_over_under_tim4 * 65536 );
}

/******************************************************************************
*	タイトル ： timer5カウント呼び出し
*	  関数名 ： tim5_count
*	  戻り値 ： int型
*	    引数 ： なし
*	  作成者 ： 成宮陽生
*	  作成日 ： 2014/11/11
******************************************************************************/
int tim5_count( void )
{
	return ( TIM_GetCounter(TIM5) + g_over_under_tim5 * 65536 );
}

/******************************************************************************
*	タイトル ： timer8カウント呼び出し
*	  関数名 ： tim8_count
*	  戻り値 ： int型
*	    引数 ： なし
*	  作成者 ： 成宮
*	  作成日 ： 2014/11/11
******************************************************************************/
int tim8_count( void )
{
	return ( TIM_GetCounter(TIM8) + g_over_under_tim8 * 65536);
}

/******************************************************************************
*	タイトル ： timer12カウント呼び出し
*	  関数名 ： tim12_count
*	  戻り値 ： int型
*	    引数 ： なし
*	  作成者 ： 成宮
*	  作成日 ： 2014/11/11
******************************************************************************/
int tim12_count( void )
{
	return ( TIM_GetCounter(TIM12) + g_over_under_tim12 * 65536);
}

/******************************************************************************
*	タイトル ： エンコーダの設定
*	  関数名 ： Init_encoder
*	  戻り値 ： void型
*	   引数1 ： TIM_TypeDef型 *tim
*	   引数2 ： GPIO_TypeDef型 *port
*	   引数3 ： uint16_t型 pin
*	  作成者 ： 成宮陽生
*	  作成日 ： 2014/11/13
******************************************************************************/
void Init_encoder(TIM_TypeDef *tim,  GPIO_TypeDef *port, uint16_t pin)
{
	unsigned short i = 0;
    //構造体宣言
	Pin_t	pin_state;

	// Define variable ------------------------------------------------------
	uint16_t PrescalerValue = 0;

	// Define InitTypeDef
	TIM_TimeBaseInitTypeDef     TIM_TimeBaseStructure;      //TIMを設定するための構造体

	// initialize InitTypeDef
	//クロックの供給 (Supply clock source)
	RCC_PeriphClock_TIM(tim);
	
	//PORTにクロックの供給を開始 (Define gpio_config)
	Init_port(GPIO_Mode_AF, port, pin, GPIO_PuPd_UP, GPIO_OType_PP);

	Analysis_GPIO_Pin(pin, &pin_state);
	
	for(i= 0; i < 16; i ++){
		if(pin_state.user_pin[i] == 1){
			GPIO_PinAFConfig( port, Pin_select_source(pin_state.pin_address[i]), Tim_select_af(tim) );//AF設定
		}
	}

	// Set up TIM_encoder function
	// プリスケーラの値の設定
	PrescalerValue = (uint16_t) ((SystemCoreClock ) / 84000000) - 1;

	// メンバの値の設定
	TIM_TimeBaseStructure.TIM_Period = 0xffff-1;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(tim, &TIM_TimeBaseStructure);

	// プリスケーラの設定
	TIM_PrescalerConfig(tim, PrescalerValue, TIM_PSCReloadMode_Immediate);
	TIM_EncoderInterfaceConfig(tim, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

	//更新割り込み設定
	TIM_ITConfig(tim, TIM_IT_Update, ENABLE );

	//タイマー許可 (TIM enable counter)
	TIM_Cmd(tim, ENABLE);

	NVIC_Configuration(Tim_irqn_select(tim));

	Reset_encoder_over_under_flow();
}

int Select_encoder_count(TIM_TypeDef* TIMx)
{
	if(TIMx == TIM1){
		return (tim1_count());
	}else if(TIMx == TIM2){
		return (tim2_count());
	}else if(TIMx == TIM3){
		return (tim3_count());
	}else if(TIMx == TIM4){
		return (tim4_count());
	}else if(TIMx == TIM5){
		return (tim5_count());
	}else if(TIMx == TIM8){
		return (tim8_count());
	}else if(TIMx == TIM12){
		return(tim12_count());
	}else{
		return 0;
	}
}


void Reset_encoder_over_under_flow(void)
{
	g_over_under_tim1 = 0;
	g_over_under_tim2 = 0;
	g_over_under_tim3 = 0;
	g_over_under_tim4 = 0;
	g_over_under_tim5 = 0;
	g_over_under_tim8 = 0;
	g_over_under_tim12 = 0;
	TIM_SetCounter(TIM1 , 0);
	TIM_SetCounter(TIM2 , 0);
	TIM_SetCounter(TIM3 , 0);
	TIM_SetCounter(TIM4 , 0);
	TIM_SetCounter(TIM5 , 0);
	TIM_SetCounter(TIM8 , 0);
	TIM_SetCounter(TIM12 , 0);
}
#endif//STM32f407VG
