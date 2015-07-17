#include "config_pwm.h"

#ifdef PRINTF_AVAILABLE
#include <stdio.h>
#endif
#define PERIOD_LIMIT						65535	//設定できるperiodの最大値
#define PERIOD_UNDER_LIMIT			100		//設定するperiodの最小値
#define PRESCALER_LIMIT					65535	//設定できるプリスケーラの最大値
#define PRESCALER_UNDER_LIMIT		1			//設定できるプリスケーラの最小値
#define FREQUENCY_UNDER_LIMIT	1			//設定できる周波数の最小値
#define RCC_TIMPRE			((MY_RCC->DCKCFGR>>24)&1)			//RCCの分周
#define MY_RCC                ((My_RCC_TypeDef *) RCC_BASE)		//永谷が設定したRCCのアドレス

//RCCのアドレスの構造体
typedef struct
{
  __IO uint32_t CR;            /*!< RCC clock control register,                                  Address offset: 0x00 */
  __IO uint32_t PLLCFGR;       /*!< RCC PLL configuration register,                              Address offset: 0x04 */
  __IO uint32_t CFGR;          /*!< RCC clock configuration register,                            Address offset: 0x08 */
  __IO uint32_t CIR;           /*!< RCC clock interrupt register,                                Address offset: 0x0C */
  __IO uint32_t AHB1RSTR;      /*!< RCC AHB1 peripheral reset register,                          Address offset: 0x10 */
  __IO uint32_t AHB2RSTR;      /*!< RCC AHB2 peripheral reset register,                          Address offset: 0x14 */
  __IO uint32_t AHB3RSTR;      /*!< RCC AHB3 peripheral reset register,                          Address offset: 0x18 */
  uint32_t      RESERVED0;     /*!< Reserved, 0x1C                                                                    */
  __IO uint32_t APB1RSTR;      /*!< RCC APB1 peripheral reset register,                          Address offset: 0x20 */
  __IO uint32_t APB2RSTR;      /*!< RCC APB2 peripheral reset register,                          Address offset: 0x24 */
  uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                               */
  __IO uint32_t AHB1ENR;       /*!< RCC AHB1 peripheral clock register,                          Address offset: 0x30 */
  __IO uint32_t AHB2ENR;       /*!< RCC AHB2 peripheral clock register,                          Address offset: 0x34 */
  __IO uint32_t AHB3ENR;       /*!< RCC AHB3 peripheral clock register,                          Address offset: 0x38 */
  uint32_t      RESERVED2;     /*!< Reserved, 0x3C                                                                    */
  __IO uint32_t APB1ENR;       /*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x40 */
  __IO uint32_t APB2ENR;       /*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x44 */
  uint32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                               */
  __IO uint32_t AHB1LPENR;     /*!< RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
  __IO uint32_t AHB2LPENR;     /*!< RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54 */
  __IO uint32_t AHB3LPENR;     /*!< RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58 */
  uint32_t      RESERVED4;     /*!< Reserved, 0x5C                                                                    */
  __IO uint32_t APB1LPENR;     /*!< RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
  __IO uint32_t APB2LPENR;     /*!< RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64 */
  uint32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                               */
  __IO uint32_t BDCR;          /*!< RCC Backup domain control register,                          Address offset: 0x70 */
  __IO uint32_t CSR;           /*!< RCC clock control & status register,                         Address offset: 0x74 */
  uint32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                               */
  __IO uint32_t SSCGR;         /*!< RCC spread spectrum clock generation register,               Address offset: 0x80 */
  __IO uint32_t PLLI2SCFGR;    /*!< RCC PLLI2S configuration register,                           Address offset: 0x84 */
  __IO uint32_t PLLSAICFGR;    /*!< 6.3.24 RCC PLL configuration register,                       Address offset: 0x88 */
  __IO uint32_t DCKCFGR;	   /*!< 6.3.25 RCC Dedicated Clock Configuration Register,           Address offset: 0x8c */
} My_RCC_TypeDef;

/******************************************************************************
*	タイトル ： PWMのduty比の設定
*	  関数名 ： Set_duty
*	  戻り値 ： void型
*	   引数1 ： TIM_TypeDef*型 TIMx  TIM
*	   引数2 ： uint8_t型 ch  TIMのチャンネル1-4
*	   引数3 ： float型 duty  duty比を0-1で入力
*	  作成者 ： 永谷 智貴
*	  作成日 ： 2014/11/10
******************************************************************************/
void  Set_duty(TIM_TypeDef* TIMx,uint8_t ch,float duty){
	switch(ch){
	case 1:
		TIMx->CCR1=TIMx->ARR*duty;
		break;
	case 2:
		TIMx->CCR2=TIMx->ARR*duty;
		break;
	case 3:
		TIMx->CCR3=TIMx->ARR*duty;
		break;
	case 4:
		TIMx->CCR4=TIMx->ARR*duty;
		break;
	default:
		break;
	}
}

float  Read_duty(TIM_TypeDef* TIMx,uint8_t ch){

	if(ch==1)			return TIMx->CCR1;
	else if(ch==2) 	return TIMx->CCR2;
	else if(ch==3)		return TIMx->CCR3;
	else if(ch==4)		return TIMx->CCR4;
	else						return 0;
}

/******************************************************************************
*	タイトル ： PWM初期設定
*	  関数名 ： Init_PWM
*	  戻り値 ： int型 0:設定できた 1:設定できない
*	   引数1 ： TIM_TypeDef *型 TIMx  TIMx TIMのポインタ
*	   引数2 ： GPIO_TypeDef型 *GPIOx  GPIOx GPIOのポインタ
*	   引数3 ： uint16_t型 pin  GPIO_Pin_x PINの設定
*	   引数4 ： int型 frequency  PWM周波数[Hz](整数)
*	  作成者 ： 永谷 智貴
*	  作成日 ： 2014/11/10
******************************************************************************/
int Init_PWM(TIM_TypeDef * TIMx,GPIO_TypeDef *GPIOx,uint16_t pin,int frequency)//エラーがあれば1、なければ0をreturnする
{
	long TIM_clock=0;
	int prescaler=0;
	int period=0;
	int calc_retry_flag=1;
	//float error_ratio=0;

	unsigned short i = 0;
	Pin_t	pin_state;//

	//システムクロックをRCC_Clocksで取得
	SystemCoreClockUpdate();
	RCC_ClocksTypeDef RCC_Clocks;
	RCC_GetClocksFreq(&RCC_Clocks);

	//TIMのクロックの取得
	if((TIM2<=TIMx&&TIMx<=TIM7)||(TIM12<=TIMx&&TIMx<=TIM14)){
		TIM_clock=RCC_Clocks.PCLK1_Frequency*((RCC_TIMPRE+1)*2);	//PCLK1のTIMプリスケーラ倍したらTIM2-7,12-14のクロックが出てくる
	}else{
		TIM_clock=RCC_Clocks.PCLK2_Frequency*((RCC_TIMPRE+1)*2);	//PCLK2のTIMプリスケーラ倍したら上のやつ以外のクロックが出てくる
	}
#ifdef PRINTF_AVAILABLE
	printf("Init_PWM() start.\nTIM_clock:%d,\n",TIM_clock);
#endif


	//上下の設定可能な周波数の中に納まっているか確認
	if(frequency<FREQUENCY_UNDER_LIMIT || frequency>TIM_clock/PRESCALER_UNDER_LIMIT/PERIOD_UNDER_LIMIT)
	{
#ifdef PRINTF_AVAILABLE
		printf("Error. Frequency value out of range. '%d' - '%d' Requested frequency '%d'\n",FREQUENCY_UNDER_LIMIT,TIM_clock/PRESCALER_UNDER_LIMIT/PERIOD_UNDER_LIMIT,frequency);
#endif
		return 1;													//おかしければエラー返して終了
	}

	//prescaler,periodを計算
	while(calc_retry_flag)											//periodが制限内の最大になるまでprescalerを上げているだけ。計算でも出せるけど、見た目だけはこっちのほうがきれい。
	{
		prescaler++;
		period=TIM_clock/prescaler/frequency;
		if(period<=PERIOD_LIMIT) calc_retry_flag=0;
		if(prescaler>=PRESCALER_LIMIT){								//prescalerが上の制限を超しちゃったらエラーを履くけどそうはならない。
#ifdef PRINTF_AVAILABLE
			printf("Error. Prescaler value out of range. '%d'-'%d' \n",PRESCALER_UNDER_LIMIT,PRESCALER_LIMIT);
#endif
			return 1;												//おかしければエラー返して終了
		}
	}
//	error_ratio=fabs(((float)TIM_clock/prescaler/period-(float)frequency)/(float)frequency)*100;//周波数の誤差をパーセントで計算。1%以内には納まる。
	frequency=TIM_clock/prescaler/period;							//設定した数値から算出される周波数　大体同じ。
#ifdef PRINTF_AVAILABLE
	printf("Result: \n period:%d,\n prescaler:%d,\n frequency:%d,\n\n",period,prescaler,frequency);
#endif


	//ここから普通のPWM設定
	//設定に使用する構造体の宣言
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;					//TIM設定用構造体宣言
	TIM_OCInitTypeDef TIM_OCInitStructure;							//OC設定用構造体宣言
	TIM_BDTRInitTypeDef 		TIM_BDTRInitStructure;

	//クロック供給
	RCC_PeriphClock_TIM(TIMx);//TIMクロック供給
	//クロック供給とGPIO設定
	Init_port(GPIO_Mode_AF,GPIOx,pin,GPIO_PuPd_NOPULL,GPIO_OType_PP);

	//TIM設定
	TIM_TimeBaseStructure.TIM_Period = period-1;					//計算したperiod-1
	TIM_TimeBaseStructure.TIM_Prescaler = prescaler-1;				//計算したprescaler-1
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;			//なんかここ変えても周波数変わらなかったんだよね　ナニコレ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;		//カウンターモードアップ設定
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;				//高機能タイマー用　基本0
	TIM_TimeBaseInit(TIMx,&TIM_TimeBaseStructure);					//設定書き込み
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;				//PWMモード1
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;		//アクティブレベル時の極性をHighレベルにセット
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	//タイマ出力を有効化
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;

	TIM_OC1Init(TIMx,&TIM_OCInitStructure);							//初期化
	TIM_OC1PreloadConfig(TIMx,TIM_OCPreload_Disable);				//プリロード不許可
	TIM_OC2Init(TIMx,&TIM_OCInitStructure);							//初期化
	TIM_OC2PreloadConfig(TIMx,TIM_OCPreload_Disable);				//プリロード不許可
	TIM_OC3Init(TIMx,&TIM_OCInitStructure);							//初期化
	TIM_OC3PreloadConfig(TIMx,TIM_OCPreload_Disable);				//プリロード不許可
	TIM_OC4Init(TIMx,&TIM_OCInitStructure);							//初期化
	TIM_OC4PreloadConfig(TIMx,TIM_OCPreload_Disable);				//プリロード不許可

	//使用するピンをTIMの出力として設定
//	GPIO_PinAFConfig(GPIOx,Pin_select_source(pin),Tim_select_af(TIMx));//AF設定

	/*TIM1とTIM8はSTM32の中でも高機能タイマに分類され、
	モータ制御用に使用されるBREAK入力機能がついている
	このBREAK機能の中でoutputの有効化という機能があり、デフォルトの設定で
	タイマー出力ごとにoutputが無効に設定されているためPWM信号が止まる。
	そのため、 タイマの更新イベントごとにoutputが自動的に再有効になるように設定する。*/
	if(TIMx == TIM1 || TIMx == TIM8){
		TIM_BDTRStructInit(&TIM_BDTRInitStructure);
		TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
		TIM_BDTRConfig(TIMx, &TIM_BDTRInitStructure);
		TIM_CtrlPWMOutputs(TIMx, ENABLE);
	}

	Analysis_GPIO_Pin(pin, &pin_state);

	for (i = 0; i < 16; i++){
		if (pin_state.user_pin[i] == 1){
			GPIO_PinAFConfig(GPIOx, Pin_select_source(pin_state.pin_address[i]), Tim_select_af(TIMx));//AF設定
		}
	}

	//ミニMDではしない方が良さげ
	TIMx->CCR1=0;TIMx->CCR2=0;TIMx->CCR3=0;TIMx->CCR4=0;			//一応duty全部0%にしておく

	TIM_ARRPreloadConfig(TIMx,ENABLE);//プリロード設定の適用
	TIM_Cmd(TIMx,ENABLE);//タイマー有効化

	return 0;
}

