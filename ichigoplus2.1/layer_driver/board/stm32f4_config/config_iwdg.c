#ifdef STM32F407VG
#include "stm32f4xx_iwdg.h"
#include "config_iwdg.h"

int Init_watchdog(){
	//LSI‚ÉˆË‘¶ 40kHz
	IWDG_SetPrescaler(IWDG_Prescaler_4);
	IWDG_SetReload(1000);
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_Enable();
	return 0;
}

void watchdog_reset(){
	IWDG_ReloadCounter();
}
#endif//STM32f407VG
