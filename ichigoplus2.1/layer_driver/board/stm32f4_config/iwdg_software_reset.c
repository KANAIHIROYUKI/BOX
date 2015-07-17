#ifdef STM32F407VG
#include "iwdg_software_reset.h"

void soft_reset_durch(void){
	IWDG_SetPrescaler(IWDG_Prescaler_4);			//10kHz‚É
	IWDG_SetReload(0);							//–ñ40ms
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_Enable();
	while(1);
}
#endif//STM32F407VG
