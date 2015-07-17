#ifdef STM32F407VG
#include "watchdog.h"
#include "config_iwdg.h"
int watchdog_setup(){
	return Init_watchdog();
}

void watchdog_cycle(){
	watchdog_reset();
}
#endif
