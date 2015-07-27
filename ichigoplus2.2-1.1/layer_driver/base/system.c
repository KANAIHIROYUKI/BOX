#include "system.h"
#include "system_stm32f4xx.h"
#include "mcutime.h"
#include <stdio.h>
#include "logging.h"
#include "serial.hpp"

int system_setup(){
	SystemInit();
	setvbuf( stdout, 0, _IONBF, 0 );
	timer_setup();
	std_char_out_setup();
	return 0;
}

void whoami(){
	int check=0;
#ifdef STM32F4DISCOVERY_2LB_3_5
	printf("board,STM32F4DISCOVERY_2LB_3_5\r");
	INFO("board,STM32F4DISCOVERY_2LB_3_5");
	check++;
#endif
#ifdef STM32F4DISCOVERY_2LB_3_6
	printf("board,STM32F4DISCOVERY_2LB_3_6\r");
	INFO("board,STM32F4DISCOVERY_2LB_3_6");
	check++;
#endif
	if(check==0) WARN("No board definition.");
	if(check>=2) WARN("Multiple board definition.");
}
