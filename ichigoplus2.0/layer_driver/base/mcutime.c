#include "mcutime.h"
#include "config_systick.h"
#ifdef STM32F407VG

static int64_t g_timer=0;

void SysTick_Handler(void){
	//任意ms毎に割り込んで、1カウントする　単位:[ms]
	g_timer++;
}

int64_t millis(){
	return micros()/1000;
}

int64_t micros(){
	return g_timer*10;
}

void timer_setup(){
	Init_SysTick(0.00001);
}

void delay_ms(int64_t wait_time){
	int time=0;
	time=millis();
	while(millis()-time<=wait_time);
}

inline void wait(int64_t wait_time){
	delay_ms(wait_time);
}
#endif//STM32F407VG
