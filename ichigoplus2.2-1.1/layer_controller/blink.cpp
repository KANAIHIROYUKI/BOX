#include "blink.hpp"
#include "mcutime.h"
Blink::Blink(Digital &pin){
	this->pin=&pin;
	btime=0;
	requestTime=500;
}
int Blink::setup(){
	return pin->setupDigitalOut();
}
void Blink::cycle(){
	if(millis()-btime>=requestTime){
		btime=millis();
		pin->digitalToggle();
	}
}
int Blink::time(int time){
	if(time<0) return 1;
	requestTime=time/2;
	return 0;
}
