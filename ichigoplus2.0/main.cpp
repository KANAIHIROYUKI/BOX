//libraries

//application

//controller
#include "layer_controller/blink.hpp"


//base
#include "system.h"
#include "mcutime.h"

//board
#include "pin.hpp"


//circuit


int main(void)
{
	system_setup();

	Led0 ledWhite;
	ledWhite.setupDigitalOut();
	ledWhite.digitalHigh();

	Blink ledW(ledWhite);


	Led1 ledGreen;
	ledGreen.setupDigitalOut();
	ledGreen.digitalHigh();

	Led2 ledYellow;
	ledYellow.setupPwmOut(10000,0.0);
	ledYellow.pwmWrite(0.0);

	A2 ledTop;
	ledTop.setupDigitalOut();
	ledTop.digitalHigh();

	Sw0 swBlue;
	swBlue.setupDigitalIn();

	int tW=0;
	int tG=0;
	float duty = 0.0,add = -0.01;
	while(1){
		if(millis() - tW > 500){
			tW = millis();
			ledWhite.digitalToggle();
		}

		if(millis() - tG > 1000){
			tG = millis();
			ledTop.digitalToggle();
		}

		if(swBlue.digitalRead()){
			ledGreen.digitalLow();
		}else{
			ledGreen.digitalHigh();
		}
		if(duty == 0.0){
			add = 0.01;
		}else if(duty == 1.0){
			add = -0.01;
		}
		duty += add;
		ledYellow.pwmWrite(duty);
		wait(10);
	}

	return 0;
}
