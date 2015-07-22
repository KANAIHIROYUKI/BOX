#include "blink.hpp"


int Blink::setup(){
    pin0->setupDigitalOut();
    timer = millis();
    return 0;
};

void Blink::cycle(){
    if(millis() - timer > cycleTime){
        timer = millis();
        pin0->digitalToggle();
        printf("%d\r",pin0->digitalRead());
    }
};

void Blink::time(int value){
    cycleTime=value;
};
