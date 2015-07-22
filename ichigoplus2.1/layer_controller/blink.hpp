#ifndef _BLINK_HPP
#define _BLINK_HPP

#include <stdio.h>
#include "../layer_driver/base/digital.hpp"
#include "../layer_driver/base/mcutime.h"


class Blink{
private:

public:
    Digital *pin0;
    int timer;
    int cycleTime;

    Blink(Digital &pin0){
        this->pin0=&pin0;
    };

    int setup();
    void cycle();
    void time(int value);

};

//int fnc();

#endif // _BLINK
