#ifndef BLINK_HPP
#define BLINK_HPP
#include "Digital.hpp"
class Blink{
private:
	int btime;
	int requestTime;
	Digital *pin;
public:
	Blink(Digital &pin);
	int setup();
	void cycle();
	int time(int time);
};


#endif//BLINK_HPP
