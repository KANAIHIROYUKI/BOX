#include "encoder.hpp"
#include <math.h>
void Encoder::rev(bool value){
	revFlag=value;
}

void Encoder::mlt(float mltArg){//to use tire, write in diameter
	mltData=mltArg;
}

void Encoder::cpr(int cprArg){
	cprData=cprArg;
}

int Encoder::cpr(){
	return cprData;
}

float Encoder::radian(){
	count();
	return 2.0*M_PI*count()/cprData;
}

float Encoder::value(){
	return radian()*mltData;
}
