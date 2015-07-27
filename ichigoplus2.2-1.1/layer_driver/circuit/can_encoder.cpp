#include "can_encoder.hpp"
#include <string.h>
#include <stdio.h>
#include "util.hpp"
#include "mcutime.h"


CanEncoder::CanEncoder(Can &can,int number){
	id=0x440+number;
	lastGetTimeData=millis();
	frequency=65535;
	can.addInterface(this);
}

CanEncoder::CanEncoder(Can &can,int number,unsigned short frequencyArg){
	id=0x440+number;
	lastGetTimeData=millis();
	frequency=frequencyArg;
	can.addInterface(this);
}

int CanEncoder::setup(){
	if(canSetup()) return 1;
	unsigned char data[8];
	canSetId(id);
	data[0]=0;
	canWrite(id-0x040,1,data);
	data[0]=1;
	ushort_to_uchar2(data+1,frequency);
	canWrite(id-0x040,3,data);
	encValue=0;
	lastGetTimeData=millis();
	return 0;
}

int CanEncoder::count(){
	if(frequency==65535){
		unsigned char data[8];
		long lastTime=lastGetTimeData;
		canWrite(id,0,data);
		while(lastTime==lastGetTimeData);
	}
	return encValue;
}

long CanEncoder::lastReadTime(){
	return lastGetTimeData;
}

int CanEncoder::canRead(int id,int number,unsigned char data[8]){
	encValue=uchar4_to_int(data);
	lastGetTimeData=millis();
	return 0;
}

int CanEncoder::canId(int id){
	return id==this->id;
}
