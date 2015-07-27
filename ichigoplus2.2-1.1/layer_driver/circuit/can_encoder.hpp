#ifndef CAN_ENCODERPLUS_HPP
#define CAN_ENCODERPLUS_HPP

/*******************************************
 * can_encoderplus ver0.1 2015/3/5
 * Classes of can encoder node for ichigo system.
 *
 * [Dependency]
 * canplus
 * encoderplus
 * mcutime
 *
 * [Note]
 *
 * [Change history]
 * ver0.1 2015/ 3/ 5 The first version.Test.
 ******************************************/

#include "can.hpp"
#include "encoder.hpp"

class CanEncoder:public CanInterface,public Encoder{
private:
	int number;
	int encValue;
	unsigned short frequency;
	long lastGetTimeData;
	int id;
public:
	CanEncoder(Can &can,int number);
	CanEncoder(Can &can,int number,unsigned short frequencyMillisArg);
	int setup();
	int count();
	long lastReadTime();
	int canRead(int id,int number,unsigned char data[8]);
	int canId(int id);
};




#endif//CAN_ENCODERPLUS_HPP
